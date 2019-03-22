#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "hypermap_msgs/SemanticMap.h"

tf2_ros::Buffer tfBuffer;

volatile bool image_received = false;

hypermap_msgs::SemanticMap map;
sensor_msgs::Image depth;
sensor_msgs::CameraInfo info;
//sensor_msgs::PointCloud2 depthCloud;
pcl::PointCloud<pcl::PointXYZ> depthCloud;
darknet_ros_msgs::BoundingBoxes boxes;

inline float depthValue(const sensor_msgs::Image &img, size_t x, size_t y)
{
    if (img.encoding == "32FC1")
    {
        const float *data = reinterpret_cast<const float*>(img.data.data());
        return *(data + img.step*y + x);
    }
    else if (img.encoding == "16UC1")
    {
        const uint16_t *data = reinterpret_cast<const uint16_t*>(img.data.data());
        uint16_t val = *(data + img.step*y + x);
        if (val == 0)
            return NAN;
        else
            return (float) val;
    }
    else
    {
        ROS_ERROR_STREAM("Wrong image format: " << img.encoding);
        return NAN;
    }
}

double averageDepth(const sensor_msgs::Image &img, const darknet_ros_msgs::BoundingBox &box)
{
    double depthSum = 0;
    double minDepth = DBL_MAX;
    double maxDepth = -DBL_MAX;
    size_t px_cnt = 0;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            float dval = depthValue(img, x, y);
            if (!std::isnan(dval) && !std::isinf(dval))
            {
                depthSum += dval;
                px_cnt++;
                if (dval > maxDepth)
                    maxDepth = dval;
                if (dval < minDepth)
                    minDepth = dval;
            }
        }
    }
    //size_t px_cnt = (box.xmax - box.xmin + 1) * (box.ymax - box.ymin + 1);
    double avg = depthSum / px_cnt;

    double sqr_err = 0;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            float dval = depthValue(img, x, y);
            if (!std::isnan(dval) && !std::isinf(dval))
                sqr_err += (dval - avg)*(dval - avg);
        }
    }
    double std_div = std::sqrt(sqr_err / (px_cnt - 1));

    std::cout << "Average: " << avg << ", Min: " << minDepth << ", Max: " << maxDepth << ", Std: " << std_div <<  ", PxCnt: " << px_cnt << std::endl;

    return avg;
}

void analyzeDepthInBox(const sensor_msgs::Image &img, const darknet_ros_msgs::BoundingBox &box)
{
    static int file_num = 0;
    std::vector<float> depths;
    int nanVals = 0;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            float dval = depthValue(img, x, y);
            if (!std::isnan(dval) && !std::isinf(dval))
                depths.push_back(dval);
            else
                nanVals++;
        }
    }
    std::sort(depths.begin(), depths.end());
    std::string filename = std::string("depths-") + std::to_string(file_num) + std::string(".csv");
    std::ofstream ofile(filename);
    for (float val : depths)
    {
        ofile << val << std::endl;
    }
    ofile.close();
    file_num++;
    std::cout << "Nan vals: " << nanVals << std::endl;
}

void analyzeDepthInBoxPC(const pcl::PointCloud<pcl::PointXYZ> &cloud, const darknet_ros_msgs::BoundingBox &box)
{
    static int file_num = 0;
    std::vector<float> depths;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            const pcl::PointXYZ &p = cloud.at(x, y);
            depths.push_back(p.z);
        }
    }
    std::sort(depths.begin(), depths.end());
    std::ofstream out("depths-" + file_num++);
    for (float val : depths)
    {
        out << val << std::endl;
    }
    out.close();
}

void receiveDepthImage(const sensor_msgs::ImageConstPtr &img)
{
    depth = *img;
    //image_received = true;
}

void receiveBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr &bx)
{
    boxes = *bx;
    if (image_received)
    {
        for (const darknet_ros_msgs::BoundingBox &box : boxes.bounding_boxes)
        {
            std::cout << "Bounding box: (" << box.xmin << " | " << box.xmax << "); (" << box.ymin << " | " << box.ymax << ")" << std::endl;
            std::cout << "Image size: " << depth.width << ", " << depth.height << std::endl;
            analyzeDepthInBoxPC(depthCloud, box);
        }
    }
}

//void receiveDepthCloud(const sensor_msgs::PointCloud2ConstPtr &cl)
void receiveDepthCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cl)
{
    depthCloud = *cl;
    image_received = true;
    /*sensor_msgs::PointCloud2 cl_out;
    tfBuffer.transform(*cl, cl_out, map.header.frame_id);
    pcl::fromROSMsg(cl_out, depthCloud);*/
}

void pclExtractPoints(const pcl::PointCloud<pcl::PointXYZ> depthCloud, const darknet_ros_msgs::BoundingBox &box)
{
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::fromROSMsg(depthCloud, cloud);

    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            const pcl::PointXYZ &p = depthCloud.at(x, y);
        }
    }
}

void transformBoxToMap()
{
    geometry_msgs::TransformStamped trans = tfBuffer.lookupTransform(map.header.frame_id, info.header.frame_id, info.header.stamp, ros::Duration(1));
    geometry_msgs::Vector3 cam_orig = trans.transform.translation;
    geometry_msgs::Quaternion cam_rot = trans.transform.rotation;

    tf2::Quaternion cam_rot_tf;
    tf2::convert(cam_rot, cam_rot_tf);
    tf2::Matrix3x3 mat(cam_rot_tf);

    double depth = 5.0; // TODO: actual depth

    tf2::Vector3 z_dir = mat.getColumn(2);
    tf2::Vector3 z_dist = depth * z_dir;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  /*ros::Subscriber depthImageSub = nh.subscribe("/camera/depth/image", 100, receiveDepthImage);
  ros::Subscriber boundingBoxSub = nh.subscribe("/darknet_ros/bounding_boxes", 100, receiveBoundingBoxes);
  ros::Subscriber depthCloudSub = nh.subscribe("/sensorring_cam3d_front/depth/points", 100, receiveDepthCloud);*/

  ros::Subscriber depthImageSub = nh.subscribe("/gibson_ros/camera/depth/image", 100, receiveDepthImage);
  ros::Subscriber boundingBoxSub = nh.subscribe("/darknet_ros/bounding_boxes", 100, receiveBoundingBoxes);
  ros::Subscriber depthCloudSub = nh.subscribe("/gibson_ros/camera/depth_registered/points", 100, receiveDepthCloud);

  ros::spin();
}
