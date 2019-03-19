#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "hypermap_msgs/SemanticMap.h"

tf2_ros::Buffer tfBuffer;

hypermap_msgs::SemanticMap map;
sensor_msgs::Image depth;
sensor_msgs::CameraInfo info;
//sensor_msgs::PointCloud2 depthCloud;
pcl::PointCloud<pcl::PointXYZ> depthCloud;
darknet_ros_msgs::BoundingBoxes boxes;

inline float depthValue(const sensor_msgs::Image &img, size_t x, size_t y)
{
    const float *data = reinterpret_cast<const float*>(img.data.data());
    return *(data + img.step*y + x);
}

double averageDepth(const sensor_msgs::Image &img, const darknet_ros_msgs::BoundingBox &box)
{
    double depthSum = 0;
    double minDepth = DBL_MAX;
    double maxDepth = -DBL_MAX;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            float dval = depthValue(img, x, y);
            depthSum += dval;
            if (dval > maxDepth)
                maxDepth = dval;
            if (dval < minDepth)
                minDepth = dval;
        }
    }
    size_t px_cnt = (box.xmax - box.xmin + 1) * (box.ymax - box.ymin + 1);
    double avg = depthSum / px_cnt;

    double sqr_err = 0;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            float dval = depthValue(img, x, y);
            sqr_err += (dval - avg)*(dval - avg);
        }
    }
    double std_div = std::sqrt(sqr_err / (px_cnt - 1));

    std::cout << "Average: " << avg << ", Min: " << minDepth << ", Max: " << maxDepth << ", Std: " << std_div << std::endl;

    return avg;
}

void receiveDepthImage(const sensor_msgs::ImageConstPtr &img)
{
    depth = *img;
}

void receiveBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr &bx)
{
    boxes = *bx;
    for (const darknet_ros_msgs::BoundingBox &box : boxes.bounding_boxes)
    {
        double avg = averageDepth(depth, box);
    }
}

void receiveDepthCloud(const sensor_msgs::PointCloud2ConstPtr &cl)
{
    sensor_msgs::PointCloud2 cl_out;
    tfBuffer.transform(*cl, cl_out, map.header.frame_id);
    pcl::fromROSMsg(cl_out, depthCloud);
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

  ros::Subscriber depthImageSub = nh.subscribe("/camera/depth/image", 100, receiveDepthImage);
  ros::Subscriber boundingBoxSub = nh.subscribe("/darknet/bounding_boxes", 100, receiveBoundingBoxes);
  ros::Subscriber depthCloudSub = nh.subscribe("/sensorring_cam3d_front/depth/points", 100, receiveDepthCloud);

  ros::spin();
}
