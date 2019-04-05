#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "hypermap_msgs/SemanticMap.h"
#include <geometry_msgs/PolygonStamped.h>

#include "boxsyncpolicy.h"
#include "boost_geometry_msgs.h"

tf2_ros::Buffer tfBuffer;
ros::Publisher detectedPgPub;

using namespace semmapping;

struct SemanticObject
{
  std::string name;
  std::vector<polygon> shapes;
  std::vector<int> shape_certainties;
  int exist_certainty;
  box bounding_box;
  std::vector<std::string> tags;
  std::vector<double> confidence;
};

typedef std::pair<box, size_t> rtree_entry;

size_t next_index;
bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
std::map<size_t, SemanticObject> objectList;

//volatile bool image_received = false;

//hypermap_msgs::SemanticMap map;
//sensor_msgs::Image depth;
//sensor_msgs::CameraInfo info;
//sensor_msgs::PointCloud2 depthCloud;
//pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);
//darknet_ros_msgs::BoundingBoxes boxes;

/*inline float depthValue(const sensor_msgs::Image &img, size_t x, size_t y)
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
    int nanVals = 0;
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            const pcl::PointXYZ &p = cloud.at(x, y);
            if (!std::isnan(p.z) && !std::isinf(p.z))
                depths.push_back(p.z);
            else
                nanVals++;
        }
    }
    std::sort(depths.begin(), depths.end());
    std::string filename = std::string("depths-") + box.Class + std::to_string(file_num) + std::string(".csv");
    std::ofstream ofile(filename);
    for (float val : depths)
    {
        ofile << val << std::endl;
    }
    ofile.close();
    file_num++;
    std::cout << "Nan vals: " << nanVals << std::endl;
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
            //analyzeDepthInBoxPC(*depthCloud, box);
            pcl::PointIndices::Ptr indices = getObjectPoints(depthCloud, box);
            geometry_msgs::Polygon::Ptr res_pg = get2DBoxInMap(depthCloud, indices);
            geometry_msgs::PolygonStamped message;
            message.polygon = std::move(*res_pg);
            message.header.frame_id = "map";
            message.header.stamp = ros::Time::now();
            detectedPgPub.publish(message);

        }
    }
}

void receiveDepthCloud(const sensor_msgs::PointCloud2ConstPtr &cl)
//void receiveDepthCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cl)
{
    //depthCloud = *cl;
    //image_received = true;
    sensor_msgs::PointCloud2 cl_out;
    tfBuffer.transform(*cl, cl_out, "map");
    //tf2::doTransform(*cl, cl_out, tfBuffer.lookupTransform("map", tf2::getFrameId(*cl), ros::Time(0), ros::Duration(0)));
    pcl::fromROSMsg(cl_out, *depthCloud);
    image_received = true;
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
}*/

geometry_msgs::Polygon::Ptr get2DBoxInMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::Ptr indices)
{
    geometry_msgs::Point32 min;
    geometry_msgs::Point32 maxmin;
    geometry_msgs::Point32 max;
    geometry_msgs::Point32 minmax;
    min.x = DBL_MAX; min.y = DBL_MAX;
    max.x = -DBL_MAX; max.y = -DBL_MAX;

    for (int index : indices->indices)
    {
        const pcl::PointXYZ &p = (*cloud)[index];
        if (p.x < min.x) min.x = p.x;
        if (p.y < min.y) min.y = p.y;
        if (p.x > max.x) max.x = p.x;
        if (p.y > max.y) max.y = p.y;
    }
    maxmin.x = max.x; maxmin.y = min.y;
    minmax.x = min.x; minmax.y = max.y;
    geometry_msgs::Polygon::Ptr pg(new geometry_msgs::Polygon);
    pg->points.push_back(min);
    pg->points.push_back(maxmin);
    pg->points.push_back(max);
    pg->points.push_back(minmax);
    return pg;
}

pcl::PointIndices::Ptr getObjectPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const darknet_ros_msgs::BoundingBox &box)
{
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> boxFilter;
    boxFilter.setInputCloud(cloud);
    boxFilter.setKeepOrganized(true);
    pcl::PointIndices::Ptr boxIndices(new pcl::PointIndices);
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            boxIndices->indices.push_back(y * cloud->width + x);
        }
    }
    boxFilter.setIndices(boxIndices);
    boxFilter.filter(*object);*/

    /*pcl::IndicesPtr indices (new std::vector <int>);
    for (size_t x = box.xmin; x <= box.xmax; x++)
    {
        for (size_t y = box.ymin; y <= box.ymax; y++)
        {
            indices->push_back(y * cloud->width + x);
        }
    }*/

    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setIndices(box.ymin, box.xmin, box.ymax - box.ymin/* + 1*/, box.xmax - box.xmin/* + 1*/);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "Cluster sizes: ";
    for (auto &cluster : clusters)
        std::cout << cluster.indices.size() << ", ";
    std::cout << std::endl;


    /*pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }*/

    size_t maxInd = 0, maxSize = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        if (clusters[i].indices.size() > maxSize)
        {
            maxInd = i;
            maxSize = clusters[i].indices.size();
        }
    }

    pcl::PointIndices::Ptr res(new pcl::PointIndices);
    *res = std::move(clusters[maxInd]);
    return res;
}

void processBoxes(const sensor_msgs::PointCloud2::Ptr &cloud, const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxes)
{
    ROS_INFO_STREAM("Time stamps comp - boxes: " << boxes->header.stamp << "; Image header: " << boxes->image_header.stamp << "; cloud: " << cloud->header.stamp);
    tfBuffer.transform(*cloud, *cloud, "map");
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pclCloud);
    for (const darknet_ros_msgs::BoundingBox &box : boxes->bounding_boxes)
    {
        //std::cout << "Bounding box: (" << box.xmin << " | " << box.xmax << "); (" << box.ymin << " | " << box.ymax << ")" << std::endl;
        pcl::PointIndices::Ptr indices = getObjectPoints(pclCloud, box);
        geometry_msgs::Polygon::Ptr res_pg = get2DBoxInMap(pclCloud, indices);
        geometry_msgs::PolygonStamped message;
        message.polygon = std::move(*res_pg);
        message.header.frame_id = "map";
        message.header.stamp = ros::Time::now();
        detectedPgPub.publish(message);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, "/sensorring_cam3d/depth/points", 1);
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfFilter(depthCloudSub, tfBuffer, "map", 10, nh);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> boundingBoxSub(nh, "/darknet_ros/bounding_boxes", 1);

  message_filters::Synchronizer<BoxSyncPolicy> sync(BoxSyncPolicy(10), tfFilter, boundingBoxSub);

  sync.registerCallback(processBoxes);


  //tfFilter.registerCallback(processBoxes);


  //tf2_ros::MessageFilter<sensor_msgs::PointCloud2> depthCloudFilter(depthCloudSub, tfBuffer, "map", 10, 0);

  //ros::Subscriber depthImageSub = nh.subscribe("/sensorring_cam3d/depth/image_raw", 100, receiveDepthImage);
  //ros::Subscriber boundingBoxSub = nh.subscribe("/darknet_ros/bounding_boxes", 100, receiveBoundingBoxes);
  //ros::Subscriber depthCloudSub = nh.subscribe("/sensorring_cam3d/depth/points", 100, receiveDepthCloud);
  //depthCloudSub.subscribe(nh, "/sensorring_cam3d/depth/points", 10);

  /*//ros::Subscriber depthImageSub = nh.subscribe("/gibson_ros/camera/depth/image", 100, receiveDepthImage);
  ros::Subscriber boundingBoxSub = nh.subscribe("/darknet_ros/bounding_boxes", 100, receiveBoundingBoxes);
  //ros::Subscriber depthCloudSub = nh.subscribe("/gibson_ros/camera/depth_registered/points", 100, receiveDepthCloud);
  depthCloudSub.subscribe(nh, "/gibson_ros/camera/depth_registered/points", 10);*/

  //depthCloudFilter.registerCallback(receiveDepthCloud);

  detectedPgPub = nh.advertise<geometry_msgs::PolygonStamped>("detected_pg", 1);

  ros::spin();
}
