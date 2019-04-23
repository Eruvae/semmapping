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
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <laser_geometry/laser_geometry.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "hypermap_msgs/SemanticMap.h"
#include <geometry_msgs/PolygonStamped.h>

#include "boxsyncpolicy.h"
#include "semanticmap.h"
#include "boost_geometry_msgs.h"

tf2_ros::Buffer tfBuffer(ros::Duration(20));
ros::Publisher detectedPgPub;
ros::Publisher observationPgPub;
ros::Publisher completeAreaPgPub;
ros::Publisher semanticMapPub;

semmapping::SemanticMap map;

inline bool operator==(const geometry_msgs::Vector3 &lhs, const geometry_msgs::Vector3 &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline bool operator==(const geometry_msgs::Quaternion &lhs, const geometry_msgs::Quaternion &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
}

inline bool operator==(const geometry_msgs::Transform &lhs, const geometry_msgs::Transform &rhs)
{
    return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation;
}

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

semmapping::polygon get2DBoxInMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::Ptr indices)
{
    semmapping::point min(DBL_MAX, DBL_MAX);
    semmapping::point maxmin;
    semmapping::point max(-DBL_MAX, -DBL_MAX);
    semmapping::point minmax;

    for (int index : indices->indices)
    {
        const pcl::PointXYZ &p = (*cloud)[index];
        if (p.x < min.x()) min.x(p.x);
        if (p.y < min.y()) min.y(p.y);
        if (p.x > max.x()) max.x(p.x);
        if (p.y > max.y()) max.y(p.y);
    }
    maxmin.x(max.x()); maxmin.y(min.y());
    minmax.x(min.x()); minmax.y(max.y());
    semmapping::polygon pg;
    semmapping::bg::append(pg.outer(), min);
    semmapping::bg::append(pg.outer(), maxmin);
    semmapping::bg::append(pg.outer(), max);
    semmapping::bg::append(pg.outer(), minmax);
    semmapping::bg::correct(pg);
    return pg;
}

semmapping::polygon getPolygonInMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::Ptr indices)
{
    // Create a set of planar coefficients with X=Y=0,Z=1 (XY-plane)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setModelCoefficients (coefficients);
    proj.setInputCloud (cloud);
    proj.setIndices(indices);
    proj.filter (*cloud_projected);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    //chull.setAlpha (10);
    chull.setDimension(2);
    chull.reconstruct (*cloud_hull);

    ROS_INFO_STREAM("Polygon found, points: " << cloud_hull->size());

    return semmapping::pclToBoost(*cloud_hull);
}

semmapping::polygon getCompleteAreaPg(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    if (cloud->size() == 0)
    {
        ROS_WARN("Point cloud was empty, could not compute area");
        return semmapping::polygon();
    }
    // Create a set of planar coefficients with X=Y=0,Z=1 (XY-plane)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setModelCoefficients (coefficients);
    proj.setInputCloud (cloud);
    proj.filter (*cloud_projected);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_projected);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_filtered);
    chull.setDimension(2);
    chull.reconstruct (*cloud_hull);

    return semmapping::pclToBoost(*cloud_hull);
}

semmapping::polygon getConvexHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    if (cloud->size() == 0)
    {
        ROS_WARN("Point cloud was empty, could not compute area");
        return semmapping::polygon();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setDimension(2);
    chull.reconstruct (*cloud_hull);

    return semmapping::pclToBoost(*cloud_hull);
}


void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::search::Search<pcl::PointXYZ>::Ptr tree, pcl::PointCloud <pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
}

pcl::PointIndices::Ptr getObjectPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::search::Search<pcl::PointXYZ>::Ptr tree,
                                       pcl::PointCloud <pcl::Normal>::Ptr normals, const darknet_ros_msgs::BoundingBox &box)
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

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setIndices(box.ymin, box.xmin, box.ymax - box.ymin/* + 1*/, box.xmax - box.xmin/* + 1*/);

    if (reg.getIndices()->size() == 0)
    {
        ROS_WARN("No points in point cloud within bounding box");
        return nullptr;
    }

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

geometry_msgs::Vector3Stamped::Ptr toVectorMsg(const cv::Point3d &point, const image_geometry::PinholeCameraModel &cam)
{
    geometry_msgs::Vector3Stamped::Ptr vec(new geometry_msgs::Vector3Stamped);
    vec->vector.x = point.x;
    vec->vector.y = point.y;
    vec->vector.z = point.z;
    vec->header.frame_id = cam.tfFrame();
    vec->header.stamp = cam.stamp();
    return vec;
}

inline semmapping::point calculateXYPlaneIntersection(const geometry_msgs::Vector3 &orig, const geometry_msgs::Vector3 &dir)
{
    double factor = - orig.z / dir.z;
    return semmapping::point(orig.x + factor * dir.x, orig.y + factor * dir.y);
}

inline semmapping::point calculateVisibilityEndPoint(const semmapping::point &startP, const geometry_msgs::Vector3 &dir, double dist = 3)
{
    double dir_len_plane = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    double factor = dist / dir_len_plane;
    return semmapping::point(startP.x() + factor * dir.x, startP.y() + factor * dir.y);
}

void processBoxes(const sensor_msgs::PointCloud2::Ptr &cloud, const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxes)
{
    static geometry_msgs::Transform lastCamPos;
    geometry_msgs::TransformStamped camPos;
    try
    {
        camPos = tfBuffer.lookupTransform("map", cloud->header.frame_id, cloud->header.stamp, ros::Duration(0));
    }
    catch (tf2::TransformException ex)
    {
        ROS_WARN_STREAM("Could not read transform: " << ex.what());
        return;
    }
    if (camPos.transform == lastCamPos)
    {
        ROS_INFO("Robot has not moved, not processing boxes");
        return;
    }
    lastCamPos = camPos.transform;

    //ROS_INFO_STREAM("Time stamps comp - boxes: " << boxes->header.stamp << "; Image header: " << boxes->image_header.stamp << "; cloud: " << cloud->header.stamp);
    //ROS_INFO_STREAM("Frames: " << boxes->image_header.frame_id << "; " << cloud->header.frame_id);
    //tfBuffer.transform(*cloud, *cloud, "map");
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pclCloud);

    if (pclCloud->empty())
    {
        ROS_WARN("Point cloud is empty, not processing boxes");
        return;
    }

    ROS_INFO_STREAM("Point cloud size: " << pclCloud->size());

    Eigen::Affine3d camPosEigen = tf2::transformToEigen(camPos.transform);
    pcl::transformPointCloud(*pclCloud, *pclCloud, camPosEigen);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    computeNormals(pclCloud, tree, normals);

    for (const darknet_ros_msgs::BoundingBox &box : boxes->bounding_boxes)
    {
        //std::cout << "Bounding box: (" << box.xmin << " | " << box.xmax << "); (" << box.ymin << " | " << box.ymax << ")" << std::endl;
        pcl::PointIndices::Ptr indices = getObjectPoints(pclCloud, tree, normals, box);
        if (indices == nullptr || indices->indices.size() == 0)
            continue;

        //semmapping::polygon res_pg = get2DBoxInMap(pclCloud, indices);
        semmapping::polygon res_pg = getPolygonInMap(pclCloud, indices);

        // DEBUG: publish detected polygon
        geometry_msgs::PolygonStamped message;
        message.polygon = semmapping::boostToPolygonMsg(res_pg);
        message.header.frame_id = "map";
        message.header.stamp = cloud->header.stamp;
        detectedPgPub.publish(message);

        map.addEvidence(box.Class, res_pg);
        hypermap_msgs::SemanticMap::Ptr map_msg = map.createMapMessage();
        semanticMapPub.publish(map_msg);
    }

    semmapping::polygon comp_area = getCompleteAreaPg(pclCloud);
    if (comp_area.outer().size() > 0)
    {
        geometry_msgs::PolygonStamped comp_area_msg;
        comp_area_msg.polygon = semmapping::boostToPolygonMsg(comp_area);
        comp_area_msg.header.frame_id = "map";
        comp_area_msg.header.stamp = cloud->header.stamp;
        completeAreaPgPub.publish(comp_area_msg);

        map.removeEvidence(comp_area);
    }
}

void removeMissing(const sensor_msgs::CameraInfo::ConstPtr &camInfo, const sensor_msgs::LaserScan::ConstPtr &laserScan)
{
    static geometry_msgs::Transform lastCamPos;
    geometry_msgs::TransformStamped camPos;
    try
    {
        camPos = tfBuffer.lookupTransform("map", camInfo->header.frame_id, camInfo->header.stamp, ros::Duration(0));
    }
    catch (tf2::TransformException ex)
    {
        ROS_WARN_STREAM("Could not read transform: " << ex.what());
        return;
    }
    if (camPos.transform == lastCamPos)
    {
        ROS_INFO("Robot has not moved, not removing evidence");
        return;
    }
    lastCamPos = camPos.transform;

    image_geometry::PinholeCameraModel camModel;
    camModel.fromCameraInfo(camInfo);

    //cv::Point2d leftP(0, 0);
    //cv::Point2d rightP(camModel.cameraInfo().width, 0);
    cv::Point2d upLeft(0, camModel.cameraInfo().height);
    cv::Point2d upRight(camModel.cameraInfo().width, camModel.cameraInfo().height);
    //cv::Point2d center(camModel.cameraInfo().width / 2, camModel.cameraInfo().height / 2);

    //cv::Point3d rayLeft = camModel.projectPixelTo3dRay(leftP);
    //cv::Point3d rayRight = camModel.projectPixelTo3dRay(rightP);
    cv::Point3d rayUpLeft = camModel.projectPixelTo3dRay(upLeft);
    cv::Point3d rayUpRight = camModel.projectPixelTo3dRay(upRight);
    //cv::Point3d rayCenter = camModel.projectPixelTo3dRay(center);

    //ROS_INFO_STREAM("Points: " << rayLeft << rayRight << rayUpLeft << rayUpRight << rayCenter);
    // Output: Points: [-0.60211, -0.802396, 1][0.599606, -0.802396, 1][-0.60211, 0.799892, 1][0.599606, 0.799892, 1][-0.00125179, -0.00125179, 1]

    //geometry_msgs::Vector3Stamped::Ptr vecLeft = toVectorMsg(rayLeft, camModel);
    //geometry_msgs::Vector3Stamped::Ptr vecRight = toVectorMsg(rayRight, camModel);
    geometry_msgs::Vector3Stamped::Ptr vecUpLeft = toVectorMsg(rayUpLeft, camModel);
    geometry_msgs::Vector3Stamped::Ptr vecUpRight = toVectorMsg(rayUpRight, camModel);


    //tfBuffer.transform(*vecLeft, *vecLeft, "map");
    //tfBuffer.transform(*vecRight, *vecRight, "map");
    //tfBuffer.transform(*vecUpLeft, *vecUpLeft, "map");
    //tfBuffer.transform(*vecUpRight, *vecUpRight, "map");
    tf2::doTransform(*vecUpLeft, *vecUpLeft, camPos);
    tf2::doTransform(*vecUpRight, *vecUpRight, camPos);

    //ROS_INFO_STREAM("Left global: " << vecLeft->vector.x << ", " << vecLeft->vector.y << ", " << vecLeft->vector.z);
    //ROS_INFO_STREAM("Right global: " << vecRight->vector.x << ", " << vecRight->vector.y << ", " << vecRight->vector.z);
    //ROS_INFO_STREAM("UpLeft global: " << vecUpLeft->vector.x << ", " << vecUpLeft->vector.y << ", " << vecUpLeft->vector.z);
    //ROS_INFO_STREAM("UpRight global: " << vecUpRight->vector.x << ", " << vecUpRight->vector.y << ", " << vecUpRight->vector.z);

    //geometry_msgs::TransformStamped camPos = tfBuffer.lookupTransform("map", camModel.tfFrame(), camModel.stamp(), ros::Duration(0));
    ROS_INFO_STREAM("Cam pos: " << camPos.transform.translation.x << ", " << camPos.transform.translation.y << ", " << camPos.transform.translation.z);
    /*
    [ INFO] [1554890337.562063526, 36.131000000]: Left global: -1.12738, 0.303181, 0.802171
    [ INFO] [1554890337.562079055, 36.131000000]: Right global: -0.261476, 1.13645, 0.802171
    [ INFO] [1554890337.562095215, 36.131000000]: UpLeft global: -1.12713, 0.302922, -0.800116
    [ INFO] [1554890337.562110103, 36.131000000]: UpRight global: -0.261227, 1.13619, -0.800116
    [ INFO] [1554890337.562140750, 36.131000000]: Cam pos: -0.0337588, 0.120642, 1.20268
     */

    semmapping::point p1 = calculateXYPlaneIntersection(camPos.transform.translation, vecUpLeft->vector);
    semmapping::point p2 = calculateXYPlaneIntersection(camPos.transform.translation, vecUpRight->vector);
    semmapping::point p3 = calculateVisibilityEndPoint(p2, vecUpRight->vector);
    semmapping::point p4 = calculateVisibilityEndPoint(p1, vecUpLeft->vector);

    semmapping::polygon obPg;
    semmapping::bg::append(obPg.outer(), p1);
    semmapping::bg::append(obPg.outer(), p2);
    semmapping::bg::append(obPg.outer(), p3);
    semmapping::bg::append(obPg.outer(), p4);
    semmapping::bg::correct(obPg);

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 laserCloud;
    try
    {
        projector.transformLaserScanToPointCloud("map", *laserScan, laserCloud, tfBuffer);
    }
    catch (tf2::TransformException ex)
    {
        ROS_WARN_STREAM("Could not read transform: " << ex.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclLaserCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(laserCloud, *pclLaserCloud);
    semmapping::polygon laserPg = getConvexHull(pclLaserCloud);

    /*if (laserPg.outer().size() > 0)
    {
        geometry_msgs::PolygonStamped comp_area_msg;
        comp_area_msg.polygon = semmapping::boostToPolygonMsg(laserPg);
        comp_area_msg.header.frame_id = "map";
        comp_area_msg.header.stamp = laserScan->header.stamp;
        completeAreaPgPub.publish(comp_area_msg);
    }*/

    semmapping::multi_polygon intersectionArea;
    semmapping::bg::intersection(obPg, laserPg, intersectionArea);
    //semmapping::bg::convex_hull(intersectionArea, obPg);

    if (intersectionArea.size() > 0 && intersectionArea[0].outer().size() > 0)
    {
        // DEBUG: publish observation area
        geometry_msgs::PolygonStamped message;
        message.polygon = semmapping::boostToPolygonMsg(intersectionArea[0]);
        message.header.frame_id = "map";
        message.header.stamp = camModel.stamp();
        observationPgPub.publish(message);

        map.removeEvidence(obPg);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tfBuffer);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, "/sensorring_cam3d/depth/points", 1);
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, "map", 20, nh);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> boundingBoxSub(nh, "/darknet_ros/bounding_boxes", 1);

  message_filters::Synchronizer<BoxSyncPolicy> sync(BoxSyncPolicy(10), tfCloudFilter, boundingBoxSub);

  //message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub(nh, "/sensorring_cam3d/depth/camera_info", 1);
  //tf2_ros::MessageFilter<sensor_msgs::CameraInfo> tfCamFilter(cameraInfoSub, tfBuffer, "map", 10, nh);

  //message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSub(nh, "/scan_unified", 1);

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::LaserScan> CamLaserSyncPolicy;
  //message_filters::Synchronizer<CamLaserSyncPolicy> syncCamLaser(CamLaserSyncPolicy(10), tfCamFilter, laserScanSub);

  sync.registerCallback(processBoxes);
  //syncCamLaser.registerCallback(removeMissing);


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
  observationPgPub = nh.advertise<geometry_msgs::PolygonStamped>("observation_pg", 1);
  completeAreaPgPub = nh.advertise<geometry_msgs::PolygonStamped>("complete_area_pg", 1);
  semanticMapPub = nh.advertise<hypermap_msgs::SemanticMap>("semantic_map", 1);

  ros::spin();
}
