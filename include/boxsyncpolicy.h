#ifndef BOXSYNCPOLICY_H
#define BOXSYNCPOLICY_H

#include <ros/message_traits.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <yolact_ros_msgs/Detections.h>
//#include <sensor_msgs/CameraInfo.h>

namespace ros
{
    namespace message_traits
    {
        /**
         * \brief TimeStamp trait.  Override default implementation for BoundingBoxes message
         * returns &m.image_header.stamp instead of &m.header.stamp
         */

        template <>
        struct TimeStamp<darknet_ros_msgs::BoundingBoxes>
        {
          static ros::Time* pointer(typename boost::remove_const<darknet_ros_msgs::BoundingBoxes>::type &m) { return &m.image_header.stamp; }
          static ros::Time const* pointer(const darknet_ros_msgs::BoundingBoxes& m) { return &m.image_header.stamp; }
          static ros::Time value(const darknet_ros_msgs::BoundingBoxes& m) { return m.image_header.stamp; }
        };
    }

}

//typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> BoxSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> BoxSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, yolact_ros_msgs::Detections> DetectionsSyncPolicy;

#endif // BOXSYNCPOLICY_H
