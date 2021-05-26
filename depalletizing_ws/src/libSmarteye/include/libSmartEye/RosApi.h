//
// Created by dzp on 2020/9/17.
//

#ifndef SMART_EYE_ROS_API_H
#define SMART_EYE_ROS_API_H

#include <ros/ros.h>

// Publishers
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>

// SmartEye
#include "SmartEyeAPI/ProcessController.h"

// Services
#include <smarteye/GetPointCloud.h>

//OpenCv
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

namespace smart_eye {

  class RosApi {
  public:
    RosApi(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~RosApi();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ProcessController controller_;
    bool init_;

    std::string camera_frame_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceServer get_pointcloud_srv_;

    bool getPointCloudSrvCb(smarteye::GetPointCloud::Request& req,
                            smarteye::GetPointCloud::Response& res);

    bool initConnect(std::string serial_no);

    bool convert2PCLPointCloud(const PointCloud_SE_Ptr& se_cloud,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud);

  };

}

#endif //SMART_EYE_ROS_API_H