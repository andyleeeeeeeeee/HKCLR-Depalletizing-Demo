//
// Created by andylee on 2021/2/24.
//

#include <cmath>
#include <libSmartEye/RosApi.h>
#include <unistd.h>

namespace smart_eye
{
  RosApi::RosApi(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), init_(false)
  {
    initConnect("");

    // Initialize get point cloud service
    std::string get_pointcloud_srv_id;
    pnh_.getParam("get_pointcloud_srv_id", get_pointcloud_srv_id);
    if (get_pointcloud_srv_id.empty())
    {
      throw std::runtime_error("get_pointcloud_srv_id is None");
    }
    get_pointcloud_srv_ = nh_.advertiseService(get_pointcloud_srv_id, &RosApi::getPointCloudSrvCb, this);
    ROS_INFO_STREAM("Advertising service " << get_pointcloud_srv_id);

    // Initialize pointcloud publisher
    std::string pub_pointcloud_msg_id;
    pnh_.getParam("pub_pointcloud_msg_id", pub_pointcloud_msg_id);
    if (pub_pointcloud_msg_id.empty())
    {
      throw std::runtime_error("pub_pointcloud_msg_id is None");
    }
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_pointcloud_msg_id, 1);
    ROS_INFO_STREAM("Publish point cloud to " << pub_pointcloud_msg_id);

    // Initialize camera frame
    pnh_.getParam("camera_frame", camera_frame_);
  }

  RosApi::~RosApi() {}

  bool RosApi::initConnect(std::string serial_no){
    printf("SmartEyeCameraController connect: %s ", serial_no.c_str());
    int result;
    result = controller_.initDevice();
    ROS_WARN("result value is: %d. If it is not 0, you should check common_type.h to debug \n", result);
    if (SE_STATUS_SUCCESS == result)
    {
      // set z range
      float min_z, max_z;
      pnh_.getParam("min_depth", min_z);
      pnh_.getParam("max_depth", max_z);
      controller_.setZRange(min_z, max_z); // in millimeter
      ROS_WARN("Initialized depth range: %.3f, %.3f mm \n", min_z, max_z);

      // set xy range
      float min_x, max_x, min_y, max_y;
      pnh_.getParam("min_length", min_x);
      pnh_.getParam("max_length", max_x);
      pnh_.getParam("min_width", min_y);
      pnh_.getParam("max_width", max_y);
      controller_.setXYRange(min_x, max_x, min_y, max_y); // in millimeter
      ROS_WARN("Initialized length range: %.3f, %.3f mm \n", min_x, max_x);
      ROS_WARN("Initialized width range: %.3f, %.3f mm \n", min_y, max_y);

      // set exposure time
      int ExposureTime2D, ExposureTime3D;
      pnh_.getParam("exp_time_2D", ExposureTime2D);
      pnh_.getParam("exp_time_3D", ExposureTime3D);
      controller_.setExposureTime2D(ExposureTime2D);
      controller_.setExposureTime3D(ExposureTime3D);
      ROS_WARN("Initialized ExposureTime2D to: %d ms \n", ExposureTime2D);
      ROS_WARN("Initialized ExposureTime3D to: %d ms \n", ExposureTime3D);

      // set MaxCoeff
      float MaxCoeff = 0.95;
      controller_.setMaxCoeff(MaxCoeff);
      ROS_WARN("Initialized MaxCoeff to: %f \n", MaxCoeff);

      init_ = true;
      ROS_WARN("Initialize device succeed.");
    }
    else
    {
      init_ = false;
      ROS_ERROR("Initialize device failed.");
    }
  }

  bool RosApi::getPointCloudSrvCb(smarteye::GetPointCloud::Request &req,
                                  smarteye::GetPointCloud::Response &res)
  {
    if (!init_)
    {
      res.result_status = res.FAILED;
      return true;
    }

    // capture 3d model
    controller_.captureThreeModel();
    // get 2d image
    cv::Mat resultImage;
    resultImage = controller_.get2DImage();
    // get 3d pcl
    PointCloud_SE_Ptr pointCloud;
    controller_.getPointCloud(pointCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (convert2PCLPointCloud(pointCloud, pcl_cloud))
    {
      ROS_WARN("Capture point cloud succeed!!!!!!!!!!!!!!!!!!!");
      // save point cloud as .ply
//      pcl::io::savePLYFile("/home/hkclr/smarteye_ws/mypointcloud.ply", *pcl_cloud);
//      ROS_WARN("Save ply file succeed!!!!!!!!!!!!!!!!!!!");

      // Respond and Publish point cloud
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*pcl_cloud, msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_frame_;
      res.points = msg;
      // Respond and Publish 2d imge
      cv_bridge::CvImage img_bridge;
      sensor_msgs::Image img_msg;
      img_bridge = cv_bridge::CvImage(msg.header, sensor_msgs::image_encodings::MONO8, resultImage);
      img_bridge.toImageMsg(img_msg);
      res.image = img_msg;
      // do respond and publish here
      res.result_status = res.SUCCEEDED;
      pointcloud_pub_.publish(msg);
    }
    else
    {
      res.result_status = res.FAILED;
    }
    return true;
  }

  bool RosApi::convert2PCLPointCloud(const PointCloud_SE_Ptr &se_cloud,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
  {
    if (nullptr == se_cloud || nullptr == pcl_cloud)
    {
      return false;
    }

    pcl_cloud->points.clear();
    int numOfPoints = se_cloud->points.size();
    for (int i = 0; i < numOfPoints; i++)
    {
      pcl::PointXYZRGB point;
      point.x = se_cloud->points[i].x;
      point.y = se_cloud->points[i].y;
      point.z = se_cloud->points[i].z;
      point.rgb = se_cloud->points[i].rgb;
      point.r = se_cloud->points[i].r;
      point.g = se_cloud->points[i].g;
      point.b = se_cloud->points[i].b;
      pcl_cloud->points.push_back(point);
    }
    pcl_cloud->width = se_cloud->width;
    pcl_cloud->height = se_cloud->height;
    return true;
  }
}