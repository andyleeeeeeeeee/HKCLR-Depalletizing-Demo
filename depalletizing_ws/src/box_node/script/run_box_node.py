#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, Image
from roport.srv import GetObjectPose, GetObjectPoseResponse

# vision dependence
import cv2
import numpy as np
import stack_detection, pcl2_img
import pcl
from sensor_msgs import point_cloud2 as pc2
import ros_numpy 
from visualize import *
import open3d as o3d


class BoxVisionHelper(object):

    def __init__(self, ):
        super(BoxVisionHelper, self).__init__()

        self.display_result_ = True
        self.is_pose_single_ = False
        self.is_pcl_updated_ = False
        self.is_img_updated_ = False

        self._get_pcl_sub = rospy.Subscriber(
            '/hv1000/point_cloud', PointCloud2, self._get_pcl_handle
        )

        self._get_img_sub = rospy.Subscriber(
            '/hv1000/twod_image', Image, self._get_img_handle
        )

        self._get_object_pose_srv = rospy.Service(
            '/box/get_object_pose', GetObjectPose, self._get_object_pose_handle
        )

    def _get_pcl_handle(self, ros_pointcloud2):
        # convert ros to pcl
        cloud = self.ros_to_pcl(ros_pointcloud2)
        pts = cloud.to_array()
        self.pts_ = pts[:, 0 : 3]
        self.is_pcl_updated_ = True
        rospy.loginfo('Point Cloud has been updated')

    def _get_img_handle(self, ros_image):
        # convert ros to cv2
        image_nparray = ros_numpy.image.image_to_numpy(ros_image)
        self.img_ = pcl2_img.rgb2gray(image_nparray)
        self.is_img_updated_ = True
        rospy.loginfo('Image has been updated')

    def _get_object_pose_handle(self, req):
        resp = GetObjectPoseResponse()
        if self.is_pose_single_:
            # if only need one pose as result, make sure the update processes of pcl and image are completed
            rospy.loginfo('Check whether the Point CLoud and Image have been updated')
            while not self.is_pcl_updated_ or not self.is_img_updated_:
                rospy.sleep(0.1)
            rospy.loginfo('Confirm that the Point CLoud and Image have been updated')
            # detect pose of target box
            success, result_tuple = stack_detection.detect_with_view_single(self.pts_, self.img_)
            if success:
                result, grasp_box, cluster_planes, idx = result_tuple
                R, t, pose_detected, rec_wid, rec_len = result
                ## visualize the detected result
                if self.display_result_:
                    scene = o3d.geometry.PointCloud()
                    scene.points = o3d.utility.Vector3dVector(self.pts_)
                    visualize_pose_in_raw_pts(scene, [], R, t)

                # assign result and respond to client
                pose = Pose()
                pose.position.x = pose_detected[0]
                pose.position.y = pose_detected[1]
                pose.position.z = pose_detected[2]
                pose.orientation.x = pose_detected[3]
                pose.orientation.y = pose_detected[4]
                pose.orientation.z = pose_detected[5]
                pose.orientation.w = pose_detected[6]
                resp.pose = pose
                resp.pose_amount = resp.SINGLE
                resp.result_status = resp.SUCCEEDED
                rospy.logwarn('Get pose %f, %f, %f, %f, %f, %f, %f' % (pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
                rospy.loginfo('detect pose successed')
            else:
                resp.result_status = resp.FAILED
                rospy.loginerr('detect pose failed')

            self.is_pcl_updated_ = False
            self.is_img_updated_ = False
        else:
            # if need many poses as result, make sure the update processes of pcl is completed
            rospy.loginfo('Check whether the Point CLoud and Image have been updated')
            while not self.is_pcl_updated_:
                rospy.sleep(0.1)
            rospy.loginfo('Confirm that the Point CLoud has been updated')
            # detect pose of target box
            success, result_list = stack_detection.detect_with_view_multi(self.pts_)
            if success:
                poses_list = []
                for result_tuple in result_list:
                    for index in range(7):
                        poses_list.append(result_tuple[2][index])
                ## visualize the detected result
                if self.display_result_:
                    scene = o3d.geometry.PointCloud()
                    scene.points = o3d.utility.Vector3dVector(self.pts_)
                    R_list = []
                    t_list = []
                    for result_tuple in result_list:
                        # result, grasp_box, cluster_planes, idx = result_tuple
                        R_list.append(result_tuple[0])
                        t_list.append(result_tuple[1])                    
                    visualize_multi_pose_in_raw_pts(scene, [], R_list, t_list)

                # assign result and respond to client
                resp.poses_list = poses_list
                resp.pose_amount = resp.MULTI
                resp.result_status = resp.SUCCEEDED
                rospy.logwarn('data size is %d'%len(resp.poses_list))
                rospy.loginfo('detect poses successed')
            else:
                resp.result_status = resp.FAILED
                rospy.loginerr('detect poses failed')

            self.is_pcl_updated_ = False             
        return resp


    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data

if __name__ == "__main__":
    try:
        rospy.init_node('box_node_server')
        helper =  BoxVisionHelper()
        rospy.loginfo("Box Node: BoxVision helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)