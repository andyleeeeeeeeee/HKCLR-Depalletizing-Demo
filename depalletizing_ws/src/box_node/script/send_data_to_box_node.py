#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, Image, PointField
from roport.srv import GetObjectPose, GetObjectPoseRequest

# vision dependence
import numpy as np
import cv2
import pcl
import ros_numpy
import ctypes
import struct



class BoxVisionTester(object):

    def __init__(self, ):
        super(BoxVisionTester, self).__init__()

        self._send_pcl_pub = rospy.Publisher(
            '/hv1000/point_cloud', PointCloud2, queue_size=1)

        self._send_img_pub = rospy.Publisher(
            '/hv1000/2d_image', Image, queue_size=1)

        self.get_object_pose_client = self._create_client(
            '/box/get_object_pose', GetObjectPose, False)

    def _publish_pcl(self):
        # convert pcl to ros
        ply_path = '/home/andylee/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo/data/qz/model11.pcd'
        cloud = pcl.load_XYZRGB(ply_path)
        cloud_msg = self.pcl_to_ros(cloud)
        self._send_pcl_pub.publish(cloud_msg)
        rospy.loginfo('Point Cloud has been sent')

    def _publish_img(self):
        # convert cv2 to ros
        img = cv2.imread('/home/andylee/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo/data/qz/leftView11.bmp')
        image_msg = ros_numpy.image.numpy_to_image(img,'bgr8')
        self._send_img_pub.publish(image_msg)        
        rospy.loginfo('Image has been sent')

    def _get_object_pose(self):
        req = GetObjectPoseRequest()
        rospy.loginfo('send request to get pose info')
        get_object_pose_resp = self.get_object_pose_client(req)
        if get_object_pose_resp.result_status == get_object_pose_resp.FAILED:
            rospy.logerr('Get pose failed')
            rospy.ROSInterruptException
        else:
            if get_object_pose_resp.pose_amount == get_object_pose_resp.SINGLE:
                rospy.logwarn('Get single pose successed')
                rospy.logwarn('Get pose %f ,%f ,%f ,%f ,%f ,%f ,%f' % (get_object_pose_resp.pose.position.x,get_object_pose_resp.pose.position.y,get_object_pose_resp.pose.position.z,get_object_pose_resp.pose.orientation.x,get_object_pose_resp.pose.orientation.y,get_object_pose_resp.pose.orientation.z,get_object_pose_resp.pose.orientation.w))
            else:
                rospy.logwarn('Get multi poses successed')
                rospy.logwarn('data size is %d'%len(get_object_pose_resp.poses_list))
                poses_1d_array = np.array(get_object_pose_resp.poses_list) 
                poses_2d_array = np.reshape(poses_1d_array,(-1,7))              
                for cnt in range(np.size(poses_2d_array,0)):
                    rospy.logwarn('Get pose %d, %f ,%f ,%f ,%f ,%f ,%f ,%f' % ((cnt+1),poses_2d_array[cnt][0],poses_2d_array[cnt][1],poses_2d_array[cnt][2],poses_2d_array[cnt][3],poses_2d_array[cnt][4],poses_2d_array[cnt][5],poses_2d_array[cnt][6]))
            return True

    def _create_client(self, srv_id, srv_type, simulation=True, timeout=1):
        """Create service client for querying simulated or real devices

        :param srv_id: str ID of the service provided by device interface
        :param srv_type: rossrv
        :param simulation: bool If true, create for simulated device, otherwise false
        "param timeout: int Timeout in sec
        """
        if simulation:
            full_id = '/simulation' + srv_id
        else:
            full_id = srv_id
        try:
            rospy.wait_for_service(full_id, timeout)
            client = rospy.ServiceProxy(full_id, srv_type)
            # self.enabled_clients.append(client)
            return client
        except rospy.ROSException:
            rospy.logwarn('Service ' + full_id + ' not available')
            return None

    def pcl_to_ros(self,pcl_array):
        """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message

            Args:
                pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

            Returns:
                PointCloud2: A ROS point cloud
        """
        ros_msg = PointCloud2()

        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "map"

        ros_msg.height = 1
        ros_msg.width = pcl_array.size

        ros_msg.fields.append(PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="rgb",
                                offset=16,
                                datatype=PointField.FLOAT32, count=1))

        ros_msg.is_bigendian = False
        ros_msg.point_step = 32
        ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
        ros_msg.is_dense = False
        buffer = []

        for data in pcl_array:
            s = struct.pack('>f', data[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

        ros_msg.data = "".join(buffer)

        return ros_msg

if __name__ == "__main__":
    try:
        rospy.init_node('box_node_tester')
        tester =  BoxVisionTester()
        rospy.loginfo("Box Node: BoxVision tester ready, start to send data for test.")
        tester._publish_pcl()
        tester._publish_img()
        tester._get_object_pose()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)