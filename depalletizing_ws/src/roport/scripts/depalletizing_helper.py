#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf2_ros

import numpy as np

from geometry_msgs.msg import Pose, Quaternion, Vector3, PoseArray

from std_srvs.srv import SetBool, Trigger
from robot_webots.srv import *

from roport.srv import *

from rotools.utility.common import sd_pose, to_ros_pose, get_param, offset_ros_pose, set_param, all_close
from rotools.utility import transform


# maybe in need in the future
# class BoxPickingHelper(object):
#     def __init__(self):
#         super(BoxPickingHelper, self).__init__()


class NaiveDepalletizingPlanner(object):

    def __init__(self):
        super(NaiveDepalletizingPlanner, self).__init__()
        # self.picking_helper = BoxPickingHelper()
        rospy.loginfo('Created helpers for picking box')
        self.pick_from_left = True

    def picking_plan(self, obj_pose):
        # the box comes from whether the left or right side
        if obj_pose[0:3, 1] >= 0:
            self.pick_from_left = True
        else: 
            self.pick_from_left = False
        
        # do some picking point planning  
        tcp_pose = transform.identity_matrix()
        angle, _, _ = transform.rotation_from_matrix(obj_pose)
        if -np.pi * 0.5 <= angle <= np.pi * 0.5:
            tcp_pose[0:3, 0] = obj_pose[0:3, 0]  # tcp.x = obj.x
            tcp_pose[0:3, 1] = -obj_pose[0:3, 1]  # tcp.y = -obj.y
        else:
            tcp_pose[0:3, 0] = -obj_pose[0:3, 0]  # tcp.x = -obj.x
            tcp_pose[0:3, 1] = obj_pose[0:3, 1]  # tcp.y = obj.y

        tcp_pose[0:3, 2] = -obj_pose[0:3, 2]  # tcp.z = -obj.z
        
        # make sure the vacuum could touch the box
        # offset = -0.02  
        # tcp_pose[0:3, 3] = obj_pose[0:3, 3] + np.array([0, 0, offset])
        return to_ros_pose(tcp_pose)

    def middle_plan(self):
        middle_pose = Pose()
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            pose = get_param('right_middle_pose')
            middle_pose.pose.position.x = pose[0]
            middle_pose.pose.position.y = pose[1]
            middle_pose.pose.position.z = pose[2]
            middle_pose.pose.orientation.x = pose[3]
            middle_pose.pose.orientation.y = pose[4]
            middle_pose.pose.orientation.z = pose[5]
            middle_pose.pose.orientation.w = pose[6]
            self.right_middle_pose = middle_pose
        # vise versa
        else:
            pose = get_param('left_middle_pose')
            middle_pose.pose.position.x = pose[0]
            middle_pose.pose.position.y = pose[1]
            middle_pose.pose.position.z = pose[2]
            middle_pose.pose.orientation.x = pose[3]
            middle_pose.pose.orientation.y = pose[4]
            middle_pose.pose.orientation.z = pose[5]
            middle_pose.pose.orientation.w = pose[6]
            self.left_middle_pose = middle_pose
        return middle_pose
                        
            

    def placing_plan(self):
        place_pose = Pose()
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            pose = get_param('right_place_pose')
            place_pose.position.x = pose[0]
            place_pose.position.y = pose[1]
            place_pose.position.z = pose[2]
            place_pose.orientation.x = pose[3]
            place_pose.orientation.y = pose[4]
            place_pose.orientation.z = pose[5]
            place_pose.orientation.w = pose[6]
        # vise versa
        else:
            pose = get_param('left_place_pose')
            place_pose.position.x = pose[0]
            place_pose.position.y = pose[1]
            place_pose.position.z = pose[2]
            place_pose.orientation.x = pose[3]
            place_pose.orientation.y = pose[4]
            place_pose.orientation.z = pose[5]
            place_pose.orientation.w = pose[6]
        return place_pose


class DepalletizingHelper(object):

    def __init__(self, ):
        super(DepalletizingHelper, self).__init__()
        
        # planner for pick and place pose
        self.planner = NaiveDepalletizingPlanner()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # switch for Parallel Running
        self.store_DI_state = False
        self.fetch_DI_state = False
        
        # switch for Parallel Running
        self.enabled_clients = []

        # Clients for querying services provided by simulated devices
        self.sim_delete_box_client = self._create_client(
            '/supervisor/delete_box', Trigger)
        self.sim_get_position_client = self._create_client(
            '/supervisor/get_position', node_get_position)
        self.sim_get_orientation_client = self._create_client(
            '/supervisor/get_orientation', node_get_orientation)
        self.sim_set_position_client = self._create_client(
            '/supervisor/set_position', field_set_vec3f)
        self.sim_set_rotation_client = self._create_client(
            '/supervisor/set_orientation', field_set_rotation)
        self.sim_run_gripper_client = self._create_client(
            '/gripper/run', SetBool)

        # Clients for querying services provided by real devices
        self.get_pointcloud_client = self._create_client(
            '/hv1000/get_pointcloud', GetPointCloud, False)
        self.get_object_info_client = self._create_client(
            '/vision/get_object_info', GetObjectPose, False)
        self.run_gripper_client = self._create_client(
            '/gripper/run', SetBool, False)
        self.get_tcp_pose_client = self._create_client(
            '/get_group_pose', GetGroupPose, False)

        # Service servers towards task scheduler
        self._execute_planning_srv = rospy.Service(
            'execute_planning', ExecutePlanning, self._execute_planning_handle
        )
        self._type_in_pose_srv = rospy.Service(
            'type_in_pose', TypeInPose, self._type_in_pose_handle
        )
        self._connect_waypoints_srv = rospy.Service(
            'connect_waypoints', ConnectWaypoints, self._connect_waypoints_handle
        )
        self._execute_suction_srv = rospy.Service(
            'execute_suction', ExecuteSuction, self._execute_suction_handle
        )
        self._sense_object_pose_srv = rospy.Service(
            'sense_object_pose', SenseObjectPose, self.sense_object_pose_handle
        )
        self._store_detected_info_srv = rospy.Service(
            'store_detected_info', StoreDetectedInfo, self._store_detected_info_handle
        )
        self._fetch_detected_info_srv = rospy.Service(
            'fetch_detected_info', FetchDetectedInfo, self._fetch_detected_info_handle
        )

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
            self.enabled_clients.append(client)
            return client
        except rospy.ROSException:
            rospy.logwarn('Service ' + full_id + ' not available')
            return None

    def _execute_suction_handle(self, req):
        resp = ExecuteSuctionResponse()
        if self.run_gripper_client in self.enabled_clients:
            run_resp = self.run_gripper_client(req.enable)
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
                return resp
        if self.sim_run_gripper_client in self.enabled_clients:
            run_resp = self.sim_run_gripper_client(req.enable)
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
        return resp

    def _type_in_pose_handle(self, req):
        # used to transform typein information into blackboard
        resp = TypeInPoseResponse()
        resp.pose_on_blackboard = req.type_in_pose
        return resp

    def _connect_waypoints_handle(self, req):
        # used to connect multiple waypoints in one path
        resp = ConnectWaypointsResponse()
        connected_waypoints = PoseArray()
        connected_waypoints.header.frame_id = 'robot_link0'
        if not req.waypoint1.position.x == 0 and not req.waypoint1.position.y == 0:
            connected_waypoints.poses.append(req.waypoint1)
        if not req.waypoint2.position.x == 0 and not req.waypoint2.position.y == 0:
            connected_waypoints.poses.append(req.waypoint2)
        if not req.waypoint3.position.x == 0 and not req.waypoint3.position.y == 0:
            connected_waypoints.poses.append(req.waypoint3)
        if not req.waypoint4.position.x == 0 and not req.waypoint4.position.y == 0:
            connected_waypoints.poses.append(req.waypoint4)
        if not req.waypoint5.position.x == 0 and not req.waypoint5.position.y == 0:
            connected_waypoints.poses.append(req.waypoint5)
        resp.connected_waypoints = connected_waypoints
        return resp

    def _execute_planning_handle(self, req):
        """Plan pick and place pose_util for given box pose_util and its category.
        All poses are relevant to the robot base frame
        """
        obj_pose = sd_pose(req.pose)
        # plan pick
        pick_tcp_pose = self.planner.picking_plan(obj_pose)
        pre_pick_offset = get_param('pre_pick_offset', [-0.2, 0, 0])
        pre_pick_tcp_pose = offset_ros_pose(pick_tcp_pose, pre_pick_offset)
        
        # plan middle
        middle_tcp_pose = self.planner.middle_plan()
    
        # plan place
        place_tcp_pose = self.planner.placing_plan()

        resp = ExecutePlanningResponse()
        resp.pick_pose = pick_tcp_pose
        resp.pre_pick_pose = pre_pick_tcp_pose
        resp.middle_pose = middle_tcp_pose
        resp.place_pose = place_tcp_pose
        return resp

    def sense_object_pose_handle(self, req):
        # 'Get pointcloud' and 'Get object pose' can be intergrated together. However, they are separeted in the code below.
        """Get the box pose in robot base frame.

        :param req: no need to give
        """        
        resp = SenseObjectPoseResponse()
        if self.get_pointcloud_client in self.enabled_clients:
            pointcloud_req = GetPointCloudRequest()
            pointcloud_resp = self.get_pointcloud_client(pointcloud_req)
            if pointcloud_resp.result_status == pointcloud_resp.FAILED:
                rospy.logerr('Get point cloud failed')
                resp.result_status = resp.FAILED
                return resp

            if self.get_object_info_client in self.enabled_clients:
                obj_info_req = GetObjectPoseRequest()
                obj_info_req.points = pointcloud_resp.points               
                # obj pose in obj info is in robot base frame
                obj_info_resp = self.get_object_info_client(obj_info_req)
                resp.pose = obj_info_resp.pose
                resp.result_status = resp.SUCCEEDED
                return resp
            else:
                rospy.logwarn('Get object info client is not enabled')
        else:
            rospy.logdebug('Get point cloud client is not enabled')

        if self.sim_get_position_client in self.enabled_clients and \
                self.sim_get_orientation_client in self.enabled_clients:
            obj_pose = Pose()
            get_position_resp = self.sim_get_position_client(0)
            obj_pose.position = get_position_resp.position
            get_orientation_resp = self.sim_get_orientation_client(0)
            obj_pose.orientation = get_orientation_resp.orientation

            # for sim, we omit translation between base frame to camera frame (hand-eye calibration)
            resp.result_status = resp.SUCCEEDED
            resp.pose = obj_pose
        return resp

    def _store_detected_info_handle(self, req):
        # store the newest dection info into ros server
        position = [req.pose.position.x,
                    req.pose.position.y, req.pose.position.z]
        set_param('detected_obj_pose_position', position)
        orientation = [req.pose.orientation.x, req.pose.orientation.y,
                       req.pose.orientation.z, req.pose.orientation.w]
        set_param('detected_obj_pose_orientation', orientation)
        rospy.sleep(0.1)
        self.store_DI_state = True

        # wait for robot to fetch this newest detection info
        while not rospy.is_shutdown() and not self.fetch_DI_state:
            rospy.sleep(0.1)
        self.fetch_DI_state = False

        # wait for the box, which is detected just ago, to be picked away and then start a new detection
        resp = StoreDetectedInfoResponse()
        if self.get_tcp_pose_client in self.enabled_clients:
            can_trigger = False
            while not rospy.is_shutdown() and not can_trigger:
                can_trigger_resp = self.get_tcp_pose_client()
                rospy.sleep(0.1)
                if self.planner.pick_from_left:
                    if all_close(self.planner.right_middle_pose.position, can_trigger_resp.pose.position, 0.05):
                        can_trigger = True
                else:
                    if all_close(self.planner.left_middle_pose.position, can_trigger_resp.pose.position, 0.05):
                        can_trigger = True
            resp.result_status = resp.SUCCEEDED
            return resp

    def _fetch_detected_info_handle(self, req):
        resp = FetchDetectedInfoResponse()
        # wait for the newest detected info to be sent in ros server.
        while not rospy.is_shutdown() and not self.store_DI_state:
            rospy.sleep(0.1)

        # robot gets the newest detection info for motion planning
        position = get_param('detected_obj_pose_position')
        resp.pose.position.x = position[0]
        resp.pose.position.y = position[1]
        resp.pose.position.z = position[2]
        orientation = get_param('detected_obj_pose_orientation')
        resp.pose.orientation.x = orientation[0]
        resp.pose.orientation.y = orientation[1]
        resp.pose.orientation.z = orientation[2]
        resp.pose.orientation.w = orientation[3]
        print('pose fetched is {}', format(resp.pose))
        self.store_DI_state = False
        self.fetch_DI_state = True
        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('roport_depalletizing_helper')
        helper = DepalletizingHelper()
        rospy.loginfo("Roport: Depalletizing helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
