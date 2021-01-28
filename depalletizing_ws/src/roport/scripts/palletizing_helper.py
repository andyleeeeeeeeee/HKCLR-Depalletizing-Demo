#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf2_ros

import numpy as np

from geometry_msgs.msg import Pose, Quaternion, Vector3, PoseArray

from std_srvs.srv import SetBool, Trigger
from webots_ros.srv import set_bool, set_int, set_intRequest, get_float, node_get_position, node_get_orientation, \
    field_set_rotation, field_set_rotationRequest, field_set_vec3f, field_set_vec3fRequest

from roport.srv import *
from hope.srv import *
from smarteye.srv import *
from libnachi.srv import *

from rotools.utility.common import sd_pose, to_ros_pose, get_param, offset_ros_pose, set_param
from rotools.utility import transform


class Pallet(object):
    def __init__(self, params):
        """Pallet object

        :param params: Pallet center coords in robot base frame + Start corner to put the first box + margin in x y
                       (x, y) = (1, 1) is left up corner, (1, -1) right up
                       corner, (-1, -1) bottom right, (-1, 1) bottom left
        """
        assert isinstance(params, list) and len(params) == 11
        self.x = params[0]
        self.y = params[1]
        self.z = params[2]
        self.start_corner_x = params[3]
        self.start_corner_y = params[4]
        self.margin_x = params[5]
        self.margin_y = params[6]

        # Whether the pallet is on the right side of the robot,
        # (see along +x axis direction) 1: right side, -1: left side
        self.right_side = 1 if self.y < 0 else -1

        self.L = params[7]
        self.W = params[8]
        self.height_limit = params[9]
        self.padding = params[10]

    @property
    def h_l(self):
        return self.L * 0.5

    @property
    def h_w(self):
        return self.W * 0.5


class Box(object):
    def __init__(self, LWH):
        """Box object.
        Through out the code, the length is along x axis, width along y, and height along z

        """
        self.L = LWH[0]
        self.W = LWH[1]
        self.H = LWH[2]

    @property
    def h_l(self):
        return self.L * 0.5

    @property
    def h_w(self):
        return self.W * 0.5

    @property
    def h_h(self):
        return self.H * 0.5


class BoxPlacingHelper(object):
    def __init__(
            self,
            box,
            pallet
    ):
        self.B = box
        self.P = pallet
        self._box_cnt = 0

        self._volume_id = -1
        self.cols = 0
        self.rows = 0

        # If force_flat, only volume_id 0 and 1 are considered
        force_flat = get_param('force_flat', True)
        if force_flat:
            rospy.logwarn('In force flat mode')
            self._volumes = [
                [self.B.L, self.B.W, self.B.H],
                [self.B.W, self.B.L, self.B.H],
            ]
        else:
            rospy.logwarn('Not force flat')
            self._volumes = [
                [self.B.L, self.B.W, self.B.H],
                [self.B.W, self.B.L, self.B.H],
                [self.B.L, self.B.H, self.B.W],
                [self.B.H, self.B.L, self.B.W],
                [self.B.W, self.B.H, self.B.L],
                [self.B.H, self.B.W, self.B.L],
            ]

        volume_capacity = 0
        for i, v in enumerate(self._volumes):
            cols = int((self.P.L - self.P.padding * 2) // (v[0] + self.P.margin_x))
            rows = int((self.P.W - self.P.padding * 2) // (v[1] + self.P.margin_y))
            layers = int(self.P.height_limit // v[2])
            if cols * rows * layers > volume_capacity:
                volume_capacity = cols * rows * layers
                self.cols = cols
                self.rows = rows
                self.layers = layers
                self._volume_id = i
                self.cell_dim = v  # the cell is the placed box's occupied volume in base frame

        # For debugging individual placing approach
        #i = 3
        #v = self._volumes[i]
        #print(v)
        #self.cols = int((self.P.L - self.P.padding * 2) // (v[0] + self.P.margin_x))
        #self.rows = int((self.P.W - self.P.padding * 2) // (v[1] + self.P.margin_y))
        #self.layers = int(self.P.height_limit // v[2])
        #self._volume_id = i
        #self.cell_dim = v

        assert self.cols > 0 and self.rows > 0

        if self._volume_id < 0:
            raise RuntimeError("No valid plan found")

        # Orientation of tcp for placing the first box wrt robot base frame
        init_tcp_orientation = transform.identity_matrix()
        if self._volume_id == 0:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
            ])
        elif self._volume_id == 1:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [0, 1, 0],
                [1, 0, 0],
                [0, 0, -1],
            ])
        elif self._volume_id == 2:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [1, 0, 0],
                [0, 0, -self.P.right_side],
                [0, self.P.right_side, 0],
            ])
        elif self._volume_id == 3:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [0, 0, -1],
                [-self.P.right_side, 0, 0],
                [0, self.P.right_side, 0],
            ])
        elif self._volume_id == 4:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [0, -1, 0],
                [0, 0, -self.P.right_side],
                [self.P.right_side, 0, 0],
            ])
        elif self._volume_id == 5:
            init_tcp_orientation[0:3, 0:3] = np.array([
                [0, 0, -1],
                [0, self.P.right_side, 0],
                [self.P.right_side, 0, 0],
            ])
        init_tcp_orientation = transform.quaternion_from_matrix(init_tcp_orientation)

        # Get initial tcp position in base frame for placing the first box
        init_box_position = np.array([
            self.P.x + self.P.start_corner_x * self.P.h_l - self.P.start_corner_x * (
                    self.P.padding + self.cell_dim[0] * 0.5),
            self.P.y + self.P.start_corner_y * self.P.h_w - self.P.start_corner_y * (
                    self.P.padding + self.cell_dim[1] * 0.5),
            self.P.z + self.cell_dim[2] * 0.5,
            ])
        if self._volume_id == 0 or self._volume_id == 1:
            init_tcp_position = init_box_position + np.array([0, 0, self.cell_dim[2] * 0.5])
        elif self._volume_id == 2 or self._volume_id == 4:
            init_tcp_position = init_box_position + np.array([0, -self.P.start_corner_y * self.cell_dim[1] * 0.5, 0])
        elif self._volume_id == 3 or self._volume_id == 5:
            init_tcp_position = init_box_position + np.array([-self.P.start_corner_x * self.cell_dim[0] * 0.5, 0, 0])
        else:
            raise ValueError()

        z_spacing = get_param('z_spacing', 0.01)
        init_tcp_position[-1] += z_spacing
        self._init_tcp_pose = np.concatenate((init_tcp_position, init_tcp_orientation))

    def get_next_pose(self):
        num_in_layer = int(self.cols * self.rows)
        layer = int(self._box_cnt // num_in_layer)
        col = (self._box_cnt - layer * num_in_layer) % self.cols
        row = (self._box_cnt - layer * num_in_layer) // self.cols

        rospy.logwarn("\nPlan #{}\nBox: #{}\nCol: {}/{} Row: {}/{} Layer: {}/{}".format(
            self._volume_id, self._box_cnt + 1, col + 1, self.cols, row + 1, self.rows, layer + 1, self.layers))
        # self.log_layout(row, col)

        next_tcp_pose = self._init_tcp_pose + np.array([
            - self.P.start_corner_x * col * (self.cell_dim[0] + self.P.margin_x),
            - self.P.start_corner_y * row * (self.cell_dim[1] + self.P.margin_y),
            layer * self.cell_dim[2],
            0, 0, 0, 0
        ])

        # decrease z for 1cm because if not box will drop from air and change orientation
        next_tcp_pose[2] -= 0.006
        next_obj_pose = sd_pose(next_tcp_pose)
        # next_obj_pose[2,3] -= (self.cell_dim[2] * 0.5 + 0.08)
        self._box_cnt += 1
        return next_tcp_pose, next_obj_pose

    def log_layout(self, row, col):
        pattern = np.chararray((self.rows, self.cols))
        pattern[:] = '#'
        pattern[self.P.start_corner_y * row, self.P.start_corner_x * col - 1] = '@'
        print(pattern)


class NaivePalletizingPlanner(object):

    def __init__(self):
        super(NaivePalletizingPlanner, self).__init__()

        self._large_box_key = '0'
        self._small_box_key = '1'

        self.boxes = {}
        large_box = get_param('large_box')
        if isinstance(large_box, list) and len(large_box) == 3:
            self.boxes[self._large_box_key] = Box(large_box)
        small_box = get_param('small_box')
        if isinstance(small_box, list) and len(small_box) == 3:
            self.boxes[self._small_box_key] = Box(small_box)

        self.box_type_num = len(self.boxes.keys())
        assert self.box_type_num > 0
        rospy.loginfo("Planning for {} types of boxes".format(self.box_type_num))

        self.pallets = {}
        if self._large_box_key in self.boxes.keys():
            left_pallet = get_param('left_pallet')
            if not isinstance(left_pallet, list) or len(left_pallet) != 11:
                raise ValueError('Left pallet for placing large box is not set')
            else:
                self.pallets[self._large_box_key] = Pallet(left_pallet)
        if self._small_box_key in self.boxes.keys():
            right_pallet = get_param('right_pallet')
            if not isinstance(right_pallet, list) or len(right_pallet) != 11:
                raise ValueError('Right pallet for placing small box is not set')
            else:
                self.pallets[self._small_box_key] = Pallet(right_pallet)

        self.pallet_num = len(self.pallets.keys())

        self.placing_helpers = {}
        for k in self.boxes.keys():
            self.placing_helpers[k] = BoxPlacingHelper(self.boxes[k], self.pallets[k])
        rospy.loginfo('Created helpers for placing {} box'.format(self.placing_helpers.keys()))

    def placing_plan(self, category):
        if category in self.placing_helpers.keys():
            place_pose, place_obj_pose = self.placing_helpers[category].get_next_pose()
        else:
            raise ValueError("Unknown category {} of type {}".format(category, type(category)))
        return to_ros_pose(place_pose), to_ros_pose(place_obj_pose)

    def picking_plan(self, category, obj_pose):
        if category in self.boxes.keys():
            tcp_pose = transform.identity_matrix()
            angle, _, _ = transform.rotation_from_matrix(obj_pose)

            if -np.pi * 0.5 <= angle <= np.pi * 0.5:
                tcp_pose[0:3, 0] = obj_pose[0:3, 0]  # tcp.x = obj.x
                tcp_pose[0:3, 1] = -obj_pose[0:3, 1]  # tcp.y = -obj.y
            else:
                tcp_pose[0:3, 0] = -obj_pose[0:3, 0]  # tcp.x = -obj.x
                tcp_pose[0:3, 1] = obj_pose[0:3, 1]  # tcp.y = obj.y

            tcp_pose[0:3, 2] = -obj_pose[0:3, 2]  # tcp.z = -obj.z
            offset = -0.02  # make sure the vacuum could touch the box
            tcp_pose[0:3, 3] = obj_pose[0:3, 3] + np.array([0, 0, offset])
            return to_ros_pose(tcp_pose)
        else:
            raise KeyError('Unknown key: {}'.format(category))


class PalletizingHelper(object):

    def __init__(self, ):
        super(PalletizingHelper, self).__init__()

        self.planner = NaivePalletizingPlanner()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # switch for Parallel Running
        self.store_DI_state = False
        self.fetch_DI_state = False

        self.enabled_clients = []

        # Clients for querying services provided by simulated devices
        self.sim_load_box_client = self._create_client('/conveyor/load_box', set_int)
        self.sim_run_conveyor_client = self._create_client('/conveyor/run', set_bool)
        self.sim_get_distance_client = self._create_client('/distance_sensor/get_distance', get_float)
        self.sim_get_position_client = self._create_client('/supervisor/get_position', node_get_position)
        self.sim_get_orientation_client = self._create_client('/supervisor/get_orientation', node_get_orientation)
        self.sim_set_position_client = self._create_client('/supervisor/set_position', field_set_vec3f)
        self.sim_set_rotation_client = self._create_client('/supervisor/set_orientation', field_set_rotation)
        self.sim_run_gripper_client = self._create_client('/gripper/run', set_bool)

        # Clients for querying services provided by real devices
        self.run_conveyor_client = self._create_client('/conveyor/run', SetBool, False)
        self.get_conveyor_state_client = self._create_client('/conveyor/get_state', Trigger, False)
        self.get_pointcloud_client = self._create_client('/hv1000/get_pointcloud', GetPointCloud, False)
        self.get_object_info_client = self._create_client('/vision/get_object_info', GetObjectPose, False)
        self.run_gripper_client = self._create_client('/gripper/run', SetBool, False)

        # Clients for querying services provided by the robot
        self.set_js_client = self._create_client('/mz25/set_joint_states', nachiSetJointStates, False)
        self.set_tcp_client = self._create_client('/mz25/set_tcp_pose', nachiSetTCPPose, False)

        # Service servers towards task scheduler
        self._execute_conveyor_srv = rospy.Service(
            'execute_conveyor_cmd', ExecuteConveyorCmd, self._execute_conveyor_handle
        )
        self._execute_load_object_srv = rospy.Service(
            'execute_load_object', ExecuteLoadObject, self._execute_load_object_handle
        )
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
        self._sense_object_exist_srv = rospy.Service(
            'sense_object_exist', SenseObjectExist, self._sense_object_exist_handle
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
        self._execute_tcp_pose_srv = rospy.Service(
            'execute_tcp_pose', ExecuteGroupPose, self.execute_tcp_pose_handle
        )
        self._execute_js = rospy.Service(
            'execute_joint_states', ExecuteGroupJointStates, self.execute_js_handle
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

    def _execute_conveyor_handle(self, req):
        resp = ExecuteConveyorCmdResponse()
        if self.run_conveyor_client in self.enabled_clients:
            run_resp = self.run_conveyor_client(req.enable)
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
                return resp
        if self.sim_run_conveyor_client in self.enabled_clients:
            run_resp = self.sim_run_conveyor_client(req.enable)
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
        return resp

    def _execute_load_object_handle(self, req):
        resp = ExecuteLoadObjectResponse()
        if self.sim_load_box_client not in self.enabled_clients:
            resp.result_status = resp.FAILED
            return resp

        # Note on type (int):
        # <0 unload, 0 random load, 1 load large, 2 load small
        sim_load_box_req = set_intRequest()
        sim_load_box_req.value = req.type
        load_resp = self.sim_load_box_client(sim_load_box_req)
        if load_resp.success:
            resp.result_status = resp.SUCCEEDED
        else:
            resp.result_status = resp.FAILED
        return resp

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
        connected_waypoints.header.frame_id = 'mz25_link0'
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
        pick_tcp_pose = self.planner.picking_plan(req.category, obj_pose)

        pre_pick_offset = get_param('pre_pick_offset', [-0.7, 0, 0.1])
        pre_pick_tcp_pose = offset_ros_pose(pick_tcp_pose, pre_pick_offset)

        place_tcp_pose, place_obj_pose = self.planner.placing_plan(req.category)
        pre_place_offset = get_param('pre_place_offset', [0, 0, 0.3])
        pre_place_tcp_pose = offset_ros_pose(place_tcp_pose, pre_place_offset)

        # waypoints_for_test = PoseArray()
        # waypoints_for_test.header.frame_id = 'mz25_link0'
        # pose1 = Pose()
        # pose1.position.x = 1.1289
        # pose1.position.y = -0.14985
        # pose1.position.z = 1.1801
        # pose1.orientation.x = 0.74252
        # pose1.orientation.y = 0.66982
        # pose1.orientation.z = 0.00078959
        # pose1.orientation.w = 0.00023822
        #
        # pose2 = Pose()
        # pose2.position.x = 0.11451
        # pose2.position.y = -0.31474
        # pose2.position.z = 1.2825
        # pose2.orientation.x = 0.79122
        # pose2.orientation.y = -0.55404
        # pose2.orientation.z = 0.21201
        # pose2.orientation.w = 0.1485
        # waypoints_for_test.poses.append(pose1)
        # waypoints_for_test.poses.append(pose2)

        resp = ExecutePlanningResponse()
        resp.pick_pose = pick_tcp_pose
        resp.pre_pick_pose = pre_pick_tcp_pose
        resp.place_pose = place_tcp_pose
        resp.pre_place_pose = pre_place_tcp_pose
        resp.place_obj_pose = place_obj_pose
        # resp.waypoints_for_test = waypoints_for_test
        return resp

    def _sense_object_exist_handle(self, req):
        resp = SenseObjectExistResponse()
        if self.get_conveyor_state_client in self.enabled_clients:
            get_state_resp = self.get_conveyor_state_client()
            if get_state_resp.success:
                resp.result_status = resp.SUCCEEDED  # Object exist
            else:
                resp.result_status = resp.FAILED
            return resp
        if self.sim_get_distance_client in self.enabled_clients:
            void_range = get_param('~void_range', 1.0)  # minimum distance for void (no object), in meter
            get_distance_resp = self.sim_get_distance_client(True)
            if get_distance_resp.value < void_range:
                resp.result_status = resp.SUCCEEDED  # Object exist
            else:
                resp.result_status = resp.FAILED
        return resp

    def sense_object_pose_handle(self, req):
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
                conveyor_height = get_param('conveyor_height')
                # Here the origin is the box's top surface's origin
                for key in self.planner.placing_helpers.keys():
                    obj_info_req.origin_heights.append(
                        conveyor_height + self.planner.placing_helpers[key].cell_dim[-1]
                    )
                # obj pose in obj info is in robot base frame
                obj_info_resp = self.get_object_info_client(obj_info_req)
                resp.category = self.planner.boxes.keys()[obj_info_resp.category]
                resp.pose = obj_info_resp.pose
                resp.result_status = resp.SUCCEEDED

                self._load_object_in_simulation(resp.category, resp.pose)
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

            # we omit translation between base frame to camera frame (hand-eye calibration)
            resp.result_status = resp.SUCCEEDED
            resp.pose = obj_pose
            # Here we take advantage of the prior knowledge of the box pose_util wrt robot base
            large_box_centroid_z = get_param('large_box_centroid_z', 0.7)
            resp.category = '0' if resp.pose.position.z > large_box_centroid_z else '1'
            # Make the position in robot base frame (originally in Webots world frame)
        return resp

    def _store_detected_info_handle(self, req):
        # store the newest dection info into ros server
        set_param('detected_category', req.category)
        position = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
        set_param('detected_obj_pose_position', position)
        orientation = [req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]
        set_param('detected_obj_pose_orientation', orientation)
        rospy.sleep(0.1)
        self.store_DI_state = True

        # wait for robot to fetch this newest detection info
        while not rospy.is_shutdown() and not self.fetch_DI_state:
            rospy.sleep(0.1)
        self.fetch_DI_state = False

        # wait for the box, which is detected just ago, to be picked away and then start a new detection
        resp = StoreDetectedInfoResponse()
        if self.get_conveyor_state_client in self.enabled_clients:
            object_exist = True
            while not rospy.is_shutdown() and object_exist:
                get_state_resp = self.get_conveyor_state_client()
                rospy.sleep(0.1)
                if not get_state_resp.success:
                    object_exist = False
            resp.result_status = resp.SUCCEEDED
            return resp

        if self.sim_get_distance_client in self.enabled_clients:
            sim_object_exist = True
            while not rospy.is_shutdown() and sim_object_exist:
                void_range = get_param('~void_range', 1.0)  # minimum distance for void (no object), in meter
                get_distance_resp = self.sim_get_distance_client(True)
                rospy.sleep(0.1)
                if not get_distance_resp.value < void_range:
                    sim_object_exist = False
            resp.result_status = resp.SUCCEEDED
            return resp

    def _fetch_detected_info_handle(self, req):
        resp = FetchDetectedInfoResponse()
        # wait for the newest detected info to be sent in ros server.
        while not rospy.is_shutdown() and not self.store_DI_state:
            rospy.sleep(0.1)

        # robot gets the newest detection info for motion planning
        resp.category = str(get_param('detected_category'))
        print('category fetched is {}', format(resp.category))

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



    def _load_object_in_simulation(self, obj_type, pose):
        """Load a box and rotate it in simulation according to the real box pose

        :param obj_type: str '0' for large box, '1' for small box
        :param pose: geometry_msgs/Pose Object pose in robot base frame
        """
        if self.sim_load_box_client in self.enabled_clients:
            sim_load_box_req = set_intRequest()
            if obj_type == '0':
                sim_load_box_req.value = 1
            elif obj_type == '1':
                sim_load_box_req.value = 2
            else:
                rospy.logwarn('Not supported box: {}'.format(obj_type))
                return

            load_box_resp = self.sim_load_box_client(sim_load_box_req)
            if load_box_resp.success:
                if self.sim_set_rotation_client in self.enabled_clients and \
                        self.sim_set_position_client in self.enabled_clients:
                    set_position_req = field_set_vec3fRequest()
                    position = Vector3(pose.position.x, pose.position.y, pose.position.z)
                    set_position_req.value = position
                    self.sim_set_position_client(set_position_req)

                    set_rotation_req = field_set_rotationRequest()
                    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                    r = transform.euler_from_quaternion(quaternion)[-1]  # assume planar rotation
                    rotation = Quaternion(0, 1, 0, r)
                    set_rotation_req.value = rotation
                    self.sim_set_rotation_client(set_rotation_req)

    def execute_tcp_pose_handle(self, req):
        resp = ExecuteGroupPoseResponse()
        if self.set_tcp_client in self.enabled_clients:
            set_tcp_pose_req = nachiSetTCPPoseRequest()
            p = req.goal.position
            q = req.goal.orientation
            set_tcp_pose_req.goal = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            set_tcp_pose_req.is_absolute = req.is_absolute
            set_tcp_pose_resp = self.set_tcp_client(set_tcp_pose_req)
            if set_tcp_pose_resp.result_status == set_tcp_pose_resp.FAILED:
                rospy.logerr('Set robot tcp pose failed')
                resp.result_status = resp.FAILED
                return resp
            else:
                resp.result_status = resp.SUCCEEDED
            return resp
        else:
            resp.result_status = resp.SUCCEEDED
            return resp

    def execute_js_handle(self, req):
        resp = ExecuteGroupJointStatesResponse()
        if self.set_js_client in self.enabled_clients:
            set_js_req = nachiSetJointStatesRequest()
            set_js_req.goal = req.goal
            set_js_req.tolerance = req.tolerance
            set_js_resp = self.set_js_client(set_js_req)
            if set_js_resp.result_status == set_js_resp.FAILED:
                rospy.logerr('Set robot joint states failed')
                resp.result_status = resp.FAILED
                return resp
            else:
                resp.result_status = resp.SUCCEEDED
            return resp
        else:
            resp.result_status = resp.SUCCEEDED
            return resp


if __name__ == "__main__":
    try:
        rospy.init_node('roport_palletizing_helper')
        helper = PalletizingHelper()
        rospy.loginfo("Roport: Palletizing helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
