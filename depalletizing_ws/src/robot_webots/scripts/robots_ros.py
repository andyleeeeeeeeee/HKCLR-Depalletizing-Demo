#!/usr/bin/env python2
import sys
import math
import argparse
import os
import rospy

from controller import Supervisor
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from rosgraph_msgs.msg import Clock

from std_srvs.srv import *
from geometry_msgs.msg import Pose
from robot_webots.srv import *



parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName',
                    default='robot_driver', help='Specifies the name of the node.')
parser.add_argument('--webots-robot-name', dest='webotsRobotName', default='SelfRobot',
                    help='Specifies the "name" field of the robot in Webots.')
arguments, unknown = parser.parse_known_args()
if arguments.webotsRobotName:
    os.environ['WEBOTS_ROBOT_NAME'] = arguments.webotsRobotName

rospy.init_node(arguments.nodeName, disable_signals=True)


jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Supervisor()
root_field_ = robot.getRoot().getField('children')
self_robot_ = robot.getFromDef('SelfRobot')
aubo_robot_ = robot.getFromDef('Aubo')

jointStatePublisher = JointStatePublisher(robot, jointPrefix)
rospy.logwarn('JointStatePublisher is initialized')

trajectoryFollower = TrajectoryFollower(
    robot, jointStatePublisher, jointPrefix)
trajectoryFollower.start()
rospy.logwarn('TrajectoryFollower is initialized')

# init gripper
gripper_ = robot.getConnector('connector')
rospy.logwarn('Gripper is initialized')
# run gripper call back
def run_gripper_srv(req):
    resp = SetBoolResponse()
    if req.data:
        gripper_.lock()
        resp.success = True
        resp.message = 'openGripper'
        rospy.logwarn('openVaccumGripper Grasp')
    else:
        gripper_.unlock()
        resp.success = True
        resp.message = 'closeGripper'
        rospy.logwarn('closeVaccumGripper Drop')
    return resp

# getPose
def getObjectPose():
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        world_to_obj_t = obj_node.getField('translation')
        world_to_obj_r = obj_node.getField('rotation')
        t_wo = world_to_obj_t.getSFVec3f()
        r_wo = world_to_obj_r.getSFRotation()
        world_to_base_t = self_robot_.getField('translation')
        t_wb = world_to_base_t.getSFVec3f()
        # obj_pose
        obj_pose = Pose()
            # position
        obj_pose.position.x = t_wo[0] - t_wb[0]  # ros.x = wo.x - wb.x
        obj_pose.position.y = -(t_wo[2] - t_wb[2])    # ros.y = -(wo.z - wb.z)
        obj_pose.position.z = t_wo[1] - t_wb[1]    # ros.z = wo.y - wb.y
            # orientation
        obj_pose.orientation.x = 0.0
        obj_pose.orientation.y = 0.0
        obj_pose.orientation.z = math.sin((r_wo[3] - math.pi/2) * 0.5)
        obj_pose.orientation.w = math.cos((r_wo[3] - math.pi/2) * 0.5)

        # surface_pose
        surface_pose = obj_pose
            # position
        children_field = obj_node.getField('children')
        Transform_node = children_field.getMFNode(-1)
        scale_field = Transform_node.getField('scale')
        obj_size = scale_field.getSFVec3f()
        surface_pose.position.x -= obj_size[1] * 0.5
            # orientation
            # box is on the left
        if obj_pose.position.y >= 0: 
            surface_pose.orientation.x = 0.5
            surface_pose.orientation.y = 0.5
            surface_pose.orientation.z = 0.5
            surface_pose.orientation.w = 0.5
            # box is on the right
        else:
            surface_pose.orientation.x = 0.5
            surface_pose.orientation.y = -0.5
            surface_pose.orientation.z = 0.5
            surface_pose.orientation.w = -0.5
        return surface_pose
    else:
        rospy.logerr('No box in scene !!!!!!!!!!!!')
        obj_pose = Pose()
        return obj_pose

# getPosition Callback
def getPositionSrvCb(req):
    pose = getObjectPose()
    resp = NodeGetPositionResponse()
    resp.position = pose.position
    return resp

# getOrientation Callback
def getOrientationSrvCb(req):
    pose = getObjectPose()
    resp = NodeGetOrientationResponse()
    resp.orientation = pose.orientation
    return resp

# setPosition Callback
def setPositionSrvCb(req):
    resp = FieldSetVec3fResponse()
    value = req.value
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        world_to_obj_t = obj_node.getField('translation')
        world_to_base_t = self_robot_.getField('translation')
        t_wb = world_to_base_t.getSFVec3f()
        t_wo_x = t_wb[0] + value.x
        t_wo_y = t_wb[1] + value.z
        t_wo_z = -(t_wb[2] + value.y)
        t_wo = [t_wo_x, t_wo_y, t_wo_z]
        world_to_obj_t.setSFVec3f(t_wo)
        resp.success = True
    else:
        rospy.logerr('Can not set object position !!!!!!!!!!!!')
        resp.success = False
    return resp

# setOrientation Callback
def setOrientationSrvCb(req):
    resp = FieldSetRotationResponse()
    value = req.value
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        world_to_obj_r = obj_node.getField('rotation')
        r_wo_x = 0
        r_wo_y = 1
        r_wo_z = 0
        r_wo_w = value.w - math.pi/2
        r_wo = [r_wo_x, r_wo_y, r_wo_z, r_wo_w]
        world_to_obj_r.setSFRotation(r_wo)
        resp.success = True
    else:
        rospy.logerr('Can not set object orientation !!!!!!!!!!!!')
        resp.success = False
    return resp

# changeBoxSize Callback
def changeBoxSizeSrvCb(req):
    resp = SetSizeResponse()
    length = req.length
    width = req.width
    height = req.height
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        # change shape
        children_field = obj_node.getField('children')
        Transform_node = children_field.getMFNode(-1)
        scale_field = Transform_node.getField('scale')
        Bsize = [length, width, height]
        scale_field.setSFVec3f(Bsize)
        # change collision
        boundingObject_field = obj_node.getField('boundingObject')
        Box_node = boundingObject_field.getSFNode()
        size_field = Box_node.getField('size')
        size_field.setSFVec3f(Bsize)
        resp.success = True
    else:
        rospy.logerr('Can not change box size !!!!!!!!!!!!')
        resp.success = False
    return resp

# deleteBox Callback
def deleteBoxSrvCb(req):
    resp = TriggerResponse()
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        obj_node.remove()
        rospy.logwarn('Box has been deleted!')
        resp.success = True
    else:
        rospy.logerr('can not delete box !!!!!!!!')
        resp.success = False
    return resp

# moveBase Callback
def moveBaseSrvCb(req):
    sf_robot_trans_field = self_robot_.getField('translation')
    t_wb = sf_robot_trans_field.getSFVec3f()
    aubo_robot_trans_field = aubo_robot_.getField('translation')
    t_wa = aubo_robot_trans_field.getSFVec3f()
    # move forward
    move_step = 0.05
    time_interval = 0.01
    move_times = abs(req.value) // move_step
    rest_move = abs(req.value) % move_step
    if req.value >= 0:
        while move_times > 0:
            t_wb[0] += move_step
            sf_robot_trans_field.setSFVec3f(t_wb)
            t_wa[0] += move_step
            aubo_robot_trans_field.setSFVec3f(t_wa)
            rospy.sleep(time_interval)
            move_times -= 1
        t_wb[0] += rest_move
        sf_robot_trans_field.setSFVec3f(t_wb)
        rospy.logwarn('Robot base has moved forward for %f m!' % abs(req.value))
    # move backward
    else:
        while move_times > 0:
            t_wb[0] -= move_step
            sf_robot_trans_field.setSFVec3f(t_wb)
            t_wa[0] -= move_step
            aubo_robot_trans_field.setSFVec3f(t_wa)
            rospy.sleep(time_interval)
            move_times -= 1
        t_wb[0] -= rest_move
        sf_robot_trans_field.setSFVec3f(t_wb)
        rospy.logwarn('Robot base has moved back for %f m!' % abs(req.value))
    resp = SetFloatResponse()
    resp.success = True
    return resp

# run gripper server    
srv_run_gripper = rospy.Service(
    'simulation/gripper/run', SetBool, run_gripper_srv)
rospy.logwarn('gripper is initialized')

# run obj handler servers
srv_get_obj_position_ = rospy.Service(
    '/simulation/supervisor/get_position', NodeGetPosition, getPositionSrvCb)
srv_get_obj_orientation_ = rospy.Service(
    '/simulation/supervisor/get_orientation', NodeGetOrientation, getOrientationSrvCb)
srv_set_obj_position_ = rospy.Service(
    '/simulation/supervisor/set_position', FieldSetVec3f, setPositionSrvCb)
srv_set_obj_orientation_ = rospy.Service(
    '/simulation/supervisor/set_orientation', FieldSetRotation, setOrientationSrvCb)
srv_change_box_size = rospy.Service(
    '/simulation/supervisor/change_box_size', SetSize, changeBoxSizeSrvCb)
srv_delete_box = rospy.Service(
    '/simulation/supervisor/delete_box', Trigger, deleteBoxSrvCb)
rospy.logwarn('Object handler is initialized')

# move base server
srv_move_base = rospy.Service(
    '/simulation/supervisor/move_base', SetFloat, moveBaseSrvCb)
rospy.logwarn('base mover is initialized')

# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    clockPublisher.publish(msg)
