Super important: Make sure you are using USE3.0 port to connect the smarteye camera

1. Add or change camera's intrinsic parameters for a new smarteye camera. You should put your 'config' folder into your ~/ros_ws/ so that the smarteye library can find it. You can get the 'config' folder by smarteye's calibration program. For more information, please contact with smarteye company.


2. Install pcl library:

	sudo apt-get install libpcl-dev


3. Put the 'libSmarteye' package into your ~/ros_ws/src and build it under ~/ros_ws/ (Note! ros_ws is a example name of ros workspace directory, replace it with yours.):

	source /opt/ros/melodic/setup.bash

	catkin_make


4. Source ros_ws's setup and export smarteye lib  (Note! hkclr is the user name of my computer, replace it with yours.):

	source ~/ros_ws/devel/setup.bash

	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/andylee/smarteye_ws/src/libSmarteye/src/lib/SmartEyeAPI


5. Launch the node to start the server:

roslaunch smarteye smarteye.launch


6. Trigger camera to capture point cloud. Use ros service call to trigger it:

rosservice call /hv1000/get_pointcloud "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''"


7. They are two ways to fetch point cloud:

A: by service response of the service call to '/hv1000/get_pointcloud'
B: Subscribe rostopic '/point_cloud' 
