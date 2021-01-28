# HKCLR-Depalletizing-Demo
Control algorithm of a self-developed depalletizing robot
## Note 
If you have Docker engine, you can skip 'Pre-requisites' and 'Install'. you can start from 'Docker Image' to build the [Dockerfile](Dockerfile) as a Docker Image and run this repo in a Docker container. If you prefer to use this repo without Docker engine, you can ignore Dockerfile and you have to meet the Pre-requests as below: 
## Pre-requests
 - Ubuntu 18.04
 - ROS Melodic
 - Webots 2020b-rev1 or newer version:  
   If you are new to Webots, copy the link below in a web browser such as Google Chrome, and download the Webots debian package, open it and click 'install'.
   `````
   https://github.com/cyberbotics/webots/releases/download/R2020b-rev1/webots_2020b-rev1_amd64.deb
   `````
- Dependence:
````
  sudo apt-get update && apt-get install -y \
    ros-melodic-moveit \
    ros-melodic-actionlib \
    ros-melodic-actionlib-tutorials \
    ros-melodic-control-msgs \
    ros-melodic-roscpp \
    ros-melodic-behaviortree-cpp-v3 \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \  
````
## Install
1. Compile this under workspace `~/HKCLR-Depalletizing-Demo/depalletizing_ws`
````
catkin_make
````
2. Add 'export' to .bashrc
````
export PYTHONPATH=${PYTHONPATH}:~/HKCLR-Depalletizing-Demo/depalletizing_ws/src
````
3. Add 'source' to .bashrc
````
source ~/HKCLR-Depalletizing-Demo/depalletizing_ws/devel/setup.bash
````
## Docker Image
The [Dockerfile](Dockerfile), which can be used to build a docker image consists of ubuntu18.04, nvidia/cudagl, webots, ros-melodic, moveit, and self-defined ROS packages. 
This is super useful and convenient. Any computer with recent Docker Engine and Nvidia GPU can easily use this repo without installing a ton of dependence. 
Also, it can beyond the limitation of operating system, eg. you can even use this repo on Windows. 
### Creat Image
Using docker without 'sudo' the cmd. If you never done this before, it is suggested that typing in following cmd:
````
sudo groupadd docker
sudo gpasswd -a ${USER} docker
sudo systemctl restart docker
sudo chmod a+rw /var/run/docker.sock
````
Create the image using Dockerfile， Run the cmd under `~/HKCLR-Depalletizing-Demo/depalletizing_ws`  
````
docker build -t webots_moveit_robot .
````
This will create the deployment image named `webots_moveit_robot`

### NVIDIA Container Toolkit
You will need the NVIDIA GPU support for the docker container. To build the bridge, run following cmd:
```
curl https://get.docker.com | sh \
  && sudo systemctl start docker \
  && sudo systemctl enable docker
```
````
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
````
````
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
````

### Run Image
To run Webots and Rviz with a graphical user interface in a docker container, you need to enable connections to the X server before starting the docker container:
````
xhost +local:root > /dev/null 2>&1
````
Run docker image in container called `hkclr_robot_interface`
````
docker run --gpus=all --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it --rm --name hkclr_robot_interface webots_moveit_robot bash
````
Tips: You can create `.bash_aliases` file in `~` and add the following to it(optional):
````
alias robot_run="docker run --gpus=all --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it --rm --name hkclr_robot_interface webots_moveit_robot bash"
````
Such that you only need to issue `robot_run` to launch the program.  

This repo needs many terminals in total, so after you do docker run in one terminal, you have to open other new terminals, and type in following cmd to connect them with the first one you opened by the same container `hkclr_robot_interface` 
````
docker exec -it hkclr_robot_interface bash
````

## Usage
No matter whether you use or not use docker, following part are the same.
## Simulation
### Launch webots world with SelfRobot and its drivers
````
roslaunch robot_webots robot_webots_simulation.launch
````
### Launch moveit move_group with ikfast kinematics plugin
````
roslaunch robot_controller moveit_group.launch use_sim_time:=true
````
Now you can control SelfRobot in Webots by MontionPlanning Plugin in Rviz
### Launch a small demo with collision free planning
````
roslaunch roport robot_minimal_demo.launch
````
or Without collision free
````
roslaunch roport test.launch
````
### Other complex function is still being developing

## Real Robot
### Launch ros controller of SelfRobot and its hardware interface
Currently, only fake controller is available.
````
roslaunch robot_controller ArmController.launch
````
### Launch moveit move_group with ikfast kinematics plugin
````
roslaunch robot_controller moveit_group.launch use_sim_time:=false
````
Now you can control real SelfRobot by MontionPlanning Plugin in Rviz
### Other complex function is still being developing
