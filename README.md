After follow the main step below, try to running ```ros2 launch orb_slam3_ros2_wrapper mono```
Because the mono type assume the depth based on the computer version so if you find the orbslam3 warning of "not initialized" please shake your camera! 


# ORB-SLAM3 ROS2 Interface Docker

This repository contains a dockerized comprehensive wrapper for ORB-SLAM3 on ROS 2 Humble for Ubuntu 22.04.
Currently, it supports both ``Monocular`` and ``Stereo`` setup for ORB-SLAM3.

# Steps to use this wrapper

## 1. Clone this repository

1. ```git clone https://github.com/xujingao13/ORB-SLAM3-Interface-Docker.git```
2. ```cd ORB-SLAM3-Interface-Docker```
3. ```git submodule update --init --recursive --remote```

## 2. Install Docker on your system

Skip this step if you have already installed docker

```bash
cd ORB-SLAM3-Interface-Docker
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

## 3. Build the image with ORB_SLAM3

1. Build the image: ```sudo docker build -t orb-slam3-humble:22.04 .```
2. Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using ```echo "xhost +" >> ~/.bashrc```
3. ```source ~/.bashrc```
4. You can see the built images on your machine by running ```sudo docker images```.

## 4. Running the container

1. ```cd ORB-SLAM3-Interface-Docker``` (ignore if you are already in the folder)
2. ```sudo docker compose run orb_slam3_22_humble```
3. This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer.
4. Once you have constructed the container, you can further work into it through ```docker exec -it -e DISPLAY=$DISPLAY @container_id /bin/bash```

## 5. Building the ORB-SLAM3 Wrapper

Launch the container using steps in (4).
```bash
cd /root/colcon_ws/
colcon build --symlink-install
source install/setup.bash
```

## 6. Launching ORB-SLAM3

Launch the container using steps in (4).
If you are inside the container, run the following:

* Monocular: ```ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py``` 
* Stereo: ```ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py```

## Running this with Olympe

1. Set up your Olympe development environment: https://developer.parrot.com/docs/olympe/installation.html. Using a virtual environment is highly recommanded.
1. Generate a parameter file of your Olympe parrot. We have prepared one if you want to use:
```bash
cd /root/colcon_ws/orb_slam3_ros2_wrapper/params
cp olympe.yaml.temp olympe.yaml
```
Then, change the ``*.yaml`` parameter file you want to use in ``unirobot_mono.launch.py``
2. Launch ORB-SLAM3: ```ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py```. You should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
3. Open another terminal and feed images to ORB-SLAM3 through Ros2. Make sure you are in the Olympe development environment
```bash
cd /root/olympe_dev
python ros2_streaming.py
```

## Running this with a Gazebo Classic simulation

1. Setup the ORB-SLAM3 ROS2 Docker using the steps above. Once you do (1) step in the ```Launching ORB-SLAM3``` section, you should see a window popup which is waiting for images. This is partially indicative of the setup correctly done.
2. Setup the simulation by following the README [here](https://github.com/suchetanrs/scout-husky-gazebo-ros2)
3. Once you are able to teleop the robot, you should be able to run ORB-SLAM3 with both the containers (simulation and wrapper) running parallely.

### Potential issues you may face.
The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics namespaced with ```scout_2``` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.
