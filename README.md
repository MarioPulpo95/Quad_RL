[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This package an extension of Crazys and the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to inlcude Reinforcemet Learning control.

Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11
-----------------------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Noetic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
$ sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev 
$ sudo apt-get install python3-rosdep python3-wstool ros-noetic-ros libgoogle-glog-dev
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

2. If you don't have ROS workspace yet you can do so by

```console
$ export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
5. Install Libraries

```console
$ pip install gym
$ pip install stable_baselines3
$ pip install tensorboard
$ source ~/.bashrc
$ export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```

> In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appear, the solution proposed in [#40](https://github.com/gsilano/CrazyS/issues/40) temporany fixes the issue.

```console
$ cp build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/
```



Basic Usage
-----------
Launch simulation with `mav.launch` located in /rotors_gazebo/launch/

Two training environments: `quadv0.py` and `quadv1.py`.

In `quadv0.py` you can train an agent without tilt regulation.



Go to `train_load.py`, choose the algorithm and tune the hyperparameters as you want.

In `start_train` function use 'Quad-v0' inside gym.make() function 

```console
$ roslaunch rotors_gazebo mav.launch
$ rosrun quadv0.py
```
In `quadv1.py` you can train an agent without tilt regulation.

Go to `train_load.py`, choose the algorithm and tune the hyperparameters as you want.

In `start_train()` function use 'Quad-v1' inside gym.make() function 

```console
$ roslaunch rotors_gazebo mav.launch
$ rosrun quadv1.py
```
In `quadv2.py` you can test your agent without/with tilt regulation.

Go to `train_load.py`, load an algorithm from models folder.

In `load()` function use 'Quad-v2' inside gym.make() function 

```console
$ roslaunch rotors_gazebo mav.launch
$ rosrun quadv2.py
```
You can analyze the train and test results in `plot_notebook.py`

The PositionController is located in `/rotor_gazebo/scripts/Position_controller.py`
