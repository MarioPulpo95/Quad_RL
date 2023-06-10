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
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python3-rosdep python3-wstool ros-noetic-ros libgoogle-glog-dev
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
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

> In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appear, the solution proposed in [#40](https://github.com/gsilano/CrazyS/issues/40) temporany fixes the issue.

```console
$ cp build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/
```

Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
-----------------------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo apt install ros-melodic-control-toolbox ros-melodic-octomap-mapping ros-melodic-mavlink
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone https://github.com/MarioPulpo95/Quad_RL
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ sudo apt install ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-rotors-description
$ sudo apt install ros-melodic-mav-msgs ros-melodic-rotors-control
$ sudo apt install ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation
$ sudo apt install ros-melodic-rotors-joy-interface ros-melodic-rotors-hil-interface
$ sudo apt install ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs
$ rosdep update
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5. Update the pre-installed Gazebo version. This fixes the issue with the `error in REST request for accessing api.ignition.org`

```console
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ sudo apt install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event that the simulation does not start, the problem may be related to Gazebo and missing packages. Therefore, run the following commands. More details are reported in [#25](https://github.com/gsilano/CrazyS/issues/25).

```console
$ sudo apt-get remove ros-melodic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event the `cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -u -e ode` appear, the solution proposed in [#40](https://github.com/gsilano/CrazyS/issues/40) temporany fixes the issue.

```console
$ cp build/rotors_gazebo_plugins/libmav_msgs.so devel/lib/
```

Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 7
---------------------------------------------------------
1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros
$ sudo apt-get install ros-kinetic-mavlink python-catkin-tools protobuf-compiler
$ sudo apt-get install libgoogle-glog-dev ros-kinetic-control-toolbox ros-kinetic-octomap-mapping
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
 ```

 2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone https://github.com/gsilano/CrazyS.git
$ git clone -b crazys https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i
$ catkin build
```

> **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ cd ~/catkin_ws/
$ catkin build
```

 4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 9
---------------------------------------------------------
To use the code developed and stored in this repository with ROS Kinetic and Gazebo 9, first follow what is reported in the previous section. Then, use the instruction below.

1. Remove Gazebo 7 and all related packages, and then install Gazebo 9:

```console
$ sudo apt-get remove ros-kinetic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*
$ sudo apt upgrade
```
> **Note** Remove `ros-kinetic-gazebo9-*` from the command `sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*` if fetch problems should appear during the installation.

 2. Additional packages are required to build the package.

```console
$ sudo apt-get install libeigen3-dev ros-kinetic-image-view ros-kinetic-parrot-arsdk libprotobuf-dev libprotoc-dev ros-kinetic-joy-teleop ros-kinetic-nav-msgs ros-kinetic-mav-msgs libyaml-cpp-dev ros-kinetic-nodelet ros-kinetic-mav-planning-msgs ros-kinetic-urdf ros-kinetic-image-transport ros-kinetic-roslint ros-kinetic-angles ros-kinetic-cv-bridge ros-kinetic-tf2-geometry-msgs ros-kinetic-xacro ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev ros-kinetic-camera-info-manager ros-kinetic-cmake-modules ros-kinetic-gazebo-msgs ros-kinetic-mavros-msgs ros-kinetic-control-toolbox ros-kinetic-mav-msgs ros-kinetic-libmavconn ros-kinetic-mavros ros-kinetic-octomap-msgs ros-kinetic-geographic-msgs ros-kinetic-mavlink ros-kinetic-mavros-extras ros-kinetic-mav-planning-msgs ros-kinetic-joy
```
> **Note** Missing packages can be found and then installed by using the command `rosdep check --from-paths src` into the `catkin_ws` folder.

 3. Make Gazebo 9 compatible with ROS Kinetic Kame

```console
$ cd ~
$ mkdir -p ros-kinetic-gazebo9-pkgs
$ cd ros-kinetic-gazebo9-pkgs
$ git clone -b feature/ros-kinetic-gazebo9-pkgs https://github.com/gsilano/BebopS.git
$ cd BebopS
$ chmod 777 gazebo9.sh
$ ./gazebo9.sh
$ cd ~
$ sudo rm -rf ros-kinetic-gazebo9-pkgs # delete the folder after the installation
```

4. Clean the workspace and compile again the code

```console
$ cd ~/catkin_ws
$ catkin clean # digit y when required
$ cd ~/catkin_ws/src/CrazyS
$ git checkout dev/gazebo9
$ cd ~/catkin_ws/src/mav_comm
$ git checkout med18_gazebo9
$ cd ~/catkin_ws
$ catkin build
$ source ~/.bashrc
```

> **Note** In case the `ERROR[rotors_gazebo_plugins]` error is displayed, run the following commands
>```console
>$ sudo apt-get install ros-kinetic-gazebo9-plugins
>$ sudo apt-get install apt ros-kinetic-gazebo9-ros
>$ sudo apt-get install apt ros-kinetic-gazebo9-dev
>```

This guide can be used a basis for fixing what has been discussed in [ethz-asl/rotors_simulator#506](https://github.com/ethz-asl/rotors_simulator/pull/506).

Installation Instructions - Ubuntu 14.04 with ROS Indigo
--------------------------------------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c ’echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list’
$ sudo apt-key adv --keyserver hkp://ha.pool.skskeyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy
$ sudo apt-get install ros-indigo-octomap-ros python-wstool python-catkin-tools
$ sudo apt-get install protobuf compiler libgoogle-glog-dev
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ catkin init
```
> **Note** for setups with multiple workspaces please refer to the [official documentation](http://docs.ros.org/independent/api/rosinstall/html/) by replacing `rosws` by `wstool`.

 3. Get the simulator and additional dependencies

```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/gsilano/CrazyS.git
$ git clone -b crazys https://github.com/gsilano/mav_comm.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i
```

> **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

> **Note** if you want to use `wstool` you can replace the above commands with
    ```console
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
> **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ cd ~/catkin_ws/
$ catkin init  # If you haven't done this before.
$ catkin build
```
 > **Note** if you are getting errors related to "future" package, you may need python future:
    ```console
    sudo apt-get install python-pip
    pip install --upgrade pip
    pip install future
    ```

 5. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
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
