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
5. Clone my repo
```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/MarioPulpo95/Quad_RL
```
6. Go to `Quad_RL/rotors_gazebo/`, copy `scripts` and `Training` folders
7. Go to `Crazys/rotors_gazebo/`, remove `scripts` folder and paste my `scripts` and `Training` folders.
8. Go to `Quad_RL/rotors_gazebo/launch` and copy `mav.launch`
9. Go to `Crazys/rotors_gazebo/launch`remove `mav.launch` file and paste my `mav.launch` file.
10. Go to `Quad_RL/rotors_description`, open `ardrone.xacro` and modify the <xacro:property name="rotor_velocity_slowdown_sim" value="10" /> with value="1".

6. Install Libraries

```console
$ pip install gym
$ pip install stable_baselines3
$ pip install tensorboard
$ source ~/.bashrc
$ export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
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

Code Documentation
------------------
Libraries Imports

`rospy` - Python library for ROS (Robot Operating System) programming.
`tf.transformations` - Module for managing transformations between quaternions and Euler angles.
`numpy` - Library for mathematical operations with multidimensional arrays.
`gym` - Library for creating learning environments for reinforcement learning.
`mav_msgs.msg` - Module for defining messages used for communication with the drone.
`gym.envs.registration` - Module for registering new Gym environments.

Definizione di alcune costanti:

`N_ACTIONS` - Number of actions available to the agent.
`SAMPLING_TIME` - Sampling interval for PID.
`LENGTH` -  Maximum length of a training episode.
`MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z` - Limits of the three-dimensional workspace in which the drone moves.
`GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y, GOAL_MIN_Z, GOAL_MAX_Z` -  Limits for generating a random goal position.

Definizione della classe Env:

__Init__(self) 
Inizializza l'ambiente, definendo lo spazio delle azioni e lo spazio delle osservazioni (‘spaces.Box’ = spazi continui)
Motor2_callback(self, data)
Funzione di callback chiamata quando viene ricevuto un messaggio sul topic /ardrone/motor_speed/2. Aggiorna il valore del motore 2.
Motor3_callback(self, data)
Funzione di callback chiamata quando viene ricevuto un messaggio sul topic /ardrone/motor_speed/3. Aggiorna il valore del motore 3.
Odometry_callback(self, data)
Funzione di callback chiamata quando viene ricevuto un messaggio sul topic /gazebo/model_states. Aggiorna i dati dell'odomotria.
get_RPY(self)
Ottiene i valori di roll, pitch e yaw dall'odometria.
get_Pose(self)
Ottiene le coordinate x, y, z dall'odometria.
get_Velocity(self)
Ottiene le velocità lineari e angolari dall'odometria.
get_MotorsVel(self)
Restituisce i valori di velocità dei motori.
check_get_pose_ready(self)
Controlla se il topic /gazebo/model_states è pronto a ricevere messaggi. Attende il messaggio fino a quando il topic è pronto o scade il timeout.
check_get_motors_ready(self)
Controlla se i topic /ardrone/motor_speed/0, /ardrone/motor_speed/1, /ardrone/motor_speed/2, /ardrone/motor_speed/3 sono pronti a ricevere messaggi. Attende i messaggi fino a quando i topic sono pronti o scade il timeout.
step(self, action)
Esegue un passo nell'ambiente. Applica il controllo di volo in base all'azione ricevuta e raccoglie le osservazioni, la ricompensa e lo stato di terminazione.
reset(self)
Resetta l'ambiente. Riporta il drone alla posizione iniziale, reimposta le variabili di stato e genera un nuovo obiettivo.
close(self)
Chiude l'ambiente. Termina i nodi ROS e Gazebo e chiude tutte le comunicazioni ROS.
send_commands(self, w1, w2, w3, w4)
Invia i comandi di velocità ai motori del drone.
get_reward(self, done)
Calcola la ricompensa in base allo stato di done.
get_observation(self)
Ottiene le osservazioni correnti dall’ambiente.
set_quad_pose(self)
Imposta la posizione iniziale del drone nel simulatore Gazebo.
is_done(self)
Verifica se l'episodio è terminato in base a condizioni come il crash o la lunghezza massima dell’episodio.
is_goal_reached(self)
Verifica se è stato raggiunto il goal
has_flipped(self)
Verifica se il quadricottero si è capovolto
is_inside_workspace(self)
Verifica se il quadricottero si trova ancora nel workspace
reset_goal(self)
Sceglie un nuovo goal
compute_distance_to_target(self, a,b)
Calcolo della distanza euclidea tra due punti
reset_variables(self)
Azzera le variabili all’inizio dell’episodio
reset_motors(self)
Azzera la velocità dei motori
make_csv(self)
Crea un file .csv
add_row(self, dati)
Aggiunge una riga al file .csv con valori in dati 
