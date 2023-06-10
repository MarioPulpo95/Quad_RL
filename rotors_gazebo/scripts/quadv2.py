#!/usr/bin/python3
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import gym
from gym import spaces
from mav_msgs.msg import Actuators
from gym.envs.registration import register
import ros_node
import ros_gazebo
from std_msgs.msg import Float32
from mav_msgs.msg import Actuators
from gazebo_msgs.msg import ModelStates
from train_test import load
import csv
import rospkg
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from Position_controller import PositionController

N_ACTIONS = 3
SAMPLING_TIME = 0.01

GOAL_MIN_X = -200
GOAL_MAX_X = 200
GOAL_MIN_Y = -200   
GOAL_MAX_Y = 200
GOAL_MIN_Z = 200
GOAL_MAX_Z = 400

LENGTH = 1000 # 10 secondi

MIN_X = -400
MAX_X = 400
MIN_Y = -400
MAX_Y = 400
MIN_Z = 0
MAX_Z = 600

msg_mot = Actuators()
msg_motor = Float32()
msg_act = Actuators()

class TestEnv(gym.Env):

    def __init__(self):
        self.action_space = spaces.Box(low = np.array([-0.5, -0.5, -1.0]), high =np.array([0.5, 0.5, 1.0]), shape=(N_ACTIONS,), dtype=np.float32) 

        # NO-TILT ENV OBSERVATION

        #low = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.radians(180), -np.radians(180), -np.radians(180), 0, 0])
        #high = np.array([ np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf, np.radians(180),  np.radians(180),  np.radians(180), 1, 1])

        # TITL ENV OBSERVATION 

        low = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,-np.inf, -np.inf, -np.inf, -np.radians(180), -np.radians(180), -np.radians(180), 0, 0])
        high = np.array([ np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf, np.inf, np.inf, np.inf, np.radians(180),  np.radians(180),  np.radians(180), 1, 1])

        self.observation_space = spaces.Box(low=low, high=high, shape=(14,), dtype=np.float64) # change shape to 11 if NOTILT ENV

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.Odometry_callback)
        rospy.Subscriber('/ardrone/motor_speed/0', Float32, self.Motor0_callback)
        rospy.Subscriber('/ardrone/motor_speed/1', Float32, self.Motor1_callback)
        rospy.Subscriber('/ardrone/motor_speed/2', Float32, self.Motor2_callback)
        rospy.Subscriber('/ardrone/motor_speed/3', Float32, self.Motor3_callback)

        self.commands_pub = rospy.Publisher('/ardrone/command/motor_speed', Actuators, queue_size=1)
        self.start_time =  None

        self.filecsv = 'RANDOMYAW_TEST.csv'
        self.dati = []
        self.crea_file_csv()
        self.PositionController = PositionController()
        self.PositionController.init_PIDs()
    
    def Motor0_callback(self,data):
        self.mot0 = data
    def Motor1_callback(self,data):
        self.mot1 = data
    def Motor2_callback(self,data):
        self.mot2 = data
    def Motor3_callback(self,data):
        self.mot3 = data
    def Odometry_callback(self, data):
        self.odometry = data
    def get_RPY(self):
        Quaternion = np.array([self.odometry.pose[1].orientation.x, self.odometry.pose[1].orientation.y, self.odometry.pose[1].orientation.z, self.odometry.pose[1].orientation.w])
        self.roll, self.pitch, self.yaw = euler_from_quaternion(Quaternion)
        self.roll *= 100
        self.pitch *= 100
        self.yaw *= 100
    def get_Pose(self):
        self.x = self.odometry.pose[1].position.x*100
        self.y = self.odometry.pose[1].position.y*100
        self.z = self.odometry.pose[1].position.z*100
    def get_Velocity(self):
        self.vx = self.odometry.twist[1].linear.x
        self.vy = self.odometry.twist[1].linear.y
        self.vz = self.odometry.twist[1].linear.z
        self.p = self.odometry.twist[1].angular.x
        self.q = self.odometry.twist[1].angular.y
        self.r = self.odometry.twist[1].angular.z
    def get_MotorsVel(self):
        return self.mot0.data, self.mot1.data, self.mot2.data, self.mot3.data
    
    def check_get_pose_ready(self):
        self.odometry = None
        rospy.logdebug("Waiting for /get_pose to be READY...")
        while self.odometry is None and not rospy.is_shutdown():
            try:
                self.odometry = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
                rospy.logdebug("Current /get_pose READY=>")

            except:
                rospy.logerr("Current /get_pose not ready yet, retrying for getting get_pose")

        return self.odometry

    def check_get_motors_ready(self):
        self.mot0 = None
        self.mot1 = None
        self.mot2 = None
        self.mot3 = None
        rospy.logdebug("Waiting for /get_motors to be READY...")
        while self.mot0 is None and not rospy.is_shutdown():
            try:
                self.mot0 = rospy.wait_for_message('/ardrone/motor_speed/0', Float32, timeout=5.0)
                rospy.logdebug("Current /get_motors READY=>")

            except:
                rospy.logerr("Current /get_motors not ready yet, retrying for getting get_motors")
        rospy.logdebug("Waiting for /get_motors to be READY...")
        while self.mot1 is None and not rospy.is_shutdown():
            try:
                self.mot1 = rospy.wait_for_message('/ardrone/motor_speed/1', Float32, timeout=5.0)
                rospy.logdebug("Current /get_motors READY=>")

            except:
                rospy.logerr("Current /get_motors not ready yet, retrying for getting get_motors")
                rospy.logdebug("Waiting for /get_motors to be READY...")
        while self.mot2 is None and not rospy.is_shutdown():
            try:
                self.mot2 = rospy.wait_for_message('/ardrone/motor_speed/2', Float32, timeout=5.0)
                rospy.logdebug("Current /get_motors READY=>")

            except:
                rospy.logerr("Current /get_motors not ready yet, retrying for getting get_motors")
                rospy.logdebug("Waiting for /get_motors to be READY...")
        while self.mot3 is None and not rospy.is_shutdown():
            try:
                self.mot4 = rospy.wait_for_message('/ardrone/motor_speed/3', Float32, timeout=5.0)
                rospy.logdebug("Current /get_motors READY=>")

            except:
                rospy.logerr("Current /get_motors not ready yet, retrying for getting get_motors")
        

        return self.mot0, self.mot1, self.mot2, self.mot3

    def step(self, action):
        
        self.info = {}
        start_time = rospy.Time.now().to_sec()

        for k in range(5):
            if self.start_time is not None:
                delay = rospy.Time.now().to_sec() - self.start_time
                if delay < SAMPLING_TIME:
                    rospy.sleep(SAMPLING_TIME - delay)
                    #rospy.logwarn('delay:{}'.format(SAMPLING_TIME - delay))
            self.get_Velocity()
            thrust_cmd = self.PositionController.ZVelocityController(action, self.vz)
            self.get_Velocity()
            roll_ref, pitch_ref, yaw_ref = self.PositionController.VelocityController(action,self.vx, self.vy, self.yaw, self.random_yaw)
            self.get_RPY()
            self.get_Velocity()
            roll_cmd, pitch_cmd, yaw_cmd = self.PositionController.AttitudeController(roll_ref, pitch_ref, yaw_ref, self.roll, self.pitch, self.r)
            W_FR, W_BL, W_FL, W_BR = self.PositionController.ControlMixer(roll_cmd, pitch_cmd, thrust_cmd, yaw_cmd)
            self.send_commands(W_FR, W_BL, W_FL, W_BR)
            self.start_time = rospy.Time.now().to_sec()

        #dtRL = rospy.Time.now().to_sec() - start_time
        #rospy.logwarn('dtRL:{}'.format(dtRL))

        ros_gazebo.gazebo_pause_physics()
        observation = self.get_observation()
        done = self.is_done()
        reward, info, done = self.get_reward(done,action)
        ros_gazebo.gazebo_unpause_physics()

        return observation, reward, done, info

    def reset(self):
        ros_gazebo.gazebo_reset_world()
        ros_gazebo.gazebo_unpause_physics()
        self.check_get_motors_ready()
        self.check_get_pose_ready()
        self.reset_variables()
        self.set_goal() # random goals
        #self.set_new_waypoint() # waypoints goals
        self.reset_motors()
        self.set_quad_pose() # to start with random yaw -> set Position controller too
        self.PositionController.reset_PIDs()
        ros_gazebo.gazebo_pause_physics()
        observation = self.get_observation()
        ros_gazebo.gazebo_unpause_physics()
        return observation 

    def close (self):
        rospy.loginfo("Closing Env")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")
        ros_gazebo.close_Gazebo()
        ros_node.ros_kill_all_nodes()
        ros_node.ros_kill_master()
        ros_node.ros_kill_all_processes()
        print("Closed ROS and Env")

    def send_commands(self,w1, w2, w3, w4):
        global msg_mot
        self.get_Pose()
        msg_mot.angular_velocities.clear()
        msg_mot.header.stamp = rospy.Time.now()
        msg_mot.angular_velocities = [w1, w2, w3, w4]
        self.commands_pub.publish(msg_mot)
        #rospy.logwarn('Commands_Published:{}'.format(msg_mot.angular_velocities))  

    def get_reward(self,done,action):

        c = 8 
        GOAL_BONUS = 0
        CRASH_PENALTY = 0
        DISTANCE_REWARD = -self.distance_to_goal/100
        self.tilt = -np.linalg.norm(self.drone_angular)/c
        TILT_PENALTY = self.tilt
        #TILT_PENALTY = 0
        self.terminated = False

        if done == True:
            CRASH_PENALTY = -100
            self.reward = CRASH_PENALTY
            self.info['is_success'] = False
            self.terminated = True

        elif done == False:
            self.reward = DISTANCE_REWARD + TILT_PENALTY  # TILT ENV
            #self.reward = DISTANCE_REWARD # NO TILT ENV
            if self.distance_to_goal < 0.15:
                GOAL_BONUS = 100
                self.reward = GOAL_BONUS
                rospy.loginfo('GoaL:{}'.format(self.random_goal))
                rospy.loginfo('Goal Reached')
                rospy.loginfo('Error:{}'.format(self.distance_to_goal))
                self.info['is_success'] = True
                self.set_goal()
                #self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
                #self.set_new_waypoint()
                rospy.loginfo('New Goal:{}'.format(self.random_goal))

            if self.counter_steps > LENGTH:
                self.terminated = True
                rospy.loginfo('Episode Terminated')
        
        self.cumulated_reward += self.reward

        R1 = DISTANCE_REWARD
        self.crash = CRASH_PENALTY
        self.bonus = GOAL_BONUS
        R4 = TILT_PENALTY
        TOTAL_REW = self.cumulated_reward
        N_STEPS = self.counter_steps
        DONE = self.terminated
        ERRORE = self.distance_to_goal
        X = self.x
        Y = self.y
        Z = self.z
        VX = self.vx
        VY = self.vy
        VZ = self.vz
        X_GOAL = self.random_goal[0]
        Y_GOAL = self.random_goal[1]
        Z_GOAL = self.random_goal[2]
        ROLL = self.roll/100
        PITCH = self.pitch/100
        YAW = self.yaw/100
        WX = self.p
        WY = self.q
        WZ = self.r
        
        self.dati = [self.crash, self.bonus, N_STEPS, DONE, ERRORE, X, Y, Z, VX, VY, VZ, X_GOAL, Y_GOAL, Z_GOAL, ROLL, PITCH, YAW, WX, WY, WZ, action[0],action[1],action[2], R1, R4, TOTAL_REW]
        self.aggiungi_riga(self.dati)
        
        return self.reward, self.info, self.terminated

    def get_observation(self):
        self.get_Pose()
        self.get_Velocity()
        self.get_RPY()
  
        self.drone_position = np.array([self.x, self.y, self.z]) 
        self.drone_velocity = np.array([self.vx, self.vy, self.vz]) 
        self.drone_angular = np.array([self.p, self.q, self.r])

        error_x = (self.random_goal[0] - self.x)/100
        error_y = (self.random_goal[1] - self.y)/100
        error_z = (self.random_goal[2] - self.z)/100
        self.distance_to_goal = self.compute_distance_to_target(self.random_goal, self.drone_position)/100

        goal_reached = self.is_goal_reached()
        has_flipped = self.has_flipped()

        if has_flipped == True:
            crash = 1
        else:
            crash = 0
        
        # TILT ENV OBS
        '''obs = np.array([error_x, error_y, error_z, 
                        self.vx, self.vy, self.vz,
                        self.p, self.q, self.r,
                        self.roll/100, self.pitch/100, self.yaw/100, 
                        goal_reached, crash])'''
        
        # NO TILT ENV OBS
        '''obs = np.array([error_x, error_y, error_z, 
                        self.vx, self.vy, self.vz,
                        self.roll/100, self.pitch/100, self.yaw/100,
                        goal_reached, crash])'''
        
        # RANDOM YAW OBS
        obs = np.array([error_x, error_y, error_z, 
                        self.vx, self.vy, self.vz,
                        self.p, self.q, self.r,
                        self.roll/100, self.pitch/100, self.random_yaw - self.yaw/100,
                        goal_reached, crash])
        #rospy.logerr("Observations:{}".format(np.array([obs])))
        return obs
    
    def set_quad_pose(self):

        model_state_msg = ModelState()
        roll = 0
        pitch = 0
        self.random_yaw = np.random.uniform(-np.radians(180), np.radians(180))
        x,y,z,w = quaternion_from_euler(roll,pitch,self.random_yaw)
        model_state_msg.model_name = 'ardrone' 
        model_state_msg.pose.position.x = 0.0
        model_state_msg.pose.position.y = 0.0
        model_state_msg.pose.position.z = 0.1
        model_state_msg.pose.orientation.x = x
        model_state_msg.pose.orientation.y = y
        model_state_msg.pose.orientation.z = z
        model_state_msg.pose.orientation.w = w
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        response = set_model_state(model_state_msg)

    def is_done(self): 
        done = False
        self.counter_steps +=1
        has_flipped = self.has_flipped()

        if has_flipped == True:
            done = True
        #if self.counter_steps == LENGTH:
            #self.plot()

        return done

    def is_goal_reached(self):
        if self.distance_to_goal < 0.15:
            goal_reached = 1
        else: 
            goal_reached = 0
        return goal_reached
    
    def set_goal(self):
        
        x_goal = np.random.uniform(GOAL_MIN_X, GOAL_MAX_X)
        y_goal = np.random.uniform(GOAL_MIN_Y, GOAL_MAX_Y)
        z_goal = np.random.uniform(GOAL_MIN_Z, GOAL_MAX_Z)

        self.random_goal = np.array([x_goal, y_goal, z_goal])

    def set_new_waypoint(self):
        # SQUARE TRAJECTORY WAYPOINTS
        self.waypoints = [[0 , 0, 200], [100, -100, 200], [100, 100, 200], [-100, 100, 200], [-100, -100, 200], [0, 0, 200]]
        
        x_goal = self.waypoints[self.waypoint_index][0]
        y_goal = self.waypoints[self.waypoint_index][1]
        z_goal = self.waypoints[self.waypoint_index][2]

        self.random_goal = np.array([x_goal, y_goal, z_goal])

    def compute_distance_to_target(self,a,b):
        distance = np.linalg.norm(a-b)
        return distance
       
    def is_inside_workspace(self):
        is_inside = False
        self.get_Pose()
        if self.x >= MIN_X and self.x <= MAX_X:
            if self.y >= MIN_Y and self.y <= MAX_Y:
                if self.z >= MIN_Z and self.z<= MAX_Z:
                    is_inside = True
        
        return is_inside

    def has_flipped(self):
        self.get_RPY()
        has_flipped = False
        if not(-np.radians(90)*100 <= self.roll <= np.radians(90)*100) or not(-np.radians(90)*100 <= self.pitch <= np.radians(90)*100) :
            has_flipped = True
        return has_flipped

    def reset_variables(self):
        self.counter_steps = 0
        self.cumulated_reward = 0.0
        self.drone_position = None
        self.drone_velocity = None
        self.drone_angular = None
        self.distance_to_goal = None
        self.reward = 0.0
        self.waypoint_index = 0

    def reset_motors(self):
        global msg_mot, msg_motor
        w1,w2,w3,w4 = self.get_MotorsVel()
        while not(round(w1,2) == 0.0 and round(w2,2) == 0.0 and round(w3,2) == 0.0 and round(w4,2) == 0.0):
            msg_mot.angular_velocities.clear()
            msg_mot.header.stamp = rospy.Time.now()
            msg_mot.angular_velocities = [0,0,0,0]
            self.commands_pub.publish(msg_mot)
            w1,w2,w3,w4 = self.get_MotorsVel()
            velocities = np.array([w1,w2,w3,w4])
            #rospy.logwarn('Motors:{}'.format(np.array([velocities])))

    def crea_file_csv(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rotors_gazebo')
        intestazioni = ['CRASH', 'GOAL', 'N_STEPS', 'DONE', 'ERRORE', 'X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'X_GOAL', 'Y_GOAL', 'Z_GOAL', 'ROLL', 'PITCH', 'YAW', 'WX', 'WY', 'WZ','VX_ACTION', 'VY_ACTION', 'VZ_ACTION', 'DISTANCE_REW', 'TILT_REW', 'TOTAL_REW', 'X_REF', 'Y_REF', 'Z_REF']
        self.file_csv = pkg_path + '/Training/NewTests/' + self.filecsv

        with open(self.file_csv, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(intestazioni)

    def aggiungi_riga(self, dati):
        with open(self.file_csv, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(dati)


if __name__ == '__main__':
    try:
        rospy.init_node('quadcopter_training', anonymous=True)
        reg = register(id='Quad-v2', entry_point='quadv2:TestEnv', max_episode_steps=LENGTH)
        load()
    except rospy.ROSInterruptException:
        pass
