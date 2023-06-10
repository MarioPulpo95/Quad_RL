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
from PID import PID
import matplotlib.pyplot as plt
from train_test import start_train
import csv
import rospkg
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

N_ACTIONS = 3
SAMPLING_TIME = 0.01                    

LENGTH = 200 # 10 secondi

MIN_X = -400
MAX_X = 400
MIN_Y = -400
MAX_Y = 400
MIN_Z = 0
MAX_Z = 600

KP_X = 0.006 # MAX_OUTPUT 2 #0.005/0.00002/ 0.0009
KI_X = 0.0002
KD_X = 0.000

KP_Y = KP_X
KI_Y = KI_X
KD_Y = KD_X

KP_Z = 0.021
KI_Z = 0.009
KD_Z = 0.0009

KP_ROLL = 2.5# MAX OUTPUT 800
KI_ROLL = 0.3                                 
KD_ROLL = 1.2              
KP_PITCH = KP_ROLL
KI_PITCH = KI_ROLL
KD_PITCH = KD_ROLL
#90 5
KP_YAW = 57
KI_YAW = 8
KD_YAW = 0

KP_VX = 12
KI_VX = 0.7
KD_VX = 0

KP_VY = KP_VX
KI_VY = KI_VX
KD_VY = KD_VX

KP_VZ = 880
KI_VZ = 600
KD_VZ = 0

KP_YAW_RATE = 0.6
KI_YAW_RATE = 0.07
KD_YAW_RATE = 0.3

MAX_THRUST_COMMAND = 800
MIN_THRUST_COMMAND = 600
MAX_ROLL_COMMAND = 13
MAX_PITCH_COMMAND = 13
MAX_YAW_COMMAND = 9

MAX_VELOCITY = 2.0
MAX_Z_VELOCITY = 2.0
MIN_Z_VELOCITY = -2.0
MAX_ROLL_VELOCITY = 800
MAX_PITCH_VELOCITY = 800
MAX_YAW_VELOCITY = 800
MIN_ROLL_VELOCITY = -800
MIN_PITCH_VELOCITY = -800
MIN_YAW_VELOCITY = -800

MAX_PROPELLER_VELOCITY = 800
MIN_PROPELLER_VELOCITY = 0

msg_mot = Actuators()
msg_motor = Float32()
msg_act = Actuators()

class QuadcopterEnv(gym.Env):

    def __init__(self):
        self.action_space = spaces.Box(low = np.array([-0.5, -0.5, -1.0]), high =np.array([0.5, 0.5, 1.0]), shape=(N_ACTIONS,), dtype=np.float32) 
    
        #low = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,-np.inf, -np.inf, -np.inf, -np.radians(180), -np.radians(180), -np.radians(180), 0, 0])
        #high = np.array([ np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf, np.inf, np.inf, np.inf, np.radians(180),  np.radians(180),  np.radians(180), 1, 1])

        low = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.radians(180), -np.radians(180), -np.radians(180), 0, 0])
        high = np.array([ np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf, np.radians(180),  np.radians(180),  np.radians(180), 1, 1])
        self.observation_space = spaces.Box(low=low, high=high, shape=(11,), dtype=np.float64)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.Odometry_callback)
        rospy.Subscriber('/ardrone/motor_speed/0', Float32, self.Motor0_callback)
        rospy.Subscriber('/ardrone/motor_speed/1', Float32, self.Motor1_callback)
        rospy.Subscriber('/ardrone/motor_speed/2', Float32, self.Motor2_callback)
        rospy.Subscriber('/ardrone/motor_speed/3', Float32, self.Motor3_callback)

        self.commands_pub = rospy.Publisher('/ardrone/command/motor_speed', Actuators, queue_size=1)
        self.start_time =  None
        
        self.init_lists()

        self.filecsv = 'SAC5_notilt.csv'
        self.dati = []
        self.init_PIDs()
        self.crea_file_csv()
        
    def init_PIDs(self):
        self.x_pid = PID(P = KP_X, I = KI_X, D = KD_X, current_time=None)
        self.y_pid = PID(P = KP_Y, I = KI_Y, D = KD_Y, current_time=None)
        self.z_pid = PID(P = KP_Z, I = KI_Z, D = KD_Z, current_time=None)
        self.vx_pid = PID(P = KP_VX, I = KI_VX, D = KD_VX, current_time=None)
        self.vy_pid = PID(P = KP_VY, I = KI_VY, D = KD_VY, current_time=None)
        self.vz_pid = PID(P = KP_VZ, I = KI_VZ, D = KD_VZ, current_time=None)
        self.roll_pid = PID(P = KP_ROLL, I = KI_ROLL, D = KD_ROLL, current_time=None)
        self.pitch_pid = PID(P = KP_PITCH, I = KI_PITCH, D = KD_PITCH, current_time=None)
        self.yaw_rate_pid =  PID(P = KP_YAW_RATE, I = KI_YAW_RATE, D = KD_YAW_RATE, current_time=None)
        self.yaw_pid = PID(P = KP_YAW, I = KI_YAW, D = KD_YAW, current_time=None)

        self.x_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.y_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.z_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.roll_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.pitch_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.yaw_rate_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.yaw_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.vx_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.vy_pid.setSampleTime(sample_time = SAMPLING_TIME)
        self.vz_pid.setSampleTime(sample_time = SAMPLING_TIME)
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
        for k in range(5): # disaccopio TsRL da TsPID
            if self.start_time is not None:
                delay = rospy.Time.now().to_sec() - self.start_time
                if delay < SAMPLING_TIME:
                    rospy.sleep(SAMPLING_TIME - delay)
                    #rospy.logwarn('delay:{}'.format(SAMPLING_TIME - delay))
            W_FR, W_BL, W_FL, W_BR = self.ControlMixer(action)
            self.send_commands(W_FR, W_BL, W_FL, W_BR)
            self.start_time = rospy.Time.now().to_sec()
        dtRL = rospy.Time.now().to_sec() - start_time
        #rospy.logwarn('dtRL:{}'.format(dtRL))
        ros_gazebo.gazebo_pause_physics()
        observation = self.get_observation()
        done = self.is_done()
        reward, info, done = self.get_reward(done, action)
        ros_gazebo.gazebo_unpause_physics()
        return observation, reward, done, info

    def reset(self):
        #rospy.logwarn('Resetting Env')
        ros_gazebo.gazebo_reset_world()
        ros_gazebo.gazebo_unpause_physics()
        self.check_get_motors_ready()
        self.check_get_pose_ready()
        self.reset_variables()
        self.reset_goal()
        self.reset_motors()
        #self.set_quad_pose()
        self.reset_PIDs()
        ros_gazebo.gazebo_pause_physics()
        observation = self.get_observation()
        self.prev_distance = self.distance_to_goal
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
        self.append_to_lists() 

    def get_reward(self,done,action):

        GOAL_BONUS = 0
        CRASH_PENALTY = 0
        DISTANCE_REWARD = -self.distance_to_goal/100
        TILT_PENALTY = -np.linalg.norm(self.drone_angular)/8
        info = {}
        terminated = False

        if done == True:
            CRASH_PENALTY = -100
            self.reward = CRASH_PENALTY
            info['is_success'] = False
            terminated = True

        elif done == False:
            self.reward = DISTANCE_REWARD
            if self.distance_to_goal < 0.15:
                GOAL_BONUS = 100
                self.reward = GOAL_BONUS
                rospy.loginfo('Goal Reached')
                rospy.loginfo('Distance:{}'.format(self.distance_to_goal))
                info['is_success'] = True
                terminated = True

        
        self.cumulated_reward += self.reward

        R1 = TILT_PENALTY
        R2 = CRASH_PENALTY
        R3 = GOAL_BONUS
        R4 = DISTANCE_REWARD
        TOTAL_REW = self.cumulated_reward
        N_STEPS = self.counter_steps
        DONE = terminated
        ERRORE = self.distance_to_goal

        self.dati = [TOTAL_REW, R1, R2, R3, R4, N_STEPS, DONE, ERRORE]
        self.aggiungi_riga(self.dati)
        
        return self.reward, info, terminated

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

        '''obs = np.array([error_x, error_y, error_z, 
                        self.vx, self.vy, self.vz,
                        self.p, self.q, self.r,
                        self.roll/100, self.pitch/100, self.yaw/100, 
                        goal_reached, crash])'''
        obs = np.array([error_x, error_y, error_z, 
                        self.vx, self.vy, self.vz,
                        self.roll/100, self.pitch/100, self.yaw/100, 
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
    
    def reset_goal(self):

        x_goal = np.random.uniform(-200,200)
        y_goal = np.random.uniform(-200,200)
        z_goal = np.random.uniform(200,400)

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

        #rospy.loginfo('Goal:{}'.format(self.random_goal))
        #self.init_lists()


    def reset_PIDs(self):
        self.x_pid.clear(setpoint = 0.0, windup_limit = 2) 
        self.y_pid.clear(setpoint = 0.0, windup_limit = 2)
        self.z_pid.clear(setpoint = 0.0, windup_limit = 2)
        self.roll_pid.clear(setpoint = 0.0, windup_limit = 800)
        self.pitch_pid.clear(setpoint = 0.0, windup_limit = 800)
        self.yaw_rate_pid.clear(setpoint = 0.0, windup_limit = MAX_YAW_COMMAND)
        self.yaw_pid.clear(setpoint = 0.0, windup_limit = 800)
        self.vx_pid.clear(setpoint = 0.0, windup_limit = MAX_PITCH_COMMAND) 
        self.vy_pid.clear(setpoint = 0.0, windup_limit = MAX_ROLL_COMMAND)
        self.vz_pid.clear(setpoint = 0.0, windup_limit = MAX_THRUST_COMMAND)

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

    def XYController(self,action):
        self.get_Pose()
        self.get_RPY()

        x_ref = 200
        y_ref = 100

        feedback_x = self.x
        feedback_y = self.y
    
        self.x_pid.SetPoint = x_ref 
        self.y_pid.SetPoint = y_ref 

        x_ref_proj = x_ref*np.cos(self.yaw) + y_ref*np.sin(self.yaw)
        y_ref_proj = -x_ref*np.sin(self.yaw) + y_ref*np.cos(self.yaw)

        feedback_x_proj = feedback_x*np.cos(self.yaw) + feedback_y*np.sin(self.yaw)
        feedback_y_proj = -feedback_x*np.sin(self.yaw) + feedback_y*np.cos(self.yaw)

        error_x_proj = x_ref_proj - feedback_x_proj
        error_y_proj = y_ref_proj - feedback_y_proj

        rospy.loginfo('Feedback_proj:{}'.format(feedback_y_proj))
        rospy.loginfo('Error:{}'.format(error_y_proj))
        
        self.vy_rate = self.y_pid.update(error_y_proj, current_time = None)
        
        self.vy_rate = np.clip(self.vy_rate, -MAX_VELOCITY, MAX_VELOCITY)
 
        rospy.loginfo('Y-ControlAction:{}'.format(self.vy_rate))

        rospy.loginfo('Feedback_proj:{}'.format(feedback_x_proj))
        rospy.loginfo('Error:{}'.format(error_x_proj))

        self.vx_rate = self.x_pid.update(error_x_proj, current_time = None)
        
        self.vx_rate = np.clip(self.vx_rate, -MAX_VELOCITY, MAX_VELOCITY)

        rospy.loginfo('X-ControlAction:{}'.format(self.vx_rate))

        return self.vx_rate, self.vy_rate
    
    def VelocityController(self, action):
        self.get_Velocity()

        #vx_ref, vy_ref = self.XYController(action)

        #vx_ref = self.map_action(action[0],-1,1,-1.5,1.5)
        #vy_ref = self.map_action(action[1],-1,1,-1.5,1.5)
        vx_ref = action[0]
        vy_ref = action[1]
        yaw_ref = 0.0

        self.vx_pid.SetPoint = vx_ref
        self.vy_pid.SetPoint = vy_ref
        self.yaw_rate_pid.SetPoint = yaw_ref

        feedback_vx = self.vx
        feedback_vy = self.vy
        feedback_yaw = self.yaw

        error_vx = self.vx_pid.SetPoint - feedback_vx
        error_vy = self.vy_pid.SetPoint - feedback_vy
        error_yaw = self.yaw_rate_pid.SetPoint - feedback_yaw

        #rospy.loginfo('Feedback:{}'.format(feedback_vy))
        #rospy.loginfo('Error:{}'.format(error_vy))

        self.roll_rate = self.vy_pid.update(error_vy, current_time = None)
        
        self.roll_rate = np.clip(self.roll_rate, -MAX_ROLL_COMMAND, MAX_ROLL_COMMAND)

        #rospy.loginfo('VY-ControlAction:{}'.format(self.roll_rate))

        #rospy.loginfo('Feedback:{}'.format(feedback_vx))
        #rospy.loginfo('Error:{}'.format(error_vx))

        self.pitch_rate = self.vx_pid.update(error_vx, current_time = None)
        
        self.pitch_rate = np.clip(self.pitch_rate,-MAX_PITCH_COMMAND, MAX_PITCH_COMMAND)

        #rospy.loginfo('VX-ControlAction:{}'.format(self.pitch_rate))
        self.yaw_rate = self.yaw_rate_pid.update(error_yaw, current_time = None)
        
        self.yaw_rate = np.clip(self.yaw_rate,-MAX_YAW_COMMAND, MAX_YAW_COMMAND)

        return -self.roll_rate, self.pitch_rate, self.yaw_rate

    def AltitudeController(self):
        self.get_Pose()

        self.z_pid.SetPoint = 100
        feedback_z = self.z
        error = self.z_pid.SetPoint - feedback_z
        
        rospy.loginfo('Feedback:{}'.format(feedback_z))
        rospy.loginfo('Error:{}'.format(error))

        output = self.z_pid.update(error, current_time = None)

        output = np.clip(output, MIN_Z_VELOCITY, MAX_Z_VELOCITY)
        
        rospy.loginfo('Z-ControlAction:{}'.format(output))
        return output
    
    def ZVelocityController(self,action):
        self.get_Velocity()
        self.get_Pose()

        #self.vz_pid.SetPoint = self.AltitudeController()

        #self.vz_pid.SetPoint = self.map_action(action[2],-1,1,-1.5,1.5)
        self.vz_pid.SetPoint = action[2]
        #self.vz_pid.SetPoint = 1

        feedback_vz = self.vz
        error_vz = self.vz_pid.SetPoint - feedback_vz 

        #rospy.loginfo('Feedback:{}'.format(feedback_vz))
        #rospy.loginfo('Error:{}'.format(error_vz))

        self.thrust_rate = self.vz_pid.update(error_vz, current_time = None)

        self.thrust_rate = np.clip(self.thrust_rate, MIN_THRUST_COMMAND, MAX_THRUST_COMMAND)
        
        #rospy.loginfo('VZ-ControlAction:{}'.format(self.thrust_rate))
        return self.thrust_rate

    def AttitudeController(self,action):
        self.get_RPY()
        roll_ref, pitch_ref, yaw_ref = self.VelocityController(action)
        #yaw_ref =  self.map_action(action[3],-1,1,-np.radians(1)*100,np.radians(1)*100)
        #yaw_ref = action[3]
        #roll_ref = 0
        #pitch_ref = 0
        #yaw_ref = np.radians(1)*100

        feedback_roll = self.roll
        feedback_pitch = self.pitch
        feedback_yaw = self.r

        self.roll_pid.SetPoint = roll_ref
        self.pitch_pid.SetPoint = pitch_ref
        self.yaw_pid.SetPoint = yaw_ref

        error_roll = roll_ref - feedback_roll
        error_pitch = pitch_ref - feedback_pitch
        error_yaw = yaw_ref - feedback_yaw

        #rospy.loginfo('Feedback:{}'.format(feedback_roll))
        #rospy.loginfo('Error:{}'.format(error_roll))

        roll_cmd = self.roll_pid.update(error_roll, current_time = None)

        roll_cmd = np.clip(roll_cmd, MIN_ROLL_VELOCITY, MAX_ROLL_VELOCITY)

        #rospy.logwarn('Roll-ControlAction:{}'.format(roll_cmd))

        #rospy.loginfo('Feedback:{}'.format(feedback_pitch))
        #rospy.loginfo('Error:{}'.format(error_pitch))

        pitch_cmd = self.pitch_pid.update(error_pitch, current_time = None)

        pitch_cmd = np.clip(pitch_cmd, MIN_ROLL_VELOCITY, MAX_ROLL_VELOCITY)

        #rospy.logwarn('Pitch-ControlAction:{}'.format(pitch_cmd))

        #rospy.loginfo('Feedback:{}'.format(feedback_yaw))
        #rospy.loginfo('Error:{}'.format(error_yaw))

        yaw_cmd = self.yaw_pid.update(error_yaw, current_time = None)
        
        yaw_cmd = np.clip(yaw_cmd, MIN_YAW_VELOCITY, MAX_YAW_VELOCITY)

        #rospy.logwarn('Yaw-ControlAction:{}'.format(yaw_cmd))

        return roll_cmd, pitch_cmd, yaw_cmd
    
    def ControlMixer(self,action):
        self.roll_cmd, self.pitch_cmd, self.yaw_cmd  = self.AttitudeController(action)
        self.thrust_cmd = self.ZVelocityController(action)

        #rospy.loginfo('Commands:{}'.format(np.array([self.thrust_cmd, self.roll_cmd, self.pitch_cmd, self.yaw_cmd])))

        w = 1

        W_FR = self.thrust_cmd - w * self.pitch_cmd - w * self.roll_cmd - self.yaw_cmd #front-right
        W_BL = self.thrust_cmd + w * self.pitch_cmd + w * self.roll_cmd - self.yaw_cmd #back_left
        W_FL = self.thrust_cmd - w * self.pitch_cmd + w * self.roll_cmd + self.yaw_cmd #front-left
        W_BR = self.thrust_cmd + w * self.pitch_cmd - w * self.roll_cmd + self.yaw_cmd #back-right

        W_FR = np.clip( W_FR, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_BL = np.clip( W_BL, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_FL = np.clip( W_FL, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_BR = np.clip( W_BR, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )

        #rospy.logwarn('MMA-Calculated Velocity:{}'.format(np.array([W_FR, W_BL, W_FL, W_BR])))
        return W_FR, W_BL, W_FL, W_BR
    
    def pos_x_graph(self,a,b,c,d):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.plot(a,d)
        plt.xlim((0,LENGTH))
        plt.ylim((-200, 500))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID X')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint', 'action'])
        plt.show()
        
    def pos_y_graph(self,a,b,c,d):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.plot(a,d)
        plt.xlim((0,LENGTH))
        plt.ylim((-200,200))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID Y')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint', 'action'])
        plt.show()

    def pos_z_graph(self,a,b,c):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.xlim((0,LENGTH))
        plt.ylim((0, 130))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID Z')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint'])
        plt.show()

    def pos_roll_pitch_graph(self,a,b,c,d,e,f,g):
        #plt.plot(a,b)
        #plt.plot(a,c)
        plt.plot(a,d)
        #plt.plot(a,e)
        #plt.plot(a,f)
        plt.plot(a,g)
        plt.xlim((0,LENGTH))
        plt.ylim((-np.radians(15)*100, np.radians(15)*100))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID ROLL PITCH YAW')
        plt.grid(visible = True)
        plt.legend(['feedback_roll', 'feedback_pitch', 'feedback_yaw', 'setpoint_roll', 'setpoint_pitch','setpoint_yaw'])
        plt.show()
    
    def pos_vx_graph(self,a,b,c,d,e):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.plot(a,d)
        plt.plot(a,e)
        plt.xlim((0,LENGTH))
        plt.ylim((-20, 20))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID VX')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint', 'action', 'pitch_angle'])
        plt.show()

    def pos_vy_graph(self,a,b,c,d):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.plot(a,d)
        plt.xlim((0,LENGTH))
        plt.ylim((-20, 20))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID VY')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint', 'action'])
        plt.show() 

    def pos_vz_graph(self,a,b,c,d):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.plot(a,d)
        plt.xlim((0,LENGTH))
        plt.ylim((-10, 1000))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID VZ')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint', 'action'])
        plt.show()
    
    def pos_yaw_rate_graph(self,a,b,c):
        plt.plot(a,b)
        plt.plot(a,c)
        plt.xlim((0,LENGTH))
        plt.ylim((-15, 15))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID YAW_RATE')
        plt.grid(visible = True)
        plt.legend(['feedback', 'setpoint'])
        plt.show()
    
    def crea_file_csv(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rotors_gazebo')
        intestazioni = ['TOTAL REWARD','TILT', 'CRASH', 'GOAL','DISTANCE' 'N_STEPS', 'DONE', 'ERRORE']
        self.file_csv = pkg_path + '/Training/NewTrains/' + self.filecsv

        with open(self.file_csv, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(intestazioni)

    def aggiungi_riga(self, dati):
        with open(self.file_csv, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(dati)

    def plot(self):
            self.pos_x_graph(self.time_list, self.feedback_x_list ,self.setpoint_x_list,self.pitch_cmd_list )
            self.pos_y_graph(self.time_list, self.feedback_y_list ,self.setpoint_y_list, self.roll_cmd_list)
            self.pos_z_graph(self.time_list, self.feedback_z_list, self.setpoint_z_list)
            self.pos_vx_graph(self.time_list, self.feedback_vx_list ,self.setpoint_vx_list,self.pitch_cmd_list, self.feedback_pitch_list)
            self.pos_vy_graph(self.time_list, self.feedback_vy_list ,self.setpoint_vy_list, self.roll_cmd_list)
            self.pos_vz_graph(self.time_list, self.feedback_vz_list ,self.setpoint_vz_list, self.thrust_cmd_list)
            self.pos_roll_pitch_graph(self.time_list, self.feedback_roll_list, self.feedback_pitch_list, self.feedback_yaw_list, self.setpoint_roll_list, self.setpoint_pitch_list, self.setpoint_yaw_list)
            self.pos_yaw_rate_graph(self.time_list, self.feedback_yaw_rate_list ,self.setpoint_yaw_rate_list)

    def init_lists(self):
        self.feedback_x_list = np.array([])
        self.feedback_y_list = np.array([])
        self.feedback_z_list = np.array([])
        self.feedback_vx_list = np.array([])
        self.feedback_vy_list = np.array([])
        self.feedback_vz_list = np.array([])
        self.feedback_yaw_list = np.array([])
        self.feedback_yaw_rate_list = np.array([])
        self.feedback_roll_list = np.array([])
        self.feedback_pitch_list = np.array([])

        self.time_list = np.array([])
        self.setpoint_z_list = np.array([])
        self.setpoint_x_list = np.array([])
        self.setpoint_y_list = np.array([])
        self.setpoint_roll_list = np.array([])
        self.setpoint_vx_list = np.array([])
        self.setpoint_vy_list = np.array([])
        self.setpoint_vz_list = np.array([])
        self.setpoint_yaw_rate_list = np.array([])
        self.setpoint_pitch_list = np.array([])
        self.setpoint_yaw_list = np.array([])
        
        self.thrust_cmd_list = np.array([])
        self.roll_cmd_list = np.array([])
        self.pitch_cmd_list = np.array([])
        self.times = np.array([])
        self.yaw_cmd_list = np.array([])

    def append_to_lists(self):
        self.feedback_z_list = np.append(self.feedback_z_list, self.z)
        self.setpoint_z_list = np.append(self.setpoint_z_list, self.z_pid.SetPoint)
        self.feedback_x_list = np.append(self.feedback_x_list, self.x)
        self.setpoint_x_list = np.append(self.setpoint_x_list, self.x_pid.SetPoint)
        self.feedback_y_list = np.append(self.feedback_y_list, self.y)
        self.setpoint_y_list = np.append(self.setpoint_y_list, self.y_pid.SetPoint)
        self.feedback_roll_list = np.append(self.feedback_roll_list, self.roll)
        self.feedback_pitch_list = np.append(self.feedback_pitch_list, self.pitch)
        self.feedback_yaw_list = np.append(self.feedback_yaw_list, self.r)
        self.setpoint_roll_list = np.append(self.setpoint_roll_list, self.roll_pid.SetPoint)
        self.setpoint_pitch_list = np.append(self.setpoint_pitch_list, self.pitch_pid.SetPoint)
        self.setpoint_yaw_list = np.append(self.setpoint_yaw_list, self.yaw_pid.SetPoint)
        self.thrust_cmd_list = np.append(self.thrust_cmd_list, self.thrust_rate)
        self.roll_cmd_list = np.append(self.roll_cmd_list, self.roll_rate)
        self.pitch_cmd_list = np.append(self.pitch_cmd_list, self.pitch_rate)
        self.yaw_cmd_list = np.append(self.yaw_cmd_list, self.yaw_cmd)
        self.feedback_vx_list = np.append(self.feedback_vx_list, self.vx)
        self.feedback_vy_list = np.append(self.feedback_vy_list, self.vy)
        self.feedback_vz_list = np.append(self.feedback_vz_list, self.vz)
        self.feedback_yaw_rate_list = np.append(self.feedback_yaw_rate_list, self.yaw)
        self.setpoint_vx_list = np.append(self.setpoint_vx_list, self.vx_pid.SetPoint)
        self.setpoint_vy_list = np.append(self.setpoint_vy_list, self.vy_pid.SetPoint)
        self.setpoint_vz_list = np.append(self.setpoint_vz_list, self.vz_pid.SetPoint)
        self.setpoint_yaw_rate_list = np.append(self.setpoint_yaw_rate_list, self.yaw_rate_pid.SetPoint)

    def map_action(self,x, in_min, in_max, out_min, out_max):
	    return ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min)

if __name__ == '__main__':
    try:
        rospy.init_node('quadcopter_training', anonymous=True)
        reg = register(id='Quad-v0', entry_point='quad_env:QuadcopterEnv', max_episode_steps=LENGTH)
        start_train()
    except rospy.ROSInterruptException:
        pass
