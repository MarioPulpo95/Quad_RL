#!/usr/bin/python3
from PID import PID
import numpy as np
import rospy

SAMPLING_TIME = 0.01                    

MIN_X = -400
MAX_X = 400
MIN_Y = -400
MAX_Y = 400
MIN_Z = 0
MAX_Z = 600

KP_X = 0.006 
KI_X = 0.0002
KD_X = 0.000

KP_Y = KP_X
KI_Y = KI_X
KD_Y = KD_X

KP_Z = 0.021
KI_Z = 0.009
KD_Z = 0.0009

KP_ROLL = 2.5
KI_ROLL = 0.3                                 
KD_ROLL = 1.2              
KP_PITCH = KP_ROLL
KI_PITCH = KI_ROLL
KD_PITCH = KD_ROLL

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

class PositionController():
    def __init__(self):

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

        self.init_PIDs()


    def init_PIDs(self):
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

    def XYController(self, x, y, x_ref, y_ref, yaw):

        feedback_x = x
        feedback_y = y
    
        self.x_pid.SetPoint = x_ref 
        self.y_pid.SetPoint = y_ref 

        x_ref_proj = x_ref*np.cos(yaw) + y_ref*np.sin(yaw)
        y_ref_proj = -x_ref*np.sin(yaw) + y_ref*np.cos(yaw)

        feedback_x_proj = feedback_x*np.cos(yaw) + feedback_y*np.sin(yaw)
        feedback_y_proj = -feedback_x*np.sin(yaw) + feedback_y*np.cos(yaw)

        error_x_proj = x_ref_proj - feedback_x_proj
        error_y_proj = y_ref_proj - feedback_y_proj

        #rospy.loginfo('Feedback_proj:{}'.format(feedback_y_proj))
        #rospy.loginfo('Error:{}'.format(error_y_proj))
        
        vy_rate = self.y_pid.update(error_y_proj, current_time = None)
        vy_rate = np.clip(vy_rate, -MAX_VELOCITY, MAX_VELOCITY)
 
        #rospy.loginfo('Y-ControlAction:{}'.format(self.vy_rate))

        #rospy.loginfo('Feedback_proj:{}'.format(feedback_x_proj))
        #rospy.loginfo('Error:{}'.format(error_x_proj))

        vx_rate = self.x_pid.update(error_x_proj, current_time = None)
        vx_rate = np.clip(vx_rate, -MAX_VELOCITY, MAX_VELOCITY)

        #rospy.loginfo('X-ControlAction:{}'.format(self.vx_rate))

        return vx_rate, vy_rate
    
    #def VelocityController(self, action, x, y, x_ref, y_ref, vx, vy, yaw): # if PID
    def VelocityController(self, action, vx, vy, yaw, rand_yaw): # if RL

        #vx_ref, vy_ref = self.XYController(action, x, y, x_ref, y_ref, yaw)

        vx_ref = action[0]
        vy_ref = action[1]
        #yaw_ref = 0.0
        yaw_ref = rand_yaw*100 # if start with rand_yaw

        self.vx_pid.SetPoint = vx_ref
        self.vy_pid.SetPoint = vy_ref
        self.yaw_rate_pid.SetPoint = yaw_ref

        feedback_vx = vx
        feedback_vy = vy
        feedback_yaw = yaw

        error_vx = self.vx_pid.SetPoint - feedback_vx
        error_vy = self.vy_pid.SetPoint - feedback_vy
        error_yaw = self.yaw_rate_pid.SetPoint - feedback_yaw

        #rospy.loginfo('Feedback:{}'.format(feedback_vy))
        #rospy.loginfo('Error:{}'.format(error_vy))

        roll_rate = self.vy_pid.update(error_vy, current_time = None)
        roll_rate = np.clip(roll_rate, -MAX_ROLL_COMMAND, MAX_ROLL_COMMAND)

        #rospy.loginfo('VY-ControlAction:{}'.format(roll_rate))

        #rospy.loginfo('Feedback:{}'.format(feedback_vx))
        #rospy.loginfo('Error:{}'.format(error_vx))

        pitch_rate = self.vx_pid.update(error_vx, current_time = None)
        pitch_rate = np.clip(pitch_rate,-MAX_PITCH_COMMAND, MAX_PITCH_COMMAND)

        #rospy.loginfo('VX-ControlAction:{}'.format(pitch_rate))
        yaw_rate = self.yaw_rate_pid.update(error_yaw, current_time = None)
        yaw_rate = np.clip(yaw_rate,-MAX_YAW_COMMAND, MAX_YAW_COMMAND)

        pitch_rate = pitch_rate*np.cos(rand_yaw) + roll_rate*np.sin(rand_yaw)  # if start with random yaw
        roll_rate = -pitch_rate*np.sin(rand_yaw) + roll_rate*np.cos(rand_yaw)

        return -roll_rate, pitch_rate, yaw_rate

    def AltitudeController(self, z_ref, z):

        self.z_pid.SetPoint = z_ref
        feedback_z = z
        error = self.z_pid.SetPoint - feedback_z
        
        #rospy.loginfo('Feedback:{}'.format(feedback_z))
        #rospy.loginfo('Error:{}'.format(error))

        vz_ref = self.z_pid.update(error, current_time = None)
        vz_ref = np.clip(vz_ref, MIN_Z_VELOCITY, MAX_Z_VELOCITY)
        
        #rospy.loginfo('Z-ControlAction:{}'.format(vz_ref))

        return vz_ref
    
    #def ZVelocityController(self,action,z_ref, z, vz): # se PID
    def ZVelocityController(self,action,vz): # se RL

        #self.vz_pid.SetPoint = self.AltitudeController(z_ref, z)

        self.vz_pid.SetPoint = action[2]
        #self.vz_pid.SetPoint = 1

        feedback_vz = vz
        error_vz = self.vz_pid.SetPoint - feedback_vz 

        #rospy.loginfo('Feedback:{}'.format(feedback_vz))
        #rospy.loginfo('Error:{}'.format(error_vz))

        thrust = self.vz_pid.update(error_vz, current_time = None)

        thrust = np.clip(thrust, MIN_THRUST_COMMAND, MAX_THRUST_COMMAND)
        
        #rospy.loginfo('VZ-ControlAction:{}'.format(thrust))
        return thrust

    def AttitudeController(self,roll_ref, pitch_ref, yaw_ref, roll, pitch, r, ):
        #roll_ref, pitch_ref, yaw_ref = self.VelocityController(action, vx, vy, yaw)
        #yaw_ref = action[3]
        #roll_ref = 0
        #pitch_ref = 0
        #yaw_ref = np.radians(1)*100

        feedback_roll = roll
        feedback_pitch = pitch
        feedback_yaw = r

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
    
    def ControlMixer(self,roll_cmd, pitch_cmd, thrust_cmd, yaw_cmd):
        #roll_cmd, pitch_cmd, yaw_cmd  = self.AttitudeController(action,vx, vy, roll, pitch, yaw, r)
        #thrust_cmd = self.ZVelocityController(action, vz)

        #rospy.loginfo('Commands:{}'.format(np.array([thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd])))

        w = 1

        W_FR = thrust_cmd - w * pitch_cmd - w * roll_cmd - yaw_cmd 
        W_BL = thrust_cmd + w * pitch_cmd + w * roll_cmd - yaw_cmd 
        W_FL = thrust_cmd - w * pitch_cmd + w * roll_cmd + yaw_cmd 
        W_BR = thrust_cmd + w * pitch_cmd - w * roll_cmd + yaw_cmd 

        W_FR = np.clip( W_FR, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_BL = np.clip( W_BL, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_FL = np.clip( W_FL, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        W_BR = np.clip( W_BR, MIN_PROPELLER_VELOCITY, MAX_PROPELLER_VELOCITY )
        #rospy.logwarn('MMA-Calculated Velocity:{}'.format(np.array([W_FR, W_BL, W_FL, W_BR])))
        return W_FR, W_BL, W_FL, W_BR
    