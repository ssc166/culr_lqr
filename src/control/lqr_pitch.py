#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy, JointState, Imu
import pylab as pl
import control
from sympy.physics.mechanics import *
from numpy.linalg import matrix_rank, eig
import matplotlib.pyplot as plt
import state_equation_pitch as sep
import WIP_utils as utils




def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

def pitch_K_gain():
    
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)
    
    return X, Y, Z

def print_graph():
    
    plt.plot(sec_store, deg_store)

    plt.xlabel('sec[s]')
    plt.ylabel('tilt angle[deg]')
    plt.title('Pitch tilt angle')
    plt.ylim(-2.5,2.5)
    plt.xlim(0, 15)
    # plt.legend(loc='upper right')
    plt.grid(True, axis='y')

    plt.show()
    
def wheel_callback(msg):
    global wheel_vel
    # rate = rospy.Rate(100)
    wheel_vel = msg.velocity[3]

    
def imu_callback(msg):
    global pitch_ang
    global pitch_vel
    
    X, Y, Z = quaternion_to_euler_angle(msg.orientation)
    
    pitch_ang= Y
    # print(msg.angular_velocity.y)
    pitch_vel = msg.angular_velocity.y
    
    # print(msg)
    
def joy_callback(msg):
    global wheel_des
    
    V = msg.axes[1] * 1.5
    wheel_des = V / 0.069
    
    # print(msg.axes)
    
#######################################################################################################
    
imu_acc_data = [0,0,0]
imu_vel_data = [0,0,0]
imu_ori_data = [0,0,0]

    
A, B, C, D = sep.Cal_Pitch_SS()

# q = [phi, theta, phi_dot, theta_dot]
Q = sp.Matrix([ [1,    0,    0,    0],
                [0,    100,    0,    0],
                [0,    0,    10,    0],
                [0,    0,    0,    100]])

R = sp.Matrix([ [1] ])

K, S, E = pitch_K_gain()
K_part = K[0][1:4]


RAD2DEG = 180/np.pi


print('K: ', K_part)
loop_cnt = 0

deg_store = []
sec_store = []

############################################################################################
 
if __name__ == '__main__':
    try:                
        
        rospy.wait_for_service('gazebo/get_model_state')
        rospy.init_node('Pitch_Controller', anonymous=False) 
        
        pub_w = rospy.Publisher('/wheeled_inverted_pendulum/wheel/command', Float64, queue_size=100)
        rate = rospy.Rate(1000)
        gazebo_setting()
        
        sec = 0
        pitch_ang = 0
        pitch_vel = 0
        wheel_vel = 0
        wheel_des = 0

        rospy.Subscriber('joy', Joy, joy_callback)
        rospy.Subscriber("/wheeled_inverted_pendulum/joint_states", JointState, wheel_callback)
        rospy.Subscriber("/imu", Imu, imu_callback)
        
        cur_time = time.time()    
         
        while True:

            last_time = cur_time
            cur_time = time.time()
            dt = cur_time - last_time 
            sec =  dt + sec
 
            x0 = np.array([pitch_ang, wheel_vel, pitch_vel])
            # wheel_vel = linvel2wheelvel(0)
            
            xd = np.array([0,wheel_des,0])
            # print(x0)
            u = -K_part @ ( x0 -xd )
            pub_w.publish(u)

            deg_store.append(pitch_ang*RAD2DEG)
            sec_store.append(sec)
            
            # print('Wheel_velocity  (rad/s): ', wheel_vel_y)
            # print('Pitch             (deg): ', body_ori_y*RAD2DEG)
            # print('====================================') 
            
            # if loop_cnt % 10 == 0:
            #     print('Wheel_velocity  (rad/s): ', wheel_vel_y)
            #     print('Pitch             (deg): ', body_ori_y*RAD2DEG)

            #     print('====================================')  
            
            
            loop_cnt= loop_cnt + 1

            # print(x0)
            # print('dt: ', dt, 'freq: ', 1/dt)
            # print('====================================')  

            rate.sleep()
            
            # if loop_cnt == 1500:
            #     print_graph()
                
                

    except rospy.ROSInterruptException:
        pass
