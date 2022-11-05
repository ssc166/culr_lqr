#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
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
        
def get_link_state(link_name_main, reference_frame):
    
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)

    return link_state

def get_link_ori(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
     
    return link_ori_x, link_ori_y, link_ori_z

def get_link_vel(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
    
    return link_vel_x, link_vel_y, link_vel_z
    
def get_model_state(body_name):
    body_state = get_body(model_name = body_name)
    
    return body_state

def get_body_ori():
    global body_state
    
    body_state = get_model_state(body_name)
    
    X, Y, Z = quaternion_to_euler_angle(body_state.pose.orientation)
    body_ori_x = X
    body_ori_y = Y
    body_ori_z = Z
    
    return body_ori_x, body_ori_y, body_ori_z

def get_body_vel():
    global body_state
    
    body_state = get_model_state(body_name)
    
    body_vel_x = body_state.twist.angular.x
    body_vel_y = body_state.twist.angular.y
    body_vel_z = body_state.twist.angular.z
    
    return body_vel_x, body_vel_y, body_vel_z

def get_wheel_state(wheel_name):
    wheel_state = get_state(link_name = wheel_name)
    
    return wheel_state
    
def get_wheel_param():
    global wheel_state
    
    wheel_state = get_wheel_state(wheel_name)
    
    X, Y, Z = utils.quaternion_to_euler_angle(wheel_state.link_state.pose.orientation)
    wheel_ori_y = Y
    wheel_vel_y = wheel_state.link_state.twist.angular.y

    return  wheel_ori_y, wheel_vel_y


def linvel2wheelvel(linvel):
    wheel_rad = 0.138/2
    wheelvel = linvel/wheel_rad
    
    return wheelvel

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
    
#######################################################################################################
    
imu_acc_data = [0,0,0]
imu_vel_data = [0,0,0]
imu_ori_data = [0,0,0]

    
A, B, C, D = sep.Cal_Pitch_SS()

# q = [phi, theta, phi_dot, theta_dot]
Q = sp.Matrix([ [1,    0,    0,    0],
                [0,    1,    0,    0],
                [0,    0,    1,    0],
                [0,    0,    0,    1]])

R = sp.Matrix([ [1] ])

# Q = sp.Matrix([ [1,    0,    0,    0],
#                 [0,    2,    0,    0],
#                 [0,    0,    0.1,    0],
#                 [0,    0,    0,    1]])

# R = sp.Matrix([ [1] ])

K, S, E = pitch_K_gain()
K_part = K[0][1:4]
ss0 = [A, B, C, D]
sys0 = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0])
sysc = sys0.feedback(K)

RAD2DEG = 180/np.pi

get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
get_body = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

wheel_state = []
wheel_name = 'wheel_link'
wheel_name_list = [wheel_name]

mid_name = 'hip_link'
low_name = 'ankle_roll_yaw_link'
top_name = 'cmg'
link_name_list = [mid_name, low_name, top_name]

body_state = []
body_name = 'wheeled_inverted_pendulum'
body_name_list = [body_name]

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
        rate = rospy.Rate(100)
        gazebo_setting()

        
        cur_time = time.time()    
        sec_time = time.time()           
         
        while True:

            last_time = cur_time
            cur_time = time.time()
            sec_cur_time = time.time()
            dt = cur_time - last_time 
            sec =  sec_cur_time - sec_time
            
            wheel_ori_y, wheel_vel_y = get_wheel_param()
            body_ori_x, body_ori_y, body_ori_z = get_body_ori()
            body_vel_x, body_vel_y, body_vel_z = get_body_vel()
            # link_ori_x, link_ori_y, link_ori_z = get_link_ori(link_name_list[2], 'world')
            # link_vel_x, link_vel_y, link_vel_z = get_link_vel(link_name_list[2], 'world')

            x0 = np.array([body_ori_y,wheel_vel_y,body_vel_y])
            # wheel_vel = linvel2wheelvel(0)
            
            # xd = np.array([0,0,wheel_vel,0])
            # print(x0)
            u = -K_part @ ( x0 )
            pub_w.publish(u)

            deg_store.append(body_ori_y*RAD2DEG)
            sec_store.append(sec)
            
            # print('Wheel_velocity  (rad/s): ', wheel_vel_y)
            # print('Pitch             (deg): ', body_ori_y*RAD2DEG)
            # print('====================================') 
            
            if loop_cnt % 10 == 0:
                print('Wheel_velocity  (rad/s): ', wheel_vel_y)
                print('Pitch             (deg): ', body_ori_y*RAD2DEG)

                print('====================================')  
            
            
            loop_cnt= loop_cnt + 1

            # print('dt: ', dt)

            rate.sleep()
            
            # if loop_cnt == 1500:
            #     print_graph()
                
                

    except rospy.ROSInterruptException:
        pass
