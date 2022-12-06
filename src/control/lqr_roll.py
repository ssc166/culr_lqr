#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import Joy, JointState, Imu
from geometry_msgs.msg import *
import pylab as pl
import control
import matplotlib.pyplot as plt
import state_equation_roll as ser
import WIP_utils as utils


def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")
    
def roll_K_gain_R():
        
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

    plt.xlabel('sec[s]', fontsize=16)
    # plt.ylabel('tilt angle[deg]')
    # plt.title('Roll tilt angle')
    plt.ylim(-19, 19)
    plt.xlim(0, 15)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.legend([r'$\theta_r$[deg]'],loc='upper right', fontsize = 20)
    plt.grid()

    plt.show()
    
def gimbal_callback(msg):
    global gimbal_pos
    global gimbal_vel
    # rate = rospy.Rate(100)
    gimbal_pos = msg.position[2]
    gimbal_vel = msg.velocity[2]

    
def imu_callback(msg):
    global roll_ang
    global roll_vel
    
    X, Y, Z = quaternion_to_euler_angle(msg.orientation)
    
    roll_ang= X
    # print(msg.angular_velocity.y)
    roll_vel = msg.angular_velocity.x
    
    # print(msg)
    
def joy_callback(msg):
    global gimbal_des
    
    gimbal_des = -msg.axes[2] * 2

    print(msg.axes)
    
# def global_gimbal():
#     global gimbal_pos
#     global gimbal_vel
#     return gimbal_pos, gimbal_vel

#################################################################################################    
z_com = 0.6752763986721315
A, B, C, D = ser.Cal_Roll_SS(z_com)

# q = [theta_R, theta_gb, theta_Rd, theta_gbd]
Q = sp.Matrix([ [100,    0,    0,    0],
                [0,    10,    0,    0],
                [0,    0,    100,    0],
                [0,    0,    0,    1]])

R = sp.Matrix([ [1] ])

K, S, E = roll_K_gain_R()

RAD2DEG = 180/np.pi       
    
loop_cnt = 1

deg_store = []
sec_store = []

print('K: ', K)
#################################################################################################
    
if __name__ == '__main__':
    try:
        rospy.init_node('Roll_Controller', anonymous=False) 

        pub_Rgb = rospy.Publisher('/wheeled_inverted_pendulum/right_gimbal/command', Float64, queue_size=100)
        pub_Lfw = rospy.Publisher('/wheeled_inverted_pendulum/left_flywheel/command', Float64, queue_size=100)
        pub_Rfw = rospy.Publisher('/wheeled_inverted_pendulum/right_flywheel/command', Float64, queue_size=100)
        # rospy.Subscriber('CoM_Roll', Float64MultiArray, callback)

        rate = rospy.Rate(1000)
        gazebo_setting()

        gimbal_pos = 0
        gimbal_vel = 0
        roll_ang = 0
        roll_vel = 0
        gimbal_des = 0
        sec = 0

        rpm = 5000
        flywheel_ang_vel = (rpm * 2 * np.pi)/60        
        flywheel_vel = np.array([flywheel_ang_vel])
        print(flywheel_vel)

        rospy.Subscriber('joy', Joy, joy_callback)
        rospy.Subscriber("/wheeled_inverted_pendulum/joint_states", JointState, gimbal_callback)
        rospy.Subscriber("/imu", Imu, imu_callback)
        
        cur_time = time.time()  
        
        while not rospy.is_shutdown():    

            pub_Lfw.publish(flywheel_vel[0])
            pub_Rfw.publish(-flywheel_vel[0])
        
            last_time = cur_time
            cur_time = time.time()
            dt = cur_time - last_time 
            sec =  dt + sec

            
            x0 = np.array([roll_ang, gimbal_pos, roll_vel, gimbal_vel])          
            xd = np.array([0, gimbal_des, 0, 0])

            u_R = - K @ (x0 - xd) 
            
            pub_Rgb.publish(u_R)
                
            deg_store.append(roll_ang*RAD2DEG)
            sec_store.append(sec)

            # print('Gimbal_angle_l      (deg): ', gimbal_ori_left_z*RAD2DEG)
            # print('FLywheel_velocity (rad/s): ', abs(flywheel_vel_y))
            # print('Roll                (deg): ', roll_ang)
            # print(x0)
            # print(u_R)
            print('dt: ', dt, 'freq: ', 1/dt)
            # print('Yaw                 (deg): ', link_ori_z*RAD2DEG)
            print('====================================')

            loop_cnt= loop_cnt + 1
            # print(deg_store)
            rate.sleep()
            
            # if loop_cnt == 1500:
            #     print_graph()
            #     sys.stdout = open('deg_store.txt','w')
            #     print(deg_store)
                
            
    except rospy.ROSInterruptException:
        pass
