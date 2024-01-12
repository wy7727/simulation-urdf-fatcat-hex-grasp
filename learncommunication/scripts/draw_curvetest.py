#! /home/wuy/anaconda3/envs/pythree/bin/python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import transforms3d as tfs
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry

plt.rcParams['font.sans-serif'] = ['SimHei']    #指定字体为SimHei
plt.rcParams['axes.unicode_minus'] = False      #让-号显示正常

def trajectory_callback(msg):
    # trajectory回调函数
    global begin_time,desire_x,desire_y,desire_z
    if not begin_time:
        begin_time = msg.header.stamp.to_sec()
    desire_x = msg.points[0].transforms[0].translation.x
    desire_y = msg.points[0].transforms[0].translation.y
    desire_z = msg.points[0].transforms[0].translation.z

def odometry_callback(msg):
    # odometry回调函数
    global begin_time,time_list,x,y,z,desire_x,desire_y,desire_z, x_theta, y_theta, z_theta, theta
    if  begin_time:
        axis_theta=tfs.quaternions.quat2axangle([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])
        time_list.append(msg.header.stamp.to_sec() - begin_time)
        x_theta.append(axis_theta[0][0])
        y_theta.append(axis_theta[0][1])
        z_theta.append(axis_theta[0][2])
        theta.append(axis_theta[1])
        x['actual'].append(msg.pose.pose.position.x)
        y['actual'].append(msg.pose.pose.position.y)
        z['actual'].append(msg.pose.pose.position.z)
        x['desire'].append(desire_x)
        y['desire'].append(desire_y)
        z['desire'].append(desire_z)

def draw_curve():
    # 绘制曲线
    global time_list,x,y,z,trajectory_sub,odometry_sub,fig, axes, x_theta, y_theta, z_theta, theta
    print('draw curve...')
    # 取消订阅话题
    trajectory_sub.unregister()
    odometry_sub.unregister()
    # 绘制曲线
    fig, axes = plt.subplots(4, 1, figsize=(10, 15))
    axes[0].plot(time_list,x['desire'],'r',time_list,x['actual'],'b')
    axes[0].grid(True)
    axes[0].legend(['期望曲线','实际曲线'])
    axes[0].set_title('X')

    axes[1].plot(time_list, y['desire'], 'r', time_list, y['actual'], 'b')
    axes[1].grid(True)
    axes[1].legend(['期望曲线', '实际曲线'])
    axes[1].set_title('Y')

    axes[2].plot(time_list, z['desire'], 'r', time_list, z['actual'], 'b')
    axes[2].grid(True)
    axes[2].legend(['期望曲线', '实际曲线'])
    axes[2].set_title('Z')

    axes[3].plot(time_list, x_theta, 'r', time_list, y_theta, 'b',time_list,z_theta,'g')
    axes[3].grid(True)
    axes[3].legend(['x', 'y','z'])
    axes[3].set_title('Z')

    plt.show()



begin_time = None
time_list = []
draw_duration = 10 # 绘制时长，单位s
desire_x = 0
desire_y = 0
desire_z = 0
x = {'desire':[],'actual':[]}
y = {'desire':[],'actual':[]}
z = {'desire':[],'actual':[]}
x_theta=[]
y_theta=[]
z_theta=[]
theta=[]
trajectory_topic = "/reed_quad_x/command/trajectory"    # 轨迹topic名称
odometry_topic = "/reed_quad_x/odometry_sensor1/odometry"   # odometry topic 名称
rospy.init_node("draw_curve_node")
trajectory_sub = rospy.Subscriber(trajectory_topic,
                                  MultiDOFJointTrajectory,
                                  trajectory_callback,queue_size=10)
odometry_sub = rospy.Subscriber(odometry_topic,
                                  Odometry,
                                  odometry_callback, queue_size=10)
print('等待服务端发送轨迹指令...')
while(not begin_time):  # 等待接受到数据
    pass
print('正在接受数据...')
time.sleep(draw_duration)
draw_curve()

