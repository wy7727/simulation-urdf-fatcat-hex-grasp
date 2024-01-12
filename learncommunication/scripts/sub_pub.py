#! /home/wuy/anaconda3/envs/pythree/bin/python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import transforms3d as tfs
import mpctools as mpc
import mpctools.plots as mpcplots
from learncommunication.msg import Trajectory
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
plt.rcParams['font.sans-serif'] = ['SimHei']    #指定字体为SimHei
plt.rcParams['axes.unicode_minus'] = False      #让-号显示正常
Nt = 40
Nx = 12
Nu = 6
Delta = 0.1

def sin(x):
    return np.sin(x)

def cos(x):
    return np.cos(x)


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
    global begin_time,posx,posy,posz,velx,vely,velz,accx,accy,accz,pos_yaw,vel_yaw,acc_yaw,x1,y,z,vx,vy,vz,yaw,dyaw,dyaw
    if  True:
        x1=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.position.z
        vx=msg.twist.twist.linear.x
        vy=msg.twist.twist.linear.y
        vz=msg.twist.twist.linear.z
        euler_theta=tfs.euler.quat2euler([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z],"sxyz")
        yaw=euler_theta[2]
        dyaw=msg.twist.twist.angular.z
       # 单杆l=0.3 theta=0
        l=0
        theta=0
        dl=0
        dtheta=0
        xl=x1
        yl=y
        zl=z
        vxl=vx
        vyl=vy
        vzl=vz
        q=np.array([vxl,vyl,vzl,dl,dyaw,dtheta,xl,yl,zl,l,yaw,theta])
        u_pub=np.zeros(6)
        dt=msg.header.stamp.to_sec() - begin_time
        accx=u_pub[0]
        accy=u_pub[1]
        accz=u_pub[2]
        acc_yaw=u_pub[4]*dt
        velx=u_pub[0]*dt
        vely=u_pub[1]*dt
        velz=u_pub[2]*dt
        vel_yaw=u_pub[4]*dt
        posx=velx*dt/2
        posy=vely*dt/2
        posz=velz*dt/2
        pos_yaw=vel_yaw*dt/2
        begin_time=msg.header.stamp.to_sec()



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
posx=0
posy=0
posz=0
velx=0
vely=0
velz=0
accx=0
accy=0
accz=0
pos_yaw=0
vel_yaw=0
acc_yaw=0
x=0
y=0
z=0
vx=0
vy=0
vz=0

trajectory_topic = "/reed_quad_x/command/trajectory"    # 轨迹topic名称
odometry_topic = "/reed_quad_x/odometry_sensor1/odometry"   # odometry topic 名称
rospy.init_node("draw_curve_node")
trajectory_sub = rospy.Subscriber(trajectory_topic,
                                  MultiDOFJointTrajectory,
                                  trajectory_callback,queue_size=10)
odometry_sub = rospy.Subscriber(odometry_topic,
                                  Odometry,
                                  odometry_callback, queue_size=10)
trajectory_pub = rospy.Publisher("/planner/trajectory",Trajectory,queue_size=1)
rate=rospy.Rate(2)
desire_tra=Trajectory()
velocities = Twist()
accelerations = Twist()
quaternion =tfs.euler.euler2quat(0,0,0,"sxyz")

while not rospy.is_shutdown():
            quaternion=tfs.euler.euler2quat(0,0,0,"sxyz")
            transforms = Transform(translation=Point(0,0,0), rotation=Quaternion(quaternion[1],quaternion[2],quaternion[3],quaternion[0]))
           # print("pi=%a\n", transforms)
            desire_tra.Pos_x=0
            desire_tra.Pos_y=0
            desire_tra.Pos_z=0
            desire_tra.Vel_x=0
            desire_tra.Vel_y=0
            desire_tra.Vel_z=0
            desire_tra.Acc_x=0
            desire_tra.Acc_y=0
            desire_tra.Acc_z=0
            print('发送期望轨迹数据...')
            trajectory_pub.publish(desire_tra)
            rate.sleep()


