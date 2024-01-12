#! /home/wuy/anaconda3/envs/pythree/bin/python

import rospy
import time
import math
import numpy as np
from trajectory_msgs.msg import *
from control_msgs.msg import *
import actionlib
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import transforms3d as tfs
import mpctools as mpc
import mpctools.plots as mpcplots
from geometry_msgs.msg import Transform, Quaternion, Point, Twist,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav_msgs.msg import Odometry
from transforms3d import quaternions
plt.rcParams['font.sans-serif'] = ['SimHei']    #指定字体为SimHei
plt.rcParams['axes.unicode_minus'] = False      #让-号显示正常

###发送夹抓和机械臂控制指令


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
stop_flag=0
z=0
vx=0
vy=0
vz=0
def odometry_callback(msg):
    # odometry回调函数
    global begin_time,posx,posy,posz,velx,vely,velz,accx,accy,accz,pos_yaw,vel_yaw,acc_yaw,x1,y,z,vx,vy,vz,stop_flag
    if  True:
        x1=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.position.z
        vx=msg.twist.twist.linear.x
        vy=msg.twist.twist.linear.y
        vz=msg.twist.twist.linear.z
        rota=quaternions.quat2mat([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])
        error=np.dot(rota[:,0],np.array([0,0,1]))
        if ((error)>0.93):
            stop_flag=1
        # else:
        #       stop_flag=0
        # print(error)
        # print(stop_flag)






rospy.init_node("pubtra_node")
client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
print("Waiting for server...")
    #等待server
client.wait_for_server()
print("Connect to server")
# begin_time= rospy.get_time() 

letrajectory_pub = rospy.Publisher("/arm_controller/command",JointTrajectory,queue_size=10)
odometry_sub = rospy.Subscriber("/base_pose_ground_truth",
                                  Odometry,
                                  odometry_callback, queue_size=10)
rate=rospy.Rate(20)
velocitiesq = Twist()
accelerationsq = Twist()
quaternion =tfs.euler.euler2quat(0,0,0,"sxyz")
gripper_cmd=JointTrajectory()  
gripper_point = JointTrajectoryPoint()
gripper_cmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
gripper_point.time_from_start = rospy.Duration.from_sec(5.0)


gripper_cmd.joint_names.append('left_knuckle_joint')
gripper_point.positions.append(0.00)
gripper_point.velocities.append(0)
gripper_point.accelerations.append(0)
gripper_point.effort.append(0)
gripper_cmd.points.append(gripper_point)

le_arm_name=['link0_link1x_joint', 'link1x_link1y_joint', 'link1y_link2x_joint',
               'link2x_link2y_joint', 'link2y_link3x_joint', 'link3x_link3y_joint',
              'link3y_link4x_joint', 'link4x_link4y_joint','link4y_link5x_joint',
              'link5x_link5y_joint']  
arm_cmd=JointTrajectory()  
arm_point = JointTrajectoryPoint()
arm_cmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
arm_point.time_from_start = rospy.Duration.from_sec(5.0)
for i in range(0, 10):
    arm_cmd.joint_names.append(le_arm_name[i])
    arm_point.positions.append(0.0*((i+1)%2)+0.5*((i)%2))
    arm_point.velocities.append(0.5*((i+1)%2)-0.0*((i)%2))
    arm_point.accelerations.append(-0.0*((i+1)%2)-0.5*((i)%2))
    arm_point.effort.append(0) 
arm_cmd.points.append(arm_point)
print(arm_point)
letrajectory_pub.publish(arm_cmd)



while not rospy.is_shutdown():
            n=160
            for i in range(n):
                   angle=i*2*math.pi/n
                   arm_cmd=JointTrajectory()  
                   arm_point = JointTrajectoryPoint()
                   arm_cmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
                   arm_point.time_from_start = rospy.Duration.from_sec(5.0)
                   if (stop_flag==0):
                         for i in range(0, 10):
                            arm_cmd.joint_names.append(le_arm_name[i])
                            arm_point.positions.append((0.50*math.sin(angle))*(((i+1)%2))+(0.50*math.cos(angle))*((i)%2))
                            arm_point.velocities.append((0.50*math.cos(angle))*((i+1)%2)-(0.50*math.sin(angle))*((i)%2))
                            arm_point.accelerations.append((-0.50*math.sin(angle))*((i+1)%2)-(0.50*math.cos(angle))*((i)%2))
                            arm_point.effort.append(0)
                         arm_cmd.points.append(arm_point)
                         letrajectory_pub.publish(arm_cmd)
                         print(stop_flag)
                         print('11111111111111111111111111111111111')
                   else:
                         for i in range(0, 10):
                            arm_cmd.joint_names.append(le_arm_name[i])
                            arm_point.positions.append((0.0*math.sin(angle))*(((i+1)%2))+(0.0*math.cos(angle))*((i)%2))
                            arm_point.velocities.append((0.0*math.cos(angle))*((i+1)%2)-(0.0*math.sin(angle))*((i)%2))
                            arm_point.accelerations.append((-0.0*math.sin(angle))*((i+1)%2)-(0.0*math.cos(angle))*((i)%2))
                            arm_point.effort.append(0)
                         arm_cmd.points.append(arm_point)
                         letrajectory_pub.publish(arm_cmd)
                         print(stop_flag)
                         print('22222222222222222222222222222222222222222')
                   print(arm_point)
                   time.sleep(0.05)
            # print('arm_point')
            rate.sleep()


