#! /home/wuy/anaconda3/envs/pythree/bin/python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import transforms3d as tfs
import mpctools as mpc
import mpctools.plots as mpcplots
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
trajectory_pub = rospy.Publisher("/planner/trajectory",MultiDOFJointTrajectory,queue_size=1)
rate=rospy.Rate(2)
desire_tra=MultiDOFJointTrajectory()
velocities = Twist()
accelerations = Twist()
quaternion =tfs.euler.euler2quat(0,0,0,"sxyz")

while not rospy.is_shutdown():
            quaternion=tfs.euler.euler2quat(0,0,0,"sxyz")
            transforms = Transform(translation=Point(0,0,0), rotation=Quaternion(quaternion[1],quaternion[2],quaternion[3],quaternion[0]))
           # print("pi=%a\n", transforms)
            velocities.linear.x=0
            velocities.linear.y=0
            velocities.linear.z=0
            velocities.angular.x=0
            velocities.angular.y=0
            velocities.angular.z=0
            accelerations.linear.x=0
            accelerations.linear.y=0
            accelerations.linear.z=0
            accelerations.angular.x=0
            accelerations.angular.y=0
            accelerations.angular.z=0
            p = MultiDOFJointTrajectoryPoint(transforms, velocities, accelerations,rospy.Time())
            desire_tra.points=p
            
            print('发送期望轨迹数据...')
            trajectory_pub.publish(desire_tra)
            rate.sleep()







print('等待服务端发送轨迹指令...')
while(not begin_time):  # 等待接受到数据
    pass
print('正在接受数据...')
time.sleep(draw_duration)
draw_curve()

