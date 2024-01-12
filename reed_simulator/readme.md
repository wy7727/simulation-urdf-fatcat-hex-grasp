# reed_simulator
本项目是基于Rotors二次开发的ROS包，用于无人机开发学习和飞行控制开发学习  
参考: https://github.com/ethz-asl/rotors_simulator.git  

## 安装Rotors
参考CSDN: https://blog.csdn.net/qq_37680545/article/details/123185002?spm=1001.2014.3001.5502  
参考: https://github.com/ethz-asl/rotors_simulator/wiki/Setting-up-the-RotorS-Simulator   

## 编译
`cd ~\catkin_ws`  
`catkin build`  

## 运行
1. PID控制器:  
打开终端:   
`roslaunch reed_gazebo mav_hovering_example.launch`  
打开另一个终端:  
`rostopic pub -1 /reed_quad_x/command/pose geometry_msgs/PoseStamped "header: 
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 1.0
    y: 1.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
`
2. 四元数控制器:  
打开终端:   
`roslaunch reed_gazebo mav_hovering_example_Q.launch`  


## 交流 
CSDN地址: https://blog.csdn.net/qq_37680545?type=blog   
Gitee: https://gitee.com/liaoluweillw/reed_simulator.git   
关注公众号: `Reed UAV`  
![输入图片说明](/picture/wechat.jpg)

## 交流