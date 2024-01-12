
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <gazebo_msgs/ApplyJointEffort.h>
int main(int argc, char **argv)
{
ros::init(argc, argv, "my_gazebo_driver");
ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

gazebo_msgs::ApplyJointEffort joint_effort;
joint_effort.request.joint_name = "link0_link1x_joint";
joint_effort.request.effort = 2.0;
joint_effort.request.start_time.fromSec(0);
joint_effort.request.duration.fromSec(100);
client.call(joint_effort);
}