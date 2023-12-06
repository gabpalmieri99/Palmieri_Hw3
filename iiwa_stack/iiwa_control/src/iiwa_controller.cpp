#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

void topicCallback(const sensor_msgs::JointState& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("/iiwa/joint_states", 1, topicCallback);

    ros::Publisher pub_j1 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J1_controller/command", 1000);
    ros::Publisher pub_j2 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J2_controller/command", 1000);
    ros::Publisher pub_j3 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J3_controller/command", 1000);
    ros::Publisher pub_j4 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J4_controller/command", 1000);
    ros::Publisher pub_j5 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J5_controller/command", 1000);
    ros::Publisher pub_j6 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J6_controller/command", 1000);
    ros::Publisher pub_j7 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J7_controller/command", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    double value_j1 = 0.0;
    double value_j2 = 1.57;
    double value_j3 = -1.57;
    double value_j4 = -1.57;
    double value_j5 = 1.57;
    double value_j6 = -1.57;
    double value_j7 = 1.57;



    while (ros::ok()) {
        std_msgs::Float64 msg;
        
        msg.data=value_j1;
        pub_j1.publish(msg);
        
        msg.data=value_j2;
        pub_j2.publish(msg);
        
        msg.data=value_j3;
        pub_j3.publish(msg);
        
        msg.data=value_j4;
        pub_j4.publish(msg);

        msg.data=value_j5;
        pub_j5.publish(msg);
        
        msg.data=value_j6;
        pub_j6.publish(msg);
        
        msg.data=value_j7;
        pub_j7.publish(msg);
        

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

void topicCallback(const sensor_msgs::JointState& msg) {
    for (int i=0; i<4; i++) {
    ROS_INFO_STREAM("Position of joint " << i + 1 << ": " << msg.position[i]);
    }
}
