#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include <array>
#include <kdl/frames.hpp>
#include <cmath> 
#include <Eigen/Dense>
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"

//Set accordingly to desired trajectory. 
//-----------Careful!--------> Comment/decomment around line 195 with the right constructor  
//Furthermore, it's possible to set the velocity profile (trapezoidal or cubic polynomial), in kdl_planner.cpp
//CAREFUL Comment/uncomment suitable gains 
bool CIRCULAR_TRAJECTORY =false;
bool LINEAR_TRAJECTORY =true;

//Set accordingly to desired control. 
bool JOINT_SPACE_CONTROL=false;
bool CARTESIAN_SPACE_CONTROL=true;

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0), aruco_pose(7,0.0);
bool robot_state_available = false, aruco_pose_available = false;


// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}


void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}


// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);
    ros::Publisher error_pub = n.advertise<geometry_msgs::Vector3>("/iiwa/end_effector_error", 1);
    ros::Publisher esse_pub = n.advertise<geometry_msgs::Vector3>("/iiwa/s", 1);
    ros::Publisher norm_error_pub = n.advertise<std_msgs::Float64>("/iiwa/cartesian_error_norm", 1);
    ros::Publisher orient_error_pub = n.advertise<std_msgs::Float64>("/iiwa/orient_norm_error", 1);

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Set robot state
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg,error_norm_msg, orient_error_norm_msg;
    std_srvs::Empty pauseSrv;
    

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);

    // Specify an end-effector: camera in flange transform
    KDL::Frame ee_T_cam;        
    ee_T_cam.p = KDL::Vector(0, 0, 0.025); //0.025
    ee_T_cam.M = KDL::Rotation::RotY(1.57) * KDL::Rotation::RotZ(-1.57);
    robot.addEE(ee_T_cam);

    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);

    // EE trajectory end position
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 10, acc_duration = 1.5, t = 0.0, init_time_slot = 1.0,traj_radius=0.1;
    
    //KDLPlanner planner(traj_duration, acc_duration, init_position, traj_radius);        //constuctor for circular trajectory    
    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);     //constructor for linear trajectory
    
    // Retrieve the first trajectory point
    vel_profile vel_prof;

    //trajectory_point p = planner.compute_trajectory(t);
    trajectory_point p;
    if (CIRCULAR_TRAJECTORY){
    trajectory_point p = planner.compute_trajectory_circ(t, vel_prof);
    }
    else if (LINEAR_TRAJECTORY){
    trajectory_point p = planner.compute_trajectory_lin(t, vel_prof);                 
    }

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M;

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        if (robot_state_available)
        {   
            // Update robot
            robot.update(jnt_pos, jnt_vel);


            // Update time
            t = (ros::Time::now()-begin).toSec();
            //std::cout << "time: " << p.pos[0] << std::endl;
            
            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();

            //compute object frame 
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]),
                                    KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));      

            // look at point: compute rotation error from angle/axis
            Eigen::Matrix<double, 3, 1> aruco_pos_n = toEigen(cam_T_object.p);
            aruco_pos_n.normalize();
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0, 0, 1)) * aruco_pos_n;
            double aruco_angle = std::acos(Eigen::Vector3d(0, 0, 1).dot(aruco_pos_n));
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);
            
            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();

        
//CIRCULAR TRAJECTORY
if (CIRCULAR_TRAJECTORY){
            if (t <= init_time_slot) // wait a second
            { 
                //p = planner.compute_trajectory(0.0);
                p = planner.compute_trajectory_circ(0.0, vel_prof);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {   
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());

                //p = planner.compute_trajectory(t-init_time_slot);
                p = planner.compute_trajectory_circ(t-init_time_slot, vel_prof);
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }
}

//LINEAR TRAJECTORY
else if (LINEAR_TRAJECTORY){
            if (t <= init_time_slot) // wait a second
            { 
                //p = planner.compute_trajectory(0.0);
                
                p = planner.compute_trajectory_lin(0.0, vel_prof);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {   
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());

                //p = planner.compute_trajectory(t-init_time_slot);
                p = planner.compute_trajectory_lin(t-init_time_slot, vel_prof);
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }
}

            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);

            //JOINT SPACE INVERSE DYNAMICS CONTROL
            if (JOINT_SPACE_CONTROL){
            des_pose.M = robot.getEEFrame().M*Re;
            

            // inverse kinematics
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
            qd = robot.getInvKin(qd,des_pose*robot.getFlangeEE().Inverse()); 
          
            dqd.data << jnt_vel[0], jnt_vel[1], jnt_vel[2], jnt_vel[3], jnt_vel[4], jnt_vel[5], jnt_vel[6];
            dqd = robot.getDesVel(des_cart_vel, J_cam); 
            
            //GAIN  
            double Kp = 20, Kd =2*0.8*sqrt(Kp);   
            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);
            }
            // CARTESIAN SPACE INVERSE DYNAMICS CONTROL
            else if (CARTESIAN_SPACE_CONTROL){ 

            des_pose.M =robot.getEEFrame().M*Re;

            //Comment/uncomment suitable gains
            //circular_trapezoidal
            //double Kp = 50 , Ko = 50;
            //tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko,2*0.8*sqrt(Kp), 2*0.8*sqrt(Ko));

            //circular_polinomial
            //double Kp = 50 , Ko = 50;
            //tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko,2*sqrt(Kp), 2*sqrt(Ko));

            //linear_
            double Kp = 30 , Ko = 30;
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko,2*sqrt(Kp), 2*sqrt(Ko));

            }
            Eigen::Vector3d orientation_error = computeOrientationError(toEigen(des_pose.M),toEigen(robot.getEEFrame().M));
            orient_error_norm_msg.data = orientation_error.norm();

            KDL::Frame current_pose = robot.getEEFrame();

            //Compute Cartesian error components
            Eigen::Vector3d error_vector = Eigen::Vector3d::Zero();
            error_vector[0] = des_pose.p.x() - current_pose.p.x();
            error_vector[1] = des_pose.p.y() - current_pose.p.y();
            error_vector[2] = des_pose.p.z() - current_pose.p.z(); 
            
            //Compute Cartesian error norm
            Eigen::Vector3d norm_error_vector=toEigen(des_pose.p)-toEigen(current_pose.p);
                  
            error_norm_msg.data = 100*norm_error_vector.norm();

            // Set torques
            //error_msg.data = errr[0];
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);
            norm_error_pub.publish(error_norm_msg);
            orient_error_pub.publish(orient_error_norm_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}
