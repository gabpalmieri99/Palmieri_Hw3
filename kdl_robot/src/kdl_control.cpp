#include "kdl_ros_control/kdl_control.h"
#include "std_msgs/Float64.h"
#include "kdl_ros_control/kdl_robot.h"
#include <cmath>
#include <Eigen/Dense>


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}


//JOINT SPACE CONTROLLER
Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp,
                                      double _Kd)
{   
    // read current state
    Eigen::VectorXd q;
    q.setZero();
    q = robot_->getJntValues();
    Eigen::VectorXd dq;
    dq.setZero();
    dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e,de;
    e.setZero();
    e = _qd.data - q;
    de.setZero();
    de = _dqd.data - dq;

    Eigen::VectorXd ddqd;
    ddqd.setZero();
    ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e) + robot_->getCoriolis() + robot_->getGravity();
} 

//CARTESIAN SPACE CONTROLLER
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)                                     
{
    
    // Declaration and initialization of the Kp and Kd matrices    
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero();

    // Block assignment for the Kp matrix
    Kp.block<3, 3>(0, 0) = _Kpp * Eigen::Matrix3d::Identity();  
    Kp.block<3, 3>(3, 3) = _Kpo * Eigen::Matrix3d::Identity();  

    // Block assignment for the Kd matrix
    Kd.block<3, 3>(0, 0) = _Kdp * Eigen::Matrix3d::Identity();  
    Kd.block<3, 3>(3, 3) = _Kdo * Eigen::Matrix3d::Identity();  


    // read current state
    KDL::Jacobian J_ee;
    J_ee.data.setZero();
    J_ee = robot_->getEEJacobian();

    //Initialization
    Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 7> M = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 6> Jpinv = Eigen::Matrix<double, 7, 6>::Zero();
    Eigen::Matrix<double,6,1> dJ_dq = Eigen::Matrix<double, 6, 1>::Zero();

    J = toEigen(J_ee);
    I = Eigen::Matrix<double,7,7>::Identity();
    M = robot_->getJsim();
    //Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
    Jpinv = pseudoinverse(J);
    dJ_dq=toEigen(robot_->getEEJacDotqDot());

    // Retrieve initial ee pose
    KDL::Frame Fi = robot_->getEEFrame();       
    Eigen::Vector3d pdi = toEigen(Fi.p);

    // position
    Eigen::Vector3d p_d(_desPos.p.data);    
    KDL::Vector effective_p = robot_->getEEFrame().p; 
    Eigen::Vector3d p_e = toEigen(effective_p);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    KDL::Rotation effective_r = robot_->getEEFrame().M;
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e = toEigen(effective_r);  
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);

    // velocity
    Eigen::Vector3d dot_p_d = Eigen::Vector3d(_desVel.vel.data);

    // obtaining end effector speed
    KDL::Twist eeVelocity = robot_->getEEVelocity();
    Eigen::Vector3d dot_p_e = toEigen(eeVelocity.vel);

    // Initialization
    Eigen::Vector3d omega_d = Eigen::Vector3d(_desVel.rot.data);

    // obtaining end effector angular velocity
    Eigen::Vector3d omega_e = toEigen(eeVelocity.rot);
   
    // acceleration
    Eigen::Matrix<double,6,1> dot_dot_x_d = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 3, 1> dot_dot_p_d = Eigen::Matrix<double, 3, 1>(_desAcc.vel.data);
    Eigen::Matrix<double, 3, 1> dot_dot_r_d = Eigen::Matrix<double, 3, 1>(_desAcc.rot.data);

    // compute linear errors
    Eigen::Matrix<double,3,1> e_p,dot_e_p;
    e_p.setZero();
    dot_e_p.setZero();
    e_p = computeLinearError(p_d,p_e);
    dot_e_p = computeLinearError(dot_p_d,dot_p_e);


    // compute orientation errors
    Eigen::Matrix<double,3,1> e_o = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double,3,1> dot_e_o = Eigen::Matrix<double, 3, 1>::Zero();
    e_o = computeOrientationError(R_d,R_e);
    dot_e_o = computeOrientationVelocityError(omega_d, omega_e,R_d, R_e);
    Eigen::Matrix<double, 3, 1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot_->getEEFrame().M));
                                                                                                                                          
    //error vector                                                            
    Eigen::Matrix<double,6,1> x_tilde = Eigen::Matrix<double, 6, 1>::Zero(); 
    Eigen::Matrix<double,6,1> dot_x_tilde= Eigen::Matrix<double, 6, 1>::Zero();
    x_tilde << e_p, e_o[0],e_o[1],e_o[2];
    dot_x_tilde << dot_e_p, dot_e_o;
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

    // null space control
    //double cost;
    //Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

    // Obtain the current speeds of the robot's joints
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // Obtain the calculated gravity forces for the current joint configuration
    Eigen::VectorXd gravity_forces = robot_->getGravity();

    // inverse dynamics
    Eigen::Matrix<double,6,1> y;
    y << dot_dot_x_d - dJ_dq + Kd*dot_x_tilde + Kp*x_tilde;

    Eigen::VectorXd tau;
    tau = M * (Jpinv*y + (I-Jpinv*J)*(/*- 10*grad*/ - robot_->getJntVelocities())) + gravity_forces + robot_->getCoriolis();
    return tau;
}

