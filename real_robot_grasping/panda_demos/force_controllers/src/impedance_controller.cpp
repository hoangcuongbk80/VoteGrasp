#include "impedance_controller.h"

#include <iostream>
#include <cmath>
#include <memory>

#include "utilities.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_kdl.h>

namespace force_controllers {

  void ImpedanceController::initialize(RobotStatePtr robot_state) {
    trajectory_subscriber_ = getControllerNodeHandle().subscribe(
      "equilibrium_pose", 1000, &ImpedanceController::trajectorySubscriberCallback, this);

    marker_array_pub_ = getControllerNodeHandle().advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 1);

    //u_ = Eigen::VectorXd::Zero(3);
    ee_vel_ = Eigen::VectorXd::Zero(6);

    fk_solver_pos_ = std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
    fk_solver_jac_ = std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);
    //fk_solver_jac_dot_ = std::make_shared<KDL::ChainJntToJacDotSolver>(kdl_chain_);

    ee_jacobian_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    ee_dot_jacobian_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    
    //set initial pose to current pose
    const KDL::JntArray& q = robot_state->kdl_jnt_array_vel_.value();
    fk_solver_pos_->JntToCart(q, ee_pose_, getTipName());
    tf::quaternionKDLToEigen(ee_pose_.M,orientation_d_target_);
    tf::vectorKDLToEigen(ee_pose_.p,position_d_target_);
    tf::quaternionKDLToEigen(ee_pose_.M,orientation_);
    tf::vectorKDLToEigen(ee_pose_.p,position_);

    q_d_nullspace_ = q.data;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    cartesian_stiffness_(0,0) = 10000;
    cartesian_stiffness_(1,1) = 10000;
    cartesian_stiffness_(2,2) = 10000;
    cartesian_stiffness_(3,3) = 5000;
    cartesian_stiffness_(4,4) = 5000;
    cartesian_stiffness_(5,5) = 5000;
    
    cartesian_damping_(0,0) = 500;
    cartesian_damping_(1,1) = 500;
    cartesian_damping_(2,2) = 500;
    cartesian_damping_(3,3) = 100;
    cartesian_damping_(4,4) = 100;
    cartesian_damping_(5,5) = 100;
  }

  void ImpedanceController::setJointAccelerations(RobotStatePtr robot_state, Eigen::VectorXd& ddq) 
  {
    const KDL::JntArray& q = robot_state->kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = robot_state->kdl_jnt_array_vel_.deriv();

    fk_solver_pos_->JntToCart(q, ee_pose_, getTipName());
    fk_solver_jac_->JntToJac(q, ee_jacobian_, getTipName());
    //fk_solver_jac_->JntToJacDot(robot_state->kdl_jnt_array_vel_, ee_dot_jacobian_);
    JntToJacDot(robot_state->kdl_jnt_array_vel_, ee_dot_jacobian_);

    Eigen::MatrixXd J = ee_jacobian_.data.block(0,0,6,getNJoints());
    Eigen::MatrixXd dJ = ee_dot_jacobian_.data.block(0,0,6,getNJoints());
    Eigen::MatrixXd Jinv = pinv(J);
    Eigen::MatrixXd dq_eig = dq.data;
    Eigen::MatrixXd q_eig = q.data;

    //convert KDL frame to eigen
    tf::quaternionKDLToEigen(ee_pose_.M,orientation_);
    tf::vectorKDLToEigen(ee_pose_.p,position_);

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position_ - position_d_target_;

    // orientation error
    if (orientation_d_target_.coeffs().dot(orientation_.coeffs()) < 0.0) {
        orientation_.coeffs() << -orientation_.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation_ * orientation_d_target_.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    //ddq = Jinv * (-error - dJ*dq.data);
    
    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(J.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    tau_task << J.transpose() *
        (-cartesian_stiffness_ * error - cartesian_damping_ * (J * dq_eig));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
            J.transpose() * jacobian_transpose_pinv) *
        (nullspace_stiffness_ * (q_d_nullspace_ - q_eig) -
         (2.0 * sqrt(nullspace_stiffness_)) * dq_eig);
    // Desired torque
    ddq = tau_task + tau_nullspace; // + coriolis;
    //ddq = J.transpose() * (-cartesian_stiffness_*error -cartesian_damping_*J*dq.data);

#if 0
    ee_vel_ = J * dq.data;
    setPseudoController(robot_state, tp);
    ddq = Jinv * (tp.ddr_ + u_ - dJ*dq.data);
#endif

    renderEndEffectorAndTrajectoryPoint();
  }

  void ImpedanceController::trajectorySubscriberCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      Eigen::Vector3d     position_d_target_loc;
      Eigen::Quaterniond  orientation_d_target_loc;

      position_d_target_loc << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z; 
      Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
      orientation_d_target_loc.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w;
      if (last_orientation_d_target.coeffs().dot(orientation_d_target_loc.coeffs()) < 0.0) {
          orientation_d_target_loc.coeffs() << -orientation_d_target_loc.coeffs();
      }

      double alpha = 0.05;
      position_d_target_ = alpha*position_d_target_loc + (1-alpha)*position_d_target_;
      Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
      Eigen::AngleAxisd aa_orientation_d_target_loc(orientation_d_target_loc);
      aa_orientation_d_target.axis() = alpha * aa_orientation_d_target_loc.axis() +
          (1.0 - alpha) * aa_orientation_d_target.axis();
      aa_orientation_d_target.angle() = alpha * aa_orientation_d_target_loc.angle() +
          (1.0 - alpha) * aa_orientation_d_target.angle();
      orientation_d_target_ = Eigen::Quaterniond(aa_orientation_d_target);


  }

  void ImpedanceController::renderEndEffectorAndTrajectoryPoint() {

    visualization_msgs::MarkerArray marker_array;

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + getRootName();
      marker.header.stamp = ros::Time::now();
      marker.ns = "current";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = position_(0);
      marker.pose.position.y = position_(1);
      marker.pose.position.z = position_(2);
      marker.pose.orientation.x = orientation_.x();
      marker.pose.orientation.y = orientation_.y();
      marker.pose.orientation.z = orientation_.z();
      marker.pose.orientation.w = orientation_.w();
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);

      {
          Eigen::Quaternion<double> rotq(0.70710678118, 0, 0, 0.70710678118);
          Eigen::Quaternion<double> resq = orientation_ * rotq;
          marker.id = 2;
          marker.pose.orientation.x = resq.x();
          marker.pose.orientation.y = resq.y();
          marker.pose.orientation.z = resq.z();
          marker.pose.orientation.w = resq.w();
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          marker_array.markers.push_back(marker);
      }
      {
          Eigen::Quaternion<double> rotq(0.70710678118, 0, -0.70710678118, 0);
          Eigen::Quaternion<double> resq = orientation_ * rotq;
          marker.id = 3;
          marker.pose.orientation.x = resq.x();
          marker.pose.orientation.y = resq.y();
          marker.pose.orientation.z = resq.z();
          marker.pose.orientation.w = resq.w();
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
          marker_array.markers.push_back(marker);
      }
    }

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/" + getRootName();
      marker.header.stamp = ros::Time::now();
      marker.ns = "target";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; 
      marker.pose.position.x = position_d_target_(0);
      marker.pose.position.y = position_d_target_(1);
      marker.pose.position.z = position_d_target_(2);
      marker.pose.orientation.x = orientation_d_target_.x();
      marker.pose.orientation.y = orientation_d_target_.y();
      marker.pose.orientation.z = orientation_d_target_.z();
      marker.pose.orientation.w = orientation_d_target_.w();
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
      {
          Eigen::Quaternion<double> rotq(0.70710678118, 0, 0, 0.70710678118);
          Eigen::Quaternion<double> resq = orientation_d_target_ * rotq;
          marker.id = 2;
          marker.pose.orientation.x = resq.x();
          marker.pose.orientation.y = resq.y();
          marker.pose.orientation.z = resq.z();
          marker.pose.orientation.w = resq.w();
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          marker_array.markers.push_back(marker);
      }
      {
          Eigen::Quaternion<double> rotq(0.70710678118, 0, -0.70710678118, 0);
          Eigen::Quaternion<double> resq = orientation_d_target_ * rotq;
          marker.id = 3;
          marker.pose.orientation.x = resq.x();
          marker.pose.orientation.y = resq.y();
          marker.pose.orientation.z = resq.z();
          marker.pose.orientation.w = resq.w();
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
          marker_array.markers.push_back(marker);
      }
    }

    marker_array_pub_.publish(marker_array);
  }

  /*! \brief This code is got from Orocos KDL but is not available in ROS indigo! */
  int ImpedanceController::JntToJacDot(const KDL::JntArrayVel& q_in, 
                                              KDL::Jacobian& jdot) {
    unsigned int segmentNr = getKDLChain().getNrOfSegments();

    jdot.data.setZero();

    KDL::Twist jac_dot_k_ = KDL::Twist::Zero();
    int k = 0;
    for (unsigned int i=0; i<segmentNr; ++i) {
      if (getKDLChain().getSegment(i).getJoint().getType()!=KDL::Joint::None) {
        for (unsigned int j=0; j<getKDLChain().getNrOfJoints(); ++j) {
            jac_dot_k_ += getPartialDerivative(ee_jacobian_,j,k) * q_in.qdot(j);
        }
        jdot.setColumn(k++, jac_dot_k_);
        jac_dot_k_ = KDL::Twist::Zero();
      }
    }

    return 0;
  }

  KDL::Twist ImpedanceController::getPartialDerivative(
    const KDL::Jacobian& bs_J_ee, 
    const unsigned int& joint_idx, 
    const unsigned int& column_idx) 
  {
    int j=joint_idx;
    int i=column_idx;

    KDL::Twist jac_j = bs_J_ee.getColumn(j);
    KDL::Twist jac_i = bs_J_ee.getColumn(i);

    KDL::Twist t_djdq = KDL::Twist::Zero();

    if (j < i) {
      t_djdq.vel = jac_j.rot * jac_i.vel;
      t_djdq.rot = jac_j.rot * jac_i.rot;
    } else if (j > i) {
      t_djdq.rot = KDL::Vector::Zero();
      t_djdq.vel = -jac_j.vel * jac_i.rot;
    }else if (j == i) {
     t_djdq.rot = KDL::Vector::Zero();
     t_djdq.vel = jac_i.rot * jac_i.vel;
   }
   return t_djdq;
 }

}

PLUGINLIB_EXPORT_CLASS(force_controllers::ImpedanceController, controller_interface::ControllerBase)
