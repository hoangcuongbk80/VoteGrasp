// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "traj_planning.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

int determineSign(double num)
  {    
    if(num > 0){
      return 1;
    } else {
      return -1;
    }
  }

// void setDefaultBehavior(franka::Robot& robot) {
//   robot.setCollisionBehavior(
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
//   robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
//   robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
// }

TrajPlanning::TrajPlanning(double speed_factor, std::array<double, 7> q_0, std::array<double, 7> q_f, std::array<double, 7> dq_0)
: q_goal_(q_f.data()) {
  dq_max *= speed_factor;
  ddq_max *= speed_factor;

  for (size_t i = 0; i < 7; i++) {
    q_0_[i] = q_0[i];
    //q_goal_[i] = q_f[i];
    dq_0_[i] = dq_0[i];             //v0
  }


  ddq_.setZero();;
  dq_m_.setZero();
  t_1_.setZero();
  t_2_.setZero();
  t_f_.setZero();

  ddq_sync_.setZero();;
  dq_m_sync_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_ = 0;

}

// for traj planning only, not motion generation
void TrajPlanning::initTminTraj(){
  // q_start_ = Vector7d(robot_state.q_d.data());
  for (size_t i = 0; i < 7; i++) {
    q_start_[i] = q_0_[i];
    q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
    sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign
  }
}

bool TrajPlanning::calculateTminTraj(){
  // traj_para: a, vm, t1, t2, tf

  
  /* Tmin traj of each joint */
  for (size_t i = 0; i < 7; i++) {
    ddq_[i] = ddq_max[i];
    dq_m_[i] = std::sqrt(std::pow(dq_0_[i], 2.0)/2.0 + ddq_max[i]*q_delta_[i]);
    if (dq_max[i] > dq_m_[i]){
      t_f_[i] = (2.0*dq_m_[i] - dq_0_[i]) / ddq_max[i];
    } else{
      t_f_[i] = q_delta_[i]/dq_max[i] + (dq_max[i] - dq_0_[i] + std::pow(dq_0_[i], 2.0)/(2.0*dq_max[i])) / ddq_max[i];
    }
  }

   /* Synchronized min motion time */ 
  t_f_sync_ = t_f_.maxCoeff();


  /* Adjust each traj to sync */
  std::array<double, 7> dq_m_tri = {0,0,0,0,0,0,0};
  for (size_t i = 0; i < 7; i++) {
    if(q_delta_[i] > kDeltaQMotionFinished){
      dq_m_tri[i] = q_delta_[i]/t_f_sync_ + std::sqrt(std::pow((q_delta_[i]/t_f_sync_ - dq_0_[i]/2.0),2.0) + std::pow(dq_0_[i],2.0)/4.0);     // t1=t2, triangle
      dq_m_sync_[i] = std::min(dq_m_tri[i],dq_max[i]);        // triangle or trapezoid
      ddq_sync_[i] = (dq_0_[i]*dq_m_sync_[i] - std::pow(dq_m_sync_[i],2.0) - std::pow(dq_0_[i],2.0)/2.0) / (q_delta_[i] - t_f_sync_*dq_m_sync_[i]);
      t_1_sync_[i] = (dq_m_sync_[i]-dq_0_[i])/ddq_sync_[i];
      t_2_sync_[i] = t_f_sync_-dq_m_sync_[i]/ddq_sync_[i];
    } else{
      dq_m_sync_[i] = 0;
      ddq_sync_[i] = 0;
      t_1_sync_[i] = 0;
      t_2_sync_[i] = 0;
    }
      // if vel exceeds limit?
      if(dq_max[i]-dq_m_sync_[i] < -std::pow(10,-10)){
        feasible_dq_ *= 0;
      }
      // if acc exceeds limit?
      if(ddq_max[i]-ddq_sync_[i] < -std::pow(10,-10)){
        feasible_ddq_ *= 0;
      }

  }



  /* Synchronized tmin traj para */
  traj_para[0] = dq_m_sync_[0];
  traj_para[1] = dq_m_sync_[1];
  traj_para[2] = dq_m_sync_[2];
  traj_para[3] = dq_m_sync_[3];
  traj_para[4] = dq_m_sync_[4];
  traj_para[5] = dq_m_sync_[5];
  traj_para[6] = dq_m_sync_[6];
  traj_para[7] = t_f_sync_;

  feasible_ = feasible_q_ * feasible_dq_ * feasible_ddq_;
  if(feasible_ == 1){
    return true;
  } else{
    return false;
  }

}


bool TrajPlanning::proportionalSlowTraj(double slow_factor){
  if(dq_0_==Vector7d::Zero()){
    t_f_sync_ *= slow_factor;
    for (int i = 0; i < 7; ++i)
    {
      t_1_sync_[i] *= slow_factor;
      t_2_sync_[i] *= slow_factor;
      dq_m_sync_[i] /= slow_factor;
      ddq_sync_[i] = ddq_sync_[i]/std::pow(slow_factor,2.0);
    }
      traj_para[0] = dq_m_sync_[0];
      traj_para[1] = dq_m_sync_[1];
      traj_para[2] = dq_m_sync_[2];
      traj_para[3] = dq_m_sync_[3];
      traj_para[4] = dq_m_sync_[4];
      traj_para[5] = dq_m_sync_[5];
      traj_para[6] = dq_m_sync_[6];
      traj_para[7] = t_f_sync_;
    return true;
  } else {
    return false;
  }

}




bool TrajPlanning::calculateDesiredValues(double t, Vector7d* delta_q_d) const {

  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(q_delta_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = dq_0_[i]*t + 0.5*ddq_sync_[i]*std::pow(t, 2.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = 
        dq_0_[i]*t_1_sync_[i] + 0.5*ddq_sync_[i]*std::pow(t_1_sync_[i], 2.0) +
        (t-t_1_sync_[i])*dq_m_sync_[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_) {
        (*delta_q_d)[i] = dq_0_[i]*t_1_sync_[i] + 0.5*ddq_sync_[i]*std::pow(t_1_sync_[i], 2.0) +
        (t_2_sync_[i]-t_1_sync_[i])*dq_m_sync_[i] +
        dq_m_sync_[i]*(t-t_2_sync_[i])-0.5*ddq_sync_[i]*std::pow(t-t_2_sync_[i], 2.0);
      } else {
        (*delta_q_d)[i] = q_delta_[i];
        joint_motion_finished[i] = true;
      }
    }
    (*delta_q_d)[i] *= sign_q_[i];
  }

  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

// offline algorithm test
bool TrajPlanning::generateMotion() {
  

  
  // if (time_ == 0.0) {

  //   for (size_t i = 0; i < 7; i++) {
  //     q_start_[i] = q_0_[i];
  //     q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
  //     sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign
  //   }
  //   calculateTminTraj();
  //   //proportionalSlowTraj(2);
  // }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  q_current = joint_positions;

  time_ += sample_time;

  return motion_finished;

}

// Online motion generator
franka::JointPositions TrajPlanning::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    for (size_t i = 0; i < 7; i++) {
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]);
    }
    calculateTminTraj();
        std::cout << "Time minimal trajectory parameter: " 
        << traj_para[0] << ", "
        << traj_para[1] << ", "
        << traj_para[2] << ", "
        << traj_para[3] << ", "
        << traj_para[4] << ", "
        << traj_para[5] << ", "
        << traj_para[6] << ", "
        << traj_para[7]
        << std::endl;
        std::cout << "Acc: " 
        << ddq_sync_[0] << ", "
        << ddq_sync_[1] << ", "
        << ddq_sync_[2] << ", "
        << ddq_sync_[3] << ", "
        << ddq_sync_[4] << ", "
        << ddq_sync_[5] << ", "
        << ddq_sync_[6]
        << std::endl;
  }

  // std::cout << robot_state.cartesian_contact[0] << ", "
  //       << robot_state.cartesian_contact[1] << ", "
  //       << robot_state.cartesian_contact[2] << ", "
  //       << robot_state.cartesian_contact[3] << ", "
  //       << robot_state.cartesian_contact[4] << ", "
  //       << robot_state.cartesian_contact[5]
  //       << std::endl;

  // std::cout << "Acc: " 
  //       << robot_state.joint_contact[0] << ", "
  //       << robot_state.joint_contact[1] << ", "
  //       << robot_state.joint_contact[2] << ", "
  //       << robot_state.joint_contact[3] << ", "
  //       << robot_state.joint_contact[4] << ", "
  //       << robot_state.joint_contact[5] << ", "
  //       << robot_state.joint_contact[6]
  //       << std::endl;



  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
