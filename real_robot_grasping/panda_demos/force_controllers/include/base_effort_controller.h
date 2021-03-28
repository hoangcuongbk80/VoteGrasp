/*! \file base_effort_controller.h
 *  \author Marcus A Johansson */

#ifndef BASE_EFFORT_CONTROLLER_H
#define BASE_EFFORT_CONTROLLER_H

#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

#include "robot_state.h"

namespace force_controllers {

  typedef 
  controller_interface::Controller<hardware_interface::EffortJointInterface>
  JointEffortController;

  typedef 
  hardware_interface::EffortJointInterface 
  JointEffortInterface;

  class BaseEffortController : public JointEffortController {
  public:
    BaseEffortController();
    ~BaseEffortController() noexcept;

    virtual bool init(JointEffortInterface *hw, 
                      ros::NodeHandle &controller_nh);

    virtual void starting(const ros::Time& time);

    void update(const ros::Time& time, 
                const ros::Duration& period);

    virtual void stopping(const ros::Time& time);

    /*! \brief A child class implements this to further initialize itself. This is called by BaseEffortController::init. */
    virtual void initialize(RobotStatePtr robot_state) = 0;

    /*! \brief A child class implements this function to compute the driving (desired) joint accelerations */
    virtual void setJointAccelerations(RobotStatePtr robot_state, Eigen::VectorXd& ddq) = 0;

  protected:
    inline ros::NodeHandle& getControllerNodeHandle() { return controller_nh_; }
    inline unsigned int getNJoints() { return n_joints_; }
    inline const std::string& getRootName() { return root_name_; }
    inline const std::string& getTipName() { return tip_name_; }
    inline const KDL::Chain& getKDLChain() { return kdl_chain_; }

  private:
    BaseEffortController(const BaseEffortController& other) = delete;
    BaseEffortController(BaseEffortController&& other) = delete;
    BaseEffortController& operator=(const BaseEffortController& other) = delete;
    BaseEffortController& operator=(BaseEffortController&& other) noexcept = delete;

    int loadUrdfToKdlTree();
    int loadJointsAndSetJointHandlesMap();
    void initInternalModel();
    void sampleJointValues();
    void computeTorqueControls();
    void setControls();

    typedef std::map<unsigned int, hardware_interface::JointHandle> JointHandleMap;

    ros::NodeHandle                       controller_nh_;
    JointEffortInterface*                 hardware_interface_;
    JointHandleMap                        joint_handles_map_;
    std::mutex                            handles_mutex_;

    RobotState                            robot_state_data_;
    RobotStatePtr                         robot_state_ptr_;

    std::shared_ptr<KDL::ChainDynParam>   id_solver_; // inverted dynamics solver

    std::string                           root_name_; // root link name
    std::string                           tip_name_; // tip (end-effector) link name

    KDL::Chain                            kdl_chain_; // chain of links from root to tip
    KDL::JntSpaceInertiaMatrix            tau_inertia_; // inertia matrix
    KDL::JntArray                         tau_coriolis_; // centrifugal and coriolis torques
    KDL::JntArray                         tau_gravity_; // gravity torques

    Eigen::VectorXd                       ddq_; // joint accelerations
    Eigen::VectorXd                       tau_u_; // driving torques
    
    unsigned int                          n_joints_;

  };

} // namespace force_controllers

#endif // include guard
