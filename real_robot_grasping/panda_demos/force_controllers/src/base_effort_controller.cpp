#include "base_effort_controller.h"
#include "utilities.h"

//#include <pluginlib/class_list_macros.h>

namespace force_controllers {

  ////////////////////////////////////////////////////////////////////////////
  //
  //                             P U B L I C
  //
  ///////////////////////////////////////////////////////////////////////////

  BaseEffortController::BaseEffortController() {}

  BaseEffortController::~BaseEffortController() {}

  bool BaseEffortController::init
  (
    JointEffortInterface *hw, 
    ros::NodeHandle &controller_nh
  )
  {
    hardware_interface_ = hw;
    controller_nh_ = controller_nh;

    robot_state_ptr_.reset(&robot_state_data_);

    loadUrdfToKdlTree();
    loadJointsAndSetJointHandlesMap();
    initInternalModel();
    sampleJointValues();

    initialize(robot_state_ptr_);
    return true;
  }

  void BaseEffortController::starting
  (
    const ros::Time& time
  )
  {
    // do nothing
  }

  void BaseEffortController::update
  (
    const ros::Time& time, 
    const ros::Duration& period
  )
  {
    sampleJointValues();
    setJointAccelerations(robot_state_ptr_, ddq_);
    //ddq_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    computeTorqueControls();
    setControls();
  }

  void BaseEffortController::stopping
  (
    const ros::Time& time
  )
  {
    // do nothing
  }

  ////////////////////////////////////////////////////////////////////////////
  //
  //                             P R I V A T E
  //
  ///////////////////////////////////////////////////////////////////////////

  int BaseEffortController::loadUrdfToKdlTree
  ()
  {
    std::string full_parameter_path;
    std::string robot_urdf;
    if (controller_nh_.searchParam("robot_description", full_parameter_path))
    {
        controller_nh_.getParam(full_parameter_path, robot_urdf);
        bool success = kdl_parser::treeFromString(robot_urdf, robot_state_data_.kdl_tree_);

        ROS_ASSERT(success);
        ROS_INFO_STREAM("Loaded the robot's urdf model and initialized the KDL tree successfully from "
                <<full_parameter_path );
    }
    else
    {
        ROS_ERROR("Could not find parameter 'robot_description' on the parameter server.");
        return -1;
    }
    return 0;
  }

  int BaseEffortController::loadJointsAndSetJointHandlesMap()
  {
    // Load the names of all joints specified in the .yaml file
    std::string param_name = "joints";
    std::vector< std::string > joint_names;
    if (!controller_nh_.getParam(param_name, joint_names))
      {
          ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
            << param_name 
            << "') in namespace '" 
            << controller_nh_.getNamespace() 
            << "' failed.");
          return -1;
      }

      // Load all joint handles for all joint name references
    for (auto&& name : joint_names) {
      try {
        int q_nr = kdl_getQNrFromJointName(robot_state_data_.kdl_tree_, name);
        if(q_nr < 0) {
            ROS_ERROR_STREAM("Could not find joint "<<name<<" in the KDL model");
            return -1;
        }
        std::cout << "Joint found: '" << name << "', qnr: " << q_nr << "\n";
        joint_handles_map_.emplace(q_nr, hardware_interface_->getHandle(name));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
              return -2;
      }
      // catch (MAP INSERT FAIL EXCEPTION)
      // catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
    }

    // Set the joint position and Effort and the control vectors to all zero
    unsigned int n_joint_names = joint_names.size();
    n_joints_ = robot_state_data_.kdl_tree_.getNrOfJoints();
    if (n_joint_names > n_joints_) {
      ROS_ERROR_STREAM("In ROSKinematicsController: The .yaml file"
        << " includes more joint names than specified in the .urdf file."
        << " number of joints in urdf "<<n_joints_<<" and in yaml "<<n_joint_names
        << " Could not succeffully initialize controller. Aborting!\n");
      return -3;
    }
    robot_state_data_.kdl_jnt_array_vel_.resize(n_joints_);
    robot_state_data_.kdl_effort_.resize(n_joints_);
    tau_u_ = Eigen::VectorXd::Zero(n_joints_);

    return 0;
  }

  void BaseEffortController::initInternalModel() {
    tau_u_ = Eigen::VectorXd::Zero(n_joints_);
    ddq_ = Eigen::VectorXd::Zero(n_joints_);
    tau_inertia_.resize(n_joints_);
    tau_coriolis_.resize(n_joints_);
    tau_gravity_.resize(n_joints_);

    controller_nh_.getParam("root_name", root_name_);
    controller_nh_.getParam("tip_name", tip_name_);

    KDL::Vector gravity = KDL::Vector::Zero();
    gravity(2) = -9.81;
    robot_state_data_.kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_);

    id_solver_ = std::make_shared<KDL::ChainDynParam>(kdl_chain_, gravity);
  }

  void BaseEffortController::sampleJointValues(){
    KDL::JntArray& q = robot_state_data_.kdl_jnt_array_vel_.q;
    KDL::JntArray& qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
    KDL::JntArray& effort = robot_state_data_.kdl_effort_;
    handles_mutex_.lock();
      for (auto&& handle : joint_handles_map_) {
        q(handle.first) = handle.second.getPosition();
        qdot(handle.first) = handle.second.getVelocity();
        effort(handle.first) = handle.second.getEffort();
      }
      ros::Time t = ros::Time::now();
      robot_state_data_.sampling_time_.setTimePoint(t.sec, t.nsec);
    handles_mutex_.unlock();
  }

  void BaseEffortController::computeTorqueControls() {
    const KDL::JntArray& q = robot_state_data_.kdl_jnt_array_vel_.value();
    const KDL::JntArray& dq = robot_state_data_.kdl_jnt_array_vel_.deriv();

    id_solver_->JntToMass(q, tau_inertia_);
    id_solver_->JntToCoriolis(q, dq, tau_coriolis_);
    id_solver_->JntToGravity(q, tau_gravity_);
    tau_u_ = tau_inertia_.data * ddq_ + tau_coriolis_.data + tau_gravity_.data;

    Eigen::VectorXd hej = tau_inertia_.data * ddq_;

    // std::cout << "g      = ";
    // for (int i=0; i<6; ++i) std::cout << tau_gravity_.data(i) << ",";
    // std::cout << "\n";

    // std::cout << "c      = ";
    // for (int i=0; i<6; ++i) std::cout << tau_coriolis_.data(i) << ",";
    // std::cout << "\n";

    // std::cout << "ddq_   = ";
    // for (int i=0; i<6; ++i) std::cout << ddq_(i) << ",";
    // std::cout << "\n";

    // std::cout << "hej    = ";
    // for (int i=0; i<6; ++i) std::cout << hej(i) << ",";
    // std::cout << "\n";

    // std::cout << "tau_inertia_.data = " << tau_inertia_.data << "\n";

    //std::cout << "tau_u_ = " << tau_u_ << "\n";
  }

  void BaseEffortController::setControls() {
    handles_mutex_.lock();
      for (auto&& handle : joint_handles_map_) {
        handle.second.setCommand(tau_u_(handle.first));
        //handle.second.setCommand(0);
      }
    handles_mutex_.unlock();
  }

}

//PLUGINLIB_EXPORT_CLASS(irb4400::BaseEffortController, controller_interface::ControllerBase)
