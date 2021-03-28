#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#define FACTOR 0.08
#define PLANNING_GROUP "panda_arm"

class move_franka_node
{
    private:
        ros::NodeHandle n, nh_;
        ros::Subscriber F_ext;

    public:
        void resetPanda();
        void move_pick();
        void move_back();
        void F_extCallback(const geometry_msgs::WrenchStamped &msg);

        moveit::planning_interface::MoveGroupInterface *move_group;
        const moveit::core::JointModelGroup* joint_model_group;
        move_franka_node():
        F_ext(n.subscribe("/panda/franka_state_controller/F_ext", 3, &move_franka_node::F_extCallback, this))
        {
            static const std::string ROBOT_DESC = "robot_description";
            moveit::planning_interface::MoveGroupInterface::Options opts(PLANNING_GROUP,ROBOT_DESC);
            move_group = new moveit::planning_interface::MoveGroupInterface(opts);
            joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        }

};

void move_franka_node::F_extCallback(const geometry_msgs::WrenchStamped& msg){
    Eigen::Vector3d f (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d t (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
    if(f.norm() > 30|| t.norm() > 75){
        ROS_INFO("Stopping! Too large torque/ force");
        ROS_INFO("Force: %f", f.norm());
        ROS_INFO("Torque: %f\n", t.norm());
        // move_group->stop();
        sleep(5);
    } 
}

void move_franka_node::resetPanda()
{
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    const std::vector<std::string>& joint_names = current_state->getVariableNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
    }

    const Eigen::Affine3d& end_effector_state = current_state->getGlobalLinkTransform("panda_link8");
    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation().eulerAngles(0, 1, 2) << "\n");

    joint_group_positions[0] = 0.067262;  
    joint_group_positions[1] = -0.794014;
    joint_group_positions[2] = -0.049588;
    joint_group_positions[3] = -2.949210;
    joint_group_positions[4] = 0.148797;
    joint_group_positions[5] = 1.854945;
    joint_group_positions[6] = 0.538748;

    move_group->setJointValueTarget(joint_group_positions);
    ROS_INFO("Resetting joints angles...");
    move_group->move();
    ROS_INFO("Resetting executed!");
    ROS_INFO("EndEffector: %s",move_group->getEndEffectorLink().c_str());
}

void move_franka_node::move_back()
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.1;
    target_pose.position.z = 1.1;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -M_PI/2);
    target_pose.orientation = tf2::toMsg(q);
    move_group->setPoseTarget(target_pose, "panda_hand");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Plan move back status: %s", success ? "SUCCESS" : "FAILIURE");
    ROS_INFO("Execution move back");
    move_group->move();
}

void move_franka_node::move_pick()
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.06;
    target_pose.position.y = 0.01;
    target_pose.position.z = 0.95;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -M_PI/2);
    target_pose.orientation = tf2::toMsg(q);
    move_group->setPoseTarget(target_pose, "panda_hand");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Plan pick status: %s", success ? "SUCCESS" : "FAILIURE");
    ROS_INFO("Execution pick");
    move_group->move();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_franka");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_franka_node mf;
 
    mf.move_group->setMaxVelocityScalingFactor(FACTOR);
    mf.move_group->setMaxAccelerationScalingFactor(FACTOR);
    //mf.resetPanda();
    mf.move_back();
    //mf.move_pick();
    return 0;
}
