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

#define DEG(x) x*180/M_PI
#define RAD(x) x*M_PI/180
#define FACTOR 0.08
#define PLANNING_GROUP "panda_arm"
#define SHOTEST_PATH_DISTANCE 0.35
class move_franka_node
{
    private:
        ros::NodeHandle n;
        ros::Publisher command;
        ros::Publisher recordCommandPub;
        
        ros::Subscriber pose;
        ros::Subscriber F_ext;
    public:
        void commandCallback(const std_msgs::String::ConstPtr& pose);
        void F_extCallback(const geometry_msgs::WrenchStamped &msg);
        void resetPanda();
        bool pandaIntersect();
        void moveCartesian(Eigen::Quaterniond orient);

        Eigen::Quaterniond orientEE(const Eigen::Vector3d ref_point);
        moveit::planning_interface::MoveGroupInterface *move_group;
        const moveit::core::JointModelGroup* joint_model_group;
        Eigen::Vector3d random_point;
        move_franka_node():
        command(n.advertise<std_msgs::String>("command", 10)), 
        pose(n.subscribe("command", 1, &move_franka_node::commandCallback, this)),
        recordCommandPub(n.advertise<std_msgs::Bool>("RecordingSignal",1)),
        F_ext(n.subscribe("/panda/franka_state_controller/F_ext", 3, &move_franka_node::F_extCallback, this))
        {
            static const std::string ROBOT_DESC = "robot_description";
            moveit::planning_interface::MoveGroupInterface::Options opts(PLANNING_GROUP,ROBOT_DESC);
            move_group = new moveit::planning_interface::MoveGroupInterface(opts);
            joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        }

};


void move_franka_node::commandCallback(const std_msgs::String::ConstPtr& msg){

    std::istringstream iss(msg->data);
    std::vector<std::string> pose(std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>());
    std::vector<double> p;
    for (int i = 0; i < pose.size(); i++){
        std::cout << pose[i] << "\n";
        p.push_back( std::stod(pose[i]));
    }

    geometry_msgs::Pose target_pose;
    target_pose.position.x = p[0];
    target_pose.position.y = p[1];
    target_pose.position.z = p[2];
    tf2::Quaternion q;
    q.setRPY( p[3]*M_PI/180, p[4]*M_PI/180, p[5]*M_PI/180 );
    target_pose.orientation = tf2::toMsg(q);

    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Plan status: %s", success ? "SUCCESS" : "FAILIURE");
    ROS_INFO("Execution");
    move_group->move();
}

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

bool move_franka_node::pandaIntersect(){
    Eigen::Vector3d panda_base(-0.3,0.305,0.7);
    Eigen::Vector3d rand = random_point-panda_base;
    double r = 0.6;
    // Intersection with panda sphere equation check
    if (rand.norm() <= r) return true; else false;
}

void move_franka_node::resetPanda(){
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = 1.7855;  
    joint_group_positions[1] = -1.3310;
    joint_group_positions[2] = -0.1737;
    joint_group_positions[3] = -2.5391;
    joint_group_positions[4] = -0.3475;
    joint_group_positions[5] = 1.00165;
    joint_group_positions[6] = 0.04;

    move_group->setJointValueTarget(joint_group_positions);
    ROS_INFO("Resetting joints angles...");
    move_group->move();
    ROS_INFO("Resetting executed!");
}

/* To decrease the moment generated around the EE, the EE should be oriented 
        towards the screwed end of the hose */
Eigen::Quaterniond move_franka_node::orientEE(const Eigen::Vector3d ref_point){
    // Building a coordinated system 
    Eigen::Vector3d x_ee_ref = ref_point - random_point;
    Eigen::Vector3d z_axis = Eigen::Vector3d(0,0,-1);
    Eigen::Vector3d y_ee_ref = z_axis.cross(x_ee_ref);
    Eigen::Vector3d z_ee_ref = x_ee_ref.cross(y_ee_ref);
    // Creating a rotation matrix   
    Eigen::Matrix3d rot; rot.col(0) = x_ee_ref.normalized(); rot.col(1) = y_ee_ref.normalized(); rot.col(2) = z_ee_ref.normalized();
    Eigen::Quaterniond q_eig(rot);
    Eigen::Vector3d euler = q_eig.toRotationMatrix().eulerAngles(0,1,2);
    // std::cout << "euler: \n" <<  DEG(euler) << "\n";

    // Shifting the angles to the chosen default EE orientation
    q_eig = Eigen::AngleAxisd( euler[0] , Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(euler[2] , Eigen::Vector3d::UnitZ());
    return q_eig;
}


void move_franka_node::moveCartesian(Eigen::Quaterniond orient){
    // geometry_msgs::Pose initState = mf.move_group->getCurrentPose().pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std_msgs::Bool recordMsg;

    geometry_msgs::Pose target_pose;
    move_group->setStartState(*move_group->getCurrentState());
    target_pose.position = tf2::toMsg(random_point);
    target_pose.orientation = tf2::toMsg(orient);
    move_group->setPoseTarget(target_pose);    
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("plan failed!");
        return;
    }

    // Computing the cartesian path
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.001;
    // double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);


    // Setting maximum velocity and accreset_eleration factor
    robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), my_plan.trajectory_);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, FACTOR, FACTOR);
    rt.getRobotTrajectoryMsg(my_plan.trajectory_);

    ROS_INFO("plan executing!");

    recordMsg.data = true;
    
    this->recordCommandPub.publish(recordMsg);
    ros::spinOnce();
    move_group->execute(my_plan);

    recordMsg.data = false;
    this->recordCommandPub.publish(recordMsg);

    ros::spinOnce();
    ROS_INFO("plan executed!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_franka");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_franka_node mf;

    /* The settings below don't work with cartesian paths because 
       Moveit! doesn't plan velocities and accelerations*/ 
    mf.move_group->setMaxVelocityScalingFactor(FACTOR);
    mf.move_group->setMaxAccelerationScalingFactor(FACTOR);

    mf.resetPanda();

    // The coordinates of ref_point are measured manually
    
    Eigen::Vector3d ref_point_for_ori(0.90, 0.21, 0.8);
    double ref_point_for_ori_offset_bounds[3][2] = {
        {-0.5,0.6},
        {-0.10,0.10},
        {-0.10,0.10} 
        };

    Eigen::Vector3d ref_point_0(0.90, 0.21, 1.23);
    Eigen::Vector3d ref_point_1(0.1, 0.21, 1.23);
    Eigen::Vector3d ref_point=ref_point_1;

    Eigen::Vector3d reset_point(-0.181, 0.235, 1.237);

    /* * * Set Bounds * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // The bounds are represented by a spherical coordinate system, r: {min, max}, theta: {min, max},phi: {min, max}:
    double bounds[3][2] = {
        {0.7,0.8},
        {-M_PI/6,M_PI/6},
        {- M_PI/6,M_PI/6} 
        };

    double bounds_1[3][2] = {
    {0.1,0.28},
    {- M_PI/4,M_PI/5}, 
    {-M_PI/2,M_PI/2}
    };
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    
    std::uniform_real_distribution<double> ref_point_for_ori_offset_x(ref_point_for_ori_offset_bounds[0][0], ref_point_for_ori_offset_bounds[0][1]);
    std::uniform_real_distribution<double> ref_point_for_ori_offset_y(ref_point_for_ori_offset_bounds[1][0], ref_point_for_ori_offset_bounds[1][1]);
    std::uniform_real_distribution<double> ref_point_for_ori_offset_z(ref_point_for_ori_offset_bounds[2][0], ref_point_for_ori_offset_bounds[2][1]);
    
    std::uniform_real_distribution<double> dist_r(bounds[0][0], bounds[0][1]);
    std::uniform_real_distribution<double> dist_theta(bounds[1][0], bounds[1][1]);
    std::uniform_real_distribution<double> dist_phi(bounds[2][0], bounds[2][1]);

    std::uniform_real_distribution<double> dist_r_1(bounds_1[0][0], bounds_1[0][1]);
    std::uniform_real_distribution<double> dist_theta_1(bounds_1[1][0], bounds_1[1][1]);
    std::uniform_real_distribution<double> dist_phi_1(bounds_1[2][0], bounds_1[2][1]);

    std::random_device device;
    //Mersenne Twister: Good quality random number generator
    std::mt19937 generator(device());
    
    double r, theta, phi, x, y, z,ref_ori_x,ref_ori_y,ref_ori_z;
    double pathLen;
    Eigen::Vector3d lastPos(reset_point);
    
    while (ros::ok())
    {
        do
        {
            ROS_INFO("Generating random points");
            // r = dist_r(generator); theta = dist_theta(generator); phi = dist_phi(generator);

            do{// make sure not locate in a area where the hose might be pulled straight. robot is tent to stop
            r = dist_r_1(generator); 
            theta = dist_theta_1(generator); 
            phi = dist_phi_1(generator);
            }while(phi<M_PI/20&&phi>-M_PI/20 && theta<M_PI/20&&theta>-M_PI/20);
            
            
            // calculating cartesian coordinates from spherical coordinates
            x = ref_point[0] - r*cos(theta)*cos(phi);
            y = ref_point[1] - r*cos(theta)*sin(phi);
            z = ref_point[2] - r*sin(theta);
            mf.random_point = Eigen::Vector3d(x,y,z);
            pathLen = (mf.random_point-lastPos).norm();
            ROS_INFO("path length: %f", pathLen);
            
        } while (pathLen<SHOTEST_PATH_DISTANCE);
       ROS_INFO("r: %f, theta: %f, phi: %f \n", r, DEG(theta) ,DEG(phi));
        // Orient and Move the robot through cartesian paths;

        lastPos = mf.random_point;
        Eigen::Vector3d ref_point_for_ori_offset = Eigen::Vector3d(ref_point_for_ori_offset_x(generator),ref_point_for_ori_offset_y(generator),ref_point_for_ori_offset_z(generator));
        Eigen::Vector3d ref_point_for_ori_cur = ref_point_for_ori_offset + ref_point_for_ori;

       ROS_INFO("x: %f, y: %f, z: %f \n ori ref x: %f, y: %f, z: %f \n", x,y,z,ref_point_for_ori_cur[0],ref_point_for_ori_cur[1],ref_point_for_ori_cur[2]);
        mf.moveCartesian( mf.orientEE(ref_point_for_ori_cur) );
        
        // mf.resetPanda();

        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
