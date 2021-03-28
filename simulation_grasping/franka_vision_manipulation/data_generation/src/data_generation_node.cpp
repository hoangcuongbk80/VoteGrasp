#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <gazebo_msgs/SetLinkState.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

using namespace cv;
using namespace std;
using namespace Eigen;

class readrgbdNode
{
  public:
    readrgbdNode();
    virtual ~readrgbdNode();
    void subcribeTopics();
    void advertiseTopics();
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cloudPublish();
    void depthToClould(cv::Mat &depth_img);
    void gripper_init(double open_dst);
    bool setup_marker_withEuler();
    void read_grasp(std::string grasp_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr gripper;
    visualization_msgs::MarkerArray multiMarker;

    bool image_save;
    std::string depth_topsub, rgb_topsub, cloud_topsub, cloud_toppub;
    std::string saved_rgb_dir, saved_depth_dir, saved_points_dir;
    std::string optical_frame_id, grasp_frame_id;
    std::string grasp_path;
    vector<vector<float>> grasps;

    double fx, fy, cx, cy, depth_factor;
    double table_height;
    float dst_gripper2object;
    float quality_thresh = 0.0;

    tf::TransformListener tf_listener;

  private:
   ros::NodeHandle nh_, nh_rgb, nh_depth, nh_cloud, nh_grasp;
   ros::Subscriber depth_sub, rgb_sub, cloud_sub;
   ros::Publisher cloud_pub, grasp_pub;
};

readrgbdNode::readrgbdNode()
{
  nh_ = ros::NodeHandle("~");
  nh_rgb = ros::NodeHandle("~");
  nh_depth = ros::NodeHandle("~");
  nh_cloud = ros::NodeHandle("~");
  nh_grasp = ros::NodeHandle("~");

  nh_depth.getParam("depth_topsub", depth_topsub);
  nh_rgb.getParam("rgb_topsub", rgb_topsub);
  nh_cloud.getParam("cloud_topsub", cloud_topsub);
  nh_.getParam("cloud_toppub", cloud_toppub);

  nh_.getParam("image_save", image_save);
  nh_.getParam("saved_rgb_dir", saved_rgb_dir);
  nh_.getParam("saved_depth_dir", saved_depth_dir);
  nh_.getParam("saved_points_dir", saved_points_dir);

  nh_.getParam("table_height", table_height);
  nh_.getParam("optical_frame_id", optical_frame_id);
  nh_.getParam("grasp_frame_id", grasp_frame_id);

  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("depth_factor", depth_factor);

  nh_.getParam("grasp_path", grasp_path);
  nh_.getParam("quality_thresh", quality_thresh);
  nh_.getParam("dst_gripper2object", dst_gripper2object);

  std::cerr << "depth topics sub: " << "\n" << depth_topsub << "\n";
  std::cerr << "rgb topics sub: " << "\n" << rgb_topsub << "\n";
  std::cerr << "cloud topics sub: " << "\n" << cloud_topsub << "\n";
  std::cerr << "save depth to: " << "\n" << saved_depth_dir << "\n";
  std::cerr << "save rgb to: " << "\n" << saved_rgb_dir << "\n";
  std::cerr << "save points to: " << "\n" << saved_points_dir << "\n";

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

readrgbdNode::~readrgbdNode()
{
};

void readrgbdNode::subcribeTopics()
{
  depth_sub = nh_depth.subscribe (depth_topsub, 1, &readrgbdNode::depthCallback, this);
  rgb_sub = nh_rgb.subscribe (rgb_topsub, 1, &readrgbdNode::rgbCallback, this);  
  cloud_sub = nh_cloud.subscribe (cloud_topsub, 1, &readrgbdNode::cloudCallback, this);
}

void readrgbdNode::advertiseTopics()
{
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud2> (cloud_toppub, 1);
  grasp_pub = nh_grasp.advertise<visualization_msgs::MarkerArray>( "myGrasp", 1);
}

void readrgbdNode::gripper_init(double open_dst)
{
/*       
       p4 -         - p5             Z -     - Y
          -         -                  -   -
          -   p1    -                  - -
          - - - - - -                  - - - - - X
          p2   -    p3
               -
               -p0

 */
  if(!gripper->empty()) gripper.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ p0;  
  p0.x = 0; p0.y = 0; p0.z = -0.1;
  gripper->push_back(p0);
  
  pcl::PointXYZ p1;  
  p1.x = 0; p1.y = 0; p1.z = 0;
  gripper->push_back(p1);

  pcl::PointXYZ p2;  
  p2.x = -open_dst/2; p2.y = 0; p2.z = 0;
  gripper->push_back(p2);

  pcl::PointXYZ p3;  
  p3.x = open_dst/2; p3.y = 0; p3.z = 0;
  gripper->push_back(p3);

  pcl::PointXYZ p4;  
  p4.x = -open_dst/2; p4.y = 0; p4.z = 0.1;
  gripper->push_back(p4);

  pcl::PointXYZ p5;  
  p5.x = open_dst/2; p5.y = 0; p5.z = 0.1;
  gripper->push_back(p5);
}

bool readrgbdNode::setup_marker_withEuler()
{
    if(grasps.empty())
    {
      std::cerr << "\n Grasps empty!";
      return false;
    }

    for(int i=0; i < grasps.size(); i++)
    {
      gripper_init(grasps[i][8]);
      visualization_msgs::Marker line_list;
      geometry_msgs::Point p;

      line_list.header.frame_id = grasp_frame_id;
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "grasps";
      line_list.id = i;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.position.x = grasps[i][1];
      line_list.pose.position.y = grasps[i][2];
      line_list.pose.position.z = grasps[i][3] + table_height +  dst_gripper2object;
      Matrix3f mat;
      mat = AngleAxisf(grasps[i][4]*M_PI/180, Vector3f::UnitX())
          * AngleAxisf(grasps[i][5]*M_PI/180, Vector3f::UnitY())
          * AngleAxisf(grasps[i][6]*M_PI/180, Vector3f::UnitZ());
      Quaternionf q(mat);
      line_list.pose.orientation.x = q.x();
      line_list.pose.orientation.y = q.y();
      line_list.pose.orientation.z = q.z();
      line_list.pose.orientation.w = q.w();
      line_list.scale.x = 0.002; line_list.scale.y = 0.002; line_list.scale.z = 0.002;
      line_list.color.r = 0.0f; line_list.color.g = 1.0f; line_list.color.b = 0.0f; line_list.color.a = 1.0;
      
      p.x = gripper->points[0].x; p.y = gripper->points[0].y; p.z = gripper->points[0].z;
      line_list.points.push_back(p);

      p.x = gripper->points[1].x; p.y = gripper->points[1].y; p.z = gripper->points[1].z;
      line_list.points.push_back(p);

      p.x = gripper->points[2].x; p.y = gripper->points[2].y; p.z = gripper->points[2].z;
      line_list.points.push_back(p);

      p.x = gripper->points[3].x; p.y = gripper->points[3].y; p.z = gripper->points[3].z;
      line_list.points.push_back(p);

      p.x = gripper->points[2].x; p.y = gripper->points[2].y; p.z = gripper->points[2].z;
      line_list.points.push_back(p);

      p.x = gripper->points[4].x; p.y = gripper->points[4].y; p.z = gripper->points[4].z;
      line_list.points.push_back(p);

      p.x = gripper->points[3].x; p.y = gripper->points[3].y; p.z = gripper->points[3].z;
      line_list.points.push_back(p);

      p.x = gripper->points[5].x; p.y = gripper->points[5].y; p.z = gripper->points[5].z;
      line_list.points.push_back(p);

      visualization_msgs::Marker graspName;
      graspName.header.frame_id = grasp_frame_id;
      graspName.header.stamp = ros::Time::now();
      graspName.id = i;
      graspName.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      graspName.action = visualization_msgs::Marker::ADD;
      graspName.pose.position.x = grasps[i][1];
      graspName.pose.position.y = grasps[i][2];
      graspName.pose.position.z = grasps[i][3] + table_height + dst_gripper2object + 0.105;
      graspName.pose.orientation.x = q.x();
      graspName.pose.orientation.y = q.y();
      graspName.pose.orientation.z = q.z();
      graspName.pose.orientation.w = q.w();
      graspName.text = std::to_string(i+2);
      graspName.scale.x = 0.005; graspName.scale.y = 0.005; graspName.scale.z = 0.005;
      graspName.color.r = 1.0f; graspName.color.g = 0.0f; graspName.color.b = 0.0f; graspName.color.a = 1.0;

      graspName.lifetime = ros::Duration();
      if(grasps[i][7] > quality_thresh)
      {
          multiMarker.markers.push_back(graspName);
          line_list.lifetime = ros::Duration();
          multiMarker.markers.push_back(line_list);
      }
        
    }
    return true;
}

void readrgbdNode::read_grasp(std::string grasp_path)
{
  ifstream grasp_file (grasp_path);
  if (grasp_file.is_open())            
    {
      string line;
      getline (grasp_file, line);
       while(!grasp_file.eof())
      {
        vector<string> st;
        vector<float> grasp;
        getline (grasp_file, line);
        boost::trim(line);
		    boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
        if(st.size() < 6) continue;
        for(int i=0; i < st.size(); i++)
          {
            grasp.push_back(std::stof(st[i]));
          }
        grasps.push_back(grasp);
        
        //trans(0) = std::stof(st[4]); trans(1) = std::stof(st[5]); trans(2) = std::stof(st[6]); //translaton
        //rot_quaternion[0] = std::stof(st[0]); rot_quaternion[1] = std::stof(st[1]); //rotation
        //rot_quaternion[2] = std::stof(st[2]); rot_quaternion[3] = std::stof(st[3]); //rotation
      }
    }
  else 
    {
      std::cerr << "Unable to open file";
    }
}

void readrgbdNode::depthToClould(cv::Mat &depth_img)
{
   scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   for(int row=0; row < depth_img.rows; row++)
    {
       for(int col=0; col < depth_img.cols; col++)       
        {
          if(isnan(depth_img.at<ushort>(row, col))) continue;
          double depth = depth_img.at<ushort>(row, col) / depth_factor;
          point.x = (col-cx) * depth / fx;
          point.y = (row-cy) * depth / fy;
          point.z = depth;
          point.b = 190;//rgb_img.at<cv::Vec3b>(row, col)[0];
          point.g = 190;//rgb_img.at<cv::Vec3b>(row, col)[1];
          point.r = 190;//rgb_img.at<cv::Vec3b>(row, col)[2];
         scene_cloud->push_back(point);
        }
    }

  // Publish
  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  scene_cloud->header.frame_id = optical_frame_id;
  pcl::toPCLPointCloud2(*scene_cloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  cloud_pub.publish(output);

  //convert to world
  tf::StampedTransform transform;
  Eigen::Affine3d Tcam_offset;
  try
  {
      tf_listener.lookupTransform(optical_frame_id, "world", ros::Time(0), transform);
      tf::poseTFToEigen(transform, Tcam_offset);
      Eigen::Matrix4d m = Tcam_offset.matrix().inverse();
      pcl::transformPointCloud (*scene_cloud, *scene_cloud, m);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
  }

  // Save pointcloud for votegrasp
  Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
  transform_pc.translation() << 0, 0, -table_height;
  pcl::transformPointCloud (*scene_cloud, *scene_cloud, transform_pc);
  pcl::io::savePLYFileBinary(saved_points_dir, *scene_cloud);
  std::cerr << "save: " << saved_points_dir << "\n";
}

void readrgbdNode::depthCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;

  try
  {
    bridge = cv_bridge::toCvCopy(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  cv::Mat depth_img;
  depth_img = bridge->image.clone();
  depth_img.convertTo(depth_img, CV_16UC1, 1000.0); //Asus
  //cv::imshow("depth", depth_img);
  //cv::waitKey(3);
  //if(image_save) cv::imwrite( saved_depth_dir, depth_img );
  //depthToClould(depth_img);
  setup_marker_withEuler();
  read_grasp(grasp_path);
  grasp_pub.publish(multiMarker);
}

void readrgbdNode::rgbCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg, "bgr8");    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform rgb image.");
    return;
  }
  cv::Mat rgb_image;
  rgb_image = bridge->image;
  //cv::imshow("RGB image", rgb_image);
  //cv::waitKey(3);
  if(image_save) cv::imwrite( saved_rgb_dir, rgb_image );
}

void readrgbdNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{   
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl_conversions::toPCL(*cloud_msg, *cloud2);
  
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2( *cloud2, *cloud_);
  // do something
  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_, *scene_cloud);
  //convert to world
  tf::StampedTransform transform;
  Eigen::Affine3d Tcam_offset;
  try
  {
      tf_listener.lookupTransform(optical_frame_id, "world", ros::Time(0), transform);
      tf::poseTFToEigen(transform, Tcam_offset);
      Eigen::Matrix4d m = Tcam_offset.matrix().inverse();
      pcl::transformPointCloud (*scene_cloud, *scene_cloud, m);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
  }

  // Save pointcloud for votegrasp
  Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
  transform_pc.translation() << 0, 0, -table_height;
  pcl::transformPointCloud (*scene_cloud, *scene_cloud, transform_pc);
  boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);

  pcl::io::savePLYFileBinary(saved_points_dir, *scene_cloud);
  std::cerr << "save: " << saved_points_dir << "\n";

  ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state"); 
 geometry_msgs::Pose start_pose;
start_pose.position.x = 0.2;
start_pose.position.y = 0;
start_pose.position.z = 0;
start_pose.orientation.x = 0.0;
start_pose.orientation.y = 0.0;
start_pose.orientation.z = 0.0;
start_pose.orientation.w = 0.0;

geometry_msgs::Twist start_twist;
start_twist.linear.x = 1.1;
start_twist.linear.y = 0;
start_twist.linear.z = 0;
start_twist.angular.x = 0.0;
start_twist.angular.y = 0.0;
start_twist.angular.z = 0.0;
//---------------------------------------------------------------------
gazebo_msgs::SetLinkState setlinkstate;
gazebo_msgs::LinkState linkstate;
linkstate.link_name = "011_banana_link"; 
linkstate.reference_frame = "world";
linkstate.twist = start_twist;
linkstate.pose = start_pose;
setlinkstate.request.link_state = linkstate;
if (client.call(setlinkstate))
{
  ROS_INFO("BRILLIANT!!!");
  ROS_INFO("%f, %f",linkstate.pose.position.x, linkstate.pose.position.y);
}
else
{
  ROS_ERROR("Failed to call service ");
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_transform");
  readrgbdNode mainNode;
  mainNode.subcribeTopics();
  mainNode.advertiseTopics();
  ros::spin();
  return 0;
}