#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

using namespace std;

std::string model_dir, object_pose_path, grasp_save_path, model_save_path;
std::ofstream grasp_save_file; 

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
Eigen::Vector3f trans;
float obj_rot[3];

void convert_grasp(string grasp_path)
{
  std::cerr << grasp_path << "\n";
  ifstream grasp_file (grasp_path);
  if (grasp_file.is_open())                     
  {
    string line;
    getline (grasp_file, line);
    while (!grasp_file.eof())                 
    {
      getline (grasp_file, line);
      vector<string> st;
      boost::trim(line);
      boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
      if(st.size() < 7) continue;
      std::cerr << line << "\n";

      Eigen::Vector3f new_tran;
      float new_rot[3];
      new_tran(0) = std::stof(st[1]);
      new_tran(1) = std::stof(st[2]); 
      new_tran(2) = std::stof(st[3]); //translaton
      new_tran = transform_pc*new_tran;
      new_rot[2] = std::stof(st[6]) - obj_rot[2]*180/M_PI; //rotation z angle
      if(new_rot[2] > 180) new_rot[2] = new_rot[2] - 360;
      if(new_rot[2] < -180) new_rot[2] = 360 + new_rot[2]; 

      grasp_save_file << st[0] << " ";
      grasp_save_file << new_tran(0) << " " << new_tran(1) << " " << new_tran(2) << " ";
      grasp_save_file << 180 << " " << 0 << " " << new_rot[2] << " ";
      grasp_save_file << st[7] << " " << st[8] << "\n";
      std::cerr << "save grasp: " << st[0] << "\n";
    }
  }
  else 
  {
    std::cerr << "Unable to open " << object_pose_path  << " file" << "\n";
    exit(0);
  }
}

void read_object_pose(std::string line)
{
  transform_pc = Eigen::Affine3f::Identity();  
  vector<string> st;
  boost::trim(line);
  boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
  if(st.size() < 7) return;
 
  trans(0) = std::stof(st[1]); trans(1) = std::stof(st[2]); trans(2) = std::stof(st[3]); //translaton
  obj_rot[0] = std::stof(st[4]); obj_rot[1] = std::stof(st[5]); obj_rot[2] = std::stof(st[6]); //euler angles

  transform_pc.translation() << trans(0), trans(1), trans(2);
  transform_pc.rotate (Eigen::AngleAxisf (obj_rot[0], Eigen::Vector3f::UnitX()));
  transform_pc.rotate (Eigen::AngleAxisf (obj_rot[1], Eigen::Vector3f::UnitY()));
  transform_pc.rotate (Eigen::AngleAxisf (obj_rot[2], Eigen::Vector3f::UnitZ()));


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
  string object_name = model_dir + st[0] + "/nontextured.ply";
  string grasp_file = model_dir + st[0] + "/grasp_euler.txt";
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (object_name, *object_pc);

  pcl::transformPointCloud(*object_pc, *object_pc, transform_pc);
  *myCloud += *object_pc;
  convert_grasp(grasp_file);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PointCloud_Visualization");
  std::cerr << "\n"<< "---------------------Point Cloud Visualization---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("my_models", 1);
  ros::Rate loop_rate(10);

  nh_ = ros::NodeHandle("~");
  nh_.getParam("model_dir", model_dir);
  nh_.getParam("object_pose_path", object_pose_path);
  nh_.getParam("grasp_save_path", grasp_save_path);  
  nh_.getParam("model_save_path", model_save_path);   
  
  ifstream object_pose_file (object_pose_path); 
  grasp_save_file.open(grasp_save_path);
  grasp_save_file << "object x y z rx ry rz quality dofValue\n";
  std::cerr << "grasp_save_path:" << grasp_save_path << "\n";

  if (object_pose_file.is_open())                     
    {
      string line;
      getline (object_pose_file, line);
      while (!object_pose_file.eof())                 
      {
        getline (object_pose_file, line);
        read_object_pose(line);
      }
    }
    else 
    {
      std::cerr << "Unable to open " << object_pose_path  << " file" << "\n";
      exit(0);
    }
    grasp_save_file.close();
    
    pcl::PCLPointCloud2 cloud_filtered;
    sensor_msgs::PointCloud2 output;
    myCloud->header.frame_id = "camera_depth_optical_frame";
    pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
    pcl_conversions::fromPCL(cloud_filtered, output);
    
    pcl::io::savePLYFileBinary(model_save_path, *myCloud);

  while (ros::ok())
  {  
    cloud_pub.publish (output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}