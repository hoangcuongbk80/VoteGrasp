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
#include <pcl/common/common.h>


using namespace std;

std::string model_dir, blender_data_dir;
std::string object_pose_path, grasp_save_path, model_save_path, obb_save_path;
std::ofstream grasp_save_file, obb_save_file; 

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
Eigen::Vector3f trans;
float obj_rot[3];
int start_scene, end_scene;

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
      //std::cerr << line << "\n";

      Eigen::Vector3f new_tran;
      float new_rot[3];
      new_tran(0) = std::stof(st[1]);
      new_tran(1) = std::stof(st[2]); 
      new_tran(2) = std::stof(st[3]); //translaton
      new_tran = transform_pc*new_tran;
      new_rot[2] = std::stof(st[6]) - obj_rot[2]*180/M_PI; // angle
      if(new_rot[2] > 180) new_rot[2] = new_rot[2] - 360;
      if(new_rot[2] < -180) new_rot[2] = 360 + new_rot[2];
      if(new_rot[2] < 0) new_rot[2] = 180 + new_rot[2];

      new_rot[0] = std::stof(st[4]) - obj_rot[0]*180/M_PI;
      new_rot[1] = std::stof(st[5]) - obj_rot[1]*180/M_PI;
      int viewpoint = (new_rot[0]/30)*(new_rot[1]/30)+(new_rot[1]/30);

      grasp_save_file << st[0] << " "; // object name
      grasp_save_file << new_tran(0) << " " << new_tran(1) << " " << new_tran(2) << " "; //grasp center
      grasp_save_file << viewpoint << " " << (int)new_rot[2] << " "; // viewpoint and angle
      grasp_save_file << st[7] << " " << st[8] << "\n";
      //std::cerr << "save grasp: " << st[0] << "\n";
    }
  }
  else 
  {
    std::cerr << "Unable to open " << object_pose_path  << " file" << "\n";
    exit(0);
  }
}

int OBB_Estimation()
{
    if(!object_pc->size())
    {
      std::cerr << "cloud is empty!" << "\n";
      return 0;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr OBB (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZRGB point;
    
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*object_pc, pcaCentroid);
    obb_save_file << pcaCentroid[0] << " " << pcaCentroid[1] << " " << pcaCentroid[2] << " ";

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*object_pc, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*object_pc, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    float lengthOBB[3];
    lengthOBB[0] = fabs(maxPoint.x - minPoint.x) / 2.0; //MAX length OBB
    lengthOBB[1] = fabs(maxPoint.y - minPoint.y) / 2.0; //MID length OBB
    lengthOBB[2] = fabs(maxPoint.z - minPoint.z) / 2.0; //MIN length OBB

    if(lengthOBB[0] < lengthOBB[1])
    {
        float buf = lengthOBB[0]; lengthOBB[0] = lengthOBB[1]; lengthOBB[1] = buf;
    }
    if(lengthOBB[0] < lengthOBB[2])
    {
        float buf = lengthOBB[0]; lengthOBB[0] = lengthOBB[2]; lengthOBB[2] = buf;
    }
    if(lengthOBB[1] < lengthOBB[2])
    {
        float buf = lengthOBB[1]; lengthOBB[1] = lengthOBB[2]; lengthOBB[2] = buf;
    }

    std::cerr << "OBB length: " << lengthOBB[0] << " " << lengthOBB[1] << " " << lengthOBB[2] << "\n";
    obb_save_file << lengthOBB[0] << " " << lengthOBB[1] << " " << lengthOBB[2] << " ";
    obb_save_file << obj_rot[2] << "\n";

    return 1;
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

  obb_save_file << st[0] << " ";
  string object_name = model_dir + st[0] + "/nontextured.ply";
  string grasp_file = model_dir + st[0] + "/grasp_euler.txt";
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (object_name, *object_pc);

  pcl::transformPointCloud(*object_pc, *object_pc, transform_pc);
  *myCloud += *object_pc;
  convert_grasp(grasp_file);
  OBB_Estimation();
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
  nh_.getParam("blender_data_dir", blender_data_dir);
  nh_.getParam("start_scene", start_scene);
  nh_.getParam("end_scene", end_scene);

  for(int s=start_scene; s <= end_scene; s++)
  {
    if(myCloud->size()) myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    object_pose_path = blender_data_dir + std::to_string(s) + "/" + "object_pose.txt";
    grasp_save_path = blender_data_dir + std::to_string(s) + "/" + "grasps.txt";
    model_save_path = blender_data_dir + std::to_string(s) + "/" + "models.ply";
    obb_save_path = blender_data_dir + std::to_string(s) + "/" + "obbs.txt";
  
    grasp_save_file.open(grasp_save_path);
    grasp_save_file << "object x y z viewpoint angle quality width\n";
    std::cerr << "grasp_save_path:" << grasp_save_path << "\n";

    obb_save_file.open(obb_save_path);
    obb_save_file << "object cx cy cz max_length mid_length min_length Rz(rad)\n";
    std::cerr << "obb_save_path:" << obb_save_path << "\n";

    ifstream object_pose_file (object_pose_path); 
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
    obb_save_file.close();
    pcl::io::savePLYFileBinary(model_save_path, *myCloud);
  }
  
  std::cerr << "\nTask done!\n";
  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  myCloud->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  

  while (ros::ok())
  {  
    cloud_pub.publish (output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}