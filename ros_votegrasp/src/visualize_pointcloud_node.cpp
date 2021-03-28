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
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  objects (new pcl::PointCloud<pcl::PointXYZRGB>);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PointCloud_Visualization");
  std::cerr << "\n"<< "---------------------Point Cloud Visualization---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("my_points", 1);
  ros::Rate loop_rate(10);

  std:string pc_path;
  
  nh_ = ros::NodeHandle("~");
  nh_.getParam("point_cloud_path", pc_path);
  
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (pc_path, *myCloud);
  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid (*myCloud, xyz_centroid);
  std::cerr << xyz_centroid << "\n";
  
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
