#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace cv;
using namespace std;
using namespace Eigen;

cv::Mat depth_img;
string img_name;
double fx, fy, cx, cy, depth_factor;
float ds_factor;
int start_X, start_Y;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
Eigen::Vector3f trans;
float rot_euler[3];

void depthToClould()
{
   myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   int start_x = start_X;
   int start_y = start_Y;
   int end_x = depth_img.cols - start_X; 
   int end_y = depth_img.rows - start_Y;

   for(int row=start_y; row < end_y; row++)
    {
       for(int col=start_x; col < end_x; col++)       
        {
          cv::Vec3f pixelColor = depth_img.at<cv::Vec3f>(row, col);
          float depth = pixelColor.val[0] / depth_factor;
          point.x = (col-cx) * depth / fx;
          point.y = (row-cy) * depth / fy;
          point.z = depth;
          myCloud->push_back(point);
        }
    }
}

bool read_cam_pose(std::string line)
{
  vector<string> st;
  boost::trim(line);
  boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
  if(st.size() < 7) return false;

  img_name = st[0];
  trans(0) = std::stof(st[1]); trans(1) = std::stof(st[2]); trans(2) = std::stof(st[3]); //translation
  rot_euler[0] = std::stof(st[4]); rot_euler[1] = std::stof(st[5]); rot_euler[2] = std::stof(st[6]); //rotation

  return true;
}

void transform_pointcloud()
{
  transform_pc = Eigen::Affine3f::Identity();
  transform_pc.rotate (Eigen::AngleAxisf (-rot_euler[0], Eigen::Vector3f::UnitX()));
  transform_pc.rotate (Eigen::AngleAxisf (rot_euler[1], Eigen::Vector3f::UnitY()));
  transform_pc.rotate (Eigen::AngleAxisf (rot_euler[2], Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*myCloud, *myCloud, transform_pc.matrix().inverse());

  transform_pc = Eigen::Affine3f::Identity();
  transform_pc.translation() << trans(0), -trans(1), -trans(2);
  pcl::transformPointCloud (*myCloud, *myCloud, transform_pc);

  transform_pc = Eigen::Affine3f::Identity();
  transform_pc.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitY()));
  transform_pc.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*myCloud, *myCloud, transform_pc);
}

void downsampling()
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (myCloud);
  sor.setLeafSize (ds_factor, ds_factor, ds_factor);
  sor.filter (*myCloud);
  std::cerr << "Num Points after downsanpling: " << myCloud->size() << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_to_pointcloud");
  std::cerr << "\n"<< "---------------------Depth To Point Cloud---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n;

  std::string camera_pose_path;
  std::string data_path;
  
  nh_ = ros::NodeHandle("~");
  nh_.getParam("camera_pose_path", camera_pose_path);
  nh_.getParam("data_path", data_path);

  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("depth_factor", depth_factor);
  nh_.getParam("downsample", ds_factor);
  nh_.getParam("start_X", start_X);
  nh_.getParam("start_Y", start_Y);

  ifstream camera_pose_file (camera_pose_path);
  if (camera_pose_file.is_open())                     
    {
      string line;
      getline (camera_pose_file, line);
      while (!camera_pose_file.eof())                 
      {
        getline (camera_pose_file, line);
        if(!read_cam_pose(line)) continue;
        string img_path;
        img_path = data_path + "depth/" + img_name;
        std::cerr << "Read: " << img_path << "\n";
        depth_img = cv::imread(img_path, -1);
        depthToClould();
        transform_pointcloud();
        downsampling();
        int pos = img_name.find("."); 
        string writePath = data_path + "pointcloud/" + img_name.substr(0, pos) + ".ply" ;
        std::cerr << "Write: " << writePath << "\n";
        pcl::io::savePLYFileBinary(writePath, *myCloud);
      }
    }
    else 
    {
      std::cerr << "Unable to open " << camera_pose_path  << " file" << "\n";
      exit(0);
    }
  return 0;
}