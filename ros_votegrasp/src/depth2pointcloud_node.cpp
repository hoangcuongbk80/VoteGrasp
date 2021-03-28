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
#include <pcl/registration/icp.h>

using namespace cv;
using namespace std;
using namespace Eigen;

cv::Mat depth_img, semantic_img;
string img_name;
double fx, fy, cx, cy, depth_factor;
float ds_factor;
int start_X, start_Y;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr  objects (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr  models (new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Affine3f transform_pc = Eigen::Affine3f::Identity();
Eigen::Vector3f trans;
float rot_euler[3];

void depthToClould()
{
   myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   objects.reset(new pcl::PointCloud<pcl::PointXYZ>);

   pcl::PointXYZRGB point;
   pcl::PointXYZ objpoint;

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
          uchar cls_id = semantic_img.at<uchar>(row, col);
          point.r = 0; point.g = 0; point.b = 0;

          if(cls_id > 0)
          {
            objpoint.x=point.x; objpoint.y=point.y; objpoint.z=point.z;
            int r=5;
            int score = 0;
            for(int i=-r; i <= r; i++)
            for(int j=-r; j <= r; j++)
            {
              int row_i = row+i;
              int col_j = col+j;
              if((row_i) > start_y & (row_i) < end_y)
              if((col_j) > start_x & (col_j) < end_x)
              {
                float d = depth_img.at<cv::Vec3f>(row_i, col_j)[0] / depth_factor;
                if(abs(d-depth) < 0.01)
                {
                  uchar id = semantic_img.at<uchar>(row_i, col_j);
                  if(id==cls_id)
                  {
                    score++;
                  }
                }
              }
            }
            if(score > 8)
            {
              objects->push_back(objpoint);
              point.r = cls_id; point.g = cls_id; point.b = cls_id;
            }
          }
          
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
  pcl::transformPointCloud (*objects, *objects, transform_pc.matrix().inverse());

  transform_pc = Eigen::Affine3f::Identity();
  transform_pc.translation() << trans(0), -trans(1), -trans(2);
  pcl::transformPointCloud (*myCloud, *myCloud, transform_pc);
  pcl::transformPointCloud (*objects, *objects, transform_pc);

  transform_pc = Eigen::Affine3f::Identity();
  transform_pc.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitY()));
  transform_pc.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*myCloud, *myCloud, transform_pc);
  pcl::transformPointCloud (*objects, *objects, transform_pc);
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

void fine_registration()
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(objects);
  icp.setInputTarget(models);
  icp.setMaximumIterations (100);
  icp.setMaxCorrespondenceDistance(0.2);
  icp.setRANSACOutlierRejectionThreshold(1);
  icp.align(*objects);
  Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
  pcl::transformPointCloud(*myCloud, *myCloud, icp_transformation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_to_pointcloud");
  std::cerr << "\n"<< "---------------------Depth To Point Cloud---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n;

  std::string blender_data_dir, save_dir;
  int start_scene, end_scene, start_save_idx;
  bool run_icp=false;
  
  nh_ = ros::NodeHandle("~");
  nh_.getParam("blender_data_dir", blender_data_dir);
  nh_.getParam("save_dir", save_dir);
  nh_.getParam("start_scene", start_scene);
  nh_.getParam("end_scene", end_scene);
  nh_.getParam("start_save_idx", start_save_idx);

  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("depth_factor", depth_factor);
  nh_.getParam("downsample", ds_factor);
  nh_.getParam("start_X", start_X);
  nh_.getParam("start_Y", start_Y);
  nh_.getParam("icp", run_icp);

  int save_idx = start_save_idx;

  for(int s=start_scene; s <= end_scene; s++)
  {
    string scene_dir = blender_data_dir + std::to_string(s) + "/";
    string model_name = scene_dir + "models.ply";
    pcl::io::loadPLYFile<pcl::PointXYZ> (model_name, *models);

    string obb_file = scene_dir + "obbs.txt";
    string grasp_file = scene_dir + "grasps.txt";
    
    string camera_pose_path = scene_dir + "camera_pose.txt";
    ifstream camera_pose_file (camera_pose_path);
    if (camera_pose_file.is_open())                     
      {
        string line;
        getline (camera_pose_file, line);
        while (!camera_pose_file.eof())                 
        {
          getline (camera_pose_file, line);
          if(!read_cam_pose(line)) continue;
          string depth_path, semantic_path;
          depth_path = scene_dir + "depth/" + img_name;
          int pos = img_name.find("."); 
          semantic_path = scene_dir + "semantic/" + img_name.substr(0, pos) + ".png";
          std::cerr << "Read: " << depth_path << "\n";
          depth_img = cv::imread(depth_path, -1);
          std::cerr << "Read: " << semantic_path << "\n";
          semantic_img = cv::imread(semantic_path, -1);
          std::cerr << "depthToClould\n";
          depthToClould();
          std::cerr << "transform_pointcloud\n";
          transform_pointcloud();
          //std::cerr << "downsampling\n";
          //downsampling();
          if(run_icp) 
          {
            std::cerr << "fine_registration\n";
            fine_registration();
          }
          
          // Save transfered point cloud, bouding boxes and grasps
          string writePath = save_dir + "pointcloud/" + std::to_string(save_idx) + ".ply";
          std::cerr << "Write: " << writePath << "\n";
          pcl::io::savePLYFileBinary(writePath, *myCloud);

          string dest_file = save_dir + "obb/" + std::to_string(save_idx) + ".txt";
          std::ofstream obb_dest( dest_file, std::ios::binary );
          std::ifstream obb_src(obb_file, std::ios::binary);  
          obb_dest << obb_src.rdbuf() ;

          dest_file = save_dir + "grasp/" + std::to_string(save_idx) + ".txt";
          std::ofstream grasp_dest( dest_file, std::ios::binary );
          std::ifstream grasp_src(grasp_file, std::ios::binary);  
          grasp_dest << grasp_src.rdbuf() ;
          
          save_idx++;
        }
      }
      else 
      {
        std::cerr << "Unable to open " << camera_pose_path  << " file" << "\n";
        exit(0);
      }
  }

  
  return 0;
}