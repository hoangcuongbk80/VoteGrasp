#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

using namespace cv;
using namespace std;

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

    bool image_save;
    std::string depth_topsub, rgb_topsub, cloud_topsub, cloud_toppub;
    std::string saved_rgb_dir, saved_depth_dir, saved_points_dir;
    std::string optical_frame_id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud;
    double fx, fy, cx, cy, depth_factor;
    double table_height;

    tf::TransformListener tf_listener;

  private:
   ros::NodeHandle nh_, nh_rgb, nh_depth, nh_cloud;
   ros::Subscriber depth_sub, rgb_sub, cloud_sub;
   ros::Publisher cloud_pub;
};

readrgbdNode::readrgbdNode()
{
  nh_ = ros::NodeHandle("~");
  nh_rgb = ros::NodeHandle("~");
  nh_depth = ros::NodeHandle("~");
  nh_cloud = ros::NodeHandle("~");

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


  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("depth_factor", depth_factor);

  std::cerr << "depth topics sub: " << "\n" << depth_topsub << "\n";
  std::cerr << "rgb topics sub: " << "\n" << rgb_topsub << "\n";
  std::cerr << "cloud topics sub: " << "\n" << cloud_topsub << "\n";
  std::cerr << "save depth to: " << "\n" << saved_depth_dir << "\n";
  std::cerr << "save rgb to: " << "\n" << saved_rgb_dir << "\n";
  std::cerr << "save points to: " << "\n" << saved_points_dir << "\n";

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

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
  depth_img.convertTo(depth_img, CV_16UC1, 1000.0);
  //cv::imshow("depth", depth_img);
  //cv::waitKey(3);
  if(image_save) cv::imwrite( saved_depth_dir, depth_img );
  depthToClould(depth_img);
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
