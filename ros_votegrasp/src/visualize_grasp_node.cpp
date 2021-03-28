#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <cmath>

using namespace std;
using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper (new pcl::PointCloud<pcl::PointXYZ>);
visualization_msgs::MarkerArray         multiMarker;
vector<vector<float> >                 grasps;
float dst_gripper2object = 0.07; //only for Euler angle marker
float quality_thresh = 0.0;

void gripper_init(double open_dst)
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
  p0.x = 0; p0.y = 0; p0.z = -0.07;
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
  p4.x = -open_dst/2; p4.y = 0; p4.z = 0.07;
  gripper->push_back(p4);

  pcl::PointXYZ p5;  
  p5.x = open_dst/2; p5.y = 0; p5.z = 0.07;
  gripper->push_back(p5);
}

bool setup_marker_withQuaternion()
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

      line_list.header.frame_id = "camera_depth_optical_frame";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "grasps";
      line_list.id = i;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.position.x = grasps[i][0];
      line_list.pose.position.y = grasps[i][1];
      line_list.pose.position.z = grasps[i][2];
      line_list.pose.orientation.x = grasps[i][3];
      line_list.pose.orientation.y = grasps[i][4];
      line_list.pose.orientation.z = grasps[i][5];
      line_list.pose.orientation.w = grasps[i][6];
      line_list.scale.x = 0.002; line_list.scale.y = 0.002; line_list.scale.z = 0.002;
      line_list.color.r = 1.0f; line_list.color.g = 0.0f; line_list.color.b = 0.0f; line_list.color.a = 1.0;
      
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

      line_list.lifetime = ros::Duration();
      multiMarker.markers.push_back(line_list);
    }
    return true;
}

bool setup_marker_withEuler()
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

      line_list.header.frame_id = "camera_depth_optical_frame";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "grasps";
      line_list.id = i;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.position.x = grasps[i][1];
      line_list.pose.position.y = grasps[i][2];
      line_list.pose.position.z = grasps[i][3] + dst_gripper2object;
      Matrix3f mat;
      mat = AngleAxisf(grasps[i][4]*M_PI/180, Vector3f::UnitX())
          * AngleAxisf(grasps[i][5]*M_PI/180, Vector3f::UnitY())
          * AngleAxisf(grasps[i][6]*M_PI/180, Vector3f::UnitZ());
      Quaternionf q(mat);
      line_list.pose.orientation.x = q.x();
      line_list.pose.orientation.y = q.y();
      line_list.pose.orientation.z = q.z();
      line_list.pose.orientation.w = q.w();
      line_list.scale.x = 0.005; line_list.scale.y = 0.005; line_list.scale.z = 0.005;
      //line_list.color.r = 1.0-grasps[i][7]; line_list.color.g = grasps[i][7]; line_list.color.b = 0.0f; line_list.color.a = 1.0;
      line_list.color.r = 0.8f; line_list.color.g = 0.6f; line_list.color.b = 0.0f; line_list.color.a = 0.8;
      if(grasps[i][7] > 1)
        {line_list.color.r = 1.0f; line_list.color.g = 0.0f; line_list.color.b = 0.0f; line_list.color.a = 1.0;}
      
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
      graspName.header.frame_id = "camera_depth_optical_frame";
      graspName.header.stamp = ros::Time::now();
      graspName.id = i;
      graspName.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      graspName.action = visualization_msgs::Marker::ADD;
      graspName.pose.position.x = grasps[i][1];
      graspName.pose.position.y = grasps[i][2];
      graspName.pose.position.z = grasps[i][3] + dst_gripper2object + 0.105;
      graspName.pose.orientation.x = q.x();
      graspName.pose.orientation.y = q.y();
      graspName.pose.orientation.z = q.z();
      graspName.pose.orientation.w = q.w();
      graspName.text = std::to_string(i+2);
      graspName.scale.x = 0.005; graspName.scale.y = 0.005; graspName.scale.z = 0.005;
      graspName.color.r = 0.0f; graspName.color.g = 1.0f; graspName.color.b = 0.0f; graspName.color.a = 1.0;

      graspName.lifetime = ros::Duration();
      if(grasps[i][7] > quality_thresh)
      {
          //multiMarker.markers.push_back(graspName);
          line_list.lifetime = ros::Duration();
          multiMarker.markers.push_back(line_list);
      }
        
    }
    return true;
}

void read_grasp(std::string grasp_path)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Grasp_Visualization");
  std::cerr << "\n"<< "---------------------Grasp Visualization---------------------" << "\n";
  ros::NodeHandle nh_, grasp_n;
  ros::Publisher grasp_pub = grasp_n.advertise<visualization_msgs::MarkerArray>( "myGrasp", 1);
  ros::Rate loop_rate(10);

  std:string grasp_path;
  int rot_type;
  
  nh_ = ros::NodeHandle("~");
  nh_.getParam("grasp_path", grasp_path);
  nh_.getParam("rot_type", rot_type);
  nh_.getParam("quality_thresh", quality_thresh);
    
  read_grasp(grasp_path);
  if(rot_type==0)
    setup_marker_withQuaternion();
  if(rot_type==1)
    setup_marker_withEuler();

  while (ros::ok())
  {  
    //cloud_pub.publish (output);
    grasp_pub.publish(multiMarker);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}