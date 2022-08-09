/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <ros/ros.h>
#include <conio.h>
#include <unistd.h>
#include <fstream>
#include <iostream>             // std::cout
#include <future>               // std::async, std::future
#include <chrono>               // std::chrono::milliseconds
#include "turn_right/tcpip_port.h"
#include <thread>
#include <queue>
#include <glog/logging.h>
#include <Eigen/Geometry>
#include "ART/DTrackSDK.hpp"
#include "turn_right/type.h"
// #include "turn_right/matplotlibcpp.h"
#include "turn_right/foot_step_planning_ljh.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h"
#include <Heuclid/geometry/Pose2D.h>
#include <vector>
using namespace std;
// 记录相机深度数据和imu数据
std::mutex m, m_tcpip, m_ros;
Eigen::Matrix4d mark_pose;
int sock_fd,client_fd;
sensor_msgs::Image image;
sensor_msgs::PointCloud2 points;
ros::Publisher steps_pub;
ros::Publisher image_pub;
ros::Publisher points_pub;
ros::Publisher rect_pub;
ros::Publisher goal_pub;
ros::Publisher path_pub;

vector<float> colors_ = {
    51/255.0, 160/255.0, 44/255.0,  //0
    166/255.0, 206/255.0, 227/255.0,
    178/255.0, 223/255.0, 138/255.0,//6
    31/255.0, 120/255.0, 180/255.0,
    251/255.0, 154/255.0, 153/255.0,// 12
    227/255.0, 26/255.0, 28/255.0,
    253/255.0, 191/255.0, 111/255.0,// 18
    106/255.0, 61/255.0, 154/255.0,
    255/255.0, 127/255.0, 0/255.0, // 24
    202/255.0, 178/255.0, 214/255.0,
    1.0, 0.0, 0.0, // red // 30
    0.0, 1.0, 0.0, // green
    0.0, 0.0, 1.0, // blue// 36
    1.0, 1.0, 0.0,
    1.0, 0.0, 1.0, // 42
    0.0, 1.0, 1.0,
    0.5, 1.0, 0.0,
    1.0, 0.5, 0.0,
    0.5, 0.0, 1.0,
    1.0, 0.0, 0.5,
    0.0, 0.5, 1.0,
    0.0, 1.0, 0.5,
    1.0, 0.5, 0.5,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0,
    0.5, 0.5, 1.0,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0};

auto getT(const double &px, const double &py, const double &pz, const double &rx, const double &ry, const double &rz)
{
  using namespace Eigen;
  Matrix4d res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisd(rz, Vector3d::UnitZ())*AngleAxisd(ry, Vector3d::UnitY())*AngleAxisd(rx, Vector3d::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

auto _deg2rad(double degree)
{
  double rad = degree/57.3;
  return rad;
}

void publish_rect(vector<Eigen::Vector3f> rectPoints)
{
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::MarkerArray ma;
  LOG(INFO)<<"publish rect plane: "<<endl;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "rect_plane";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;
  const double r = colors_[3]*255.0;
  const double g = colors_[4]*255.0;
  const double b = colors_[5]*255.0;
  marker.points.reserve(8);
  marker.colors.reserve(8);
  for (size_t j = 0; j < 4; j++)
  {
    point.x = rectPoints.at(j)(0);
    point.y = rectPoints.at(j)(1);
    point.z = rectPoints.at(j)(2);
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);
    point.x = rectPoints.at((j+1)%4)(0);
    point.y = rectPoints.at((j+1)%4)(1);
    point.z = rectPoints.at((j+1)%4)(2);
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);
    marker.frame_locked = true;
  }
  ma.markers.push_back(marker);
  rect_pub.publish(ma);
}

vector<Eigen::Vector3d> computeStepMarker(footstep& f)
{
  vector<Eigen::Vector3d> return_v;
  vector<Eigen::Vector3d> v;
  v.reserve(4); return_v.reserve(4);
  Eigen::Vector3d v1(0.15, 0.05, 0.0);
  Eigen::Vector3d v2(0.15,  - 0.09, 0.0);
  Eigen::Vector3d v3(- 0.09, - 0.09, 0.0);
  Eigen::Vector3d v4(-0.09, 0.05, 0.0);
  v.emplace_back(v1);
  v.emplace_back(v2);
  v.emplace_back(v3);
  v.emplace_back(v4);
  Eigen::AngleAxisd r_v(f.theta, Eigen::Vector3d(0,0,1));
  for (auto & iter : v)
  {
    return_v.emplace_back(r_v.matrix()*iter + Eigen::Vector3d(f.x, f.y, f.z));
  }
  return return_v;
}

void publish_steps(vector<footstep>& steps)
{
  LOG(INFO)<<"enter pub steps"<<endl;
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::MarkerArray ma;
  for (size_t j =1 ; j < steps.size(); j++)
  {
    ma.markers.clear();
    vector<footstep> tmpsteps(steps.begin(), steps.begin() + j);
    for (size_t i = 0; i < tmpsteps.size (); i++)
    {
      // cout<<"step "<<i+1<<endl;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "step_" + std::to_string(i);
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      const double r = colors_[0]*255.0;
      const double g = colors_[1]*255.0;
      const double b = colors_[2]*255.0;

      marker.points.reserve(8);
      marker.colors.reserve(8);
      // 偏离12cm
      // 脚宽12cm
      // cout<<"compute points"<<endl;
      vector<Eigen::Vector3d> step = computeStepMarker(tmpsteps.at(i));
      // cout<<"load points"<<endl;
      for (size_t j = 0; j < step.size(); j++)
      {
        point.x = step[j](0);
        point.y = step[j](1);
        point.z = step[j](2);
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);

        point.x = step[(j+1)%step.size()](0);
        point.y = step[(j+1)%step.size()](1);
        point.z = step[(j+1)%step.size()](2);
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);
      }
      // cout<<"ok"<<endl;
      marker.frame_locked = true;
      ma.markers.push_back(marker);
    }
    steps_pub.publish(ma);
    usleep(300000);
  }
}

void publish_goal(Eigen::Vector3f & start, Eigen::Vector3f & goal)
{
  LOG(INFO)<<"enter pub path"<<endl;
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker_start;
  marker_start.header.frame_id = "world";
  marker_start.header.stamp = ros::Time::now();
  marker_start.ns = "start_point";
  marker_start.id = 0;
  marker_start.type = visualization_msgs::Marker::POINTS;
  marker_start.action = visualization_msgs::Marker::ADD;
  marker_start.pose.position.x = 0;
  marker_start.pose.position.y = 0;
  marker_start.pose.position.z = 0;
  marker_start.pose.orientation.x = 0.0;
  marker_start.pose.orientation.y = 0.0;
  marker_start.pose.orientation.z = 0.0;
  marker_start.pose.orientation.w = 1.0;
  marker_start.scale.x = 0.05;
  marker_start.scale.y = 0.05;
  marker_start.scale.z = 0.05;
  marker_start.color.r = colors_[6]*255.0;
  marker_start.color.g = colors_[7]*255.0;
  marker_start.color.b = colors_[8]*255.0;
  marker_start.color.a = 1.0;
  geometry_msgs::Point p1;
  p1.x = start(0); p1.y = start(1); p1.z = 0.0;
  marker_start.points.emplace_back(p1);
  ma.markers.emplace_back(marker_start);

  visualization_msgs::Marker mark_goal;
  mark_goal.header.frame_id = "world";
  mark_goal.header.stamp = ros::Time::now();
  mark_goal.ns = "goal_point";
  mark_goal.id = 0;
  mark_goal.type = visualization_msgs::Marker::ARROW;
  mark_goal.action = visualization_msgs::Marker::ADD;
  mark_goal.pose.position.x = 0;
  mark_goal.pose.position.y = 0;
  mark_goal.pose.position.z = 0;
  mark_goal.pose.orientation.x = 0.0;
  mark_goal.pose.orientation.y = 0.0;
  mark_goal.pose.orientation.z = 0.0;
  mark_goal.pose.orientation.w = 1.0;
  mark_goal.scale.x = 0.05;
  mark_goal.scale.y = 0.05;
  mark_goal.scale.z = 0.05;
  mark_goal.color.r = colors_[6]*255.0;
  mark_goal.color.g = colors_[7]*255.0;
  mark_goal.color.b = colors_[8]*255.0;
  mark_goal.color.a = 1.0;
  geometry_msgs::Point goal_p1;
  goal_p1.x = goal(0); goal_p1.y = goal(1); goal_p1.z = 0.0;
  std_msgs::ColorRGBA point_color_goal;
  point_color_goal.r = colors_[6]*255.0;
  point_color_goal.g = colors_[7]*255.0;
  point_color_goal.b = colors_[8]*255.0;
  point_color_goal.a = 1.0;
  mark_goal.colors.emplace_back(point_color_goal);
  mark_goal.points.emplace_back(goal_p1);
  Eigen::AngleAxisf v_r(goal(2), Eigen::Vector3f::UnitZ());
  Eigen::Vector3f d = v_r.toRotationMatrix()*Eigen::Vector3f::UnitX()*0.2;
  geometry_msgs::Point goal_p2;
  goal_p2.x = goal(0) + d(0); goal_p2.y = goal(1) + d(1); goal_p2.z = 0.0 + d(2);
  mark_goal.colors.emplace_back(point_color_goal);
  mark_goal.points.emplace_back(goal_p2);
  ma.markers.emplace_back(mark_goal);
  goal_pub.publish(ma);
  std::cout<<"pub start and goal"<<std::endl;

  ljh::mathlib::Pose2D<double> start_pose(start(0), start(1), start(2));
  ljh::mathlib::Pose2D<double> goal_pose(goal(0), goal(1), goal(2));
  ljh::path::footstep_planner::Simple2DBodyPathHolder gen_path;
  gen_path.initialize(start_pose, goal_pose);
  std::vector<ljh::mathlib::Pose2D<double> > path = gen_path.getWayPointPath();
  std::cout<<"path points size "<<path.size()<<std::endl;
  visualization_msgs::Marker path_points;
  path_points.header.frame_id = "world";
  path_points.header.stamp = ros::Time::now();
  path_points.ns = "path";
  path_points.id = 0;
  path_points.type = visualization_msgs::Marker::LINE_LIST;
  path_points.action = visualization_msgs::Marker::ADD;
  path_points.pose.position.x = 0;
  path_points.pose.position.y = 0;
  path_points.pose.position.z = 0;
  path_points.pose.orientation.x = 0.0;
  path_points.pose.orientation.y = 0.0;
  path_points.pose.orientation.z = 0.0;
  path_points.pose.orientation.w = 1.0;
  path_points.scale.x = 0.03;
  path_points.scale.y = 0.03;
  path_points.scale.z = 0.03;
  path_points.color.r = colors_[9]*255.0;
  path_points.color.g = colors_[10]*255.0;
  path_points.color.b = colors_[11]*255.0;
  path_points.color.a = 1.0;
  geometry_msgs::Point tmpPoint;
  std_msgs::ColorRGBA point_color;
  
  for (size_t i = 1; i < path.size(); i++)
  {
    std::vector<ljh::mathlib::Pose2D<double> > tmppath(path.begin(), path.begin() + i);
    path_points.colors.clear();
    path_points.points.clear();
    for (size_t i = 0; i < tmppath.size() - 1; i++)
    {
      tmpPoint.x = path.at(i).getPosition().getX();
      tmpPoint.y = path.at(i).getPosition().getY();
      tmpPoint.z = 0,0;
      point_color.r = colors_[9]*255.0;
      point_color.g = colors_[10]*255.0;
      point_color.b = colors_[11]*255.0; 
      path_points.colors.push_back(point_color);
      path_points.points.push_back(tmpPoint);

      tmpPoint.x = path.at(i+1).getPosition().getX();
      tmpPoint.y = path.at(i+1).getPosition().getY();
      tmpPoint.z = 0,0;
      point_color.r = colors_[9]*255.0;
      point_color.g = colors_[10]*255.0;
      point_color.b = colors_[11]*255.0; 
      path_points.colors.push_back(point_color);
      path_points.points.push_back(tmpPoint);
    }
    path_pub.publish(path_points);
    usleep(10000);
  }
}

// 均位于局部世界坐标系
void publish_info(vector<footstep>& steps, Eigen::Vector3f & start, Eigen::Vector3f & goal, vector<Eigen::Vector3f> rectPoints)
{
  // 发布图像
  unique_lock<mutex> g(m_ros, std::defer_lock);
  g.lock();
  image_pub.publish(image);
  g.unlock();
  // 画四边形
  publish_rect(rectPoints);
  // // 画起点终点
  publish_goal(start, goal);
  // // 画步态点
  publish_steps(steps);
}

// goal pose: 0.601 -0.662 -1.634
// point 1:  1.087 -0.893
// point 2:  0.089 -0.830
// point 3:  0.076 -1.029
// point 4:  1.074 -1.092
// 0 0.0409836 -0.142715 -0.1571
// 1 0.0660164 0.0153148 -0.1571
// 0 0.0822756 -0.203484 -0.3142
// 1 0.131724 -0.0513165 -0.3142
// 0 0.124176 -0.262378 -0.4713
// 1 0.196824 -0.119822 -0.4713
// 0 0.166972 -0.319518 -0.6284
// 1 0.261028 -0.190082 -0.6284
// 0 0.210926 -0.375063 -0.7855
// 1 0.324074 -0.261937 -0.7855
// 0 0.256273 -0.429215 -0.9426
// 1 0.385727 -0.335185 -0.9426
// 0 0.303214 -0.482209 -1.0997
// 1 0.445786 -0.409591 -1.0997
// 0 0.351911 -0.534309 -1.2568
// 1 0.504089 -0.484891 -1.2568
// 0 0.402483 -0.5858 -1.4139
// 1 0.560517 -0.5608 -1.4139
// 0 0.455 -0.636984 -1.571
// 1 0.615 -0.637016 -1.571
void getImage(const sensor_msgs::Image::ConstPtr& msg)
{
  std::cout<<"get image"<<std::endl;
  unique_lock<mutex> g(m_ros, std::defer_lock);
  g.lock();
  image = *msg;
  g.unlock();
  Eigen::Vector3f start_pose(0.015, 0.0 , 0.0);
  Eigen::Vector3f goal_pose(0.601, -0.662 , -1.634);
  Eigen::Vector3f p1(1.087, -0.893, 0.0);
  Eigen::Vector3f p2(0.089, -0.830, 0.0);
  Eigen::Vector3f p3(0.076, -1.029, 0.0);
  Eigen::Vector3f p4(1.074, -1.092, 0.0);
  vector<Eigen::Vector3f> rect; 
  rect.emplace_back(p1);
  rect.emplace_back(p2);
  rect.emplace_back(p3);
  rect.emplace_back(p4);
  footstep f1(0, 0.0409836, -0.142715, -0.1571);
  footstep f2(1, 0.0660164, 0.0153148, -0.1571);
  footstep f3(0, 0.0822756, -0.203484, -0.3142);
  footstep f4(1, 0.131724, -0.0513165, -0.3142);
  footstep f5(0, 0.124176, -0.262378, -0.4713);
  footstep f6(1, 0.196824, -0.119822, -0.4713);
  vector<footstep> footsteps;
  footsteps.emplace_back(f1);
  footsteps.emplace_back(f2);
  footsteps.emplace_back(f3);
  footsteps.emplace_back(f4);
  footsteps.emplace_back(f5);
  footsteps.emplace_back(f6);
  publish_info(footsteps, start_pose, goal_pose, rect);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"turn_right_walk");
  ros::NodeHandle nh;
  cout<<"enter main func"<<endl;
  ros::Subscriber sub_image = nh.subscribe("/camera/color/image_raw", 1, getImage);
  image_pub = nh.advertise<sensor_msgs::Image>("image_pub", 1);
  rect_pub = nh.advertise<visualization_msgs::MarkerArray>("rect_plane", 1);
  steps_pub = nh.advertise<visualization_msgs::MarkerArray>("steps_pub", 1);
  points_pub = nh.advertise<sensor_msgs::PointCloud2>("points_pub", 1);
  goal_pub = nh.advertise<visualization_msgs::MarkerArray>("start_goal", 1);
  path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // 这两个参数不对，对应的老版本链接件
    double BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
    double BaseVisionX = 0.15995;
    double BaseVisionY = 0.0;
    double BaseVisionPitchDeg = 27.5;
    Eigen::Matrix<double,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
    Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
    Eigen::Matrix3d Base_R_VisionTemp;
    Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
    Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisd(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3d::UnitX())).matrix();
    // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
    // 这来自控制端发送的数据
    World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
    Vision_T_Tar.setIdentity();
    World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
    tf::Vector3 t(World_T_Tar(0, 3), World_T_Tar(1, 3), World_T_Tar(2, 3));
    Eigen::Quaterniond qd; qd = World_T_Tar.block<3,3>(0,0);
    transform.setOrigin(t);
    double qqx = qd.x();
    double qqy = qd.y();
    double qqz = qd.z();
    double qqw = qd.w();
    tf::Quaternion q(qqx, qqy, qqz, qqw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_depth_optical_frame"));
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}