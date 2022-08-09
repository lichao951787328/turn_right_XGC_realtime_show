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
#include <cv_bridge/cv_bridge.h>
#include "FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h"
#include <Heuclid/geometry/Pose2D.h>
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp> 
using namespace std;
// 记录相机深度数据和imu数据
// std::mutex m, m_tcpip, m_ros;
// Eigen::Matrix4d mark_pose;
// int sock_fd,client_fd;
// sensor_msgs::Image image;
ros::Publisher image_pub;
cv::Mat image;

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

auto getT(const float &px, const float &py, const float &pz, const float &rx, const float &ry, const float &rz)
{
  using namespace Eigen;
  Matrix4f res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisf(rz, Vector3f::UnitZ())*AngleAxisf(ry, Vector3f::UnitY())*AngleAxisf(rx, Vector3f::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

auto _deg2rad(float degree)
{
  float rad = degree/57.3;
  return rad;
}

std::pair<size_t, size_t> Word2Pixel(Eigen::Vector3f & p)
{
  // 世界坐标系到相机坐标系的转换矩阵
  float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
  float BaseVisionX = 0.15995;
  float BaseVisionY = 0.0;
  float BaseVisionPitchDeg = 27.5;
  Eigen::Matrix<float,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
  Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
  Eigen::Matrix3f Base_R_VisionTemp;
  Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
  Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3f::UnitX())).matrix();
  // // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
  // // 这来自控制端发送的数据
  World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
  Vision_T_Tar.setIdentity();
  World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;

  Eigen::Vector4f p_I; p_I.head(3) = p; p_I(3) = 1;
  Eigen::Vector3f point_vision = (World_T_Tar*p_I).head(3);
  // const float kFx = 902.9238891601562;  const float kFy = 902.9104614257812;  const float kCx = 651.3723754882812;  const float kCy = 370.8451232910156;
  const double kFx = 460.2265625;
  const double kFy = 460.25;
  const double kCx = 325.44140625;
  const double kCy = 236.3984375;
  // // 相机坐标系转换为像素坐标系
  // 像素，以像素中点为原点
  size_t j = point_vision.x() * kFx/point_vision.z() + kCx;// 列
  size_t i = point_vision.y() * kFy/point_vision.z() + kCy;// 行
  // 将其转换为以相机左上角的点为原点
  return std::make_pair(i, j);
}

std::vector<std::pair<size_t, size_t>> PointsWord2Pixel(std::vector<Eigen::Vector3f> ps)
{
  std::vector<std::pair<size_t, size_t>> return_pixels;
  for (auto & iter_point : ps)
  {
    return_pixels.emplace_back(Word2Pixel(iter_point));
  }
  return return_pixels;
}

void draw_rect(vector<Eigen::Vector3f> rectPoints)
{
  std::cout<<"draw rect"<<std::endl;
  std::vector<std::pair<size_t, size_t>> pixels = PointsWord2Pixel(rectPoints);
  std::vector<cv::Point> fillContSingle;
  //add all points of the contour to the vector
  for (auto & iter_pixel : pixels)
  {
    std::cout<<iter_pixel.first<<" "<<iter_pixel.second<<std::endl;
    // fillContSingle.push_back(cv::Point(iter_pixel.first, iter_pixel.second));
    // cv::Point 列 行
    fillContSingle.push_back(cv::Point(iter_pixel.second, iter_pixel.first));
  }
  std::vector<std::vector<cv::Point> > fillContAll;
  //fill the single contour 
  //(one could add multiple other similar contours to the vector)
  fillContAll.push_back(fillContSingle);
  cv::fillPoly(image, fillContAll, cv::Scalar(128));
}

vector<Eigen::Vector3f> computeStepMarker(footstep& f)
{
  vector<Eigen::Vector3f> return_v;
  vector<Eigen::Vector3f> v;
  v.reserve(4); return_v.reserve(4);
  Eigen::Vector3f v1(0.15, 0.05, 0.0);
  Eigen::Vector3f v2(0.15,  - 0.09, 0.0);
  Eigen::Vector3f v3(- 0.09, - 0.09, 0.0);
  Eigen::Vector3f v4(-0.09, 0.05, 0.0);
  v.emplace_back(v1);
  v.emplace_back(v2);
  v.emplace_back(v3);
  v.emplace_back(v4);
  Eigen::AngleAxisf r_v(f.theta, Eigen::Vector3f(0,0,1));
  for (auto & iter : v)
  {
    return_v.emplace_back(r_v.matrix()*iter + Eigen::Vector3f(f.x, f.y, f.z));
  }
  return return_v;
}

void draw_steps(vector<footstep>& steps)
{
  std::cout<<"enter draw steps"<<std::endl;
  for (auto & iter_step : steps)
  {
    vector<Eigen::Vector3f> step = computeStepMarker(iter_step);
    vector<std::pair<size_t, size_t>> pixels = PointsWord2Pixel(step);
    std::vector<cv::Point> fillContSingle;
    //add all points of the contour to the vector
    for (auto & iter_pixel : pixels)
    {
      std::cout<<iter_pixel.first<<" "<<iter_pixel.second<<std::endl;
      fillContSingle.push_back(cv::Point(iter_pixel.second, iter_pixel.first));
      // fillContSingle.push_back(cv::Point(iter_pixel.first, iter_pixel.second));
    }
    std::vector<std::vector<cv::Point> > fillContAll;
    //fill the single contour 
    //(one could add multiple other similar contours to the vector)
    fillContAll.push_back(fillContSingle);
    cv::fillPoly(image, fillContAll, cv::Scalar(128));
  }
}

void draw_goal(Eigen::Vector3f & start, Eigen::Vector3f & goal)
{
  Eigen::Vector3f arrow_start(goal(0), goal(1), 0.0);
  Eigen::AngleAxisf v_r(goal(2), Eigen::Vector3f::UnitZ());
  Eigen::Vector3f d = v_r.toRotationMatrix()*Eigen::Vector3f::UnitX()*0.2;
  Eigen::Vector3f arrow_end(arrow_start(0) + d(0), arrow_start(1) + d(1), arrow_start(2) + d(2));

  pair<size_t, size_t> arrow_start_pixel = Word2Pixel(arrow_start);
  pair<size_t, size_t> arrow_end_pixel = Word2Pixel(arrow_end);
  std::cout<<"arrow start "<<arrow_start_pixel.first<<" "<<arrow_start_pixel.second<<std::endl;
  std::cout<<"arrow end "<<arrow_end_pixel.first<<" "<<arrow_end_pixel.second<<std::endl;
  cv::arrowedLine(image, cv::Point2d(arrow_start_pixel.second, arrow_start_pixel.first), cv::Point2d(arrow_end_pixel.second, arrow_end_pixel.first), cv::Scalar(128));
  // cv::arrowedLine(image, cv::Point2d(arrow_start_pixel.first, arrow_start_pixel.second), cv::Point2d(arrow_end_pixel.first, arrow_end_pixel.second), cv::Scalar(128));
  ljh::mathlib::Pose2D<double> start_pose(start(0), start(1), start(2));
  ljh::mathlib::Pose2D<double> goal_pose(goal(0), goal(1), goal(2));
  ljh::path::footstep_planner::Simple2DBodyPathHolder gen_path;
  gen_path.initialize(start_pose, goal_pose);
  std::vector<ljh::mathlib::Pose2D<double> > path = gen_path.getWayPointPath();
  std::cout<<"path points size "<<path.size()<<std::endl;
  vector<cv::Point> pixel_points;
  for (auto & iter_point : path)
  {
    Eigen::Vector3f point(iter_point.getPosition().getX(), iter_point.getPosition().getY(), 0.0);
    pair<size_t, size_t> pixel = Word2Pixel(point);
    std::cout<<pixel.first<<" "<<pixel.second<<std::endl;
    pixel_points.emplace_back(cv::Point(pixel.second, pixel.first));
    // pixel_points.emplace_back(cv::Point(pixel.first, pixel.second));
  }
  std::cout<<"push back finish"<<std::endl;
  // std::vector<std::vector<cv::Point> > fillContAll;
  // fillContAll.emplace_back(pixel_points);
  cv::polylines(image, pixel_points, false, cv::Scalar(128));
}

void draw_test()
{
  cv::circle(image, cv::Point(100,100), 3, cv::Scalar(0, 255, 120), -1);
  cv::circle(image, cv::Point(200,200), 3, cv::Scalar(0, 255, 120), -1);
  cv::circle(image, cv::Point(300,300), 3, cv::Scalar(0, 255, 120), -1);
}
// 均位于局部世界坐标系
void publish_info(vector<footstep>& steps, Eigen::Vector3f & start, Eigen::Vector3f & goal, vector<Eigen::Vector3f> rectPoints)
{
  // 画四边形
  draw_rect(rectPoints);
  // // 画起点终点
  draw_goal(start, goal);
  // // // 画步态点
  draw_steps(steps);
  // draw_test();
}


void getImage(const sensor_msgs::Image::ConstPtr& msg)
{
  std::cout<<"get image"<<std::endl;
  std::cout<<"get image"<<std::endl;
  // unique_lock<mutex> g(m_ros, std::defer_lock);
  // g.lock();
  // image = *msg;
  // g.unlock();
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  cv::Mat raw_image = cv_ptr->image;
  std::cout<<"........."<<raw_image.rows<<" "<<raw_image.cols<<std::endl;
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
  std::cout<<"get pu info"<<std::endl;
  cv::resize(raw_image, image, cv::Size(640 ,480), 0, 0, cv::INTER_LINEAR);
  std::cout<<"image rows "<<image.rows<<" cols "<<image.cols<<std::endl;
  publish_info(footsteps, start_pose, goal_pose, rect);
  cv::imshow("show image",image);
  cv::waitKey(0);
  sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  pub_msg->header.frame_id = "pub_camera";
  image_pub.publish(pub_msg);
  std::cout<<"pub image"<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"turn_right_walk");
  ros::NodeHandle nh;
  cout<<"enter main func"<<endl;
  ros::Subscriber sub_image = nh.subscribe("/camera/color/image_raw", 1, getImage);
  image_pub = nh.advertise<sensor_msgs::Image>("image_pub", 1);
  ros::spin();
  return 0;
}