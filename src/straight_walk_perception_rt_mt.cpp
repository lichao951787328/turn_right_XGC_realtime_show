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
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
// #include "FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h"
// #include <Heuclid/geometry/Pose2D.h>
#include <vector>
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp> 
#include <cv_bridge/cv_bridge.h>
using namespace std;
// 记录相机深度数据和imu数据
std::mutex m, m_tcpip, m_ros, m_flag;
// 实时marker相对于art世界坐标系的位姿
Eigen::Matrix4f ARTWORLD_T_MARKER;
int sock_fd,client_fd;

cv::Mat image;// 存储ros的图像
cv::Mat draw_image;// 画图

// bool get_plane_result = false;
// bool in_planning = false;
// bool get_initial_marker = true;
bool pub_flag = false;
bool initial_time = true;
bool has_send_perception = false;
std::vector<Eigen::Vector3f> rectPoints;
ros::Publisher image_pub;//图像发布器
vector<footstep> pub_steps;
ros::Time start_pub;
size_t pub_step_index = 0;
#define PI 3.1415926
// ROBOTWORLD_T_CAMERA 检测下蹲时刻camera相对于机器人世界坐标系的位姿
// ROBOTWORLD_T_MARKER 检测下蹲时刻marker相对于机器人世界坐标系的位姿
// MARKER_T_CAMERA camera相对于marker的位姿
// initial_ARTWORLD_T_MARKER 检测下蹲时刻marker相对于art世界坐标系下的位姿
// initROBOTWORLD_T_CAMERA 实时相机相对于机器人检测时刻世界坐标系位姿
Eigen::Matrix4f ROBOTWORLD_T_CAMERA, ROBOTWORLD_T_MARKER, MARKER_T_CAMERA, initial_ARTWORLD_T_MARKER, initROBOTWORLD_T_CAMERA;
vector<float> colors_ = {
    51, 160, 44,  //0
    166, 206, 227,
    178, 223, 138,//6
    31, 120, 180,
    251, 154, 153,// 12
    227, 26, 28,
    253, 191, 111,// 18
    106, 61, 154,
    255, 127, 0, // 24
    202, 178, 214};

static DTrackSDK* dt = NULL;

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

// 初始化检测时 marker到相机的位姿
void initial_pose()
{
    float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
    // 对于新的工装
    BaseVisionZ -= (77.51 - 12.81);
    float BaseVisionX = 0.15995;
    // 对于新的工装
    BaseVisionX -= (82.4 - 65.17);
    float BaseVisionY = 0.0;
    float BaseVisionPitchDeg = 27.5;
    Eigen::Matrix<float,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
    Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
    Eigen::Matrix3f Base_R_VisionTemp;
    Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
    Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg), Eigen::Vector3f::UnitX())).matrix();
    // // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
    // // 这来自控制端发送的数据
    World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
    Vision_T_Tar.setIdentity();
    World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
    ROBOTWORLD_T_CAMERA = World_T_Tar;

    // 确定mark相对于机器人世界坐标的位姿
    // 115标定数据 
    // Eigen::AngleAxisf z_rot(_deg2rad(-61.85), Eigen::Vector3f::UnitZ());
    // Eigen::AngleAxisf y_rot(_deg2rad(88.4), Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf x_rot(_deg2rad(69.42), Eigen::Vector3f::UnitX());

    // 校工厂标定数据 8月1日
    Eigen::AngleAxisf z_rot(_deg2rad(-60.21), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf y_rot(_deg2rad(86.74), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf x_rot(_deg2rad(68.85), Eigen::Vector3f::UnitX());
    // 绕固定坐标系，左乘

    Eigen::Matrix3f mark_In_word = x_rot.matrix() * y_rot.matrix() * z_rot.matrix();
    // Eigen::Quaterniond q_mark_IN_BW(0.145, 0.952, 0.212, 0.165);

    Eigen::Matrix4f mark_IN_RW = Eigen::Matrix4f::Identity();
    // mark_IN_BW.block<3,3>(0,0) = q_mark_IN_BW.toRotationMatrix();
    mark_IN_RW.block<3,3>(0,0) = mark_In_word;
    // mark_IN_BW(0,3) = -213.656/1000;
    // mark_IN_BW(1,3) = -84.426/1000;
    // mark_IN_BW(2,3) = 1099.334/1000;

    // 115标定的数据 
    // mark_IN_RW(0,3) = 125.85/1000;
    // mark_IN_RW(1,3) = -108.9/1000;
    // mark_IN_RW(2,3) = 1112.94/1000;

    // 校工厂标定数据 8月1日
    mark_IN_RW(0,3) = 98.64/1000;
    mark_IN_RW(1,3) = -120.59/1000;
    mark_IN_RW(2,3) = 1117.35/1000;
    // Eigen::Matrix4d BW_IN_RW = Eigen::Matrix4d::Identity();
    // BW_IN_RW(0, 3) = 0.3;
    // Eigen::Matrix4d mark_IN_RW = mark_IN_BW * BW_IN_RW;
    LOG(INFO)<<"mark_IN_RW: "<<endl<<mark_IN_RW<<endl;
    ROBOTWORLD_T_MARKER = mark_IN_RW;

    MARKER_T_CAMERA = ROBOTWORLD_T_MARKER.inverse() * ROBOTWORLD_T_CAMERA;
}

std::pair<size_t, size_t> Word2Pixel(Eigen::Vector3f & p)
{
  // 世界坐标系到相机坐标系的转换矩阵
//   float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
//   // 对于新的工装
//   BaseVisionZ -= 0.06435;
//   float BaseVisionX = 0.15995;
//   // 对于新的工装
//   BaseVisionX -= 0.00298;
//   float BaseVisionY = 0.0;
//   float BaseVisionPitchDeg = 27.5;
//   Eigen::Matrix<float,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
//   Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
//   Eigen::Matrix3f Base_R_VisionTemp;
//   Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
//   Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3f::UnitX())).matrix();
//   // // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
//   // // 这来自控制端发送的数据
//   World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
//   Vision_T_Tar.setIdentity();
//   World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar；
//   Eigen::Vector4f p_I; p_I.head(3) = p; p_I(3) = 1;
//   Eigen::Vector3f point_vision = (World_T_Tar.inverse()*p_I).head(3);
  Eigen::Vector3f point_vision = p;
  // // 相机坐标系转换为像素坐标系
  const double kFx = 460.2265625;
  const double kFy = 460.25;
  const double kCx = 325.44140625;
  const double kCy = 236.3984375;
  // 像素
  size_t j = point_vision.x() * kFx/point_vision.z() + kCx;
  size_t i = point_vision.y() * kFy/point_vision.z() + kCy;
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

void draw_rect(/* vector<Eigen::Vector3f> rectPoints */)
{
    std::cout<<"draw rect"<<std::endl;
    vector<Eigen::Vector3f> rectPoints_inVersion;
    for (auto & iter_point : rectPoints)
    {
        Eigen::Vector4f point; 
        point.head(3) = iter_point;
        point(3) = 1;
        rectPoints_inVersion.emplace_back((initROBOTWORLD_T_CAMERA.inverse()*point).head(3));
    }
    std::vector<std::pair<size_t, size_t>> pixels = PointsWord2Pixel(rectPoints_inVersion);
    std::vector<cv::Point> fillContSingle;
    //add all points of the contour to the vector
    for (auto & iter_pixel : pixels)
    {
        // std::cout<<iter_pixel.first<<" "<<iter_pixel.second<<std::endl;
        // fillContSingle.push_back(cv::Point(iter_pixel.first, iter_pixel.second));
        fillContSingle.push_back(cv::Point(iter_pixel.second, iter_pixel.first));
    }
    std::vector<std::vector<cv::Point> > fillContAll;
    //fill the single contour 
    //(one could add multiple other similar contours to the vector)
    fillContAll.push_back(fillContSingle);
    cv::polylines(draw_image, fillContAll, true, cv::Scalar(colors_.at(3), colors_.at(4), colors_.at(5)), 5);
    // cv::Mat pub_image;
    // cv::resize(draw_image, pub_image, cv::Size(1280 ,720), 0, 0, cv::INTER_LINEAR);
    // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
    // pub_msg->header.frame_id = "pub_camera";
    // image_pub.publish(pub_msg);
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
    // 点坐标转换
    // 这个点是初始世界坐标系下的点
    // 初始世界坐标系转为相机坐标系
    // maker相对与相机的位姿、当前marker相对于初始世界坐标系下的位姿
    // maker相对与相机的位姿 marker相对于当前世界坐标系的位姿 相机相对于当前世界坐标系的位姿，检测时可以计算得到
    // 当前marker相对于初始世界坐标系下的位姿 当前marker相对于art的位姿 初始marker相对于art的位姿 marker相对于初始世界坐标系下的位
    std::cout<<"enter draw steps"<<std::endl;
    size_t index = 1;
    for (auto & iter_step : steps)
    {
        vector<Eigen::Vector3f> step = computeStepMarker(iter_step);

        vector<Eigen::Vector3f> step_inVersion;
        for (auto & iter_point : step)
        {
            Eigen::Vector4f point; 
            point.head(3) = iter_point;
            point(3) = 1;
            step_inVersion.emplace_back((initROBOTWORLD_T_CAMERA.inverse()*point).head(3));
        }

        vector<std::pair<size_t, size_t>> pixels = PointsWord2Pixel(step_inVersion);
        std::vector<cv::Point> fillContSingle;
        //add all points of the contour to the vector
        for (auto & iter_pixel : pixels)
        {
            // std::cout<<iter_pixel.first<<" "<<iter_pixel.second<<std::endl;
        //   fillContSingle.push_back(cv::Point(iter_pixel.first, iter_pixel.second));
            fillContSingle.push_back(cv::Point(iter_pixel.second, iter_pixel.first));
        
        }
        std::vector<std::vector<cv::Point> > fillContAll;
        //fill the single contour 
        //(one could add multiple other similar contours to the vector)
        fillContAll.push_back(fillContSingle);
        cv::polylines(draw_image, fillContAll, true, cv::Scalar(colors_.at(6), colors_.at(7), colors_.at(8)), 5);
        // Eigen::Vector3f center_w(iter_step.x, iter_step.y, iter_step.z);
        Eigen::Vector4f center_txt(iter_step.x, iter_step.y, iter_step.z, 1);
        Eigen::Vector3f center_txt_pixel = (initROBOTWORLD_T_CAMERA.inverse() * center_txt).head(3);
        std::pair<size_t, size_t> center = Word2Pixel(center_txt_pixel);
        cv::putText(draw_image, std::to_string(index), cv::Point(center.second, center.first), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(colors_.at(9), colors_.at(10), colors_.at(11)), 2);
        index++;
        // cv::Mat pub_image;
        // cv::resize(draw_image, pub_image, cv::Size(1280 ,720), 0, 0, cv::INTER_LINEAR);
        // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
        // pub_msg->header.frame_id = "pub_camera";
        // image_pub.publish(pub_msg);
        // usleep(300000);
    }
}

void draw_goal(Eigen::Vector3f & start, Eigen::Vector3f & goal)
{
  Eigen::Vector3f arrow_start(goal(0), goal(1), 0.0);
  Eigen::AngleAxisf v_r(goal(2), Eigen::Vector3f::UnitZ());
  Eigen::Vector3f d = v_r.toRotationMatrix()*Eigen::Vector3f::UnitX()*0.1;
  Eigen::Vector3f arrow_end(arrow_start(0) + d(0), arrow_start(1) + d(1), arrow_start(2) + d(2));

  pair<size_t, size_t> arrow_start_pixel = Word2Pixel(arrow_start);
  pair<size_t, size_t> arrow_end_pixel = Word2Pixel(arrow_end);
//   std::cout<<"arrow start "<<arrow_start_pixel.first<<" "<<arrow_start_pixel.second<<std::endl;
//   std::cout<<"arrow end "<<arrow_end_pixel.first<<" "<<arrow_end_pixel.second<<std::endl;
//   cv::arrowedLine(draw_image, cv::Point2d(arrow_start_pixel.first, arrow_start_pixel.second), cv::Point2d(arrow_end_pixel.first, arrow_end_pixel.second), cv::Scalar(0, 255, 120));
  cv::arrowedLine(draw_image, cv::Point2d(arrow_start_pixel.second, arrow_start_pixel.first), cv::Point2d(arrow_end_pixel.second, arrow_end_pixel.first), cv::Scalar(0, 0, 255), 3, 8, 0, 0.4);
  cv::Mat pub_image;
  cv::resize(draw_image, pub_image, cv::Size(1280 ,720), 0, 0, cv::INTER_LINEAR);
  sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
  pub_msg->header.frame_id = "pub_camera";
  image_pub.publish(pub_msg);
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
    // std::cout<<pixel.first<<" "<<pixel.second<<std::endl;
    // pixel_points.emplace_back(cv::Point(pixel.first, pixel.second));
    pixel_points.emplace_back(cv::Point(pixel.second, pixel.first));
    usleep(10000);
    cv::polylines(draw_image, pixel_points, false, cv::Scalar(0, 0, 255), 3);
    cv::Mat pub_image;
    cv::resize(draw_image, pub_image, cv::Size(1280 ,720), 0, 0, cv::INTER_LINEAR);
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
    pub_msg->header.frame_id = "pub_camera";
    image_pub.publish(pub_msg);
  }
  std::cout<<"push back finish"<<std::endl;
  // std::vector<std::vector<cv::Point> > fillContAll;
  // fillContAll.emplace_back(pixel_points);
//   cv::polylines(draw_image, pixel_points, false, cv::Scalar(0, 255, 120));
}

void draw_test()
{
  cv::circle(draw_image, cv::Point(100,100), 3, cv::Scalar(0, 255, 120), -1);
  cv::circle(draw_image, cv::Point(200,200), 3, cv::Scalar(0, 255, 120), -1);
  cv::circle(draw_image, cv::Point(300,300), 3, cv::Scalar(0, 255, 120), -1);
}
// 均位于局部世界坐标系
void draw_info(vector<footstep>& steps, Eigen::Vector3f & start, Eigen::Vector3f & goal, vector<Eigen::Vector3f> rectPoints)
{
  // 画四边形
  draw_rect();
  // // 画起点终点
  draw_goal(start, goal);
  // // // 画步态点
  draw_steps(steps);
//   draw_test();
}

void draw(vector<footstep>& steps)
{
    Eigen::Matrix4f MARKER_T_INITWORLD;
    unique_lock<mutex> g(m, std::defer_lock);
    g.lock();
    std::cout<<"refresh matrix"<<std::endl;
    // marker相对于初始世界坐标系的位姿 = 实时marker位姿、初始检测时刻marker相对于世界坐标系的位姿、marker相对于机器人初始世界坐标系的位姿
    MARKER_T_INITWORLD = ARTWORLD_T_MARKER.inverse() * initial_ARTWORLD_T_MARKER * ROBOTWORLD_T_MARKER.inverse();
    g.unlock();
    initROBOTWORLD_T_CAMERA = MARKER_T_INITWORLD.inverse() * MARKER_T_CAMERA;
    // draw_test();
    draw_steps(steps);
    draw_rect();
}

void communicate_PC104()
{
    LOG(INFO)<<"enter communicate PC 104 function"<<endl;
    char currentFlag = 'A';
    char lastFlag = 'A';
    clock_t start, finish;
    double duration;
    tcpip_port com;
    com.initial();
    LOG(INFO)<<"tcp ip port initial finish"<<endl;
    while (1)
    {
        com.accept_client();
        if (com.recvData() == -1)
        {
            printf("recv in server error.");
            exit(1);
        }
        LOG(INFO)<<"recev data"<<endl;
        // cout<<"have recev data"<<endl;
        com.analysisBuf(); 
        // perception.publishWindata(0.0, 0.0);
        lastFlag = currentFlag;
        currentFlag = com.getRecvFlag();
        if (currentFlag == 'C' && initial_time && has_send_perception)
        {   
            LOG(INFO)<<"start walk ......";
            unique_lock<mutex> g_flag(m_flag, std::defer_lock);
            g_flag.lock();
            pub_flag = true;
            has_send_perception = false;
            g_flag.unlock();
            lastFlag = 'A';
            currentFlag = 'A';
        }
        
        // std::cout<<"current letter is "<<currentFlag<<std::endl;
        // com.resetRecvFlag();
        // std::cout<<"the com recv flag is change to "<<com.getRecvFlag()<<std::endl;
        if (lastFlag == 'A' && currentFlag == 'B')
        {
            LOG(INFO)<<"GOOOOOOOOOOO"<<endl;

            // vector<footstep> send_steps;
            // send_steps.reserve(19);
            // ifstream in("../steps.txt");
            // for (size_t i = 0; i < 19; i++)
            // {
            //     footstep tmpstep;
            //     in>>tmpstep.is_left;
            //     in>>tmpstep.x;
            //     in>>tmpstep.y;
            //     in>>tmpstep.z;
            //     in>>tmpstep.theta;
            //     send_steps.emplace_back(tmpstep);
            // }
            // in.close();
            // LOG(INFO)<<"STEP PLANNING RESULTS:"<<endl;
            // for (auto & iter_step : send_steps)
            // {
            //     LOG(INFO)<<iter_step.is_left<<" "<<iter_step.x<<" "<<iter_step.y<<" "<<iter_step.z<<" "<<iter_step.theta<<endl;
            // }
            // struct sendddata
            // {
            //     int n;
            //     footstep steps[25];
            // };
            // assert(send_steps.size() <= 25 && "the plan steps is big than 25, you should turn the senddata buffer more ...");
            // sendddata SENDdata;
            // SENDdata.n = send_steps.size();
            // for (size_t i = 0; i < send_steps.size(); i++)
            // {
            //     SENDdata.steps[i].is_left = send_steps.at(i).is_left;
            //     SENDdata.steps[i].x = send_steps.at(i).x;
            //     SENDdata.steps[i].y = send_steps.at(i).y;
            //     SENDdata.steps[i].z = send_steps.at(i).z;
            //     SENDdata.steps[i].theta = send_steps.at(i).theta;
            // }
            // cout<<"size of sendddata "<<sizeof(sendddata)<<endl;
            // if (com.sendSteps((char*)(&SENDdata), sizeof(sendddata)) == -1)
            // {
            //     perror("send error");
            //     exit(1);
            // }
            // lastFlag = 'A';
            // currentFlag = 'A';
            // continue;
            unique_lock<mutex> g_flag(m_flag, std::defer_lock);
            g_flag.lock();
            has_send_perception = true;
            initial_time = true;
            g_flag.unlock();
            start = clock();
            unique_lock<mutex> g(m, std::defer_lock);
            g.lock();
            initial_ARTWORLD_T_MARKER = ARTWORLD_T_MARKER;
            g.unlock();

            // 转换到aim坐标系求取step ROBOTWORLD_T_MARKER    artWORLD_T_MARKER
            
            Eigen::Matrix4f ROBOTWORLD_T_ARTWORLD = ROBOTWORLD_T_MARKER * initial_ARTWORLD_T_MARKER.inverse();

            // ROBOTWORLD_T_artWORLD = ROBOTWORLD_T_MARKER* artWORLD_T_MARKER.inverse()

            Eigen::Matrix4f AW_IN_RW = ROBOTWORLD_T_ARTWORLD;
            LOG(INFO)<<"AW_IN_RW: "<<endl<<AW_IN_RW<<endl;
            // 台阶朝向
            // Eigen::Vector3d direct_AW = Eigen::Vector3d::UnitX();
            // Eigen::Vector3d direct_RW = AW_IN_RW.block<3,3>(0,0) * direct_AW;
            // LOG(INFO)<<"direct_RW: "<<endl<<direct_RW.transpose()<<endl;
            // assert(direct_RW(0)>0 &&  "direct error");
            // Eigen::Vector2d goal_AW = AW_IN_RW.block<2,1>(0, 3);
            // LOG(INFO)<<"goal_AW: "<<endl<<goal_AW.transpose()<<endl;
            // Eigen::Vector2d direct_2d = direct_RW.head(2).normalized();
            // double dis_AW = goal_AW.dot(direct_2d);
            // LOG(INFO)<<dis_AW<<endl;
            // double dis_walk = dis_AW - 0.15;// 前脚掌
            // assert(dis_walk>0 && "dis error");
            // double theta = acos(abs(direct_2d(0)));
            // LOG(INFO)<<"THETA IS "<<theta * 57.3<<endl;
            // double goal_dis = dis_walk;

            Eigen::Vector2f direct2d = (AW_IN_RW.block<3,3>(0,0) * Eigen::Vector3f::UnitX()).head(2).normalized();
            Eigen::Vector2f goal2d = AW_IN_RW.block<2,1>(0, 3) - direct2d *(0.18 + 0.01) ;// 0.18 前脚掌余量，验证台阶与终点距离 偏置修改需额外在0.18基础上修正 0.01 ensure not touch the step
            double yaw = std::acos(std::abs(direct2d.dot(Eigen::Vector2f::UnitX())));
            if (direct2d(0) > 0)
            {
                if (direct2d(1) < 0)
                {
                    yaw = -yaw;
                }
            }
            else if(direct2d(0) < 0)
            {
                if (direct2d(1) > 0)
                {
                    yaw = PI - yaw;
                }
                else
                {
                    yaw = yaw - PI;
                }
            }
            else
            {
                if (direct2d(1) > 0)
                {
                    yaw = PI/2.0;
                }
                else
                {
                    yaw = - PI/2.0;
                }
            }
            
            Eigen::Vector2f directY = (AW_IN_RW.block<3,3>(0,0) * Eigen::Vector3f::UnitY()).head(2).normalized();
            Eigen::Vector2f ref = goal2d + direct2d * 0.2; // 0.2 前脚掌余量，验证台阶与终点距离
            Eigen::Vector2f p1 = ref + directY * 0.5;
            Eigen::Vector2f p2 = ref - directY * 0.5;
            Eigen::Vector2f p3 = p2 + direct2d * 0.2;
            Eigen::Vector2f p4 = p1 + direct2d * 0.2;

            std::cout<<"goal pose: "<<goal2d(0)<<" "<<goal2d(1)<<" "<<yaw<<std::endl;
            std::cout<<"point 1: "<<p1.transpose()<<std::endl;
            std::cout<<"point 2: "<<p2.transpose()<<std::endl;
            std::cout<<"point 3: "<<p3.transpose()<<std::endl;
            std::cout<<"point 4: "<<p4.transpose()<<std::endl;
            // Eigen::Vector2d goal_ljh = line_point - direct_2d * (0.19 - 0.08);
            std::vector<double> param;
            // double yaw = direct_2d(1) > 0 ? theta : - theta;
            // 0.015是从脚踝坐标系转到base坐标系
            param.emplace_back(goal2d(0) + 0.015);
            param.emplace_back(goal2d(1));
            param.emplace_back(yaw);
            // param.emplace_back(-PI/2.0);

            param.emplace_back(p1.x());
            param.emplace_back(p1.y());
            param.emplace_back(p2.x());
            param.emplace_back(p2.y());
            param.emplace_back(p3.x());
            param.emplace_back(p3.y());
            param.emplace_back(p4.x());
            param.emplace_back(p4.y());
            assert(param.size() == 11);
            // 0.015 是base坐标系是x是0，脚踝相对于base往前了0.015
            vector<std::pair<Eigen::Vector3d, bool> > steps = foot_step_planning(param, 0, 0.015);
            // vector<std::pair<Eigen::Vector3d, bool> > steps;
            vector<footstep> steps_result;
            steps_result.clear();
            vector<std::pair<Eigen::Vector3d, bool> >::iterator iter_foot_step_ljh = steps.begin()+2;
            for (;iter_foot_step_ljh != steps.end(); iter_foot_step_ljh++)
            {
                footstep tmpstep;
                tmpstep.is_left = iter_foot_step_ljh->second;
                tmpstep.x = iter_foot_step_ljh->first(0);
                tmpstep.y = iter_foot_step_ljh->first(1);
                tmpstep.z = 0;
                tmpstep.theta = iter_foot_step_ljh->first(2);
                steps_result.emplace_back(tmpstep);
            }
            pub_steps = steps_result;
            // 输入的起点终点三个坐标点对应的是x, y, yaw
            
            rectPoints.emplace_back(Eigen::Vector3f(p1.x(), p1.y(), 0.0));
            rectPoints.emplace_back(Eigen::Vector3f(p2.x(), p2.y(), 0.0));
            rectPoints.emplace_back(Eigen::Vector3f(p3.x(), p3.y(), 0.0));
            rectPoints.emplace_back(Eigen::Vector3f(p4.x(), p4.y(), 0.0));
            Eigen::Vector3f pub_start(0.015, 0.0, 0.0);
            Eigen::Vector3f pub_goal = Eigen::Vector3f(0.015, 0.0, 0.0) + Eigen::Vector3f(goal2d(0), goal2d(1), yaw);
            
            // return steps_result.size() > 0;
            // 假设为正确的x方向
            // Eigen::Vector4d x_d;
            // x_d.head(3) = Eigen::Vector3d::UnitY();
            // x_d(3) = 1.0;
            // Eigen::Vector3d direct = (World_T_Tar * x_d).head(3);
            // LOG(INFO)<<"DIRECT 3D: "<<direct.transpose()<<endl;
            // Eigen::Vector2d direct_2d = direct.head(2).normalized();
            // LOG(INFO)<<"goal 3d: "<<World_T_Tar.block<3, 1>(0, 3).transpose()<<endl;
            // Eigen::Vector2d goal = World_T_Tar.block<2, 1>(0, 3);
            // LOG(INFO)<<"goal position: "<<goal.transpose()<<endl;
            // double dis_tag = 0.1 + 0.17;// 此为粘贴时测量
            // Eigen::Vector2d walk_goal = goal - dis_tag * direct_2d;
            // // 至此，便得到了方向和目标点
            // // double dis = abs(walk_goal.dot(direct_2d));
            // double goal_dis = walk_goal.norm();
            // // double goal_dis = dis - 0.17;//前脚长15cm + 1cm阈值
            // LOG(INFO)<<"aim direct "<<direct_2d.transpose()<<endl;
            // LOG(INFO)<<"goal distance : "<<goal_dis<<endl;
            // double theta = acos(Eigen::Vector2d::UnitX().dot(direct_2d));
            // CHECK_GE(theta, 0.0)<<"theta is > 0"<<endl;
            // LOG(ERROR)<<"THETA : "<<theta<<endl;
            // struct line_step
            // {
            //     double x, y, theta;
            // };
            // vector<footstep> steps_result;
            // if (direct_2d(1) < 0)//右转
            // {
            //     LOG(INFO)<<"TURN RIGHT ..."<<endl;
            //     int num_foot_len = (int)(goal_dis/0.25 + 0.8);
            //     int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
            //     int num_foot = max(num_foot_len, num_foot_angle);
            //     double length_step = goal_dis / num_foot;
            //     double theta_step = theta / num_foot;
            //     LOG(INFO)<<"step length "<<length_step<<endl;
            //     LOG(INFO)<<"step angle "<<theta_step<<endl;
            //     vector<line_step> line_steps;
            //     line_steps.reserve(num_foot);
            //     for (size_t i = 0; i < num_foot; i++)
            //     {
            //         Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
            //         double tmptheta = (i+1) * theta_step;
            //         line_step tmp_line_step;
            //         tmp_line_step.x = line_cor(0);
            //         tmp_line_step.y = line_cor(1);
            //         tmp_line_step.theta = tmptheta;
            //         line_steps.emplace_back(tmp_line_step);
            //     }
            //     LOG(INFO)<<"line steps :"<<endl;
            //     for (auto & iter_line_step : line_steps)
            //     {
            //         LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
            //     }
            //     for (auto & iter_line_step : line_steps)
            //     {
            //         Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
            //         Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
            //         Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
            //         Eigen::AngleAxisd rotate_vector( - iter_line_step.theta, Eigen::Vector3d::UnitZ());
            //         Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
            //         Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
            //         footstep tmpstep1;
            //         tmpstep1.is_left = false;
            //         tmpstep1.x = right_foot_cor_ro(0);
            //         tmpstep1.y = right_foot_cor_ro(1);
            //         tmpstep1.z = right_foot_cor_ro(2);
            //         tmpstep1.theta = - iter_line_step.theta;
            //         steps_result.emplace_back(tmpstep1);
            //         footstep tmpstep2;
            //         tmpstep2.is_left = true;
            //         tmpstep2.x = left_foot_cor_ro(0);
            //         tmpstep2.y = left_foot_cor_ro(1);
            //         tmpstep2.z = left_foot_cor_ro(2);
            //         tmpstep2.theta = - iter_line_step.theta;
            //         steps_result.emplace_back(tmpstep2);
            //     }
            //     LOG(INFO)<<"foot step:"<<endl;
            //     for (auto & iter_footstep : steps_result)
            //     {
            //         LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
            //     }
            // }
            // else//左转
            // {
            //     LOG(INFO)<<"TURN LEFT ..."<<endl;
            //     int num_foot_len = (int)(goal_dis/0.25 + 0.8);
            //     int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
            //     int num_foot = max(num_foot_len, num_foot_angle);
            //     double length_step = goal_dis / num_foot;
            //     double theta_step = theta / num_foot;
            //     LOG(INFO)<<"step length "<<length_step<<endl;
            //     LOG(INFO)<<"step angle "<<theta_step<<endl;
            //     vector<line_step> line_steps;
            //     line_steps.reserve(num_foot);
            //     steps_result.reserve(num_foot * 2);
            //     for (size_t i = 0; i < num_foot; i++)
            //     {
            //         Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
            //         double tmptheta = (i+1) * theta_step;
            //         line_step tmp_line_step;
            //         tmp_line_step.x = line_cor(0);
            //         tmp_line_step.y = line_cor(1);
            //         tmp_line_step.theta = tmptheta;
            //         line_steps.emplace_back(tmp_line_step);
            //     }
            //     for (auto & iter_line_step : line_steps)
            //     {
            //         LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
            //     }
            //     for (auto & iter_line_step : line_steps)
            //     {
            //         Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
            //         Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
            //         Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
            //         Eigen::AngleAxisd rotate_vector(iter_line_step.theta, Eigen::Vector3d::UnitZ());
            //         Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
            //         Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
            //         footstep tmpstep1;
            //         tmpstep1.is_left = true;
            //         tmpstep1.x = left_foot_cor_ro(0);
            //         tmpstep1.y = left_foot_cor_ro(1);
            //         tmpstep1.z = left_foot_cor_ro(2);
            //         tmpstep1.theta = iter_line_step.theta;
            //         steps_result.emplace_back(tmpstep1);
            //         footstep tmpstep2;
            //         tmpstep2.is_left = false;
            //         tmpstep2.x = right_foot_cor_ro(0);
            //         tmpstep2.y = right_foot_cor_ro(1);
            //         tmpstep2.z = right_foot_cor_ro(2);
            //         tmpstep2.theta = iter_line_step.theta;
            //         steps_result.emplace_back(tmpstep2);
            //     }
            //     LOG(INFO)<<"foot step:"<<endl;
            //     for (auto & iter_footstep : steps_result)
            //     {
            //         LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
            //     }
            // }
            bool return_flag = steps_result.size() > 0;
            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            cout<<"plane detection and step planning cost "<<duration<<endl;
            LOG(INFO)<<"step size "<<steps_result.size()<<endl;
            // 画步态点
            // for (auto step_iter : steps_result)
            // {
            //     std::array<Eigen::Vector3d, 4> edge_points;// 27 10 5 14.5
            //     edge_points.at(0) = Eigen::Vector3d(0.17, -0.05, 0.0);
            //     edge_points.at(1) = Eigen::Vector3d(0.17,  0.095, 0.0);
            //     edge_points.at(2) = Eigen::Vector3d(- 0.1, 0.095, 0.0);
            //     edge_points.at(3) = Eigen::Vector3d(-0.1, -0.05, 0.0);
            //     LOG(INFO)<<"GET EDGE POINTS"<<endl;
            //     Eigen::AngleAxisd r_v(step_iter.theta, Eigen::Vector3d(0,0,1));
            //     std::array<Eigen::Vector3d, 4> draw_points;
            //     std::vector<double> x, y;
            //     x.reserve(4); y.reserve(4);
            //     for (size_t i = 0; i < 4; i++)
            //     {
            //         draw_points.at(i) = r_v.matrix() * edge_points.at(i) + Eigen::Vector3d(step_iter.x, step_iter.y, step_iter.z);
            //         x.emplace_back(draw_points.at(i)(0));
            //         y.emplace_back(draw_points.at(i)(1));
            //     }
            //     LOG(INFO)<<"get x and y"<<endl;
            //     matplotlibcpp::plot(y, x);
            // }
            // matplotlibcpp::show();
            
            if (!return_flag)
            {
                LOG(INFO)<<"perception wrong, send an empty data to win. "<<endl;
                int buf_size = sizeof(int);
                char* senddata = new char[buf_size];
                int steps_num = 0;
                memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
                if (com.sendSteps(senddata, buf_size) == -1)
                {
                    perror("send error");
                    exit(1);
                }
                cout<<"has send 0 to win, you can ansys it use type int at buffer"<<endl;
            }
            else
            {
                vector<footstep> send_steps = steps_result;
                // send_steps.emplace_back(perception.steps_result.at(2));
                // data to file
                if (send_steps.size() <= 2)
                {
                    LOG(INFO)<<"some logic error occured in map or stepplanning, please check..."<<endl;
                }
                
                // for (size_t i = 0; i < perception.steps_result.size(); i++)
                // {
                //     fs_communicate<<"step "<<i+1<<": ";
                //     fs_communicate<<perception.steps_result.at(i).is_left<<" "<<perception.steps_result.at(i).x<<" "<<perception.steps_result.at(i).z<<endl;
                // }
                
                //  = perception.steps_result;
                LOG(INFO)<<"STEP PLANNING RESULTS:"<<endl;
                for (auto & iter_step : send_steps)
                {
                    LOG(INFO)<<iter_step.is_left<<" "<<iter_step.x<<" "<<iter_step.y<<" "<<iter_step.z<<" "<<iter_step.theta<<endl;
                }
                // 改了最大步数
                struct sendddata
                {
                    int n;
                    footstep steps[25];
                };
                assert(send_steps.size() <= 25 && "the plan steps is big than 25, you should turn the senddata buffer more ...");
                CHECK(send_steps.size() <= 25);
                sendddata SENDdata;
                SENDdata.n = send_steps.size();
                for (size_t i = 0; i < send_steps.size(); i++)
                {
                    SENDdata.steps[i].is_left = send_steps.at(i).is_left;
                    SENDdata.steps[i].x = send_steps.at(i).x;
                    SENDdata.steps[i].y = send_steps.at(i).y;
                    SENDdata.steps[i].z = send_steps.at(i).z;
                    SENDdata.steps[i].theta = send_steps.at(i).theta;

                }
                cout<<"size of sendddata "<<sizeof(sendddata)<<endl;
                if (com.sendSteps((char*)(&SENDdata), sizeof(sendddata)) == -1)
                {
                    perror("send error");
                    exit(1);
                }
                // DRAW
            }
            // 画完之后增加一个按键检测环节，用于切换图像界面
            // 直接使用cv：：ishow成一个动态显示的效果，显示完之后再将其缩小放置在右上角
            // 也可以考虑使用发布器，只不过将采集到的图像右上方换成规划的图像，实际上就是在一张图像上显示了
            // cv::Mat raw_image;
            // unique_lock<mutex> g_image(m_ros, std::defer_lock);
            // g_image.lock();
            // raw_image = image;
            // g_image.unlock();
            // cv::resize(raw_image, draw_image, cv::Size(640 ,480), 0, 0, cv::INTER_LINEAR);
            // unique_lock<mutex> g_result(m_result, std::defer_lock);
            // g_result.lock();
            // get_plane_result = true;
            // in_planning = true;
            // g_result.unlock();
            // draw_info(steps_result, pub_start, pub_goal, rectPoints);
            // usleep(2000000);
            // cv::Mat embed_image;
            // cv::resize(draw_image, embed_image, cv::Size(320, 180), 0, 0, cv::INTER_LINEAR);
            // cv::Mat insetImage(raw_image, cv::Rect(960, 0, 320, 180));
            // embed_image.copyTo(insetImage);
            // cv::rectangle(raw_image, cv::Rect(960, 0, 320, 180), cv::Scalar(255, 0, 255), 3);
            // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
            // pub_msg->header.frame_id = "pub_camera";
            // image_pub.publish(pub_msg);
            // // unique_lock<mutex> g_result(m_result, std::defer_lock);
            // g_result.lock();
            // in_planning = false;
            // g_result.unlock();
            // cv::imshow("plane image", draw_image);
            // cv::waitKey(0);
            lastFlag = 'A';
            currentFlag = 'A';
        }
        else
        {
            LOG(INFO)<<"do not need to perception, send an empty data to win. "<<endl;
            int buf_size = sizeof(int);
            char* senddata = new char[buf_size];
            int steps_num = 0;
            memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
            if (com.sendSteps(senddata, buf_size) == -1)
            {
                perror("send error");
                exit(1);
            }
            // cout<<"has send 0 to win"<<endl;
            // 发布数据
            // cv::Mat pub_image;
            // if (get_plane_result)
            // {
            //     LOG(INFO)<<"show result"<<endl;
            //     unique_lock<mutex> g_image(m_ros, std::defer_lock);
            //     g_image.lock();
            //     pub_image = image;
            //     g_image.unlock();
            //     cv::Mat embed_image;
            //     cv::resize(draw_image, embed_image, cv::Size(320, 180), 0, 0, cv::INTER_LINEAR);
            //     cv::Mat insetImage(pub_image, cv::Rect(960, 0, 320, 180));
            //     embed_image.copyTo(insetImage);
            //     cv::rectangle(pub_image, cv::Rect(960, 0, 320, 180), cv::Scalar(255, 0, 255), 3);
            // }
            // else
            // {
            //     LOG(INFO)<<"not show result"<<endl;
            //     unique_lock<mutex> g_image(m_ros, std::defer_lock);
            //     g_image.lock();
            //     pub_image = image;
            //     g_image.unlock();
            // }
            // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
            // pub_msg->header.frame_id = "pub_camera";
            // image_pub.publish(pub_msg);
        }
        LOG(INFO)<<"last flag is "<<lastFlag<<endl;
        LOG(INFO)<<"current flag is "<<currentFlag<<endl;
        com.close_client();
        LOG(INFO)<<"CLOSE CLIENT FINISH"<<endl;
    }
}

int communicate_ART()
{
    // 此处初始化
    LOG(INFO)<<"listen art data..."<<endl;
    unsigned short port = 6324;
	dt = new DTrackSDK( port );
	if ( ! dt->isDataInterfaceValid() )
	{
		std::cout << "DTrackSDK init error" << std::endl;
		return -3;
	}
	std::cout << "listening at local data port " << dt->getDataPort() << std::endl;
	// measurement:
    uint16_t index_time = 0;
    while (1)
    {
        if(dt->receive())
        {
            std::cout.precision( 3 );
            std::cout.setf( std::ios::fixed, std::ios::floatfield );

            // std::cout << std::endl << "frame " << dt->getFrameCounter() << " ts " << dt->getTimeStamp()
            //         << " nbod " << dt->getNumBody() << " nfly " << dt->getNumFlyStick()
            //         << " nmea " << dt->getNumMeaTool() << " nmearef " << dt->getNumMeaRef() 
            //         << " nhand " << dt->getNumHand() << " nmar " << dt->getNumMarker() 
            //         << " nhuman " << dt->getNumHuman() << " ninertial " << dt->getNumInertial()
            //         << std::endl;

            const DTrackBody* body = dt->getBody( 0 );
            if ( body == NULL )
            {
                std::cout << "DTrackSDK fatal error: invalid body id " << 0 << std::endl;
                break;
            }

            if ( ! body->isTracked() )
            {
                if (index_time >= 200)
                {
                    index_time = 0;
                    std::cout << "bod " << body->id << " not tracked" << std::endl;
                }
            }
            else
            {
                // std::cout << "bod " << body->id << " qu " << body->quality
                //         << " loc " << body->loc[ 0 ] << " " << body->loc[ 1 ] << " " << body->loc[ 2 ]
                //         << " rot " << body->rot[ 0 ] << " " << body->rot[ 1 ] << " " << body->rot[ 2 ]
                //         << " "     << body->rot[ 3 ] << " " << body->rot[ 4 ] << " " << body->rot[ 5 ]
                //         << " "     << body->rot[ 6 ] << " " << body->rot[ 7 ] << " " << body->rot[ 8 ]
                //         << std::endl;

                unique_lock<mutex> g(m, std::defer_lock);
                g.lock();
                ARTWORLD_T_MARKER.setIdentity();
                ARTWORLD_T_MARKER<<(float)body->rot[0], (float)body->rot[3], (float)body->rot[6], (float)body->loc[0]/1000,
                            (float)body->rot[1], (float)body->rot[4], (float)body->rot[7], (float)body->loc[1]/1000,
                            (float)body->rot[2], (float)body->rot[5], (float)body->rot[8], (float)body->loc[2]/1000,
                            0,            0,             0,           1;
                g.unlock();
                // std::cout<<mark_pose<<endl;
                DTrackQuaternion quat = body->getQuaternion();
                // std::cout << "bod " << body->id << " quatw " << quat.w
                //         << " quatxyz " << quat.x << " " << quat.y << " " << quat.z << std::endl;
                Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
                // std::cout<<q.toRotationMatrix()<<std::endl;
            }
        }
        else
        {
            if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
            {
                std::cout << "--- timeout while waiting for tracking data" << std::endl;
                return -1;
            }

            if ( dt->getLastDataError() == DTrackSDK::ERR_NET )
            {
                std::cout << "--- error while receiving tracking data" << std::endl;
                return -1;
            }

            if ( dt->getLastDataError() == DTrackSDK::ERR_PARSE )
            {
                std::cout << "--- error while parsing tracking data" << std::endl;
                return -1;
            }
        }
        index_time++;
    }
	delete dt;  // clean up
	return 0;
}

void keyBoard()
{
    LOG(INFO)<<"enter exit all function"<<endl;
    int ch;
    while (1){
		if (_kbhit())
        {//如果有按键按下，则_kbhit()函数返回真
            LOG(INFO)<<"PRESS KEY"<<endl;
			ch = getch();//使用_getch()获取按下的键值
            LOG(INFO)<<"GET CHAR "<<ch<<endl;
			cout << ch;
			if (ch == 27)
            { 
                close(sock_fd);
                close(client_fd);
                delete dt;
                LOG(INFO)<<"CLOSE TCPIP PORT..."<<endl;
                break; 
                //当按下ESC时退出循环，ESC键的键值是27.
            }

            // if (ch == 99 || ch == 67)//c建
            // {
            //     LOG(INFO)<<"show raw image"<<endl;
            //     unique_lock<mutex> g_result(m_result, std::defer_lock);
            //     g_result.lock();
            //     get_plane_result = false;
            //     g_result.unlock();
            // }
            
        }
	}
}

// 每次获取图像均发布tf
// void getImage(const sensor_msgs::Image::ConstPtr& msg)
// {
//     std::cout<<"get image"<<std::endl;
//     unique_lock<mutex> g(m_ros, std::defer_lock);
//     g.lock();
//     image = *msg;
//     g.unlock();
//     // 这两个参数不对，对应的老版本链接件
//     double BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
//     double BaseVisionX = 0.15995;
//     double BaseVisionY = 0.0;
//     double BaseVisionPitchDeg = 27.5;
//     Eigen::Matrix<double,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
//     Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
//     Eigen::Matrix3d Base_R_VisionTemp;
//     Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
//     Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisd(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3d::UnitX())).matrix();
//     // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
//     // 这来自控制端发送的数据
//     World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
//     Vision_T_Tar.setIdentity();
//     World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
//     tf::Vector3 t(World_T_Tar(0, 3), World_T_Tar(1, 3), World_T_Tar(2, 3));
//     Eigen::Quaterniond qd; qd = World_T_Tar.block<3,3>(0,0);
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     transform.setOrigin(t);
//     double qqx = qd.x();
//     double qqy = qd.y();
//     double qqz = qd.z();
//     double qqw = qd.w();
//     tf::Quaternion q(qqx, qqy, qqz, qqw);
//     transform.setRotation(q);
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_color_optical_frame"));
// }

void getColorImage(const sensor_msgs::Image::ConstPtr& msg)
{
    // std::cout<<"get image"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    unique_lock<mutex> g(m_ros, std::defer_lock);
    g.lock();
    image = cv_ptr->image;
    g.unlock();
    
    // if (in_planning)
    // {
    //     return;
    // }
    // cv::Mat pub_image;
    // if (get_plane_result)
    // {
    //     LOG(INFO)<<"show result"<<endl;
    //     unique_lock<mutex> g_image(m_ros, std::defer_lock);
    //     g_image.lock();
    //     pub_image = image;
    //     g_image.unlock();
    //     cv::Mat embed_image;
    //     cv::resize(draw_image, embed_image, cv::Size(320, 180), 0, 0, cv::INTER_LINEAR);
    //     cv::Mat insetImage(pub_image, cv::Rect(960, 0, 320, 180));
    //     embed_image.copyTo(insetImage);
    //     cv::rectangle(pub_image, cv::Rect(960, 0, 320, 180), cv::Scalar(255, 0, 255), 3);
    // }
    // else
    // {
    //     // LOG(INFO)<<"not show result"<<endl;
    //     unique_lock<mutex> g_image(m_ros, std::defer_lock);
    //     g_image.lock();
    //     pub_image = image;
    //     g_image.unlock();
    // }
    cv::Mat pub_image;
    // 表示需要画落脚点图
    if (pub_flag)
    {
        if (initial_time)
        {
            start_pub = msg->header.stamp;
            initial_time = false;
        }
        if ((msg->header.stamp - start_pub).toNSec() > (int64_t)(1000000000 * 0.8))
        {
            start_pub = msg->header.stamp;
            if (pub_step_index < pub_steps.size())
            {
                std::cout<<"show index: "<<pub_step_index<<std::endl;
                vector<footstep> pub_steps_now(pub_steps.begin() + pub_step_index, pub_step_index + 4 >pub_steps.size() ? pub_steps.end() : pub_steps.begin() + pub_step_index + 4);
                pub_step_index++;
                if (pub_steps_now.empty())
                {
                    std::cout<<"do not need to publish steps"<<std::endl;
                    unique_lock<mutex> g_flag(m_flag, std::defer_lock);
                    g_flag.lock();
                    pub_flag =  false;
                    g_flag.unlock();
                }
                else
                {
                    cv::resize(image, draw_image, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
                    // cv::imshow("draw_image", draw_image);
                    // cv::waitKey(0);
                    // draw_image = image;
                    draw(pub_steps_now);
                    // cv::imshow("draw_image", draw_image);
                    // cv::waitKey(0);
                    cv::resize(draw_image, pub_image, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
                    // pub_image = draw_image;
                    // cv::imshow("pub_image", pub_image);
                    // cv::waitKey(0);
                    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
                    pub_msg->header.frame_id = "pub_camera";
                    image_pub.publish(pub_msg);
                    // std::cout<<"publish out"<<std::endl;
                }
            }
            else
            {
                std::cout<<"do not need to publish steps"<<std::endl;
                unique_lock<mutex> g_flag(m_flag, std::defer_lock);
                g_flag.lock();
                pub_flag =  false;
                g_flag.unlock();
                pub_image = image;
                sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
                pub_msg->header.frame_id = "pub_camera";
                image_pub.publish(pub_msg);
                // std::cout<<"publish out"<<std::endl;
            }
        }
    }
    else
    {
        pub_image = image;
        sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
        pub_msg->header.frame_id = "pub_camera";
        image_pub.publish(pub_msg);
        // std::cout<<"publish out"<<std::endl;
    }
    // cv::imshow("pub_image", pub_image);
    // cv::waitKey(0);

    // 奇怪，放在这里怎么不行
    // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
    // pub_msg->header.frame_id = "pub_camera";
    // image_pub.publish(pub_msg);
    // std::cout<<"publish out"<<std::endl;
}

int main(int argc, char** argv)
{
    initial_pose();
    // google::ParseCommandLineFlags(&argc, &argv, true); 
    // std::cout<<"龙哥哭他"<<std::endl;
    google::InitGoogleLogging(argv[0]); 
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "./log"; 
    FLAGS_alsologtostderr = true;
    LOG(INFO)<<"initial glog finish"<<endl;
    LOG(WARNING)<<"WARNING"<<endl;

    thread th1(keyBoard);
    thread th2(communicate_PC104);
    thread th3(communicate_ART);
    
    ros::init(argc, argv,"turn_right_walk");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, getColorImage);
    image_pub = nh.advertise<sensor_msgs::Image>("image_pub", 1);
    ros::spin();
    // rect_pub = nh.advertise<visualization_msgs::MarkerArray>("rect_plane", 1);
    // steps_pub = nh.advertise<visualization_msgs::MarkerArray>("steps_pub", 1);
    // goal_pub = nh.advertise<visualization_msgs::MarkerArray>("start_goal", 1);
    // path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);
    // tf::TransformBroadcaster br;
    // tf::Transform transform;
    // ros::Rate rate(10.0);
    // while (nh.ok())
    // {
    //     // 这两个参数不对，对应的老版本链接件
    //     double BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
    //     double BaseVisionX = 0.15995;
    //     double BaseVisionY = 0.0;
    //     double BaseVisionPitchDeg = 27.5;
    //     Eigen::Matrix<double,4,4> Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
    //     Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
    //     Eigen::Matrix3d Base_R_VisionTemp;
    //     Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
    //     Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisd(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3d::UnitX())).matrix();
    //     // World_T_Base = getT(-0.021746375, 0.0, 0.60419, _deg2rad(-0.52),_deg2rad(-1.54), 0.0);// 仅供自己测试使用
    //     // 这来自控制端发送的数据
    //     World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
    //     Vision_T_Tar.setIdentity();
    //     World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
    //     tf::Vector3 t(World_T_Tar(0, 3), World_T_Tar(1, 3), World_T_Tar(2, 3));
    //     Eigen::Quaterniond qd; qd = World_T_Tar.block<3,3>(0,0);
    //     transform.setOrigin(t);
    //     double qqx = qd.x();
    //     double qqy = qd.y();
    //     double qqz = qd.z();
    //     double qqw = qd.w();
    //     tf::Quaternion q(qqx, qqy, qqz, qqw);
    //     transform.setRotation(q);
    //     br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "camera_depth_optical_frame", "world"));
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    th1.join();
    th2.join();
    th3.join();
    google::ShutdownGoogleLogging();
    return 0;
}

