/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
using namespace std;

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

void getImage(const sensor_msgs::PointCloud2 msg)
{
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
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(t);
  double qqx = qd.x();
  double qqy = qd.y();
  double qqz = qd.z();
  double qqw = qd.w();
  tf::Quaternion q(qqx, qqy, qqz, qqw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_depth_optical_frame"));
  cout<<"get tf"<<endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"turn_right_walk");
  ros::NodeHandle nh;
  cout<<"enter main func"<<endl;
  ros::Subscriber sub_image = nh.subscribe("/camera/depth/color/points", 1, getImage);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);
  while (nh.ok())
  {
    transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_depth_optical_frame"));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}