#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H

#include <sstream>
#include <ros/ros.h>
#include <QObject>

#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/default_plugin/tools/pose_tool.h"

namespace rviz
{
class StringProperty;

class WaypointTool : public PoseTool
{
  Q_OBJECT
public:
  WaypointTool(); // 생성자
  virtual ~WaypointTool()
  {
  } // 소멸자
  virtual void onInitialize(); // 초기화 함수

protected:
  virtual void odomHandler(const nav_msgs::Odometry::ConstPtr& odom); // 오도메트리 메시지 핸들러
  virtual void onPoseSet(double x, double y, double theta); // 포즈 설정 함수

private Q_SLOTS:
  void updateTopic(); // 토픽 업데이트 함수

private:
  float vehicle_z; // 차량의 Z 좌표

  ros::NodeHandle nh_; // ROS 노드 핸들
  ros::Subscriber sub_; // ROS 구독자
  ros::Publisher pub_; // ROS 퍼블리셔
  ros::Publisher pub_joy_; // ROS 퍼블리셔 (Joy 메시지용)

  StringProperty* topic_property_; // 토픽 속성
};
}

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
