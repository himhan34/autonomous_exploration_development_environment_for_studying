#include "waypoint_tool.h"

namespace rviz
{
// WaypointTool 클래스 생성자
WaypointTool::WaypointTool()
{
  shortcut_key_ = 'w'; // 단축키 설정

  // 토픽 속성 설정
  topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

// 초기화 함수
void WaypointTool::onInitialize()
{
  PoseTool::onInitialize(); // 부모 클래스의 초기화 함수 호출
  setName("Waypoint"); // 이름 설정
  updateTopic(); // 토픽 업데이트
  vehicle_z = 0; // 차량의 Z 좌표 초기화
}

// 토픽 업데이트 함수
void WaypointTool::updateTopic()
{
  // 오도메트리 메시지 구독자 설정
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, &WaypointTool::odomHandler, this);
  // 웨이포인트 메시지 퍼블리셔 설정
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  // Joy 메시지 퍼블리셔 설정
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
}

// 오도메트리 메시지 핸들러
void WaypointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z; // 차량의 Z 좌표 업데이트
}

// 포즈 설정 함수
void WaypointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::Joy joy;

  // Joy 메시지의 축 값 설정
  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  // Joy 메시지의 버튼 값 설정
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = ros::Time::now(); // 현재 시간 설정
  joy.header.frame_id = "waypoint_tool"; // 프레임 ID 설정
  pub_joy_.publish(joy); // Joy 메시지 퍼블리시

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map"; // 프레임 ID 설정
  waypoint.header.stamp = joy.header.stamp; // 타임스탬프 설정
  waypoint.point.x = x; // 웨이포인트의 X 좌표 설정
  waypoint.point.y = y; // 웨이포인트의 Y 좌표 설정
  waypoint.point.z = vehicle_z; // 웨이포인트의 Z 좌표 설정

  pub_.publish(waypoint); // 웨이포인트 퍼블리시
  usleep(10000); // 10ms 대기
  pub_.publish(waypoint); // 웨이포인트 다시 퍼블리시
}
}

// 플러그인 등록 매크로
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)
