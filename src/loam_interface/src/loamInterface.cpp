#include <math.h> // 수학 라이브러리 포함
#include <time.h> // 시간 관련 라이브러리 포함
#include <stdio.h> // 표준 입출력 라이브러리 포함
#include <stdlib.h> // 표준 라이브러리 포함
#include <ros/ros.h> // ROS 라이브러리 포함

#include <message_filters/subscriber.h> // 메시지 필터 구독자 포함
#include <message_filters/synchronizer.h> // 메시지 필터 동기화기 포함
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화 정책 포함

#include <std_msgs/Float32.h> // 표준 Float32 메시지 포함
#include <nav_msgs/Odometry.h> // 네비게이션 오도메트리 메시지 포함
#include <geometry_msgs/PointStamped.h> // 지오메트리 PointStamped 메시지 포함
#include <geometry_msgs/PolygonStamped.h> // 지오메트리 PolygonStamped 메시지 포함
#include <sensor_msgs/PointCloud2.h> // 센서 PointCloud2 메시지 포함

#include <tf/transform_datatypes.h> // tf 변환 데이터 타입 포함
#include <tf/transform_broadcaster.h> // tf 변환 브로드캐스터 포함

#include <pcl_conversions/pcl_conversions.h> // PCL 변환 포함
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 포함
#include <pcl/point_types.h> // PCL 포인트 타입 포함
#include <pcl/filters/voxel_grid.h> // PCL Voxel Grid 필터 포함
#include <pcl/kdtree/kdtree_flann.h> // PCL KD 트리 포함

using namespace std; // 표준 네임스페이스 사용

const double PI = 3.1415926; // 원주율 상수 정의

// 파라미터 초기화
string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

// 포인트 클라우드 객체 생성
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData; // 오도메트리 데이터
tf::StampedTransform odomTrans; // tf 변환 객체
ros::Publisher *pubOdometryPointer = NULL; // 오도메트리 퍼블리셔 포인터
tf::TransformBroadcaster *tfBroadcasterPointer = NULL; // tf 브로드캐스터 포인터
ros::Publisher *pubLaserCloudPointer = NULL; // 레이저 클라우드 퍼블리셔 포인터

// 오도메트리 콜백 함수
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw; // 회전 각도 변수
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation; // 쿼터니언 가져오기
  odomData = *odom; // 오도메트리 데이터 저장

  // 오도메트리 플립 여부에 따라 변환
  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  // 오도메트리 메시지 퍼블리시
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // tf 메시지 퍼블리시
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  // tf 메시지 전송 여부에 따라 전송
  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

// 레이저 클라우드 콜백 함수
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  laserCloud->clear(); // 레이저 클라우드 초기화
  pcl::fromROSMsg(*laserCloudIn, *laserCloud); // ROS 메시지를 PCL 포인트 클라우드로 변환

  // 레이저 스캔 플립 여부에 따라 변환
  if (flipRegisteredScan) {
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      float temp = laserCloud->points[i].x;
      laserCloud->points[i].x = laserCloud->points[i].z;
      laserCloud->points[i].z = laserCloud->points[i].y;
      laserCloud->points[i].y = temp;
    }
  }

  // 변환된 레이저 클라우드를 ROS 메시지로 변환하여 퍼블리시
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = "map";
  pubLaserCloudPointer->publish(laserCloud2);
}

// 메인 함수
int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface"); // ROS 노드 초기화
  ros::NodeHandle nh; // 노드 핸들러 생성
  ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 프라이빗 노드 핸들러 생성

  // 파라미터 가져오기
  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  // 각 토픽 구독자 설정
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  // 퍼블리셔 설정
  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin(); // ROS 이벤트 처리 루프 실행

  return 0;
}
