#include <math.h> // 수학 라이브러리 포함
#include <time.h> // 시간 관련 라이브러리 포함
#include <stdio.h> // 표준 입출력 라이브러리 포함
#include <stdlib.h> // 표준 라이브러리 포함
#include <ros/ros.h> // ROS 라이브러리 포함

#include <nav_msgs/Odometry.h> // 네비게이션 오도메트리 메시지 포함
#include <sensor_msgs/PointCloud2.h> // 센서 포인트 클라우드 메시지 포함

#include <tf/transform_datatypes.h> // tf 변환 데이터 타입 포함
#include <tf/transform_broadcaster.h> // tf 변환 브로드캐스터 포함

#include <pcl_conversions/pcl_conversions.h> // PCL 변환 포함
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 포함
#include <pcl/point_types.h> // PCL 포인트 타입 포함

#include <message_filters/subscriber.h> // 메시지 필터 구독자 포함
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화 정책 포함
#include <message_filters/synchronizer.h> // 메시지 필터 동기화기 포함

using namespace std; // 표준 네임스페이스 사용

// 포인트 클라우드 포인터 정의 및 초기화
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0; // 로봇의 X 위치
double robotY = 0; // 로봇의 Y 위치
double robotZ = 0; // 로봇의 Z 위치
double roll = 0; // 롤 각도
double pitch = 0; // 피치 각도
double yaw = 0; // 요 각도

bool newTransformToMap = false; // 맵으로의 새로운 변환 여부

nav_msgs::Odometry odometryIn; // 오도메트리 메시지
ros::Publisher *pubOdometryPointer = NULL; // 오도메트리 퍼블리셔 포인터
tf::StampedTransform transformToMap; // 맵으로의 변환
tf::TransformBroadcaster *tfBroadcasterPointer = NULL; // tf 브로드캐스터 포인터

ros::Publisher pubLaserCloud; // 레이저 클라우드 퍼블리셔

// 레이저 클라우드와 오도메트리 콜백 함수
void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudIn->clear(); // 입력 레이저 클라우드 초기화
  laserCLoudInSensorFrame->clear(); // 센서 프레임의 레이저 클라우드 초기화

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn); // ROS 메시지를 PCL 포인트 클라우드로 변환

  odometryIn = *odometry; // 오도메트리 메시지 저장

  // 오도메트리 메시지로부터 변환 설정
  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size(); // 입력 레이저 클라우드의 포인트 수

  pcl::PointXYZ p1; // 포인트 변수
  tf::Vector3 vec; // tf 벡터 변수

  for (int i = 0; i < laserCloudInNum; i++) // 레이저 클라우드의 모든 포인트에 대해
  {
    p1 = laserCloudIn->points[i]; // 현재 포인트 저장
    vec.setX(p1.x); // 벡터 X 설정
    vec.setY(p1.y); // 벡터 Y 설정
    vec.setZ(p1.z); // 벡터 Z 설정

    vec = transformToMap.inverse() * vec; // 벡터를 맵 좌표계로 변환

    p1.x = vec.x(); // 변환된 X 값 저장
    p1.y = vec.y(); // 변환된 Y 값 저장
    p1.z = vec.z(); // 변환된 Z 값 저장

    laserCLoudInSensorFrame->points.push_back(p1); // 센서 프레임의 레이저 클라우드에 추가
  }

  // 오도메트리 메시지 업데이트 및 퍼블리시
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  // tf 변환 설정 및 퍼블리시
  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = "map";
  transformToMap.child_frame_id_ = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformToMap);

  // 센서 프레임의 레이저 클라우드를 ROS 메시지로 변환하여 퍼블리시
  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud.publish(scan_data);
}

// 메인 함수
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan"); // ROS 노드 초기화
  ros::NodeHandle nh; // 노드 핸들러 생성
  ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 프라이빗 노드 핸들러 생성

  // ROS 메시지 필터 설정
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, "/state_estimation", 1);
  subLaserCloud.subscribe(nh, "/registered_scan", 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  // 퍼블리셔 설정
  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 2);

  ros::spin(); // ROS 이벤트 처리 루프 실행

  return 0;
}
