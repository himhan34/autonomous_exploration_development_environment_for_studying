#include <math.h> // 수학 라이브러리 포함
#include <time.h> // 시간 관련 라이브러리 포함
#include <stdio.h> // 표준 입출력 라이브러리 포함
#include <stdlib.h> // 표준 라이브러리 포함
#include <ros/ros.h> // ROS 라이브러리 포함

#include <message_filters/subscriber.h> // 메시지 필터 구독자 포함
#include <message_filters/synchronizer.h> // 메시지 필터 동기화기 포함
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화 정책 포함

#include <std_msgs/Float32.h> // 표준 Float32 메시지 포함
#include <nav_msgs/Odometry.h> // 내비게이션 Odometry 메시지 포함
#include <geometry_msgs/PointStamped.h> // 지오메트리 PointStamped 메시지 포함
#include <geometry_msgs/PolygonStamped.h> // 지오메트리 PolygonStamped 메시지 포함
#include <sensor_msgs/PointCloud2.h> // 센서 PointCloud2 메시지 포함

#include <tf/transform_datatypes.h> // tf 변환 데이터 타입 포함
#include <tf/transform_broadcaster.h> // tf 변환 브로드캐스터 포함

#include <pcl/io/ply_io.h> // PCL PLY 입출력 포함
#include <pcl_conversions/pcl_conversions.h> // PCL 변환 포함
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 포함
#include <pcl/point_types.h> // PCL 포인트 타입 포함
#include <pcl/filters/voxel_grid.h> // PCL Voxel Grid 필터 포함
#include <pcl/kdtree/kdtree_flann.h> // PCL KD 트리 포함

using namespace std; // 표준 네임스페이스 사용

const double PI = 3.1415926; // 원주율 상수 정의

// 파일 경로 및 관련 변수 정의
string metricFile;
string trajFile;
string mapFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

// 포인트 클라우드 객체 생성
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

// 시스템 초기화 변수
const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

// 차량 상태 변수
float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

// Voxel Grid 필터 객체 생성
pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

// PointCloud2 메시지 객체 생성
sensor_msgs::PointCloud2 overallMap2;

// ROS 퍼블리셔 포인터 생성
ros::Publisher *pubExploredAreaPtr = NULL;
ros::Publisher *pubTrajectoryPtr = NULL;
ros::Publisher *pubExploredVolumePtr = NULL;
ros::Publisher *pubTravelingDisPtr = NULL;
ros::Publisher *pubTimeDurationPtr = NULL;

// 파일 포인터 생성
FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;


void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 시스템 시간을 오도메트리 메시지의 타임스탬프로 설정
  systemTime = odom->header.stamp.toSec();

  // 오도메트리 메시지에서 쿼터니언을 롤, 피치, 요 각도로 변환
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  // 차량의 현재 요(yaw) 값과 이전 요 값의 차이 계산
  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI - dYaw;

  // 차량의 현재 위치와 이전 위치의 차이 계산
  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  // 시스템 초기화가 완료되지 않은 경우 현재 위치와 요 값을 저장하고 반환
  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  // 시스템이 초기화된 경우 실행 시간 계산 및 퍼블리시
  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  // 이동 거리와 요 차이가 일정 임계값 이하인 경우 반환
  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  // 시스템 초기화 중인 경우 초기화 시간 설정
  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  // 총 이동 거리 업데이트
  travelingDis += dis;

  // 차량의 현재 위치와 요 값을 저장
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  // 트래젝토리 파일에 현재 위치와 자세를 기록
  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  // 현재 위치를 포인트 클라우드에 추가
  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  // 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시
  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  // 시스템 초기화가 지연되었는지 확인
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  // 시스템이 초기화되지 않은 경우 함수 종료
  if (!systemInited) {
    return;
  }

  // 입력된 레이저 클라우드를 PCL 포인트 클라우드로 변환
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  // 탐색된 볼륨 클라우드에 현재 레이저 클라우드 추가
  *exploredVolumeCloud += *laserCloud;

  // 탐색된 볼륨 클라우드를 다운샘플링
  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  // 포인터 교체를 통해 클라우드 업데이트
  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  // 탐색된 볼륨 계산
  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  // 탐색된 영역 클라우드에 현재 레이저 클라우드 추가
  *exploredAreaCloud += *laserCloud;

  // 탐색된 영역 디스플레이 카운트 증가
  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    // 탐색된 영역 클라우드를 다운샘플링
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    // 포인터 교체를 통해 클라우드 업데이트
    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    // 탐색된 영역 클라우드를 ROS 메시지로 변환하여 퍼블리시
    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    // 탐색된 영역 디스플레이 카운트 초기화
    exploredAreaDisplayCount = 0;
  }

  // 메트릭 파일에 현재 탐색된 볼륨, 이동 거리, 실행 시간, 전체 시간을 기록
  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  // 탐색된 볼륨을 ROS 메시지로 퍼블리시
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  // 이동 거리를 ROS 메시지로 퍼블리시
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

// 실행 시간을 처리하는 콜백 함수
void runtimeHandler(const std_msgs::Float32::ConstPtr& runtimeIn)
{
  runtime = runtimeIn->data;
}


int main(int argc, char** argv)
{
  // ROS 노드 초기화
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 파라미터 가져오기
  nhPrivate.getParam("metricFile", metricFile);
  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  // 각 토픽 구독자 설정
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/registered_scan", 5, laserCloudHandler);
  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32> ("/runtime", 5, runtimeHandler);

  // 퍼블리셔 설정
  ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::PointCloud2> ("/overall_map", 5);
  ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::PointCloud2> ("/explored_areas", 5);
  pubExploredAreaPtr = &pubExploredArea;

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("/trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;

  ros::Publisher pubExploredVolume = nh.advertise<std_msgs::Float32> ("/explored_volume", 5);
  pubExploredVolumePtr = &pubExploredVolume;

  ros::Publisher pubTravelingDis = nh.advertise<std_msgs::Float32> ("/traveling_distance", 5);
  pubTravelingDisPtr = &pubTravelingDis;

  ros::Publisher pubTimeDuration = nh.advertise<std_msgs::Float32> ("/time_duration", 5);
  pubTimeDurationPtr = &pubTimeDuration;

  // Voxel Grid 필터 설정
  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  // PLY 파일 읽기
  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    printf("\nCouldn't read pointcloud.ply file.\n\n");
  }

  // 전체 맵 클라우드 다운샘플링
  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  // 다운샘플링된 전체 맵을 ROS 메시지로 변환
  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  // 로그 파일 이름 설정
  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");

  // ROS 루프 설정
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    // 전체 맵 디스플레이 카운트 증가 및 퍼블리시
    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      overallMap2.header.stamp = ros::Time().fromSec(systemTime);
      overallMap2.header.frame_id = "map";
      pubOverallMap.publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    status = ros::ok();
    rate.sleep();
  }

  // 파일 닫기
  fclose(metricFilePtr);
  fclose(trajFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.\n\n");

  return 0;
}
