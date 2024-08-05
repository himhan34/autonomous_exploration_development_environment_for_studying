#include <math.h>  // 수학 함수들을 포함
#include <time.h>  // 시간 관련 함수들을 포함
#include <stdio.h>  // 표준 입출력 함수들을 포함
#include <stdlib.h>  // 표준 라이브러리 함수들을 포함
#include <ros/ros.h>  // ROS 기본 기능들을 포함

#include <message_filters/subscriber.h>  // 메시지 필터를 위한 구독자 포함
#include <message_filters/synchronizer.h>  // 메시지 필터를 위한 동기화 포함
#include <message_filters/sync_policies/approximate_time.h>  // 근사 시간 동기화 정책 포함

#include <std_msgs/Float32.h>  // std_msgs 패키지의 Float32 메시지 타입 포함
#include <nav_msgs/Odometry.h>  // nav_msgs 패키지의 Odometry 메시지 타입 포함
#include <geometry_msgs/PointStamped.h>  // geometry_msgs 패키지의 PointStamped 메시지 타입 포함
#include <geometry_msgs/PolygonStamped.h>  // geometry_msgs 패키지의 PolygonStamped 메시지 타입 포함
#include <sensor_msgs/PointCloud2.h>  // sensor_msgs 패키지의 PointCloud2 메시지 타입 포함

#include <tf/transform_datatypes.h>  // TF 변환 데이터 타입 포함
#include <tf/transform_broadcaster.h>  // TF 변환 브로드캐스터 포함

#include <pcl_conversions/pcl_conversions.h>  // PCL 변환 기능 포함
#include <pcl/point_cloud.h>  // PCL 포인트 클라우드 포함
#include <pcl/point_types.h>  // PCL 포인트 타입 포함
#include <pcl/filters/voxel_grid.h>  // PCL의 Voxel Grid 필터 포함
#include <pcl/kdtree/kdtree_flann.h>  // PCL의 KD-트리 포함

using namespace std;  // 표준 네임스페이스 사용

const double PI = 3.1415926;  // 원주율 상수 정의

string waypoint_file_dir;  // 웨이포인트 파일 경로 문자열 변수
string boundary_file_dir;  // 경계 파일 경로 문자열 변수
double waypointXYRadius = 0.5;  // 웨이포인트 XY 반경
double waypointZBound = 5.0;  // 웨이포인트 Z 범위
double waitTime = 0;  // 대기 시간
double waitTimeStart = 0;  // 대기 시작 시간
bool isWaiting = false;  // 대기 중 여부
double frameRate = 5.0;  // 프레임 레이트
double speed = 1.0;  // 속도
bool sendSpeed = true;  // 속도 전송 여부
bool sendBoundary = true;  // 경계 전송 여부

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());  // PCL 포인트 클라우드 포인터: 웨이포인트
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());  // PCL 포인트 클라우드 포인터: 경계

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;  // 차량의 현재 위치 좌표
double curTime = 0, waypointTime = 0;  // 현재 시간과 웨이포인트 시간


// 웨이포인트를 파일에서 읽어오는 함수
void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");  // 파일을 읽기 모드로 엶
  if (waypoint_file == NULL) {  // 파일을 열 수 없는 경우
    printf ("\nCannot read input files, exit.\n\n");  // 오류 메시지 출력
    exit(1);  // 프로그램 종료
  }

  char str[50];  // 문자열을 저장할 배열
  int val, pointNum;  // 읽어들인 값과 포인트 개수
  string strCur, strLast;  // 현재 문자열과 마지막 문자열
  while (strCur != "end_header") {  // "end_header" 문자열을 만날 때까지 반복
    val = fscanf(waypoint_file, "%s", str);  // 파일에서 문자열을 읽음
    if (val != 1) {  // 읽기 실패 시
      printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
      exit(1);  // 프로그램 종료
    }

    strLast = strCur;  // 현재 문자열을 마지막 문자열로 저장
    strCur = string(str);  // 읽어들인 문자열을 현재 문자열로 저장

    if (strCur == "vertex" && strLast == "element") {  // 현재 문자열이 "vertex"이고 마지막 문자열이 "element"인 경우
      val = fscanf(waypoint_file, "%d", &pointNum);  // 포인트 개수를 읽음
      if (val != 1) {  // 읽기 실패 시
        printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
        exit(1);  // 프로그램 종료
      }
    }
  }

  waypoints->clear();  // 웨이포인트 클라우드를 초기화
  pcl::PointXYZ point;  // 포인트 객체 생성
  int val1, val2, val3;  // 읽어들인 값
  for (int i = 0; i < pointNum; i++) {  // 포인트 개수만큼 반복
    val1 = fscanf(waypoint_file, "%f", &point.x);  // x 좌표 읽기
    val2 = fscanf(waypoint_file, "%f", &point.y);  // y 좌표 읽기
    val3 = fscanf(waypoint_file, "%f", &point.z);  // z 좌표 읽기

    if (val1 != 1 || val2 != 1 || val3 != 1) {  // 읽기 실패 시
      printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
      exit(1);  // 프로그램 종료
    }

    waypoints->push_back(point);  // 읽어들인 포인트를 웨이포인트 클라우드에 추가
  }

  fclose(waypoint_file);  // 파일을 닫음
}

// 경계를 파일에서 읽어오는 함수
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");  // 파일을 읽기 모드로 엶
  if (boundary_file == NULL) {  // 파일을 열 수 없는 경우
    printf ("\nCannot read input files, exit.\n\n");  // 오류 메시지 출력
    exit(1);  // 프로그램 종료
  }

  char str[50];  // 문자열을 저장할 배열
  int val, pointNum;  // 읽어들인 값과 포인트 개수
  string strCur, strLast;  // 현재 문자열과 마지막 문자열
  while (strCur != "end_header") {  // "end_header" 문자열을 만날 때까지 반복
    val = fscanf(boundary_file, "%s", str);  // 파일에서 문자열을 읽음
    if (val != 1) {  // 읽기 실패 시
      printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
      exit(1);  // 프로그램 종료
    }

    strLast = strCur;  // 현재 문자열을 마지막 문자열로 저장
    strCur = string(str);  // 읽어들인 문자열을 현재 문자열로 저장

    if (strCur == "vertex" && strLast == "element") {  // 현재 문자열이 "vertex"이고 마지막 문자열이 "element"인 경우
      val = fscanf(boundary_file, "%d", &pointNum);  // 포인트 개수를 읽음
      if (val != 1) {  // 읽기 실패 시
        printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
        exit(1);  // 프로그램 종료
      }
    }
  }

  boundary->clear();  // 경계 클라우드를 초기화
  pcl::PointXYZ point;  // 포인트 객체 생성
  int val1, val2, val3;  // 읽어들인 값
  for (int i = 0; i < pointNum; i++) {  // 포인트 개수만큼 반복
    val1 = fscanf(boundary_file, "%f", &point.x);  // x 좌표 읽기
    val2 = fscanf(boundary_file, "%f", &point.y);  // y 좌표 읽기
    val3 = fscanf(boundary_file, "%f", &point.z);  // z 좌표 읽기

    if (val1 != 1 || val2 != 1 || val3 != 1) {  // 읽기 실패 시
      printf ("\nError reading input files, exit.\n\n");  // 오류 메시지 출력
      exit(1);  // 프로그램 종료
    }

    boundary->push_back(point);  // 읽어들인 포인트를 경계 클라우드에 추가
  }

  fclose(boundary_file);  // 파일을 닫음
}

// 차량 위치 콜백 함수
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();  // 현재 시간을 헤더의 타임스탬프에서 초 단위로 가져옴

  vehicleX = pose->pose.pose.position.x;  // 차량의 x 좌표 업데이트
  vehicleY = pose->pose.pose.position.y;  // 차량의 y 좌표 업데이트
  vehicleZ = pose->pose.pose.position.z;  // 차량의 z 좌표 업데이트
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointExample");  // ROS 노드 초기화
  ros::NodeHandle nh;  // 기본 노드 핸들 생성
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");  // 프라이빗 노드 핸들 생성

  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);  // 파라미터로부터 웨이포인트 파일 경로를 가져옴
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);  // 파라미터로부터 경계 파일 경로를 가져옴
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);  // 파라미터로부터 웨이포인트 XY 반경을 가져옴
  nhPrivate.getParam("waypointZBound", waypointZBound);  // 파라미터로부터 웨이포인트 Z 범위를 가져옴
  nhPrivate.getParam("waitTime", waitTime);  // 파라미터로부터 대기 시간을 가져옴
  nhPrivate.getParam("frameRate", frameRate);  // 파라미터로부터 프레임 레이트를 가져옴
  nhPrivate.getParam("speed", speed);  // 파라미터로부터 속도를 가져옴
  nhPrivate.getParam("sendSpeed", sendSpeed);  // 파라미터로부터 속도 전송 여부를 가져옴
  nhPrivate.getParam("sendBoundary", sendBoundary);  // 파라미터로부터 경계 전송 여부를 가져옴

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);  // 위치 구독자 설정

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);  // 웨이포인트 퍼블리셔 설정
  geometry_msgs::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";  // 웨이포인트 메시지의 프레임 설정

  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> ("/speed", 5);  // 속도 퍼블리셔 설정
  std_msgs::Float32 speedMsgs;

  ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5);  // 경계 퍼블리셔 설정
  geometry_msgs::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";  // 경계 메시지의 프레임 설정

  // 파일에서 웨이포인트 읽기
  readWaypointFile();

  // 파일에서 경계 읽기
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  int wayPointID = 0;  // 현재 웨이포인트 ID
  int waypointSize = waypoints->points.size();  // 웨이포인트의 총 개수

  if (waypointSize == 0) {  // 웨이포인트가 없는 경우
    printf ("\nNo waypoint available, exit.\n\n");  // 오류 메시지 출력
    exit(1);  // 프로그램 종료
  }

  ros::Rate rate(100);  // 루프 주기를 설정
  bool status = ros::ok();  // ROS 상태 확인
  while (status) {
    ros::spinOnce();  // 콜백 함수 호출

    float disX = vehicleX - waypoints->points[wayPointID].x;  // 현재 위치와 웨이포인트의 X 거리 계산
    float disY = vehicleY - waypoints->points[wayPointID].y;  // 현재 위치와 웨이포인트의 Y 거리 계산
    float disZ = vehicleZ - waypoints->points[wayPointID].z;  // 현재 위치와 웨이포인트의 Z 거리 계산

    // 현재 웨이포인트에 도달했는지 확인하고 대기 시작
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
      waitTimeStart = curTime;  // 대기 시작 시간 설정
      isWaiting = true;  // 대기 상태로 전환
    }

    // 대기 시간이 지나면 다음 웨이포인트로 이동
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;  // 다음 웨이포인트로 이동
      isWaiting = false;  // 대기 상태 해제
    }

    // 일정 프레임 레이트에 맞춰 웨이포인트, 속도, 경계 메시지 퍼블리시
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);  // 현재 시간 설정
        waypointMsgs.point.x = waypoints->points[wayPointID].x;  // 웨이포인트 X 좌표 설정
        waypointMsgs.point.y = waypoints->points[wayPointID].y;  // 웨이포인트 Y 좌표 설정
        waypointMsgs.point.z = waypoints->points[wayPointID].z;  // 웨이포인트 Z 좌표 설정
        pubWaypoint.publish(waypointMsgs);  // 웨이포인트 퍼블리시
      }

      if (sendSpeed) {
        speedMsgs.data = speed;  // 속도 설정
        pubSpeed.publish(speedMsgs);  // 속도 퍼블리시
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);  // 현재 시간 설정
        pubBoundary.publish(boundaryMsgs);  // 경계 퍼블리시
      }

      waypointTime = curTime;  // 웨이포인트 시간 업데이트
    }

    status = ros::ok();  // ROS 상태 업데이트
    rate.sleep();  // 루프 주기 유지
  }

  return 0;  // 프로그램 종료
}
