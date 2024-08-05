#include <math.h> // 수학 함수들을 사용하기 위한 헤더 파일
#include <time.h> // 시간 관련 함수들을 사용하기 위한 헤더 파일
#include <stdio.h> // 표준 입출력 함수들을 사용하기 위한 헤더 파일
#include <stdlib.h> // 일반적인 유틸리티 함수들을 사용하기 위한 헤더 파일
#include <ros/ros.h> // ROS (Robot Operating System) 기능들을 사용하기 위한 헤더 파일

#include <message_filters/subscriber.h> // 메시지 필터를 사용하여 토픽을 구독하기 위한 헤더 파일
#include <message_filters/synchronizer.h> // 여러 토픽의 메시지를 동기화하기 위한 헤더 파일
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화 정책을 사용하기 위한 헤더 파일

#include <std_msgs/Int8.h> // ROS 표준 메시지 타입 Int8를 사용하기 위한 헤더 파일
#include <std_msgs/Float32.h> // ROS 표준 메시지 타입 Float32를 사용하기 위한 헤더 파일
#include <nav_msgs/Path.h> // 경로 정보를 담은 메시지를 사용하기 위한 헤더 파일
#include <nav_msgs/Odometry.h> // 로봇의 위치 및 자세 정보를 담은 메시지를 사용하기 위한 헤더 파일
#include <geometry_msgs/TwistStamped.h> // 기하학적 속도 정보를 담은 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/Imu.h> // 관성 측정 장치(IMU) 데이터를 담은 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 데이터를 담은 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/Joy.h> // 조이스틱 데이터를 담은 메시지를 사용하기 위한 헤더 파일

#include <tf/transform_datatypes.h> // 변환 관련 데이터 타입을 사용하기 위한 헤더 파일
#include <tf/transform_broadcaster.h> // 변환을 방송하기 위한 헤더 파일

#include <pcl_conversions/pcl_conversions.h> // PCL (Point Cloud Library)과 ROS 간의 변환을 위한 헤더 파일
#include <pcl/point_cloud.h> // PCL에서 포인트 클라우드를 사용하기 위한 헤더 파일
#include <pcl/point_types.h> // PCL에서 다양한 포인트 타입을 사용하기 위한 헤더 파일
#include <pcl/filters/voxel_grid.h> // PCL에서 볼륨 그리드 필터를 사용하기 위한 헤더 파일
#include <pcl/kdtree/kdtree_flann.h> // PCL에서 KD 트리를 사용하기 위한 헤더 파일

using namespace std; // 표준 네임스페이스 사용 선언

const double PI = 3.1415926; // 파이 상수 정의

double sensorOffsetX = 0; // 센서의 X축 오프셋
double sensorOffsetY = 0; // 센서의 Y축 오프셋
int pubSkipNum = 1; // 퍼블리시 생략 횟수
int pubSkipCount = 0; // 퍼블리시 생략 카운트
bool twoWayDrive = true; // 양방향 주행 여부
double lookAheadDis = 0.5; // 전방 주시 거리
double yawRateGain = 7.5; // 요율 이득
double stopYawRateGain = 7.5; // 정지 시 요율 이득
double maxYawRate = 45.0; // 최대 요율
double maxSpeed = 1.0; // 최대 속도
double maxAccel = 1.0; // 최대 가속도
double switchTimeThre = 1.0; // 전환 시간 임계값
double dirDiffThre = 0.1; // 방향 차이 임계값
double stopDisThre = 0.2; // 정지 거리 임계값
double slowDwnDisThre = 1.0; // 감속 거리 임계값
bool useInclRateToSlow = false; // 경사율을 사용한 감속 여부
double inclRateThre = 120.0; // 경사율 임계값
double slowRate1 = 0.25; // 감속율 1
double slowRate2 = 0.5; // 감속율 2
double slowTime1 = 2.0; // 감속 시간 1
double slowTime2 = 2.0; // 감속 시간 2
bool useInclToStop = false; // 경사를 사용한 정지 여부
double inclThre = 45.0; // 경사 임계값
double stopTime = 5.0; // 정지 시간
bool noRotAtStop = false; // 정지 시 회전 금지 여부
bool noRotAtGoal = true; // 목표 도착 시 회전 금지 여부
bool autonomyMode = false; // 자율 모드 여부
double autonomySpeed = 1.0; // 자율 모드 속도
double joyToSpeedDelay = 2.0; // 조이스틱 속도 지연 시간

float joySpeed = 0; // 조이스틱 속도
float joySpeedRaw = 0; // 조이스틱 원시 속도
float joyYaw = 0; // 조이스틱 요
int safetyStop = 0; // 안전 정지

float vehicleX = 0; // 차량의 X 좌표
float vehicleY = 0; // 차량의 Y 좌표
float vehicleZ = 0; // 차량의 Z 좌표
float vehicleRoll = 0; // 차량의 롤
float vehiclePitch = 0; // 차량의 피치
float vehicleYaw = 0; // 차량의 요

float vehicleXRec = 0; // 기록된 차량의 X 좌표
float vehicleYRec = 0; // 기록된 차량의 Y 좌표
float vehicleZRec = 0; // 기록된 차량의 Z 좌표
float vehicleRollRec = 0; // 기록된 차량의 롤
float vehiclePitchRec = 0; // 기록된 차량의 피치
float vehicleYawRec = 0; // 기록된 차량의 요

float vehicleYawRate = 0; // 차량의 요율
float vehicleSpeed = 0; // 차량의 속도

double odomTime = 0; // 오도메트리 시간
double joyTime = 0; // 조이스틱 시간
double slowInitTime = 0; // 감속 초기화 시간
double stopInitTime = 0; // 정지 초기화 시간
int pathPointID = 0; // 경로 포인트 ID
bool pathInit = false; // 경로 초기화 여부
bool navFwd = true; // 전방 주행 여부
double switchTime = 0; // 전환 시간

nav_msgs::Path path; // 경로 메시지

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec(); // 오도메트리 메시지의 시간 기록

  double roll, pitch, yaw; // 롤, 피치, 요 각도 변수 선언
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation; // 오도메트리에서 쿼터니언 추출
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw); // 쿼터니언을 RPY(롤, 피치, 요)로 변환

  vehicleRoll = roll; // 차량의 롤 각도 갱신
  vehiclePitch = pitch; // 차량의 피치 각도 갱신
  vehicleYaw = yaw; // 차량의 요 각도 갱신
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY; // 차량의 X 좌표 갱신 (센서 오프셋 보정)
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY; // 차량의 Y 좌표 갱신 (센서 오프셋 보정)
  vehicleZ = odomIn->pose.pose.position.z; // 차량의 Z 좌표 갱신

  // 롤 또는 피치가 임계값을 초과하고 경사를 사용하여 정지할 경우
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec(); // 정지 초기화 시간 갱신
  }

  // 각속도가 임계값을 초과하고 경사율을 사용하여 감속할 경우
  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec(); // 감속 초기화 시간 갱신
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size(); // 입력 경로의 크기
  path.poses.resize(pathSize); // 경로 포즈 크기 조정
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x; // 경로 포즈의 X 좌표 갱신
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y; // 경로 포즈의 Y 좌표 갱신
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z; // 경로 포즈의 Z 좌표 갱신
  }

  vehicleXRec = vehicleX; // 기록된 차량의 X 좌표 갱신
  vehicleYRec = vehicleY; // 기록된 차량의 Y 좌표 갱신
  vehicleZRec = vehicleZ; // 기록된 차량의 Z 좌표 갱신
  vehicleRollRec = vehicleRoll; // 기록된 차량의 롤 갱신
  vehiclePitchRec = vehiclePitch; // 기록된 차량의 피치 갱신
  vehicleYawRec = vehicleYaw; // 기록된 차량의 요 갱신

  pathPointID = 0; // 경로 포인트 ID 초기화
  pathInit = true; // 경로 초기화 플래그 설정
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec(); // 현재 시간을 조이스틱 시간으로 기록

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]); // 조이스틱 축 값으로부터 원시 속도 계산
  joySpeed = joySpeedRaw; // 조이스틱 속도 설정
  if (joySpeed > 1.0) joySpeed = 1.0; // 최대 속도를 1로 제한
  if (joy->axes[4] == 0) joySpeed = 0; // 축 값이 0일 경우 속도를 0으로 설정
  joyYaw = joy->axes[3]; // 조이스틱 요 값 설정
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0; // 속도가 0이고 정지 시 회전 금지일 경우 요 값을 0으로 설정

  // 조이스틱 축 값이 음수이고 양방향 주행이 불가능한 경우 속도와 요 값을 0으로 설정
  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  // 조이스틱의 특정 축 값이 -0.1보다 크면 자율 모드 비활성화, 그렇지 않으면 자율 모드 활성화
  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec(); // 현재 시간을 속도 시간으로 기록

  // 자율 모드가 활성화되고 조이스틱 속도가 0이며, 조이스틱 시간과 속도 시간이 지연 시간을 초과한 경우
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed; // 속도를 최대 속도로 정규화하여 조이스틱 속도로 설정

    // 속도를 0과 1 사이로 제한
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data; // 수신된 정지 명령 데이터를 안전 정지 변수에 저장
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower"); // ROS 초기화
  ros::NodeHandle nh; // 기본 NodeHandle 생성
  ros::NodeHandle nhPrivate("~"); // 개인 파라미터 NodeHandle 생성

  // 매개변수 가져오기
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX); // 센서의 X축 오프셋
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY); // 센서의 Y축 오프셋
  nhPrivate.getParam("pubSkipNum", pubSkipNum); // 퍼블리시 생략 횟수
  nhPrivate.getParam("twoWayDrive", twoWayDrive); // 양방향 주행 여부
  nhPrivate.getParam("lookAheadDis", lookAheadDis); // 전방 주시 거리
  nhPrivate.getParam("yawRateGain", yawRateGain); // 요율 이득
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain); // 정지 시 요율 이득
  nhPrivate.getParam("maxYawRate", maxYawRate); // 최대 요율
  nhPrivate.getParam("maxSpeed", maxSpeed); // 최대 속도
  nhPrivate.getParam("maxAccel", maxAccel); // 최대 가속도
  nhPrivate.getParam("switchTimeThre", switchTimeThre); // 전환 시간 임계값
  nhPrivate.getParam("dirDiffThre", dirDiffThre); // 방향 차이 임계값
  nhPrivate.getParam("stopDisThre", stopDisThre); // 정지 거리 임계값
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre); // 감속 거리 임계값
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow); // 경사율을 사용한 감속 여부
  nhPrivate.getParam("inclRateThre", inclRateThre); // 경사율 임계값
  nhPrivate.getParam("slowRate1", slowRate1); // 감속율 1
  nhPrivate.getParam("slowRate2", slowRate2); // 감속율 2
  nhPrivate.getParam("slowTime1", slowTime1); // 감속 시간 1
  nhPrivate.getParam("slowTime2", slowTime2); // 감속 시간 2
  nhPrivate.getParam("useInclToStop", useInclToStop); // 경사를 사용한 정지 여부
  nhPrivate.getParam("inclThre", inclThre); // 경사 임계값
  nhPrivate.getParam("stopTime", stopTime); // 정지 시간
  nhPrivate.getParam("noRotAtStop", noRotAtStop); // 정지 시 회전 금지 여부
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal); // 목표 도착 시 회전 금지 여부
  nhPrivate.getParam("autonomyMode", autonomyMode); // 자율 모드 여부
  nhPrivate.getParam("autonomySpeed", autonomySpeed); // 자율 모드 속도
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay); // 조이스틱 속도 지연 시간

  // 오도메트리 구독
  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);

  // 경로 구독
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);

  // 조이스틱 구독
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  // 속도 구독
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  // 정지 명령 구독
  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> ("/stop", 5, stopHandler);

  // 속도 퍼블리셔 설정
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle"; // 프레임 아이디 설정

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed; // 자율 모드 속도 설정

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100); // 루프 속도 설정 (100Hz)
  bool status = ros::ok(); // ROS 상태 확인
  while (status) {
    ros::spinOnce(); // 콜백 함수 실행

    if (pathInit) { // 경로가 초기화되었을 때
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec); // 상대 X 좌표 계산
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec); // 상대 Y 좌표 계산

      int pathSize = path.poses.size(); // 경로 크기
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel; // 경로 끝점의 X 거리
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel; // 경로 끝점의 Y 거리
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY); // 경로 끝점까지의 거리

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY); // 특정 경로 포인트까지의 거리 계산
        if (dis < lookAheadDis) {
          pathPointID++; // 경로 포인트 ID 증가
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY); // 현재 경로 포인트까지의 거리 계산
      float pathDir = atan2(disY, disX); // 경로 방향 계산

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir; // 방향 차이 계산
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) { // 양방향 주행일 경우
        double time = ros::Time::now().toSec(); // 현재 시간
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false; // 후진으로 전환
          switchTime = time; // 전환 시간 갱신
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true; // 전진으로 전환
          switchTime = time; // 전환 시간 갱신
        }
      }

      float joySpeed2 = maxSpeed * joySpeed; // 조이스틱 속도 계산
      if (!navFwd) { // 후진일 경우
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }
      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) 
        vehicleYawRate = -stopYawRateGain * dirDiff; // 차량 속도가 낮을 경우 정지 요율 이득을 사용하여 요율 설정
      else 
        vehicleYawRate = -yawRateGain * dirDiff; // 차량 속도가 높을 경우 일반 요율 이득을 사용하여 요율 설정

      // 최대 요율 제한
      if (vehicleYawRate > maxYawRate * PI / 180.0) 
        vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) 
        vehicleYawRate = -maxYawRate * PI / 180.0;

      // 자율 모드가 아니고 조이스틱 속도가 0일 경우 요율을 조이스틱 값으로 설정
      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } 
      // 경로 크기가 1 이하이거나, 목표에 도착하여 회전 금지일 경우 요율을 0으로 설정
      else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      // 경로 크기가 1 이하일 경우 속도를 0으로 설정
      if (pathSize <= 1) {
        joySpeed2 = 0;
      } 
      // 종료 지점까지의 거리가 감속 거리 임계값보다 작으면 속도를 조정
      else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2; // 조정된 속도 설정
      // 감속 초기화 시간과 현재 시간을 비교하여 감속률 적용
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) 
        joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) 
        joySpeed3 *= slowRate2;

      // 방향 차이가 작고 거리 임계값을 초과할 경우 가속/감속
      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) 
          vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) 
          vehicleSpeed -= maxAccel / 100.0;
      } 
      // 방향 차이가 크거나 거리가 임계값보다 작을 경우 감속
      else {
        if (vehicleSpeed > 0) 
          vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) 
          vehicleSpeed += maxAccel / 100.0;
      }

      // 정지 초기화 시간과 현재 시간을 비교하여 정지 유지
      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      // 안전 정지 신호에 따른 속도 및 요율 제어
      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      // 퍼블리시 생략 카운트 감소
      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime); // 현재 시간을 메시지에 설정
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) 
          cmd_vel.twist.linear.x = 0;
        else 
          cmd_vel.twist.linear.x = vehicleSpeed; // 선형 속도 설정
        cmd_vel.twist.angular.z = vehicleYawRate; // 각속도 설정
        pubSpeed.publish(cmd_vel); // 속도 명령 퍼블리시

        pubSkipCount = pubSkipNum; // 퍼블리시 생략 카운트 초기화
      }
    }

    status = ros::ok(); // ROS 상태 확인
    rate.sleep(); // 설정된 속도로 루프 속도 유지
  }

  return 0;
}




