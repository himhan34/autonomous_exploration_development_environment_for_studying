#include <math.h> // 수학 함수들을 사용하기 위한 헤더 파일
#include <ros/ros.h> // ROS (Robot Operating System) 기능들을 사용하기 위한 헤더 파일
#include <stdio.h> // 표준 입출력 함수들을 사용하기 위한 헤더 파일
#include <stdlib.h> // 일반적인 유틸리티 함수들을 사용하기 위한 헤더 파일
#include <time.h> // 시간 관련 함수들을 사용하기 위한 헤더 파일

#include <message_filters/subscriber.h> // 메시지 필터를 사용하여 토픽을 구독하기 위한 헤더 파일
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화 정책을 사용하기 위한 헤더 파일
#include <message_filters/synchronizer.h> // 여러 토픽의 메시지를 동기화하기 위한 헤더 파일

#include <nav_msgs/Odometry.h> // 로봇의 위치 및 자세 정보를 담은 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/Joy.h> // 조이스틱 데이터를 담은 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 데이터를 담은 메시지를 사용하기 위한 헤더 파일
#include <std_msgs/Float32.h> // ROS 표준 메시지 타입 Float32를 사용하기 위한 헤더 파일

#include <tf/transform_broadcaster.h> // 변환을 방송하기 위한 헤더 파일
#include <tf/transform_datatypes.h> // 변환 관련 데이터 타입을 사용하기 위한 헤더 파일

#include <pcl/filters/voxel_grid.h> // PCL에서 볼륨 그리드 필터를 사용하기 위한 헤더 파일
#include <pcl/kdtree/kdtree_flann.h> // PCL에서 KD 트리를 사용하기 위한 헤더 파일
#include <pcl/point_cloud.h> // PCL에서 포인트 클라우드를 사용하기 위한 헤더 파일
#include <pcl/point_types.h> // PCL에서 다양한 포인트 타입을 사용하기 위한 헤더 파일
#include <pcl_conversions/pcl_conversions.h> // PCL (Point Cloud Library)과 ROS 간의 변환을 위한 헤더 파일

using namespace std; // 표준 네임스페이스 사용 선언

const double PI = 3.1415926; // 파이 상수 정의

double scanVoxelSize = 0.05; // 스캔 볼륨 그리드의 크기
double decayTime = 2.0; // 데이터 감쇠 시간
double noDecayDis = 4.0; // 감쇠가 없는 거리
double clearingDis = 8.0; // 클리어링 거리
bool clearingCloud = false; // 클리어링 클라우드 사용 여부

// 사실 정렬 사용같은 경우, 아직 잘 모르겠음. 
// 여기서만 나오는 것 같은데, 궁금함. 
bool useSorting = true; // 정렬 사용 여부

//여기서 말하는 분위수는 z축에서 값들이 가장 큰 값들에 대한 ? 건가..
double quantileZ = 0.25; // Z 축 분위수

// 여기서 말하는 드롭 같은 경우, 기억하기로는 .....
// 물체가 떨어지는 걸로 기억함. 
bool considerDrop = false; // 드롭 고려 여부

bool limitGroundLift = false; // 지면 상승 제한 여부
double maxGroundLift = 0.15; // 최대 지면 상승 높이

// 사실 launch 파일을 보면, 동적인 장애물에 대해서 하는 걸 본 적이 있는데, 잘 모르겠음. 
// 그래서 이걸 어떻게 해야 할지 고민이 되더라고요.  
// 동적인 장애물에 대해서 한번 확인은 해봐야 할 것 같습니다. 
// 이게 그래도 재밌어 보이네용 
// 다이나믹 옵스터클이라니... 
// 여기서 동적 장애물 !? 어떻게 판단하나요. 
bool clearDyObs = false; // 동적 장애물 클리어링 여부
double minDyObsDis = 0.3; // 최소 동적 장애물 거리
double minDyObsAngle = 0; // 최소 동적 장애물 각도
double minDyObsRelZ = -0.5; // 최소 동적 장애물 상대 Z 값
double absDyObsRelZThre = 0.2; // 절대 동적 장애물 상대 Z 임계값
double minDyObsVFOV = -16.0; // 최소 동적 장애물 수직 시야각
double maxDyObsVFOV = 16.0; // 최대 동적 장애물 수직 시야각
int minDyObsPointNum = 1; // 최소 동적 장애물 포인트 수

bool noDataObstacle = false; // 데이터 없는 장애물 여부
int noDataBlockSkipNum = 0; // 데이터 없는 블록 생략 수
int minBlockPointNum = 10; // 최소 블록 포인트 수

// 또 궁금한게 이거에요. 
// 이거 보니까... 이걸 기준으로 포인트 클라우드를 인식하는 범위가 갈라지는 것 같더라고요. 
// 그래서 이걸 좀 나중에 꼭 봐야할 것 같습니다. 
double vehicleHeight = 1.5; // 차량 높이

int voxelPointUpdateThre = 100; // 볼륨 그리드 포인트 업데이트 임계값
double voxelTimeUpdateThre = 2.0; // 볼륨 그리드 시간 업데이트 임계값
double minRelZ = -1.5; // 최소 상대 Z 값
double maxRelZ = 0.2; // 최대 상대 Z 값\

// 여기서 말하는 disratioz는 뭔가요? 
double disRatioZ = 0.2; // 거리 비율 Z 값

// 지형 볼륨 그리드 파라미터
float terrainVoxelSize = 1.0; // 지형 볼륨 그리드 크기
int terrainVoxelShiftX = 0; // 지형 볼륨 그리드 X축 이동
int terrainVoxelShiftY = 0; // 지형 볼륨 그리드 Y축 이동
const int terrainVoxelWidth = 21; // 지형 볼륨 그리드 너비
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2; // 지형 볼륨 그리드 반너비
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 지형 볼륨 그리드 수

// 평면 볼륨 그리드 파라미터
float planarVoxelSize = 0.2; // 평면 볼륨 그리드 크기
const int planarVoxelWidth = 51; // 평면 볼륨 그리드 너비 
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2; // 평면 볼륨 그리드 반너비
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth; // 평면 볼륨 그리드 수


// 포인트 클라우드 객체 생성
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 원본 레이저 스캔 포인트 클라우드
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 잘린 레이저 스캔 포인트 클라우드
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 다운샘플링된 레이저 스캔 포인트 클라우드
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 지형 포인트 클라우드
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>()); // 지형 고도 포인트 클라우드
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum]; // 지형 볼륨 클라우드 배열, 각 지형 볼륨을 저장

int terrainVoxelUpdateNum[terrainVoxelNum] = {0}; // 각 지형 볼륨의 업데이트 횟수
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 각 지형 볼륨의 마지막 업데이트 시간
float planarVoxelElev[planarVoxelNum] = {0}; // 각 평면 볼륨의 고도 값

// 근데 여기서 왜 엣지 정보를 받아오는 건가요? 
int planarVoxelEdge[planarVoxelNum] = {0}; // 각 평면 볼륨의 엣지 정보
int planarVoxelDyObs[planarVoxelNum] = {0}; // 각 평면 볼륨의 동적 장애물 정보
// 몰랐는데, 동적 장애물을 고려를 하는 구나... 너무 슬프누. 
vector<float> planarPointElev[planarVoxelNum]; // 각 평면 볼륨의 포인트 고도 값을 저장하는 벡터

double laserCloudTime = 0; // 레이저 클라우드의 타임스탬프
bool newlaserCloud = false; // 새로운 레이저 클라우드 데이터 수신 여부

double systemInitTime = 0; // 시스템 초기화 시간
bool systemInited = false; // 시스템 초기화 여부
int noDataInited = 0; // 데이터 초기화 여부

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0; // 차량의 롤, 피치, 요 각도
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; // 차량의 X, Y, Z 위치

// 여기서 말하는 거, rec 은 나중 프로세스를 위해서 해야 하는 건가요?  
float vehicleXRec = 0, vehicleYRec = 0; // 기록된 차량의 X, Y 위치


float sinVehicleRoll = 0, cosVehicleRoll = 0; // 롤 각도의 사인 및 코사인 값
float sinVehiclePitch = 0, cosVehiclePitch = 0; // 피치 각도의 사인 및 코사인 값
float sinVehicleYaw = 0, cosVehicleYaw = 0; // 요 각도의 사인 및 코사인 값

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 다운샘플링을 위한 볼륨 그리드 필터



// 상태 추정 콜백 함수
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
  double roll, pitch, yaw; // 롤, 피치, 요 각도 변수 선언
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation; // 오도메트리 메시지에서 쿼터니언 추출
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw); // 쿼터니언을 RPY(롤, 피치, 요)로 변환

  vehicleRoll = roll; // 차량의 롤 각도 갱신
  vehiclePitch = pitch; // 차량의 피치 각도 갱신
  vehicleYaw = yaw; // 차량의 요 각도 갱신
  vehicleX = odom->pose.pose.position.x; // 차량의 X 좌표 갱신
  vehicleY = odom->pose.pose.position.y; // 차량의 Y 좌표 갱신
  vehicleZ = odom->pose.pose.position.z; // 차량의 Z 좌표 갱신

  sinVehicleRoll = sin(vehicleRoll); // 롤 각도의 사인 값 계산 및 저장
  cosVehicleRoll = cos(vehicleRoll); // 롤 각도의 코사인 값 계산 및 저장
  sinVehiclePitch = sin(vehiclePitch); // 피치 각도의 사인 값 계산 및 저장
  cosVehiclePitch = cos(vehiclePitch); // 피치 각도의 코사인 값 계산 및 저장
  sinVehicleYaw = sin(vehicleYaw); // 요 각도의 사인 값 계산 및 저장
  cosVehicleYaw = cos(vehicleYaw); // 요 각도의 코사인 값 계산 및 저장

  if (noDataInited == 0) { // 데이터가 초기화되지 않은 경우
    // 이게 데이터를 초기화 하는 이유가 뭔가요? 
    vehicleXRec = vehicleX; // 기록된 차량의 X 좌표 갱신
    vehicleYRec = vehicleY; // 기록된 차량의 Y 좌표 갱신
    noDataInited = 1; // 데이터 초기화 상태를 1로 설정
  }
  if (noDataInited == 1) { // 데이터가 초기화된 경우
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec)); // 기록된 위치와 현재 위치 간의 거리 계산
    if (dis >= noDecayDis) // 거리 값이 감쇠 거리 이상일 경우
      noDataInited = 2; // 데이터 초기화 상태를 2로 설정
  }
}


// 레이저 스캔 콜백 함수
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
  laserCloudTime = laserCloud2->header.stamp.toSec(); // 레이저 클라우드의 타임스탬프 저장

  if (!systemInited) { // 시스템이 초기화되지 않은 경우
    systemInitTime = laserCloudTime; // 시스템 초기화 시간 설정
    systemInited = true; // 시스템 초기화 상태 설정
  }

  laserCloud->clear(); // 원본 레이저 클라우드 초기화
  pcl::fromROSMsg(*laserCloud2, *laserCloud); // ROS 메시지를 PCL 포인트 클라우드로 변환

  pcl::PointXYZI point; // 포인트 객체 생성
  laserCloudCrop->clear(); // 잘린 레이저 클라우드 초기화
  int laserCloudSize = laserCloud->points.size(); // 레이저 클라우드의 포인트 개수 저장
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i]; // 현재 포인트 가져오기

    float pointX = point.x; // 포인트의 X 좌표
    float pointY = point.y; // 포인트의 Y 좌표
    float pointZ = point.z; // 포인트의 Z 좌표

    // 여기서 다음 10줄 같은 경우, 왜 이렇게 되는 거고 이걸 어떻게 해야 하는 건가요? 
    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY)); // 차량과 포인트 간의 거리 계산
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) { // 포인트가 유효한 범위 내에 있는지 확인
      point.x = pointX; // 포인트의 X 좌표 설정
      point.y = pointY; // 포인트의 Y 좌표 설정
      point.z = pointZ; // 포인트의 Z 좌표 설정
      point.intensity = laserCloudTime - systemInitTime; // 포인트의 강도 값을 타임스탬프 차이로 설정
      laserCloudCrop->push_back(point); // 잘린 레이저 클라우드에 포인트 추가
    }
  }

  newlaserCloud = true; // 새로운 레이저 클라우드 데이터가 수신되었음을 표시
}

// 조이스틱 콜백 함수
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
  if (joy->buttons[5] > 0.5) { // 조이스틱의 특정 버튼(여기서는 버튼 5)이 눌렸을 경우
    noDataInited = 0; // 데이터 초기화 상태를 0으로 설정
    clearingCloud = true; // 클라우드 클리어링 플래그를 설정
  }
}

// 클라우드 클리어링 콜백 함수
void clearingHandler(const std_msgs::Float32::ConstPtr &dis) {
  noDataInited = 0; // 데이터 초기화 상태를 0으로 설정
  clearingDis = dis->data; // 수신된 거리 데이터를 클리어링 거리로 설정
  clearingCloud = true; // 클라우드 클리어링 플래그를 설정
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "terrainAnalysis"); // ROS 노드 초기화 및 노드 이름 설정
  ros::NodeHandle nh; // 기본 NodeHandle 생성
  ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 개인 파라미터 NodeHandle 생성

  // 파라미터 서버에서 변수 값 가져오기
  nhPrivate.getParam("scanVoxelSize", scanVoxelSize); // 스캔 볼륨 그리드 크기
  nhPrivate.getParam("decayTime", decayTime); // 데이터 감쇠 시간
  nhPrivate.getParam("noDecayDis", noDecayDis); // 감쇠가 없는 거리
  nhPrivate.getParam("clearingDis", clearingDis); // 클리어링 거리
  nhPrivate.getParam("useSorting", useSorting); // 정렬 사용 여부
  nhPrivate.getParam("quantileZ", quantileZ); // Z 축 분위수
  nhPrivate.getParam("considerDrop", considerDrop); // 드롭 고려 여부
  nhPrivate.getParam("limitGroundLift", limitGroundLift); // 지면 상승 제한 여부
  nhPrivate.getParam("maxGroundLift", maxGroundLift); // 최대 지면 상승 높이
  nhPrivate.getParam("clearDyObs", clearDyObs); // 동적 장애물 클리어링 여부

// 그래요... 여기서는 동적 장애물을 어떻게 처리하는 지에 대해서 깊게 봐야 할 것 같습니다. 
  nhPrivate.getParam("minDyObsDis", minDyObsDis); // 최소 동적 장애물 거리
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle); // 최소 동적 장애물 각도
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ); // 최소 동적 장애물 상대 Z 값
  nhPrivate.getParam("absDyObsRelZThre", absDyObsRelZThre); // 절대 동적 장애물 상대 Z 임계값
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV); // 최소 동적 장애물 수직 시야각
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV); // 최대 동적 장애물 수직 시야각
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum); // 최소 동적 장애물 포인트 수
//////////////// 여기까지임. 

  nhPrivate.getParam("noDataObstacle", noDataObstacle); // 데이터 없는 장애물 여부
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum); // 데이터 없는 블록 생략 수
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum); // 최소 블록 포인트 수
  nhPrivate.getParam("vehicleHeight", vehicleHeight); // 차량 높이
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre); // 볼륨 그리드 포인트 업데이트 임계값
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre); // 볼륨 그리드 시간 업데이트 임계값
  nhPrivate.getParam("minRelZ", minRelZ); // 최소 상대 Z 값
  nhPrivate.getParam("maxRelZ", maxRelZ); // 최대 상대 Z 값
  nhPrivate.getParam("disRatioZ", disRatioZ); // 거리 비율 Z 값


  // 오도메트리 데이터를 구독하여 상태 추정 콜백 함수 호출
  ros::Subscriber subOdometry =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  // 레이저 클라우드 데이터를 구독하여 레이저 클라우드 콜백 함수 호출
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/registered_scan", 5, laserCloudHandler);

  // 조이스틱 데이터를 구독하여 조이스틱 콜백 함수 호출
  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  // 맵 클리어링 데이터를 구독하여 클리어링 콜백 함수 호출
  // 이게 어디에서... clearHandler를 .,... 받아오는지 기억이 안나유...
  ros::Subscriber subClearing =
      nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);

  // 지형 맵 데이터를 퍼블리시하기 위한 퍼블리셔 설정
  ros::Publisher pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

// terrainVoxelNum 만큼 terrainVoxelCloud 초기화 (PointCloud를 저장할 포인터 배열)
for (int i = 0; i < terrainVoxelNum; i++) {
  terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
}

// downSizeFilter에 scanVoxelSize로 Leaf Size 설정
downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

// ROS 루프 속도를 100Hz로 설정
ros::Rate rate(100);

// ROS 상태를 true로 설정
bool status = ros::ok();

// ROS 루프 시작
while (status) {
  ros::spinOnce(); // 콜백 함수 호출

  if (newlaserCloud) { // 새로운 레이저 클라우드 데이터가 있을 때
    newlaserCloud = false; // 새로운 데이터 플래그를 false로 설정

    // 지형 보셀 중심 좌표 계산
    // 아 보니까... 아... 
    // 근데 제가 아직 이게 이해가 안되는 게... 여기서 size와 shift에 대해서 아직은 잘 모르겠습니다. 
    float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

    // 차량이 지형 보셀 중심보다 왼쪽에 있을 때 보셀 이동
    // 근데 이게 보니까.... 음.... ?   
    // 아 결국 맞춰주는 거다? 이건가요. 
    while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        // 마지막 열의 포인터 저장
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
        // 오른쪽으로 한 칸씩 이동
        for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
        }
        // 첫 열에 마지막 열 포인터 할당 및 초기화
        terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud[indY]->clear();
      }
      terrainVoxelShiftX--; // X축 이동
      terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX; // 중심 좌표 갱신
    }

    // 차량이 지형 보셀 중심보다 오른쪽에 있을 때 보셀 이동
    while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        // 첫 열의 포인터 저장
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[indY];
        // 왼쪽으로 한 칸씩 이동
        for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
        }
        // 마지막 열에 첫 열 포인터 할당 및 초기화
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
      }
      terrainVoxelShiftX++; // X축 이동
      terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX; // 중심 좌표 갱신
    }

    // 차량이 지형 보셀 중심보다 아래에 있을 때 보셀 이동
    while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        // 마지막 행의 포인터 저장
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
        // 위로 한 칸씩 이동
        for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
        }
        // 첫 행에 마지막 행 포인터 할당 및 초기화
        terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
      }
      terrainVoxelShiftY--; // Y축 이동
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY; // 중심 좌표 갱신
    }

    // 차량이 지형 보셀 중심보다 위에 있을 때 보셀 이동
    while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        // 첫 행의 포인터 저장
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX];
        // 아래로 한 칸씩 이동
        for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
        }
        // 마지막 행에 첫 행 포인터 할당 및 초기화
        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
      }
      terrainVoxelShiftY++; // Y축 이동
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY; // 중심 좌표 갱신
    }


// 레이저 스캔 데이터를 쌓기 위한 변수를 선언
pcl::PointXYZI point; 
int laserCloudCropSize = laserCloudCrop->points.size(); // 레이저 클라우드의 포인트 수를 가져옴
for (int i = 0; i < laserCloudCropSize; i++) { // 각 포인트에 대해 루프
  point = laserCloudCrop->points[i]; // 현재 포인트를 가져옴

  // 보셀 인덱스를 계산
  int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
  int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;

  // 포인트가 보셀 크기의 절반보다 작은 경우 인덱스를 조정
  if (point.x - vehicleX + terrainVoxelSize / 2 < 0) indX--;
  if (point.y - vehicleY + terrainVoxelSize / 2 < 0) indY--;

  // 인덱스가 유효한 범위 내에 있는지 확인
  if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
    // 유효한 경우 해당 보셀에 포인트를 추가하고 업데이트 횟수를 증가
    terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
  }
}

// 보셀 데이터 갱신
for (int ind = 0; ind < terrainVoxelNum; ind++) { // 각 보셀에 대해 루프
  // 업데이트 조건이 충족되었는지 확인
  if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
      laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre ||
      clearingCloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind]; // 현재 보셀을 가져옴

    laserCloudDwz->clear(); // 다운샘플링된 클라우드를 초기화
    downSizeFilter.setInputCloud(terrainVoxelCloudPtr); // 다운샘플러에 현재 보셀을 입력
    downSizeFilter.filter(*laserCloudDwz); // 다운샘플링 수행

    terrainVoxelCloudPtr->clear(); // 현재 보셀을 초기화
    int laserCloudDwzSize = laserCloudDwz->points.size(); // 다운샘플링된 클라우드의 포인트 수를 가져옴
    for (int i = 0; i < laserCloudDwzSize; i++) { // 각 포인트에 대해 루프
      point = laserCloudDwz->points[i]; // 현재 포인트를 가져옴
      float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY)); // 포인트와 차량 간의 거리를 계산
      // 조건에 맞는 포인트만 현재 보셀에 추가
      if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
          point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
          (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
          !(dis < clearingDis && clearingCloud)) {
        terrainVoxelCloudPtr->push_back(point);
      }
    }

    terrainVoxelUpdateNum[ind] = 0; // 업데이트 횟수를 초기화
    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime; // 업데이트 시간을 갱신
  }
}

// 지형 클라우드 갱신
terrainCloud->clear(); // 지형 클라우드를 초기화
for (int indX = terrainVoxelHalfWidth - 5; indX <= terrainVoxelHalfWidth + 5; indX++) { // 주변 보셀을 순회
  for (int indY = terrainVoxelHalfWidth - 5; indY <= terrainVoxelHalfWidth + 5; indY++) {
    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY]; // 현재 보셀의 클라우드를 지형 클라우드에 추가
  }
}

// 지면 추정 및 각 점에 대한 고도 계산 초기화
for (int i = 0; i < planarVoxelNum; i++) {
  planarVoxelElev[i] = 0; // 고도 초기화
  planarVoxelEdge[i] = 0; // 엣지 초기화
  planarVoxelDyObs[i] = 0; // 동적 장애물 초기화
  planarPointElev[i].clear(); // 고도 점 리스트 초기화
}

int terrainCloudSize = terrainCloud->points.size(); // 지형 클라우드의 포인트 수를 가져옴
for (int i = 0; i < terrainCloudSize; i++) { // 각 포인트에 대해 루프
  point = terrainCloud->points[i]; // 현재 포인트를 가져옴

  // 보셀 인덱스를 계산
  int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
  int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

  // 포인트가 보셀 크기의 절반보다 작은 경우 인덱스를 조정
  if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
  if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

  // 포인트가 고도 범위 내에 있는지 확인
  if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
    // 주변 3x3 보셀에 포인트의 고도 값을 추가
    for (int dX = -1; dX <= 1; dX++) {
      for (int dY = -1; dY <= 1; dY++) {
        if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth) {
          planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
        }
      }
    }
  }

  // 동적 장애물 검출을 위한 처리
  // 여기야..!!! 여기라구... 
  // 이 코드 보면 동적인 장애물에 대해서 처리를 하는 것 같애. 
  // 그래서 이 코드를 보면, 동적인 장애물에 대해서 확인을 하쥬. 
  if (clearDyObs) {
    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
      float pointX1 = point.x - vehicleX; // 차량 기준 X 좌표
      float pointY1 = point.y - vehicleY; // 차량 기준 Y 좌표
      float pointZ1 = point.z - vehicleZ; // 차량 기준 Z 좌표

      float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1); // XY 평면 거리 계산
      if (dis1 > minDyObsDis) { // 최소 거리 조건 확인
        float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI; // 각도 계산
        if (angle1 > minDyObsAngle) { // 최소 각도 조건 확인
          // 차량의 회전 행렬을 적용하여 좌표 변환
          float pointX2 = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
          float pointY2 = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
          float pointZ2 = pointZ1;

          float pointX3 = pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
          float pointY3 = pointY2;
          float pointZ3 = pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

          float pointX4 = pointX3;
          float pointY4 = pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
          float pointZ4 = -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

          float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4); // 최종 거리 계산
          float angle4 = atan2(pointZ4, dis4) * 180.0 / PI; // 최종 각도 계산
          // 동적 장애물 조건 확인
          if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV || fabs(pointZ4) < absDyObsRelZThre) {
            planarVoxelDyObs[planarVoxelWidth * indX + indY]++; // 동적 장애물 수 증가
          }
        }
      } else {
        planarVoxelDyObs[planarVoxelWidth * indX + indY] += minDyObsPointNum; // 최소 동적 장애물 포인트 수 추가
      }
    }
  }
}


        // 동적 장애물 제거를 위한 처리
        // 근데 여기서 동적 장애물 제거를 해야 하는 건가요? 
        // 이번 랩 미팅때는 이걸 말씀 드려야 할 것 같음....  
       // 이 코드에서는 동적인 장애물을 인식을 하지만... 제거를 하는 것 같다. 
      // 그래서 이걸 인식하고... 이거에 대해서 속도 퍼블리싱 하는 것을 중요시 해야 한다...
    if (clearDyObs) {
      for (int i = 0; i < laserCloudCropSize; i++) { // 각 레이저 클라우드 포인트에 대해 루프
        point = laserCloudCrop->points[i]; // 현재 포인트를 가져옴
    
        // 보셀 인덱스 계산
        int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
        int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
    
        // 포인트가 보셀 크기의 절반보다 작은 경우 인덱스를 조정
        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;
    
        // 인덱스가 유효한 범위 내에 있는지 확인
        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
          // 차량 기준 좌표 계산
          float pointX1 = point.x - vehicleX;
          float pointY1 = point.y - vehicleY;
          float pointZ1 = point.z - vehicleZ;
    
          // XY 평면 거리와 각도 계산
          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
          
          // 각도가 최소 동적 장애물 각도보다 큰 경우
          // 그러면 최소 동적 장애물보다 크니... 0 이 되는 것 같네요. 허허   
        if (angle1 > minDyObsAngle) {
            planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0; // 해당 보셀의 동적 장애물 수를 0으로 설정
            
          }
        }
      }
    }
    
    // 고도 정렬을 사용하는 경우
    // 고도 정렬을 한다는 것은... 결국 이거를 넘지 말라는 뜻 같습니다. 
    if (useSorting) {
      for (int i = 0; i < planarVoxelNum; i++) { // 각 보셀에 대해 루프
        int planarPointElevSize = planarPointElev[i].size(); // 현재 보셀의 고도 포인트 수를 가져옴
        if (planarPointElevSize > 0) {
          // 고도 포인트를 정렬
          sort(planarPointElev[i].begin(), planarPointElev[i].end());
    
          int quantileID = int(quantileZ * planarPointElevSize); // 분위수 인덱스를 계산
          if (quantileID < 0) quantileID = 0; // 인덱스 조정
          else if (quantileID >= planarPointElevSize) quantileID = planarPointElevSize - 1;
    
          // 고도 리프트 제한 조건 확인
          if (planarPointElev[i][quantileID] > planarPointElev[i][0] + maxGroundLift && limitGroundLift) {
            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift; // 최대 고도 리프트 적용
          } else {
            planarVoxelElev[i] = planarPointElev[i][quantileID]; // 정렬된 고도 값을 설정
          }
        }
      }
    } else { // 고도 정렬을 사용하지 않는 경우
      for (int i = 0; i < planarVoxelNum; i++) { // 각 보셀에 대해 루프
        int planarPointElevSize = planarPointElev[i].size(); // 현재 보셀의 고도 포인트 수를 가져옴
        if (planarPointElevSize > 0) {
          float minZ = 1000.0; // 최소 고도를 저장할 변수
          int minID = -1; // 최소 고도의 인덱스를 저장할 변수
          for (int j = 0; j < planarPointElevSize; j++) { // 각 고도 포인트에 대해 루프
            if (planarPointElev[i][j] < minZ) {
              minZ = planarPointElev[i][j]; // 최소 고도를 갱신
              minID = j; // 최소 고도의 인덱스를 갱신
            }
          }
    
          if (minID != -1) { // 최소 고도 인덱스가 유효한 경우
            planarVoxelElev[i] = planarPointElev[i][minID]; // 최소 고도를 보셀 고도로 설정
          }
        }
      }
    }
        // 고도 클라우드를 초기화
    terrainCloudElev->clear();
    int terrainCloudElevSize = 0;
    for (int i = 0; i < terrainCloudSize; i++) { // 각 지형 클라우드 포인트에 대해 루프
      point = terrainCloud->points[i]; // 현재 포인트를 가져옴

      //아 .... 야발 그렇구나, 그러니까 이렇게 되는 구나 싶다. 
      // 결론적으로 포인트의 고도가 기준 최소 및 최대 고도 사이를 확인하고, 이걸 기준으로 한다... 야호... 
      // 포인트의 고도가 차량 기준 최소 및 최대 고도 사이에 있는지 확인
      // 역시 코드에 대해서는 확실히 이해를 하고 코드를 돌리는 것이 맞다는 걸 확인한 것 같다. 
      if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
      
        // 보셀 인덱스 계산
        int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
        int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
    
        // 포인트가 보셀 크기의 절반보다 작은 경우 인덱스를 조정
        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;
    
        // 인덱스가 유효한 범위 내에 있는지 확인
        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
          
          // 동적 장애물 포인트 수가 최소 동적 장애물 포인트 수보다 작거나 동적 장애물을 제거하지 않는 경우
          // 여기서도 동적인 장애물에 대해서 체크를 한다잉. 
          if (planarVoxelDyObs[planarVoxelWidth * indX + indY] < minDyObsPointNum || !clearDyObs) {
            float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY]; // 고도 차이 계산
            if (considerDrop) disZ = fabs(disZ); // 고도 차이를 절대값으로 변환
            int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size(); // 현재 보셀의 고도 포인트 수를 가져옴
            // 고도 차이가 0 이상 차량 높이 이하이고, 보셀의 고도 포인트 수가 최소 블록 포인트 수 이상인 경우
            if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum) {
              terrainCloudElev->push_back(point); // 고도 클라우드에 포인트 추가
              terrainCloudElev->points[terrainCloudElevSize].intensity = disZ; // 고도 차이를 인텐시티 값으로 설정
              terrainCloudElevSize++;
            }
          }
        }
      }
    }
    
    // 데이터가 없는 장애물 처리
    if (noDataObstacle && noDataInited == 2) {
      for (int i = 0; i < planarVoxelNum; i++) { // 각 보셀에 대해 루프
        int planarPointElevSize = planarPointElev[i].size(); // 현재 보셀의 고도 포인트 수를 가져옴
        if (planarPointElevSize < minBlockPointNum) {
          planarVoxelEdge[i] = 1; // 최소 블록 포인트 수보다 작은 경우 엣지로 설정
        }
      }
    
      for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < noDataBlockSkipNum; noDataBlockSkipCount++) {
        for (int i = 0; i < planarVoxelNum; i++) { // 각 보셀에 대해 루프
          if (planarVoxelEdge[i] >= 1) { // 엣지 보셀인 경우
            int indX = int(i / planarVoxelWidth); // 보셀의 X 인덱스 계산
            int indY = i % planarVoxelWidth; // 보셀의 Y 인덱스 계산
            bool edgeVoxel = false;
            // 주변 3x3 보셀에 대해 검사
            for (int dX = -1; dX <= 1; dX++) {
              for (int dY = -1; dY <= 1; dY++) {
                // 주변 보셀 인덱스가 유효한지 확인
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                  // 주변 보셀이 현재 보셀보다 엣지 값이 작은지 확인
                  if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY + dY] < planarVoxelEdge[i]) {
                    edgeVoxel = true;
                  }
                }
              }
            }
    
            // 엣지 보셀이 아닌 경우 엣지 값을 증가
            if (!edgeVoxel) planarVoxelEdge[i]++;
          }
        }
      }

for (int i = 0; i < planarVoxelNum; i++) { // 각 평면 보셀에 대해 루프
  if (planarVoxelEdge[i] > noDataBlockSkipNum) { // 보셀 엣지 값이 임계값보다 큰 경우
    int indX = int(i / planarVoxelWidth); // 보셀의 X 인덱스를 계산
    int indY = i % planarVoxelWidth; // 보셀의 Y 인덱스를 계산

    // 보셀 중심 좌표를 계산
    point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
    point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
    point.z = vehicleZ;
    point.intensity = vehicleHeight;

    // 보셀의 네 꼭지점 좌표를 설정
    point.x -= planarVoxelSize / 4.0;
    point.y -= planarVoxelSize / 4.0;
    terrainCloudElev->push_back(point); // 첫 번째 꼭지점 추가

    point.x += planarVoxelSize / 2.0;
    terrainCloudElev->push_back(point); // 두 번째 꼭지점 추가

    point.y += planarVoxelSize / 2.0;
    terrainCloudElev->push_back(point); // 세 번째 꼭지점 추가

    point.x -= planarVoxelSize / 2.0;
    terrainCloudElev->push_back(point); // 네 번째 꼭지점 추가
  }
}

clearingCloud = false; // 클라우드 정리를 완료

// 고도 정보를 포함한 포인트 클라우드를 퍼블리시
sensor_msgs::PointCloud2 terrainCloud2;
pcl::toROSMsg(*terrainCloudElev, terrainCloud2); // PCL 클라우드를 ROS 메시지로 변환
terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime); // 타임스탬프 설정
terrainCloud2.header.frame_id = "map"; // 프레임 ID 설정
pubLaserCloud.publish(terrainCloud2); // 포인트 클라우드 퍼블리시

status = ros::ok(); // ROS 상태를 업데이트
rate.sleep(); // 루프 주기 조절
}

return 0; // 프로그램 종료
}



          
