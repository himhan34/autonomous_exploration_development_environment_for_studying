#include <math.h>  // 수학 라이브러리를 포함합니다.
#include <ros/ros.h>  // ROS 기본 기능을 포함합니다.
#include <stdio.h>  // 표준 입출력 라이브러리를 포함합니다.
#include <stdlib.h>  // 표준 라이브러리를 포함합니다.
#include <time.h>  // 시간 관련 라이브러리를 포함합니다.
#include <queue>  // 큐 자료 구조를 포함합니다.

#include <message_filters/subscriber.h>  // 메시지 필터의 구독자를 포함합니다.
#include <message_filters/sync_policies/approximate_time.h>  // 근사 시간 동기화 정책을 포함합니다.
#include <message_filters/synchronizer.h>  // 메시지 동기화기를 포함합니다.

#include <nav_msgs/Odometry.h>  // 내비게이션 메시지의 오도메트리를 포함합니다.
#include <sensor_msgs/Joy.h>  // 센서 메시지의 조이스틱 데이터를 포함합니다.
#include <sensor_msgs/PointCloud2.h>  // 센서 메시지의 포인트 클라우드 데이터를 포함합니다.
#include <std_msgs/Float32.h>  // 표준 메시지의 Float32 데이터를 포함합니다.
#include <std_msgs/Float32MultiArray.h>  // 표준 메시지의 Float32 배열 데이터를 포함합니다.

#include <tf/transform_broadcaster.h>  // TF 변환 브로드캐스터를 포함합니다.
#include <tf/transform_datatypes.h>  // TF 변환 데이터 타입을 포함합니다.

#include <pcl/filters/voxel_grid.h>  // PCL 필터의 보셀 그리드를 포함합니다.
#include <pcl/kdtree/kdtree_flann.h>  // PCL KD 트리의 FLANN을 포함합니다.
#include <pcl/point_cloud.h>  // PCL 포인트 클라우드를 포함합니다.
#include <pcl/point_types.h>  // PCL 포인트 타입을 포함합니다.
#include <pcl_conversions/pcl_conversions.h>  // PCL 변환을 포함합니다.

using namespace std;  // 표준 네임스페이스를 사용합니다.

const double PI = 3.1415926;  // 파이 상수를 정의합니다.

double scanVoxelSize = 0.1;  // 스캔 보셀 크기를 설정합니다.
double decayTime = 10.0;  // 소멸 시간을 설정합니다.
double noDecayDis = 0;  // 소멸하지 않는 거리를 설정합니다.
double clearingDis = 30.0;  // 클리어링 거리를 설정합니다.
bool clearingCloud = false;  // 클리어링 클라우드 사용 여부를 설정합니다.
bool useSorting = false;  // 정렬 사용 여부를 설정합니다.
double quantileZ = 0.25;  // Z 축의 분위수를 설정합니다.
double vehicleHeight = 1.5;  // 차량 높이를 설정합니다.
int voxelPointUpdateThre = 100;  // 보셀 포인트 업데이트 임계값을 설정합니다.
double voxelTimeUpdateThre = 2.0;  // 보셀 시간 업데이트 임계값을 설정합니다.
double lowerBoundZ = -1.5;  // Z 축의 하한값을 설정합니다.
double upperBoundZ = 1.0;  // Z 축의 상한값을 설정합니다.
double disRatioZ = 0.1;  // Z 축 거리 비율을 설정합니다.
bool checkTerrainConn = true;  // 지형 연결성 확인 여부를 설정합니다.
double terrainUnderVehicle = -0.75;  // 차량 아래의 지형 값을 설정합니다.
double terrainConnThre = 0.5;  // 지형 연결성 임계값을 설정합니다.
double ceilingFilteringThre = 2.0;  // 천장 필터링 임계값을 설정합니다.
double localTerrainMapRadius = 4.0;  // 로컬 지형 맵 반경을 설정합니다.

// 지형 보셀 파라미터
float terrainVoxelSize = 2.0;  // 지형 보셀 크기를 설정합니다.
int terrainVoxelShiftX = 0;  // 지형 보셀 X축 이동값을 설정합니다.
int terrainVoxelShiftY = 0;  // 지형 보셀 Y축 이동값을 설정합니다.
const int terrainVoxelWidth = 41;  // 지형 보셀 너비를 설정합니다.
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;  // 지형 보셀 반 너비를 설정합니다.
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;  // 지형 보셀 개수를 설정합니다.


// 평면 보셀 파라미터
float planarVoxelSize = 0.4;  // 평면 보셀 크기를 설정합니다.
const int planarVoxelWidth = 101;  // 평면 보셀 너비를 설정합니다.
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;  // 평면 보셀 반 너비를 설정합니다.
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;  // 평면 보셀 개수를 설정합니다.

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());  // 레이저 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());  // 잘린 레이저 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());  // 다운샘플된 레이저 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());  // 지형 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());  // 고도 지형 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());  // 로컬 지형 클라우드를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];  // 지형 보셀 클라우드 배열을 초기화합니다.

int terrainVoxelUpdateNum[terrainVoxelNum] = { 0 };  // 지형 보셀 업데이트 개수 배열을 초기화합니다.
float terrainVoxelUpdateTime[terrainVoxelNum] = { 0 };  // 지형 보셀 업데이트 시간을 초기화합니다.
float planarVoxelElev[planarVoxelNum] = { 0 };  // 평면 보셀 고도 배열을 초기화합니다.
int planarVoxelConn[planarVoxelNum] = { 0 };  // 평면 보셀 연결 배열을 초기화합니다.
vector<float> planarPointElev[planarVoxelNum];  // 평면 포인트 고도 벡터 배열을 초기화합니다.
queue<int> planarVoxelQueue;  // 평면 보셀 큐를 초기화합니다.

double laserCloudTime = 0;  // 레이저 클라우드 시간을 초기화합니다.
bool newlaserCloud = false;  // 새로운 레이저 클라우드 여부를 초기화합니다.

double systemInitTime = 0;  // 시스템 초기화 시간을 초기화합니다.
bool systemInited = false;  // 시스템 초기화 여부를 초기화합니다.

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;  // 차량의 롤, 피치, 요 값을 초기화합니다.
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;  // 차량의 X, Y, Z 위치를 초기화합니다.

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;  // 다운사이즈 필터를 초기화합니다.
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;  // KD 트리를 초기화합니다.

// 상태 추정 콜백 함수
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;  // 롤, 피치, 요 값을 저장할 변수를 선언합니다.
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;  // 오도메트리 메시지에서 쿼터니언을 가져옵니다.
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);  // 쿼터니언을 롤, 피치, 요로 변환합니다.

  vehicleRoll = roll;  // 차량의 롤 값을 업데이트합니다.
  vehiclePitch = pitch;  // 차량의 피치 값을 업데이트합니다.
  vehicleYaw = yaw;  // 차량의 요 값을 업데이트합니다.
  vehicleX = odom->pose.pose.position.x;  // 차량의 X 위치를 업데이트합니다.
  vehicleY = odom->pose.pose.position.y;  // 차량의 Y 위치를 업데이트합니다.
  vehicleZ = odom->pose.pose.position.z;  // 차량의 Z 위치를 업데이트합니다.
}


// 등록된 레이저 스캔 콜백 함수
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudTime = laserCloud2->header.stamp.toSec();  // 레이저 클라우드 메시지의 시간을 가져옵니다.

  if (!systemInited)  // 시스템이 초기화되지 않았으면
  {
    systemInitTime = laserCloudTime;  // 시스템 초기화 시간을 설정합니다.
    systemInited = true;  // 시스템을 초기화 상태로 변경합니다.
  }

  laserCloud->clear();  // 기존 레이저 클라우드를 비웁니다.
  pcl::fromROSMsg(*laserCloud2, *laserCloud);  // ROS 메시지를 PCL 포인트 클라우드로 변환합니다.

  pcl::PointXYZI point;  // 포인트 객체를 선언합니다.
  laserCloudCrop->clear();  // 잘린 레이저 클라우드를 비웁니다.
  int laserCloudSize = laserCloud->points.size();  // 레이저 클라우드의 포인트 개수를 가져옵니다.
  for (int i = 0; i < laserCloudSize; i++)  // 모든 포인트를 순회합니다.
  {
    point = laserCloud->points[i];  // 현재 포인트를 가져옵니다.

    float pointX = point.x;  // 포인트의 X 좌표를 가져옵니다.
    float pointY = point.y;  // 포인트의 Y 좌표를 가져옵니다.
    float pointZ = point.z;  // 포인트의 Z 좌표를 가져옵니다.

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));  // 포인트와 차량 간의 거리를 계산합니다.
    if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis && pointZ - vehicleZ < upperBoundZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1))  // 포인트가 유효 범위 내에 있는지 확인합니다.
    {
      point.x = pointX;  // 포인트의 X 좌표를 설정합니다.
      point.y = pointY;  // 포인트의 Y 좌표를 설정합니다.
      point.z = pointZ;  // 포인트의 Z 좌표를 설정합니다.
      point.intensity = laserCloudTime - systemInitTime;  // 포인트의 강도를 설정합니다.
      laserCloudCrop->push_back(point);  // 잘린 레이저 클라우드에 포인트를 추가합니다.
    }
  }

  newlaserCloud = true;  // 새로운 레이저 클라우드가 존재함을 표시합니다.
}


// 로컬 지형 클라우드 콜백 함수
void terrainCloudLocalHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloudLocal2)
{
  terrainCloudLocal->clear();  // 기존 로컬 지형 클라우드를 비웁니다.
  pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);  // ROS 메시지를 PCL 포인트 클라우드로 변환합니다.
}

// 조이스틱 콜백 함수
void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->buttons[5] > 0.5)  // 조이스틱의 특정 버튼이 눌렸는지 확인합니다.
  {
    clearingCloud = true;  // 클라우드를 클리어링하도록 설정합니다.
  }
}

// 클라우드 클리어링 콜백 함수
void clearingHandler(const std_msgs::Float32::ConstPtr& dis)
{
  clearingDis = dis->data;  // 클리어링 거리를 업데이트합니다.
  clearingCloud = true;  // 클라우드를 클리어링하도록 설정합니다.
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "terrainAnalysisExt");  // ROS 노드를 초기화합니다.
  ros::NodeHandle nh;  // 기본 노드 핸들을 생성합니다.
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");  // 개인 노드 핸들을 생성합니다 (노드의 파라미터를 읽기 위함).

  // 개인 노드 핸들을 통해 파라미터 서버에서 파라미터를 가져옵니다.
  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);  // 스캔 보셀 크기를 가져옵니다.
  nhPrivate.getParam("decayTime", decayTime);  // 소멸 시간을 가져옵니다.
  nhPrivate.getParam("noDecayDis", noDecayDis);  // 소멸하지 않는 거리를 가져옵니다.
  nhPrivate.getParam("clearingDis", clearingDis);  // 클리어링 거리를 가져옵니다.
  nhPrivate.getParam("useSorting", useSorting);  // 정렬 사용 여부를 가져옵니다.
  nhPrivate.getParam("quantileZ", quantileZ);  // Z 축의 분위수를 가져옵니다.
  nhPrivate.getParam("vehicleHeight", vehicleHeight);  // 차량 높이를 가져옵니다.
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);  // 보셀 포인트 업데이트 임계값을 가져옵니다.
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);  // 보셀 시간 업데이트 임계값을 가져옵니다.
  nhPrivate.getParam("lowerBoundZ", lowerBoundZ);  // Z 축의 하한값을 가져옵니다.
  nhPrivate.getParam("upperBoundZ", upperBoundZ);  // Z 축의 상한값을 가져옵니다.
  nhPrivate.getParam("disRatioZ", disRatioZ);  // Z 축 거리 비율을 가져옵니다.
  nhPrivate.getParam("checkTerrainConn", checkTerrainConn);  // 지형 연결성 확인 여부를 가져옵니다.
  nhPrivate.getParam("terrainUnderVehicle", terrainUnderVehicle);  // 차량 아래의 지형 값을 가져옵니다.
  nhPrivate.getParam("terrainConnThre", terrainConnThre);  // 지형 연결성 임계값을 가져옵니다.
  nhPrivate.getParam("ceilingFilteringThre", ceilingFilteringThre);  // 천장 필터링 임계값을 가져옵니다.
  nhPrivate.getParam("localTerrainMapRadius", localTerrainMapRadius);  // 로컬 지형 맵 반경을 가져옵니다.

  // 오도메트리 메시지를 구독합니다.
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  // 등록된 레이저 스캔 메시지를 구독합니다.
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  // 조이스틱 메시지를 구독합니다.
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  // 클라우드 클리어링 메시지를 구독합니다.
  ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("/cloud_clearing", 5, clearingHandler);

  // 로컬 지형 클라우드 메시지를 구독합니다.
  ros::Subscriber subTerrainCloudLocal = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudLocalHandler);

  // 지형 클라우드 메시지를 발행할 퍼블리셔를 생성합니다.
  ros::Publisher pubTerrainCloud = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map_ext", 2);



  // 각 지형 보셀에 대한 포인트 클라우드를 초기화합니다.
  for (int i = 0; i < terrainVoxelNum; i++)
  {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());  // 새로운 포인트 클라우드를 생성하고 지형 보셀 배열에 저장합니다.
  }

  // 다운사이즈 필터의 리프 크기를 설정합니다. 
  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);  // X, Y, Z 방향으로의 리프 크기를 설정하여 포인트 클라우드를 다운사이징합니다.

  std::vector<int> pointIdxNKNSearch;  // KNN 검색을 위한 포인트 인덱스 벡터를 선언합니다.
  std::vector<float> pointNKNSquaredDistance;  // KNN 검색을 위한 거리 벡터를 선언합니다.

  ros::Rate rate(100);  // 루프 주기를 100Hz로 설정합니다.
  bool status = ros::ok();  // ROS가 정상적으로 작동 중인지 확인합니다.
  while (status)
  {
    ros::spinOnce();  // 콜백 함수가 호출되도록 합니다. 이 함수는 구독된 메시지가 수신되었을 때 해당 콜백을 실행합니다.

    if (newlaserCloud)  // 새로운 레이저 클라우드 데이터가 있으면
    {
      newlaserCloud = false;  // 새로운 레이저 클라우드 플래그를 리셋하여 처리 중임을 표시합니다.

      // 지형 보셀을 차량의 위치에 맞게 이동시키는 과정입니다.
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;  // 지형 보셀의 X 축 중심 좌표를 계산합니다.
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;  // 지형 보셀의 Y 축 중심 좌표를 계산합니다.

      // 차량이 지형 보셀의 왼쪽 경계를 넘어갔을 때
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          // 현재 가장 오른쪽 열의 보셀 클라우드를 저장합니다.
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          // 오른쪽 열의 클라우드를 왼쪽으로 이동합니다.
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          // 가장 오른쪽 열에 저장된 클라우드를 가장 왼쪽 열로 이동합니다.
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();  // 기존의 클라우드를 비웁니다.
        }
        terrainVoxelShiftX--;  // 지형 보셀의 X 이동 값을 감소시킵니다.
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;  // 새로운 X 중심 좌표를 계산합니다.
      }

      // 차량이 지형 보셀의 오른쪽 경계를 넘어갔을 때
      while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          // 현재 가장 왼쪽 열의 보셀 클라우드를 저장합니다.
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[indY];
          // 왼쪽 열의 클라우드를 오른쪽으로 이동합니다.
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          // 가장 왼쪽 열에 저장된 클라우드를 가장 오른쪽 열로 이동합니다.
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();  // 기존의 클라우드를 비웁니다.
        }
        terrainVoxelShiftX++;  // 지형 보셀의 X 이동 값을 증가시킵니다.
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;  // 새로운 X 중심 좌표를 계산합니다.
      }

      // 차량이 지형 보셀의 아래쪽 경계를 넘어갔을 때
      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          // 현재 가장 오른쪽 행의 보셀 클라우드를 저장합니다.
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          // 오른쪽 행의 클라우드를 아래쪽으로 이동합니다.
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          // 가장 오른쪽 행에 저장된 클라우드를 가장 위쪽 행으로 이동합니다.
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();  // 기존의 클라우드를 비웁니다.
        }
        terrainVoxelShiftY--;  // 지형 보셀의 Y 이동 값을 감소시킵니다.
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;  // 새로운 Y 중심 좌표를 계산합니다.
      }

      // 차량이 지형 보셀의 위쪽 경계를 넘어갔을 때
      while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          // 현재 가장 위쪽 행의 보셀 클라우드를 저장합니다.
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * indX];
          // 위쪽 행의 클라우드를 아래쪽으로 이동합니다.
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          // 가장 위쪽 행에 저장된 클라우드를 가장 아래쪽 행으로 이동합니다.
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();  // 기존의 클라우드를 비웁니다.
        }
        terrainVoxelShiftY++;  // 지형 보셀의 Y 이동 값을 증가시킵니다.
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;  // 새로운 Y 중심 좌표를 계산합니다.
      }










  // 레이저 클라우드를 스택에 쌓는 과정
  pcl::PointXYZI point;  // 포인트 변수 선언
  int laserCloudCropSize = laserCloudCrop->points.size();  // 레이저 클라우드의 크기
  for (int i = 0; i < laserCloudCropSize; i++)
  {
    point = laserCloudCrop->points[i];  // 포인트를 추출

    // 포인트의 위치를 기반으로 지형 보셀의 인덱스를 계산
    int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
    int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;

    // 좌표가 음수일 경우 인덱스를 조정
    if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
      indX--;
    if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
      indY--;

    // 인덱스가 유효한 범위 내에 있는지 확인
    if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth)
    {
      // 포인트를 해당 보셀에 추가하고 업데이트 수를 증가
      terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
      terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
    }
  }

  // 지형 보셀을 업데이트하는 과정
  for (int ind = 0; ind < terrainVoxelNum; ind++)
  {
    // 포인트 업데이트 수가 임계값을 초과하거나, 시간 경과 또는 클라우드 정리가 필요한 경우
    if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
        laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre || clearingCloud)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];  // 현재 보셀 클라우드를 가져옴

      laserCloudDwz->clear();  // 다운사이즈 필터링을 위한 클라우드 초기화
      downSizeFilter.setInputCloud(terrainVoxelCloudPtr);  // 필터에 입력 클라우드를 설정
      downSizeFilter.filter(*laserCloudDwz);  // 다운사이즈 필터를 적용

      terrainVoxelCloudPtr->clear();  // 현재 보셀 클라우드 비우기
      int laserCloudDwzSize = laserCloudDwz->points.size();  // 다운사이즈 클라우드의 크기
      for (int i = 0; i < laserCloudDwzSize; i++)
      {
        point = laserCloudDwz->points[i];  // 포인트를 추출
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));  // 차량과 포인트 간 거리 계산

        // 포인트가 유효한 높이 범위 내에 있는지 확인하고, 필요시 포인트를 클라우드에 추가
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
            (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
            !(dis < clearingDis && clearingCloud))
        {
          terrainVoxelCloudPtr->push_back(point);
        }
      }

      // 보셀 업데이트 수 및 시간을 리셋
      terrainVoxelUpdateNum[ind] = 0;
      terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
    }
  }

  // 전체 지형 클라우드를 갱신
  terrainCloud->clear();  // 지형 클라우드를 초기화
  for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++)
  {
    for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++)
    {
      // 특정 범위 내의 모든 보셀을 결합하여 지형 클라우드를 업데이트
      *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
    }
  }

  // 지면을 추정하고 각 포인트에 대한 높이를 계산
  for (int i = 0; i < planarVoxelNum; i++)
  {
    planarVoxelElev[i] = 0;  // 평면 보셀의 높이를 초기화
    planarVoxelConn[i] = 0;  // 평면 보셀의 연결 수를 초기화
    planarPointElev[i].clear();  // 평면 보셀의 포인트 높이 리스트를 초기화
  }

  int terrainCloudSize = terrainCloud->points.size();  // 지형 클라우드의 크기
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];  // 포인트를 추출
    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));  // 차량과 포인트 간 거리 계산

    // 포인트가 유효한 높이 범위 내에 있는지 확인
    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis)
    {
      // 포인트의 위치를 기반으로 평면 보셀의 인덱스를 계산
      int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
      int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

      // 좌표가 음수일 경우 인덱스를 조정
      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      // 주변의 평면 보셀에 포인트 높이를 추가
      for (int dX = -1; dX <= 1; dX++)
      {
        for (int dY = -1; dY <= 1; dY++)
        {
          // 인덱스가 유효한 범위 내에 있는지 확인
          if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
          {
            planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);  // 포인트 높이를 추가
              }
            }
          }
        }
      }



        // 평면 보셀의 높이를 계산하는 방법을 결정합니다.
  if (useSorting)
  {
    // 정렬을 사용하는 경우
    for (int i = 0; i < planarVoxelNum; i++)
    {
      int planarPointElevSize = planarPointElev[i].size();  // 현재 평면 보셀의 포인트 높이 리스트 크기
      if (planarPointElevSize > 0)  // 포인트가 존재하는 경우
      {
        // 포인트의 높이를 정렬합니다.
        sort(planarPointElev[i].begin(), planarPointElev[i].end());

        // 분위수(quantile) 위치를 계산합니다.
        int quantileID = int(quantileZ * planarPointElevSize);
        if (quantileID < 0)
          quantileID = 0;  // 분위수 ID가 0 미만일 경우
        else if (quantileID >= planarPointElevSize)
          quantileID = planarPointElevSize - 1;  // 분위수 ID가 최대 크기보다 클 경우

        // 분위수 위치에 해당하는 높이를 보셀 높이로 설정합니다.
        planarVoxelElev[i] = planarPointElev[i][quantileID];
      }
    }
  }
  else
  {
    // 정렬을 사용하지 않는 경우
    for (int i = 0; i < planarVoxelNum; i++)
    {
      int planarPointElevSize = planarPointElev[i].size();  // 현재 평면 보셀의 포인트 높이 리스트 크기
      if (planarPointElevSize > 0)  // 포인트가 존재하는 경우
      {
        // 가장 낮은 높이를 찾기 위한 초기화
        float minZ = 1000.0;
        int minID = -1;
        for (int j = 0; j < planarPointElevSize; j++)
        {
          // 최소 높이와 해당 인덱스를 찾습니다.
          if (planarPointElev[i][j] < minZ)
          {
            minZ = planarPointElev[i][j];
            minID = j;
          }
        }

        // 가장 낮은 높이를 보셀 높이로 설정합니다.
        if (minID != -1)
        {
          planarVoxelElev[i] = planarPointElev[i][minID];
        }
      }
    }
  }

  // 지형의 연결성을 확인하여 천장을 제거합니다.
  if (checkTerrainConn)
  {
    // 중앙 평면 보셀을 시작점으로 설정
    int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
    if (planarPointElev[ind].size() == 0)
      planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;  // 포인트가 없는 경우 초기 높이 설정

    planarVoxelQueue.push(ind);  // 큐에 시작점 추가
    planarVoxelConn[ind] = 1;  // 시작점의 연결 상태를 1로 설정

    // BFS를 통해 지형 연결성 확인
    while (!planarVoxelQueue.empty())
    {
      int front = planarVoxelQueue.front();  // 큐에서 앞의 인덱스를 가져옴
      planarVoxelConn[front] = 2;  // 연결 상태를 2로 설정하여 방문 완료 표시
      planarVoxelQueue.pop();  // 큐에서 제거

      // 현재 평면 보셀의 X, Y 좌표를 계산
      int indX = int(front / planarVoxelWidth);
      int indY = front % planarVoxelWidth;

      // 주어진 범위 내의 이웃 보셀을 검사
      for (int dX = -10; dX <= 10; dX++)
      {
        for (int dY = -10; dY <= 10; dY++)
        {
          if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
          {
            // 이웃 보셀의 인덱스를 계산
            ind = planarVoxelWidth * (indX + dX) + indY + dY;

            if (planarVoxelConn[ind] == 0 && planarPointElev[ind].size() > 0)  // 아직 방문하지 않았고 포인트가 있는 경우
            {
              // 현재 보셀과 이웃 보셀 간의 높이 차이를 확인
              if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) < terrainConnThre)
              {
                // 높이 차이가 임계값 이하인 경우, 이웃 보셀을 큐에 추가하고 연결 상태를 1로 설정
                planarVoxelQueue.push(ind);
                planarVoxelConn[ind] = 1;
              }
              else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) > ceilingFilteringThre)
              {
                // 높이 차이가 천장 필터링 임계값을 초과하는 경우, 이웃 보셀의 연결 상태를 -1로 설정
                planarVoxelConn[ind] = -1;
                  }
                }
              }
            }
          }
        }
      }
    
    

    // 지역 지형 맵 반경(localTerrainMapRadius) 너머의 지형 맵을 계산합니다.
    terrainCloudElev->clear();  // 지형 클라우드 높이(terrainCloudElev) 컨테이너를 비웁니다.
    int terrainCloudElevSize = 0;  // 지형 클라우드 높이의 크기를 0으로 초기화합니다.
    for (int i = 0; i < terrainCloudSize; i++)  // 지형 클라우드의 모든 포인트를 반복합니다.
    {
        point = terrainCloud->points[i];  // 현재 포인트를 가져옵니다.
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));  // 차량과 포인트 간의 거리 계산
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis && dis > localTerrainMapRadius)  // 높이와 거리 조건을 만족하는지 확인
        {
            int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;  // X 좌표의 인덱스 계산
            int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;  // Y 좌표의 인덱스 계산
    
            if (point.x - vehicleX + planarVoxelSize / 2 < 0)  // X 좌표가 음수일 경우 인덱스 조정
                indX--;
            if (point.y - vehicleY + planarVoxelSize / 2 < 0)  // Y 좌표가 음수일 경우 인덱스 조정
                indY--;
    
            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth)  // 인덱스가 유효한 범위 내에 있는지 확인
            {
                int ind = planarVoxelWidth * indX + indY;  // 2D 인덱스를 1D 인덱스로 변환
                float disZ = fabs(point.z - planarVoxelElev[ind]);  // Z 축 방향으로의 거리 계산
                if (disZ < vehicleHeight && (planarVoxelConn[ind] == 2 || !checkTerrainConn))  // Z 축 거리와 연결 조건을 만족하는지 확인
                {
                    terrainCloudElev->push_back(point);  // 포인트를 지형 클라우드 높이에 추가
                    terrainCloudElev->points[terrainCloudElevSize].x = point.x;  // X 좌표 설정
                    terrainCloudElev->points[terrainCloudElevSize].y = point.y;  // Y 좌표 설정
                    terrainCloudElev->points[terrainCloudElevSize].z = point.z;  // Z 좌표 설정
                    terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;  // 강도(높이 차이) 설정
    
                    terrainCloudElevSize++;  // 지형 클라우드 높이의 크기 증가
                }
            }
        }
    }

  // 지역 지형 맵 반경(localTerrainMapRadius) 내의 지형 클라우드를 병합합니다.
  int terrainCloudLocalSize = terrainCloudLocal->points.size();  // 지역 지형 클라우드의 포인트 개수를 가져옵니다.
  for (int i = 0; i < terrainCloudLocalSize; i++) {  // 모든 지역 지형 클라우드 포인트를 반복합니다.
      point = terrainCloudLocal->points[i];  // 현재 포인트를 가져옵니다.
      float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));  // 차량과 포인트 간의 거리 계산
      if (dis <= localTerrainMapRadius)  // 거리 조건이 반경 내에 있는지 확인
      {
          terrainCloudElev->push_back(point);  // 조건을 만족하는 포인트를 지형 클라우드 높이에 추가합니다.
      }
  }
  
  clearingCloud = false;  // 클라우드 비우기 상태를 false로 설정합니다.
  
    // 고도 정보가 포함된 포인트를 발행합니다.
    sensor_msgs::PointCloud2 terrainCloud2;  // ROS 메시지 포인트 클라우드 2를 선언합니다.
    pcl::toROSMsg(*terrainCloudElev, terrainCloud2);  // PCL 포인트 클라우드를 ROS 메시지로 변환합니다.
    terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);  // 타임스탬프를 설정하여 메시지의 시간 정보를 지정합니다.
    terrainCloud2.header.frame_id = "map";  // 메시지의 프레임 ID를 "map"으로 설정합니다.
    pubTerrainCloud.publish(terrainCloud2);  // 포인트 클라우드 메시지를 발행합니다.
  }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
