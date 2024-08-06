#include <math.h> // 수학 라이브러리를 포함합니다.
#include <time.h> // 시간 라이브러리를 포함합니다.
#include <stdio.h> // 표준 입출력 라이브러리를 포함합니다.
#include <stdlib.h> // 표준 라이브러리를 포함합니다.
#include <ros/ros.h> // ROS 기본 기능을 사용하기 위해 ROS 헤더 파일을 포함합니다.

#include <message_filters/subscriber.h> // 메시지 필터링을 위한 subscriber를 포함합니다.
#include <message_filters/synchronizer.h> // 메시지 동기화를 위한 synchronizer를 포함합니다.
#include <message_filters/sync_policies/approximate_time.h> // 근사 시간 동기화를 위한 정책을 포함합니다.

#include <std_msgs/Bool.h> // std_msgs 패키지의 Bool 메시지를 포함합니다.
#include <std_msgs/Float32.h> // std_msgs 패키지의 Float32 메시지를 포함합니다.
#include <nav_msgs/Path.h> // nav_msgs 패키지의 Path 메시지를 포함합니다.
#include <nav_msgs/Odometry.h> // nav_msgs 패키지의 Odometry 메시지를 포함합니다.
#include <geometry_msgs/PointStamped.h> // geometry_msgs 패키지의 PointStamped 메시지를 포함합니다.
#include <geometry_msgs/PolygonStamped.h> // geometry_msgs 패키지의 PolygonStamped 메시지를 포함합니다.
#include <sensor_msgs/Imu.h> // sensor_msgs 패키지의 Imu 메시지를 포함합니다.
#include <sensor_msgs/PointCloud2.h> // sensor_msgs 패키지의 PointCloud2 메시지를 포함합니다.
#include <sensor_msgs/Joy.h> // sensor_msgs 패키지의 Joy 메시지를 포함합니다.

#include <tf/transform_datatypes.h> // tf 패키지의 변환 데이터 타입을 포함합니다.
#include <tf/transform_broadcaster.h> // tf 패키지의 변환 브로드캐스터를 포함합니다.

#include <pcl_conversions/pcl_conversions.h> // PCL(Point Cloud Library) 변환 기능을 포함합니다.
#include <pcl/point_cloud.h> // PCL의 포인트 클라우드 기능을 포함합니다.
#include <pcl/point_types.h> // PCL의 포인트 타입 기능을 포함합니다.
#include <pcl/filters/voxel_grid.h> // PCL의 Voxel Grid 필터를 포함합니다.
#include <pcl/kdtree/kdtree_flann.h> // PCL의 K-d 트리 기능을 포함합니다.

using namespace std; // 표준 라이브러리의 모든 이름을 전역 네임스페이스로 가져옵니다.
const double PI = 3.1415926; // 원주율 값을 상수로 정의합니다.

#define PLOTPATHSET 1 // PLOTPATHSET을 1로 정의합니다.

string pathFolder; // 경로 폴더를 저장할 문자열 변수를 선언합니다.

// 보통 차량에 따라서 vehicle length, vehicle width를 설정해줘야 합니다. 
double vehicleLength = 0.6; // 차량의 길이를 미터 단위로 초기화합니다.
double vehicleWidth = 0.6; // 차량의 너비를 미터 단위로 초기화합니다.
double sensorOffsetX = 0; // 센서의 X축 오프셋을 초기화합니다.
double sensorOffsetY = 0; // 센서의 Y축 오프셋을 초기화합니다.

bool twoWayDrive = true; // 양방향 주행 여부를 초기화합니다.
double laserVoxelSize = 0.05; // 레이저 보셀의 크기를 초기화합니다.
double terrainVoxelSize = 0.2; // 지형 보셀의 크기를 초기화합니다.
bool useTerrainAnalysis = false; // 지형 분석 사용 여부를 초기화합니다.
bool checkObstacle = true; // 장애물 확인 여부를 초기화합니다.
bool checkRotObstacle = false; // 회전 장애물 확인 여부를 초기화합니다.
double adjacentRange = 3.5; // 인접 범위를 초기화합니다.
double obstacleHeightThre = 0.2; // 장애물 높이 임계값을 초기화합니다.
double groundHeightThre = 0.1; // 지면 높이 임계값을 초기화합니다.
double costHeightThre = 0.1; // 비용 높이 임계값을 초기화합니다.
double costScore = 0.02; // 비용 점수를 초기화합니다.
bool useCost = false; // 비용 사용 여부를 초기화합니다.
const int laserCloudStackNum = 1; // 레이저 클라우드 스택 수를 초기화합니다.
int laserCloudCount = 0; // 레이저 클라우드 카운트를 초기화합니다.
int pointPerPathThre = 2; // 경로당 포인트 임계값을 초기화합니다.
double minRelZ = -0.5; // 최소 상대 Z값을 초기화합니다.
double maxRelZ = 0.25; // 최대 상대 Z값을 초기화합니다.
double maxSpeed = 1.0; // 최대 속도를 초기화합니다.
double dirWeight = 0.02; // 방향 가중치를 초기화합니다.
double dirThre = 90.0; // 방향 임계값을 초기화합니다.
bool dirToVehicle = false; // 차량 방향 여부를 초기화합니다.
double pathScale = 1.0; // 경로 스케일을 초기화합니다.
double minPathScale = 0.75; // 최소 경로 스케일을 초기화합니다.
double pathScaleStep = 0.25; // 경로 스케일 단계값을 초기화합니다.
bool pathScaleBySpeed = true; // 속도에 따른 경로 스케일 사용 여부를 초기화합니다.
double minPathRange = 1.0; // 최소 경로 범위를 초기화합니다.
double pathRangeStep = 0.5; // 경로 범위 단계값을 초기화합니다.
bool pathRangeBySpeed = true; // 속도에 따른 경로 범위 사용 여부를 초기화합니다.
bool pathCropByGoal = true; // 목표에 따른 경로 자르기 여부를 초기화합니다.
bool autonomyMode = false; // 자율 주행 모드 여부를 초기화합니다.
double autonomySpeed = 1.0; // 자율 주행 속도를 초기화합니다.
double joyToSpeedDelay = 2.0; // 조이스틱 속도 지연 시간을 초기화합니다.
double joyToCheckObstacleDelay = 5.0; // 조이스틱 장애물 확인 지연 시간을 초기화합니다.
double goalClearRange = 0.5; // 목표 클리어 범위를 초기화합니다.
double goalX = 0; // 목표의 X 좌표를 초기화합니다.
double goalY = 0; // 목표의 Y 좌표를 초기화합니다.

float joySpeed = 0; // 조이스틱 속도를 초기화합니다.
float joySpeedRaw = 0; // 원시 조이스틱 속도를 초기화합니다.
float joyDir = 0; // 조이스틱 방향을 초기화합니다.

const int pathNum = 343; // 경로 수를 상수로 정의합니다.
const int groupNum = 7; // 그룹 수를 상수로 정의합니다.
float gridVoxelSize = 0.02; // 그리드 보셀 크기를 초기화합니다.
float searchRadius = 0.45; // 검색 반경을 초기화합니다.
float gridVoxelOffsetX = 3.2; // 그리드 보셀 X축 오프셋을 초기화합니다.
float gridVoxelOffsetY = 4.5; // 그리드 보셀 Y축 오프셋을 초기화합니다.
const int gridVoxelNumX = 161; // 그리드 보셀의 X축 개수를 상수로 정의합니다.
const int gridVoxelNumY = 451; // 그리드 보셀의 Y축 개수를 상수로 정의합니다.
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY; // 그리드 보셀의 총 개수를 상수로 정의합니다.

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 레이저 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 잘린 레이저 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 다운샘플링된 레이저 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 지형 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 잘린 지형 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 다운샘플링된 지형 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum]; // 레이저 클라우드 스택을 저장할 배열을 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 플래너 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 잘린 플래너 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 경계 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>()); // 추가된 장애물 클라우드를 저장할 포인터를 초기화합니다.
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum]; // 시작 경로들을 저장할 배열을 초기화합니다.
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum]; // 경로들을 저장할 배열을 초기화합니다.
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>()); // 자유 경로들을 저장할 포인터를 초기화합니다.
#endif

int pathList[pathNum] = {0}; // 경로 목록을 초기화합니다.
float endDirPathList[pathNum] = {0}; // 경로의 끝 방향 목록을 초기화합니다.
int clearPathList[36 * pathNum] = {0}; // 클리어 경로 목록을 초기화합니다.
float pathPenaltyList[36 * pathNum] = {0}; // 경로 페널티 목록을 초기화합니다.
float clearPathPerGroupScore[36 * groupNum] = {0}; // 그룹당 클리어 경로 점수를 초기화합니다.
std::vector<int> correspondences[gridVoxelNum]; // 그리드 보셀 수에 해당하는 대응 관계 벡터를 초기화합니다.

bool newLaserCloud = false; // 새로운 레이저 클라우드 여부를 초기화합니다.
bool newTerrainCloud = false; // 새로운 지형 클라우드 여부를 초기화합니다.

double odomTime = 0; // 오돔 시간 값을 초기화합니다.
double joyTime = 0; // 조이스틱 시간 값을 초기화합니다.

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0; // 차량의 롤, 피치, 요 값을 초기화합니다.
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; // 차량의 X, Y, Z 좌표 값을 초기화합니다.

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter; // 레이저 및 지형 클라우드의 VoxelGrid 필터를 초기화합니다.


void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 오돔 메시지의 타임스탬프를 초 단위로 저장합니다.
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  // 오돔 메시지의 쿼터니언을 가져옵니다.
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  // 쿼터니언을 롤, 피치, 요로 변환합니다.
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  // 차량의 롤, 피치, 요 값을 업데이트합니다.
  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  // 차량의 X 좌표를 업데이트합니다.
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  // 차량의 Y 좌표를 업데이트합니다.
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  // 차량의 Z 좌표를 업데이트합니다.
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {
    // 기존 레이저 클라우드를 지우고 새로운 메시지를 레이저 클라우드로 변환합니다.
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear(); // 잘린 레이저 클라우드를 지웁니다.
    int laserCloudSize = laserCloud->points.size(); // 레이저 클라우드의 포인트 수를 가져옵니다.

    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i]; // 각 포인트를 가져옵니다.

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      // 현재 포인트와 차량 사이의 거리를 계산합니다.
      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) { // 거리가 인접 범위 내에 있는 경우
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point); // 잘린 레이저 클라우드에 포인트를 추가합니다.
      }
    }

    laserCloudDwz->clear(); // 다운샘플링된 레이저 클라우드를 지웁니다.
    laserDwzFilter.setInputCloud(laserCloudCrop); // 다운샘플링 필터의 입력 클라우드를 설정합니다.
    laserDwzFilter.filter(*laserCloudDwz); // 필터를 적용하여 다운샘플링합니다.

    newLaserCloud = true; // 새로운 레이저 클라우드가 있음을 표시합니다.
  }
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {
    // 기존 지형 클라우드를 지우고 새로운 메시지를 지형 클라우드로 변환합니다.
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear(); // 잘린 지형 클라우드를 지웁니다.
    int terrainCloudSize = terrainCloud->points.size(); // 지형 클라우드의 포인트 수를 가져옵니다.

    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i]; // 각 포인트를 가져옵니다.

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      // 현재 포인트와 차량 사이의 거리를 계산합니다.
      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) { // 조건에 맞는 포인트를 필터링합니다.
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point); // 잘린 지형 클라우드에 포인트를 추가합니다.
      }
    }

    terrainCloudDwz->clear(); // 다운샘플링된 지형 클라우드를 지웁니다.
    terrainDwzFilter.setInputCloud(terrainCloudCrop); // 다운샘플링 필터의 입력 클라우드를 설정합니다.
    terrainDwzFilter.filter(*terrainCloudDwz); // 필터를 적용하여 다운샘플링합니다.

    newTerrainCloud = true; // 새로운 지형 클라우드가 있음을 표시합니다.
  }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  // 현재 시간을 저장합니다.
  joyTime = ros::Time::now().toSec();

  // 조이스틱 속도를 계산합니다.
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  // 조이스틱 속도를 1.0으로 제한합니다.
  if (joySpeed > 1.0) joySpeed = 1.0;
  // 조이스틱의 Y축 값이 0이면 속도를 0으로 설정합니다.
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    // 조이스틱 방향을 계산합니다.
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    // 조이스틱의 Y축 값이 음수이면 방향 값을 반전합니다.
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  // 조이스틱의 Y축 값이 음수이고 양방향 주행이 아니면 속도를 0으로 설정합니다.
  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  // 조이스틱의 축 값에 따라 자율 주행 모드를 설정합니다.
  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  // 조이스틱의 축 값에 따라 장애물 확인 여부를 설정합니다.
  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}


void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  // 기존의 경계 클라우드를 지웁니다.
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  // 경계가 존재하면 첫 번째 점을 초기화합니다.
  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    // 현재 점을 업데이트합니다.
    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    // 두 점의 z 값이 같으면 경계를 처리합니다.
    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      // 경계 점들 사이의 거리와 보셀 크기를 기준으로 점의 수를 계산합니다.
      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        // 선형 보간을 통해 경계 점들을 생성합니다.
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0; // 경계 점의 z 값을 0으로 설정합니다.
        point.intensity = 100.0; // 경계 점의 강도를 설정합니다.

        // 경계 클라우드에 점을 추가합니다.
        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  // 추가된 장애물 클라우드를 지우고 새로운 메시지를 추가된 장애물 클라우드로 변환합니다.
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  // 추가된 장애물 클라우드의 포인트 수를 가져옵니다.
  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    // 각 포인트의 강도를 200.0으로 설정합니다.
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  // 현재 시간을 초 단위로 가져옵니다.
  double checkObsTime = ros::Time::now().toSec();

  // 자율 주행 모드이고 조이스틱 시간 이후 일정 지연 시간이 지난 경우에만 장애물 확인 여부를 업데이트합니다.
  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50]; // 문자열을 읽어올 버퍼를 선언합니다.
  int val, pointNum = 0; // 파일에서 읽은 값을 저장할 변수와 포인트 수를 초기화합니다.
  string strCur, strLast; // 현재 문자열과 이전 문자열을 저장할 변수를 선언합니다.

  // 파일의 끝(header의 끝)을 만날 때까지 반복합니다.
  while (strCur != "end_header") {
    // 파일에서 문자열을 읽어옵니다.
    val = fscanf(filePtr, "%s", str);
    // 파일에서 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    // 이전 문자열을 현재 문자열로 업데이트합니다.
    strLast = strCur;
    // 현재 문자열을 새로 읽은 문자열로 업데이트합니다.
    strCur = string(str);

    // 현재 문자열이 "vertex"이고 이전 문자열이 "element"인 경우
    if (strCur == "vertex" && strLast == "element") {
      // 다음 값(포인트 수)을 읽어옵니다.
      val = fscanf(filePtr, "%d", &pointNum);
      // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  // 읽어온 포인트 수를 반환합니다.
  return pointNum;
}

void readStartPaths()
{
  // 시작 경로 파일의 경로를 설정합니다.
  string fileName = pathFolder + "/startPaths.ply";

  // 파일을 읽기 모드로 엽니다.
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    // 파일을 열 수 없는 경우 오류 메시지를 출력하고 프로그램을 종료합니다.
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  // PLY 헤더를 읽고 포인트 수를 가져옵니다.
  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point; // 포인트를 저장할 변수를 선언합니다.
  int val1, val2, val3, val4, groupID; // 파일에서 읽은 값을 저장할 변수를 선언합니다.

  // 모든 포인트를 읽을 때까지 반복합니다.
  for (int i = 0; i < pointNum; i++) {
    // 포인트의 x, y, z 좌표와 그룹 ID를 읽어옵니다.
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    // 그룹 ID가 유효한 범위에 있는 경우 해당 그룹의 시작 경로에 포인트를 추가합니다.
    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  // 파일을 닫습니다.
  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  // 경로 파일의 경로를 설정합니다.
  string fileName = pathFolder + "/paths.ply";

  // 파일을 읽기 모드로 엽니다.
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    // 파일을 열 수 없는 경우 오류 메시지를 출력하고 프로그램을 종료합니다.
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  // PLY 헤더를 읽고 포인트 수를 가져옵니다.
  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point; // 포인트를 저장할 변수를 선언합니다.
  int pointSkipNum = 30; // 포인트 건너뛰기 수를 설정합니다.
  int pointSkipCount = 0; // 포인트 건너뛰기 카운트를 초기화합니다.
  int val1, val2, val3, val4, val5, pathID; // 파일에서 읽은 값을 저장할 변수를 선언합니다.

  // 모든 포인트를 읽을 때까지 반복합니다.
  for (int i = 0; i < pointNum; i++) {
    // 포인트의 x, y, z 좌표, 경로 ID 및 강도를 읽어옵니다.
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    // 경로 ID가 유효한 범위에 있는 경우 해당 경로에 포인트를 추가합니다.
    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  // 파일을 닫습니다.
  fclose(filePtr);
}
#endif

void readPathList()
{
  // 경로 목록 파일의 경로를 설정합니다.
  string fileName = pathFolder + "/pathList.ply";

  // 파일을 읽기 모드로 엽니다.
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    // 파일을 열 수 없는 경우 오류 메시지를 출력하고 프로그램을 종료합니다.
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  // PLY 헤더를 읽고 포인트 수가 예상 값과 일치하는지 확인합니다.
  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID; // 파일에서 읽은 값을 저장할 변수를 선언합니다.
  float endX, endY, endZ; // 경로의 끝 점을 저장할 변수를 선언합니다.

  // 모든 경로 정보를 읽을 때까지 반복합니다.
  for (int i = 0; i < pathNum; i++) {
    // 경로의 끝 점 좌표와 경로 ID, 그룹 ID를 읽어옵니다.
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    // 경로 ID와 그룹 ID가 유효한 범위에 있는 경우
    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      // 경로 목록에 그룹 ID를 저장하고, 경로의 끝 방향을 계산하여 저장합니다.
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  // 파일을 닫습니다.
  fclose(filePtr);
}

void readCorrespondences()
{
  // 대응 관계 파일의 경로를 설정합니다.
  string fileName = pathFolder + "/correspondences.txt";

  // 파일을 읽기 모드로 엽니다.
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    // 파일을 열 수 없는 경우 오류 메시지를 출력하고 프로그램을 종료합니다.
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID; // 파일에서 읽은 값을 저장할 변수를 선언합니다.

  // 모든 그리드 보셀을 반복합니다.
  for (int i = 0; i < gridVoxelNum; i++) {
    // 그리드 보셀 ID를 읽어옵니다.
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    // 경로 ID를 계속 읽습니다.
    while (true) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        // 읽기가 실패하면 오류 메시지를 출력하고 프로그램을 종료합니다.
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      // 경로 ID가 -1이 아닌 경우 대응 관계에 추가합니다.
      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        // 경로 ID가 -1이면 현재 그리드 보셀의 대응 관계 읽기를 종료합니다.
        break;
      }
    }
  }

  // 파일을 닫습니다.
  fclose(filePtr);
}



int main(int argc, char** argv)
{
  // ROS 초기화, 노드 이름은 "localPlanner"
  ros::init(argc, argv, "localPlanner");
  // 기본 노드 핸들러 생성
  ros::NodeHandle nh;
  // 프라이빗 노드 핸들러 생성
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 파라미터 서버에서 매개변수들을 가져옴
  nhPrivate.getParam("pathFolder", pathFolder); // 경로 폴더
  nhPrivate.getParam("vehicleLength", vehicleLength); // 차량 길이
  nhPrivate.getParam("vehicleWidth", vehicleWidth); // 차량 폭
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX); // 센서의 X축 오프셋
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY); // 센서의 Y축 오프셋
  nhPrivate.getParam("twoWayDrive", twoWayDrive); // 양방향 주행 여부
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize); // 라이다 보셀 크기
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize); // 지형 보셀 크기
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis); // 지형 분석 사용 여부
  nhPrivate.getParam("checkObstacle", checkObstacle); // 장애물 체크 여부
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle); // 회전 장애물 체크 여부
  nhPrivate.getParam("adjacentRange", adjacentRange); // 인접 범위
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre); // 장애물 높이 임계값
  nhPrivate.getParam("groundHeightThre", groundHeightThre); // 지면 높이 임계값
  nhPrivate.getParam("costHeightThre", costHeightThre); // 비용 높이 임계값
  nhPrivate.getParam("costScore", costScore); // 비용 점수
  nhPrivate.getParam("useCost", useCost); // 비용 사용 여부
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre); // 경로당 포인트 임계값
  nhPrivate.getParam("minRelZ", minRelZ); // 최소 상대 Z값
  nhPrivate.getParam("maxRelZ", maxRelZ); // 최대 상대 Z값
  nhPrivate.getParam("maxSpeed", maxSpeed); // 최대 속도
  nhPrivate.getParam("dirWeight", dirWeight); // 방향 가중치
  nhPrivate.getParam("dirThre", dirThre); // 방향 임계값
  nhPrivate.getParam("dirToVehicle", dirToVehicle); // 차량으로의 방향
  nhPrivate.getParam("pathScale", pathScale); // 경로 스케일
  nhPrivate.getParam("minPathScale", minPathScale); // 최소 경로 스케일
  nhPrivate.getParam("pathScaleStep", pathScaleStep); // 경로 스케일 단계
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed); // 속도에 따른 경로 스케일
  nhPrivate.getParam("minPathRange", minPathRange); // 최소 경로 범위
  nhPrivate.getParam("pathRangeStep", pathRangeStep); // 경로 범위 단계
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed); // 속도에 따른 경로 범위
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal); // 목표에 따른 경로 크롭
  nhPrivate.getParam("autonomyMode", autonomyMode); // 자율 모드
  nhPrivate.getParam("autonomySpeed", autonomySpeed); // 자율 주행 속도
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay); // 조이스틱에서 속도로의 지연
  nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay); // 조이스틱에서 장애물 체크로의 지연
  nhPrivate.getParam("goalClearRange", goalClearRange); // 목표 클리어 범위
  nhPrivate.getParam("goalX", goalX); // 목표 X 좌표
  nhPrivate.getParam("goalY", goalY); // 목표 Y 좌표

  // 주행 상태 추정 정보를 수신하는 구독자 설정
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/state_estimation", 5, odometryHandler);

  // 라이다 포인트 클라우드 데이터를 수신하는 구독자 설정
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/registered_scan", 5, laserCloudHandler);

  // 지형 정보를 수신하는 구독자 설정
  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/terrain_map", 5, terrainCloudHandler);

  // 조이스틱 입력을 수신하는 구독자 설정
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  // 목표 위치 정보를 수신하는 구독자 설정
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  // 속도 정보를 수신하는 구독자 설정
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  // 경계 정보를 수신하는 구독자 설정
  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5, boundaryHandler);

  // 추가 장애물 정보를 수신하는 구독자 설정
  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> ("/added_obstacles", 5, addedObstaclesHandler);

  // 장애물 체크 여부를 수신하는 구독자 설정
  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool> ("/check_obstacle", 5, checkObstacleHandler);

  // 경로를 퍼블리시하는 퍼블리셔 설정
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);

  nav_msgs::Path path; // 경로 메시지 객체 생성

  #if PLOTPATHSET == 1
  // 자유 경로를 퍼블리시하는 퍼블리셔 설정
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> ("/free_paths", 2);
  #endif

  // 라이다 포인트 클라우드를 퍼블리시하는 퍼블리셔 (주석 처리됨)
  // ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stacked_scans", 2);

  printf ("\nReading path files.\n"); // 경로 파일을 읽기 시작

  if (autonomyMode) { // 자율 모드일 경우
    joySpeed = autonomySpeed / maxSpeed; // 자율 주행 속도를 최대 속도로 나눈 비율

    if (joySpeed < 0) joySpeed = 0; // 속도가 0보다 작으면 0으로 설정
    else if (joySpeed > 1.0) joySpeed = 1.0; // 속도가 1.0보다 크면 1.0으로 설정
  }

  // 라이다 클라우드 스택 초기화
  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  // 시작 경로 초기화
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  // 경로 초기화
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  // 대응 관계 초기화
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  // 라이다 다운 샘플링 필터 설정
  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  // 지형 다운 샘플링 필터 설정
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  // 시작 경로 읽기
  readStartPaths();
  #if PLOTPATHSET == 1
  // 경로 읽기
  readPaths();
  #endif
  // 경로 목록 읽기
  readPathList();
  // 대응 관계 읽기
  readCorrespondences();

  printf ("\nInitialization complete.\n\n"); // 초기화 완료

  ros::Rate rate(100); // 루프 주기 설정 (100Hz)
  bool status = ros::ok(); // ROS 상태 확인
  while (status) { // ROS 상태가 OK일 때 루프 실행
    ros::spinOnce(); // 콜백 함수 실행

    if (newLaserCloud || newTerrainCloud) { // 새로운 라이다 또는 지형 클라우드가 있을 경우
      if (newLaserCloud) { // 새로운 라이다 클라우드가 있을 경우
        newLaserCloud = false; // 새로운 라이다 클라우드 플래그 리셋

        laserCloudStack[laserCloudCount]->clear(); // 현재 스택 클라우드 초기화
        *laserCloudStack[laserCloudCount] = *laserCloudDwz; // 다운 샘플링된 클라우드 복사
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum; // 스택 카운트 업데이트

        plannerCloud->clear(); // 플래너 클라우드 초기화
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i]; // 스택 클라우드를 플래너 클라우드에 추가
        }
      }

      if (newTerrainCloud) { // 새로운 지형 클라우드가 있을 경우
        newTerrainCloud = false; // 새로운 지형 클라우드 플래그 리셋

        plannerCloud->clear(); // 플래너 클라우드 초기화
        *plannerCloud = *terrainCloudDwz; // 다운 샘플링된 지형 클라우드 복사
      }

      // 차량의 롤, 피치, 요우 각도를 계산
      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point; // 포인트 객체 생성
      plannerCloudCrop->clear(); // 크롭된 플래너 클라우드 초기화
      int plannerCloudSize = plannerCloud->points.size(); // 플래너 클라우드 포인트 수
      
      for (int i = 0; i < plannerCloudSize; i++) {
        // 포인트의 차량 좌표계를 기준으로 변환
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        // 포인트와 차량의 거리 계산
        float dis = sqrt(point.x * point.x + point.y * point.y);
        // 인접 범위 내에 있고 지형 분석을 사용하거나 상대 Z값이 범위 내에 있을 경우
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point); // 크롭된 플래너 클라우드에 추가
        }
      }

            int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        // 경계 클라우드 포인트를 차량 좌표계를 기준으로 변환
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        // 포인트와 차량의 거리 계산
        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point); // 인접 범위 내 포인트를 크롭된 플래너 클라우드에 추가
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        // 추가 장애물 포인트를 차량 좌표계를 기준으로 변환
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        // 포인트와 차량의 거리 계산
        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point); // 인접 범위 내 포인트를 크롭된 플래너 클라우드에 추가
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed; // 속도에 따른 경로 범위 조정
      if (pathRange < minPathRange) pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;

      if (autonomyMode) {
        // 목표 위치를 차량 좌표계를 기준으로 변환
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY); // 목표까지의 거리 계산
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI; // 목표 방향 계산

        if (!twoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0; // 양방향 주행이 아닐 경우 방향 제한
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed; // 속도에 따른 경로 스케일 조정
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        // 경로 클리어 및 페널티 리스트 초기화
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          // 크롭된 플래너 클라우드 포인트를 경로 스케일로 변환
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              // 경로 방향이 임계값을 초과하면 스킵
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              // 회전 각도를 고려하여 포인트 변환
              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              // 그리드 인덱스 계산
              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  // 장애물 높이가 임계값을 초과하면 클리어 경로 리스트에 추가, 그렇지 않으면 페널티 리스트에 추가
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }
        
          // 경로 내 장애물 회전 체크
          if (dis < diameter / pathScale && 
              (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
              (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            // 포인트의 각도를 계산하여 angObs에 저장 (라디안에서 도 단위로 변환)
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) { // 포인트의 각도가 양수일 경우
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset; // 반시계 방향 최소 각도 업데이트
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0; // 시계 방향 최소 각도 업데이트
            } else { // 포인트의 각도가 음수일 경우
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset; // 시계 방향 최소 각도 업데이트
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset; // 반시계 방향 최소 각도 업데이트
            }
          }
        }

        // 최소 장애물 각도 조정
        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;
        
        // 경로 점수 계산
        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum); // 현재 경로의 회전 방향 인덱스 계산
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0)); // 조이스틱 방향과 현재 회전 방향 간의 각도 차이 계산
          if (angDiff > 180.0) { // 각도 차이가 180도를 넘으면
            angDiff = 360.0 - angDiff; // 360도에서 뺀 값을 사용하여 작은 각도로 변환
          }
          // 각도 차이가 임계값을 넘거나, 차량 방향과의 관계에 따라 특정 조건을 만족하지 않으면 해당 경로를 스킵
          if ((angDiff > dirThre && !dirToVehicle) || 
              (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }
        
          if (clearPathList[i] < pointPerPathThre) { // 경로가 충분히 클리어 되지 않았을 경우
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre; // 장애물 높이에 따른 페널티 점수 계산
            if (penaltyScore < costScore) penaltyScore = costScore; // 페널티 점수가 최소 비용 점수보다 작으면 최소 비용 점수로 설정
        
            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0)); // 조이스틱 방향과 경로 끝 방향 간의 각도 차이 계산
            if (dirDiff > 360.0) { // 각도 차이가 360도를 넘으면
              dirDiff -= 360.0; // 360도에서 뺀 값을 사용
            }
            if (dirDiff > 180.0) { // 각도 차이가 180도를 넘으면
              dirDiff = 360.0 - dirDiff; // 360도에서 뺀 값을 사용하여 작은 각도로 변환
            }
        
            // 회전 방향 가중치 계산
            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1); // 중앙 방향에 가까울수록 가중치가 높음
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            // 방향 가중치, 회전 방향 가중치 및 페널티 점수를 조합하여 최종 경로 점수 계산
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) { // 점수가 0보다 크면
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score; // 해당 경로 그룹의 점수에 추가
            }
          }
        }

               // 경로 점수 중 최대 점수를 찾기 위한 변수 초기화
        float maxScore = 0;
        int selectedGroupID = -1;

        // 각 그룹에 대해 경로 점수를 계산
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum); // 현재 그룹의 회전 방향 인덱스 계산
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180; // 회전 각도 계산 (라디안)
          float rotDeg = 10.0 * rotDir; // 회전 각도 계산 (도)
          if (rotDeg > 180.0) rotDeg -= 360.0; // 각도가 180도 이상이면 360도에서 뺀 값을 사용

          // 최대 점수와 비교하여 더 높은 점수를 가진 경로를 선택
          if (maxScore < clearPathPerGroupScore[i] && 
              ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        // 선택된 경로가 있을 경우
        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum); // 선택된 그룹의 회전 방향 인덱스 계산
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180; // 회전 각도 계산 (라디안)

          selectedGroupID = selectedGroupID % groupNum; // 선택된 그룹 ID 계산
          int selectedPathLength = startPaths[selectedGroupID]->points.size(); // 선택된 경로의 길이
          path.poses.resize(selectedPathLength); // 경로 포즈 배열 크기 조정

          // 선택된 경로의 각 포인트에 대해 변환하여 경로 설정
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y); // 포인트와 차량의 거리 계산

            // 경로 범위와 목표 거리 내에 있는 포인트만 경로에 추가
            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i); // 경로 포즈 배열 크기 조정
              break;
            }
          }



          // 경로 메시지의 헤더 설정 및 퍼블리시
          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);

          #if PLOTPATHSET == 1
          // 자유 경로 포인트 클라우드 초기화
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum); // 회전 방향 인덱스 계산
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180; // 회전 각도 계산 (라디안)
            float rotDeg = 10.0 * rotDir; // 회전 각도 계산 (도)
            if (rotDeg > 180.0) rotDeg -= 360.0; // 각도가 180도 이상이면 360도에서 뺀 값을 사용
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0)); // 조이스틱 방향과 회전 방향 간의 각도 차이 계산
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff; // 각도 차이가 180도를 넘으면 360도에서 뺀 값을 사용
            }
            // 각도 차이가 임계값을 넘거나, 차량 방향과의 관계에 따라 특정 조건을 만족하지 않으면 해당 경로를 스킵
            if ((angDiff > dirThre && !dirToVehicle) || 
                (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
              continue;
            }

            // 클리어된 경로 리스트에 포인트 추가
            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y); // 포인트와 차량의 거리 계산
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                  // 포인트를 경로 스케일 및 회전 각도로 변환하여 자유 경로에 추가
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          // 자유 경로를 퍼블리시
          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "vehicle";
          pubFreePaths.publish(freePaths2);
          #endif
        }

        // 선택된 그룹 ID가 없을 경우
        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep; // 경로 스케일 감소
            pathRange = adjacentRange * pathScale / defPathScale; // 경로 범위 업데이트
          } else {
            pathRange -= pathRangeStep; // 경로 범위 감소
          }
        } else {
          pathFound = true; // 경로를 찾았음을 표시
          break;
        }
      }
      pathScale = defPathScale; // 경로 스케일 초기화

      // 경로를 찾지 못했을 경우
      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear(); // 자유 경로 클리어
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2); // 자유 경로 퍼블리시
        #endif
      }

      // 주석 처리된 코드: 크롭된 플래너 클라우드를 퍼블리시
      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok(); // ROS 상태 업데이트
    rate.sleep(); // 루프 주기 유지
  }

  return 0;
}


