#!/usr/bin/python3

# numpy와 matplotlib 라이브러리를 임포트하여 데이터 처리를 하고 그래프를 그리기 위해 사용
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# ROS 관련 라이브러리를 임포트하여 ROS 노드 및 메시지 처리에 사용
import rospy
from std_msgs.msg import Float32

# matplotlib의 툴바를 비활성화하고, 인터랙티브 모드를 활성화
mpl.rcParams['toolbar'] = 'None'
plt.ion()

# 변수 초기화
time_duration = 0
start_time_duration = 0
first_iteration = 'True'

explored_volume = 0
traveling_distance = 0
run_time = 0
max_explored_volume = 0
max_traveling_diatance = 0
max_run_time = 0

# 데이터를 저장할 numpy 배열 초기화
time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
run_time_list = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])

# /time_duration 토픽의 메시지를 처리하는 콜백 함수
def timeDurationCallback(msg):
    global time_duration, start_time_duration, first_iteration
    time_duration = msg.data
    if first_iteration == 'True':
        start_time_duration = time_duration
        first_iteration = 'False'

# /runtime 토픽의 메시지를 처리하는 콜백 함수
def runTimeCallback(msg):
    global run_time
    run_time = msg.data

# /explored_volume 토픽의 메시지를 처리하는 콜백 함수
def exploredVolumeCallback(msg):
    global explored_volume
    explored_volume = msg.data

# /traveling_distance 토픽의 메시지를 처리하는 콜백 함수
def travelingDistanceCallback(msg):
    global traveling_distance
    traveling_distance = msg.data

# ROS 노드를 초기화하고 구독자 설정
def listener():
  global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list

  # ROS 노드 초기화
  rospy.init_node('realTimePlot')
  
  # 각 토픽을 구독하여 해당 콜백 함수를 호출
  rospy.Subscriber("/time_duration", Float32, timeDurationCallback)
  rospy.Subscriber("/runtime", Float32, runTimeCallback)
  rospy.Subscriber("/explored_volume", Float32, exploredVolumeCallback)
  rospy.Subscriber("/traveling_distance", Float32, travelingDistanceCallback)

  # 그래프 설정
  fig = plt.figure(figsize=(8,7))
  
  # 첫 번째 서브플롯: 탐색된 부피
  fig1 = fig.add_subplot(311)
  plt.title("Exploration Metrics\n", fontsize=14)
  plt.margins(x=0.001)
  fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
  l1, = fig1.plot(time_list2, explored_volume_list, color='r', label='Explored Volume')
  
  # 두 번째 서브플롯: 이동 거리
  fig2 = fig.add_subplot(312)
  fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
  l2, = fig2.plot(time_list3, traveling_distance_list, color='r', label='Traveling Distance')
  
  # 세 번째 서브플롯: 알고리즘 실행 시간
  fig3 = fig.add_subplot(313)
  fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
  fig3.set_xlabel("Time Duration (s)", fontsize=12) # 한 번만 설정
  l3, = fig3.plot(time_list1, run_time_list, color='r', label='Algorithm Runtime')

  # 데이터 수집 및 그래프 갱신을 위한 루프
  count = 0
  r = rospy.Rate(100) # 100Hz로 루프 설정
  while not rospy.is_shutdown():
      r.sleep()
      count += 1

      # 25번 루프마다 데이터를 업데이트
      if count % 25 == 0:
        # 최대 탐색 부피와 최대 이동 거리 업데이트
        max_explored_volume = explored_volume
        max_traveling_diatance = traveling_distance
        if run_time > max_run_time:
            max_run_time = run_time

        # 데이터 리스트에 현재 값 추가
        time_list2 = np.append(time_list2, time_duration)
        explored_volume_list = np.append(explored_volume_list, explored_volume)
        time_list3 = np.append(time_list3, time_duration)
        traveling_distance_list = np.append(traveling_distance_list, traveling_distance)
        time_list1 = np.append(time_list1, time_duration)
        run_time_list = np.append(run_time_list, run_time)

      # 100번 루프마다 그래프를 갱신
      if count >= 100:
        count = 0
        l1.set_xdata(time_list2)
        l2.set_xdata(time_list3)
        l3.set_xdata(time_list1)
        l1.set_ydata(explored_volume_list)
        l2.set_ydata(traveling_distance_list)
        l3.set_ydata(run_time_list)

        # 그래프 축 설정
        fig1.set_ylim(0, max_explored_volume + 500)
        fig1.set_xlim(start_time_duration, time_duration + 10)
        fig2.set_ylim(0, max_traveling_diatance + 20)
        fig2.set_xlim(start_time_duration, time_duration + 10)
        fig3.set_ylim(0, max_run_time + 0.2)
        fig3.set_xlim(start_time_duration, time_duration + 10)

        # 그래프 갱신
        fig.canvas.draw()

# 메인 함수
if __name__ == '__main__':
  listener()
  print("1")
