#can0올리기

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

그리고 다 했으면 
수신
candump can0



1부터 32

for id in $(seq 1 32); do
  printf -v txid "%03X" $((0x140 + id))
  echo "try motor ID=$id, CAN ID=$txid"
  cansend can0 ${txid}#9A00000000000000
  sleep 0.1
done

상태 읽기 송신
cansend can0 141#9A00000000000000
엔코더 읽기 송신
cansend can0 141#9000000000000000
모델명 읽기 송신
cansend can0 141#B500000000000000

모터 10도 돌리기
cansend can0 141#A8001E00E8030000d


#캔아이디 부여

cansend can0 300#7900000000000001
cansend can0 300#7900000000000002
cansend can0 300#7900000000000004

조이스틱으로 모터 구동 테스트

터미널1 can0 키기
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

ip -details link show can0
위에 줄 이건 생략 가능일 걸
candump can0

터미널2 조이스틱 노드 키기
source /opt/ros/humble/setup.bash

ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p autorepeat_rate:=20.0

터미널3 /joy 수신 확인
source /opt/ros/humble/setup.bash

ros2 topic hz /joy

터미널4 빌드 및 모터제어
cd /home/s/joy_deg_test_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select joy_motor_test_deg

source install/setup.bash
ros2 run joy_motor_test_deg joy_rmd_increment_test