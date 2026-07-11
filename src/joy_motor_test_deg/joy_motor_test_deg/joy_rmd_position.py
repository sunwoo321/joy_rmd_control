import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import can

class JoyRmdPositionControl(Node):
    def __init__(self):
        super().__init__('joy_rmd_position_control')

        # =========================================================
        # 1. CAN / RMD 기본 설정
        # =========================================================
        self.can_channel = 'can0'
        self.left_motor_id = 1
        self.right_motor_id = 2

        # =========================================================
        # 2. 제어 및 안전 파라미터 (각도 기반)
        # =========================================================
        self.deadzone = 0.15
        
        # 각도 제어를 위한 현재 타겟 각도 변수 (누적식)
        self.target_angle_left = 0.0
        self.target_angle_right = 0.0
        
        # 조이스틱 스틱을 끝까지 밀었을 때 1주기(0.02초)당 증가할 최대 각도 (단위: 도 [degree])
        # 값이 클수록 조이스틱을 밀 때 모터가 더 빠르게 회전합니다. (초기 테스트용으로 보수적 설정)
        self.max_delta_angle = 2.0 

        # =========================================================
        # 3. 조이스틱 매핑 및 상태 변수
        # =========================================================
        self.axis_wz = 0      # 왼쪽 스틱 좌우 (회전)
        self.axis_vx = 1      # 왼쪽 스틱 상하 (직진)
        self.button_share = 8
        self.button_ps = 10

        self.manual_mode = True
        self.emergency_stop = False
        self.prev_share = 0

        # =========================================================
        # 4. 왓치독(Watchdog) 설정 
        # =========================================================
        # 마지막으로 조이스틱 데이터를 받은 시간 기록
        self.last_joy_time = self.get_clock().now()
        # 왓치독 허용 타임아웃 시간 (0.2초 동안 패킷이 없으면 끊긴 것으로 판단)
        self.watchdog_timeout_sec = 0.2 
        
        # 왓치독 상태 점검을 위한 주기적 타이머 생성 (50Hz, 20ms 주기)
        self.watchdog_timer = self.create_timer(0.02, self.watchdog_check)

        # =========================================================
        # 5. CAN 버스 연결 및 ROS2 구독
        # =========================================================
        try:
            self.bus = can.interface.Bus(channel=self.can_channel, interface='socketcan')
            self.get_logger().info(f'[{self.can_channel}] SocketCAN 연결 성공')
        except Exception as e:
            self.get_logger().error(f'CAN 연결 실패: {e}')
            exit()

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('위치(각도) 제어 및 왓치독 활성화 노드가 시작되었습니다.')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        # 조이스틱 데이터 수신 시간 최신화 (왓치독 리셋)
        self.last_joy_time = self.get_clock().now()

        if len(msg.axes) <= max(self.axis_wz, self.axis_vx) or len(msg.buttons) <= max(self.button_share, self.button_ps):
            return

        raw_wz = self.apply_deadzone(msg.axes[self.axis_wz])
        raw_vx = self.apply_deadzone(msg.axes[self.axis_vx])

        # 비상정지 및 모드 전환 처리
        if msg.buttons[self.button_ps] == 1:
            self.emergency_stop = True
            self.get_logger().error('비상 정지 발동! (PS 버튼)')

        if msg.buttons[self.button_share] == 1 and self.prev_share == 0:
            self.manual_mode = not self.manual_mode
            self.get_logger().info(f"모드 변경 -> {'MANUAL' if self.manual_mode else 'AUTO'}")
        self.prev_share = msg.buttons[self.button_share]

        if self.emergency_stop or not self.manual_mode:
            return

        # 조이스틱 입력에 따른 좌우 휠 타겟 각도 증분 계산 (차동 매커니즘 응용)
        delta_left = (raw_vx - raw_wz) * self.max_delta_angle
        delta_right = (raw_vx + raw_wz) * self.max_delta_angle

        # 현재 각도 목표치에 누적
        self.target_angle_left += delta_left
        self.target_angle_right += delta_right

        # 모터로 각도 명령 송신
        self.send_rmd_position(self.left_motor_id, self.target_angle_left)
        self.send_rmd_position(self.right_motor_id, self.target_angle_right)

    def watchdog_check(self):
        """ 주기적으로 조이스틱과의 통신 유실을 감시하는 왓치독 함수 """
        if self.emergency_stop:
            return

        now = self.get_clock().now()
        time_since_last_joy = (now - self.last_joy_time).nanoseconds / 1e9

        # 설정한 타임아웃(0.2초)보다 오랜 시간 동안 조이스틱 패킷이 안 들어왔다면 끊긴 것으로 간주 
        if time_since_last_joy > self.watchdog_timeout_sec:
            self.get_logger().error(f' 조이스틱 무선 연결 유실 감지! ({time_since_last_joy:.2f}초간 패킷 없음) 안전 정지합니다.') [cite: 377]
            self.emergency_stop = True
            self.stop_motors()

    def send_rmd_position(self, motor_id, angle_deg):
        """ RMD Position Control (0xA8) - Little Endian 변환 """
        can_id = 0x140 + motor_id
        
        # RMD 모터의 위치 제어 단위를 0.01도/LSB로 변환
        # 예: 10도 회전 명령어 -> 1000 주입
        pos_control = int(angle_deg * 100)

        # 32비트 int형 데이터를 Little-endian 바이트 배열로 변환 
        pos_bytes = pos_control.to_bytes(4, byteorder='little', signed=True)

        data = [
            0xA8,  # 위치 제어 커맨드 레지스터
            0x00,  # 속도 제한 (0x00 사용 시 기본 내부 최대 속도 적용)
            0x00,
            0x00,
            pos_bytes[0],
            pos_bytes[1],
            pos_bytes[2],
            pos_bytes[3]
        ]

        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            pass

    def stop_motors(self):
        # 정지 시 현재 위치를 목표값으로 고정하여 멈추게 함
        self.send_rmd_position(self.left_motor_id, self.target_angle_left)
        self.send_rmd_position(self.right_motor_id, self.target_angle_right)

def main(args=None):
    rclpy.init(args=args)
    node = JoyRmdPositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()