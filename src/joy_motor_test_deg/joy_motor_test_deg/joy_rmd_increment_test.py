#!/usr/bin/env python3

import time
from typing import List

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


class JoyRmdIncrementTest(Node):
    """
    PS4 조이스틱으로 RMD 모터 한 개를 ±10도씩 움직이는 안전 시험 노드.

    조작:
    - R1 + Triangle : +10도
    - R1 + Cross    : -10도
    - R1 해제       : 즉시 정지
    - PS 또는 L1+R1 : 비상정지 고정
    - /joy 단절     : 정지 후 비상정지 고정
    """

    def __init__(self) -> None:
        super().__init__('joy_rmd_increment_test')

        # =========================================================
        # 1. CAN / 모터 설정
        # =========================================================

        self.can_channel = 'can0'
        self.motor_id = 1
        self.can_id = 0x140 + self.motor_id

        # 한 번 누를 때 이동할 출력축 각도
        self.step_angle_deg = 10.0

        # 출력축 최대 이동 속도 [degree/s]
        # 처음에는 15 dps 이하를 권장
        self.max_speed_dps = 15

        # =========================================================
        # 2. PS4 버튼 번호
        # =========================================================
        # 주의:
        # 조이스틱 드라이버에 따라 번호가 다를 수 있으므로
        # ros2 topic echo /joy로 반드시 확인해야 함.

        self.button_cross = 0
        self.button_triangle = 2

        self.button_l1 = 4
        self.button_r1 = 5

        self.button_share = 8
        self.button_options = 9
        self.button_ps = 10

        # =========================================================
        # 3. 안전 설정
        # =========================================================

        # /joy가 이 시간 이상 들어오지 않으면 정지
        self.joy_timeout_sec = 0.30

        # 버튼 중복 입력 방지
        self.command_cooldown_sec = 0.50

        # 비상정지는 한 번 걸리면 프로그램을 재시작해야 해제
        self.emergency_stop = False

        # 마지막 /joy 수신 시각
        self.last_joy_time = time.monotonic()

        # 프로세스 시작 후 실제 /joy를 한 번이라도 받았는지 여부
        self.joy_received = False

        # 한 번의 /joy 단절 구간에서 정지 명령이 반복 송신되지 않도록 함
        self.joy_timeout_stop_sent = False

        # 최근 명령 시각
        self.last_command_time = 0.0

        # 모터가 이동 중이라고 간주하는 상태
        self.motion_active = False

        # 예상 이동 종료 시각
        self.motion_deadline = 0.0

        # 이전 버튼 상태: 한 번 누를 때 한 번만 동작시키기 위함
        self.prev_cross = 0
        self.prev_triangle = 0
        self.prev_r1 = 0

        # 경고 로그 반복 방지
        self.last_timeout_warning = 0.0

        # =========================================================
        # 4. CAN 버스 열기
        # =========================================================

        try:
            self.bus = can.interface.Bus(
                channel=self.can_channel,
                interface='socketcan'
            )
        except Exception as exc:
            self.get_logger().fatal(
                f'CAN bus open failed: {exc}'
            )
            raise

        # 시작과 동시에 정지 명령 전송
        self.send_motor_stop()

        # =========================================================
        # 5. ROS2 /joy 구독 및 watchdog
        # =========================================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile_sensor_data
        )

        # 50 ms마다 안전 상태 확인
        self.watchdog_timer = self.create_timer(
            0.05,
            self.watchdog_callback
        )

        self.get_logger().info('======================================')
        self.get_logger().info('RMD incremental joystick test started')
        self.get_logger().info(f'CAN channel : {self.can_channel}')
        self.get_logger().info(f'Motor ID    : {self.motor_id}')
        self.get_logger().info(f'CAN ID      : {hex(self.can_id)}')
        self.get_logger().info(
            f'Step angle  : {self.step_angle_deg:.1f} deg'
        )
        self.get_logger().info(
            f'Max speed   : {self.max_speed_dps} deg/s'
        )
        self.get_logger().info('R1 + Triangle : +10 deg')
        self.get_logger().info('R1 + Cross    : -10 deg')
        self.get_logger().info('Release R1    : Stop')
        self.get_logger().info('PS or L1+R1   : Emergency stop')
        self.get_logger().info(
            'Waiting for /joy (commands remain disabled until received)'
        )
        self.get_logger().info('======================================')

    # =============================================================
    # 공통 CAN 송신 함수
    # =============================================================

    def send_can_data(self, data: List[int]) -> bool:
        if len(data) != 8:
            self.get_logger().error(
                f'CAN data length must be 8, received={len(data)}'
            )
            return False

        message = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(message, timeout=0.1)
            return True

        except can.CanError as exc:
            self.get_logger().error(
                f'CAN send failed: {exc}'
            )
            return False

    # =============================================================
    # RMD 0xA8 증분 위치제어
    # =============================================================

    def send_increment_angle(
        self,
        angle_deg: float,
        max_speed_dps: int
    ) -> bool:
        """
        Send an RMD incremental position command (0xA8).

        DATA[0]     = 0xA8
        DATA[1]     = 0x00
        DATA[2:4]   = maxSpeed, uint16, little-endian, 1 dps/LSB
        DATA[4:8]   = angleControl, int32, little-endian, 0.01 deg/LSB
        """
        if self.emergency_stop:
            self.get_logger().error(
                'Command rejected: emergency stop is active'
            )
            return False

        if not 1 <= max_speed_dps <= 100:
            self.get_logger().error(
                f'Invalid max speed: {max_speed_dps} dps'
            )
            return False

        # 0.01 degree / LSB
        angle_control = int(round(angle_deg * 100.0))

        # int32 범위 확인
        if not -(2**31) <= angle_control <= (2**31 - 1):
            self.get_logger().error(
                f'Angle command is out of int32 range: {angle_deg}'
            )
            return False

        speed_bytes = max_speed_dps.to_bytes(
            2,
            byteorder='little',
            signed=False
        )

        angle_bytes = angle_control.to_bytes(
            4,
            byteorder='little',
            signed=True
        )

        data = [
            0xA8,
            0x00,
            speed_bytes[0],
            speed_bytes[1],
            angle_bytes[0],
            angle_bytes[1],
            angle_bytes[2],
            angle_bytes[3],
        ]

        success = self.send_can_data(data)

        if success:
            estimated_move_time = (
                abs(angle_deg) / float(max_speed_dps)
            )

            # 실제 제어 지연을 고려하여 약간 여유를 둠
            self.motion_deadline = (
                time.monotonic()
                + estimated_move_time
                + 0.30
            )

            self.motion_active = True

            self.get_logger().info(
                f'Increment command sent: '
                f'angle={angle_deg:+.1f} deg, '
                f'max_speed={max_speed_dps} dps'
            )

        return success

    # =============================================================
    # RMD 0x81 정지
    # =============================================================

    def send_motor_stop(self) -> bool:
        """
        Send an RMD motor stop command (0x81).

        현재 폐루프 모드를 유지하면서 속도를 정지시킴.
        """
        success = self.send_can_data(
            [0x81, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00]
        )

        self.motion_active = False
        self.motion_deadline = 0.0

        if success:
            self.get_logger().info('Motor stop command sent')

        return success

    # =============================================================
    # 비상정지
    # =============================================================

    def activate_emergency_stop(self, reason: str) -> None:
        if self.emergency_stop:
            return

        self.emergency_stop = True

        # 한 번이 아니라 여러 번 보내어 정지 명령 누락 가능성을 낮춤
        for _ in range(3):
            self.send_motor_stop()
            time.sleep(0.02)

        self.get_logger().error(
            f'EMERGENCY STOP: {reason}'
        )
        self.get_logger().error(
            'Restart this node to clear emergency stop'
        )

    # =============================================================
    # 조이스틱 콜백
    # =============================================================

    def joy_callback(self, msg: Joy) -> None:
        self.last_joy_time = time.monotonic()
        self.joy_received = True
        self.joy_timeout_stop_sent = False

        required_button_index = max(
            self.button_cross,
            self.button_triangle,
            self.button_l1,
            self.button_r1,
            self.button_ps
        )

        if len(msg.buttons) <= required_button_index:
            self.activate_emergency_stop(
                f'Joy button array is too short: {len(msg.buttons)}'
            )
            return

        cross = msg.buttons[self.button_cross]
        triangle = msg.buttons[self.button_triangle]
        l1 = msg.buttons[self.button_l1]
        r1 = msg.buttons[self.button_r1]
        ps = msg.buttons[self.button_ps]

        # ---------------------------------------------------------
        # 1. 비상정지
        # ---------------------------------------------------------

        if ps == 1:
            self.activate_emergency_stop(
                'PS button pressed'
            )
            return

        if l1 == 1 and r1 == 1:
            self.activate_emergency_stop(
                'L1 and R1 pressed together'
            )
            return

        if self.emergency_stop:
            return

        # ---------------------------------------------------------
        # 2. R1 데드맨 스위치
        # ---------------------------------------------------------

        # R1을 놓는 순간, 이동 중이면 즉시 정지
        if self.prev_r1 == 1 and r1 == 0:
            if self.motion_active:
                self.send_motor_stop()
                self.get_logger().warn(
                    'R1 released while moving: motor stopped'
                )

        # R1이 눌리지 않았으면 이동 명령 금지
        if r1 == 0:
            self.prev_cross = cross
            self.prev_triangle = triangle
            self.prev_r1 = r1
            return

        # ---------------------------------------------------------
        # 3. 버튼 상승 에지 검출
        # ---------------------------------------------------------

        cross_pressed = (
            cross == 1 and self.prev_cross == 0
        )

        triangle_pressed = (
            triangle == 1 and self.prev_triangle == 0
        )

        now = time.monotonic()

        cooldown_ok = (
            now - self.last_command_time
            >= self.command_cooldown_sec
        )

        if cooldown_ok:
            if triangle_pressed:
                if self.send_increment_angle(
                    +self.step_angle_deg,
                    self.max_speed_dps
                ):
                    self.last_command_time = now

            elif cross_pressed:
                if self.send_increment_angle(
                    -self.step_angle_deg,
                    self.max_speed_dps
                ):
                    self.last_command_time = now

        self.prev_cross = cross
        self.prev_triangle = triangle
        self.prev_r1 = r1

    # =============================================================
    # Watchdog
    # =============================================================

    def watchdog_callback(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_joy_time

        # 예상 이동 시간이 지나면 이동 완료로 간주
        if (
            self.motion_active
            and self.motion_deadline > 0.0
            and now >= self.motion_deadline
        ):
            self.motion_active = False
            self.motion_deadline = 0.0

        # /joy 단절
        if elapsed > self.joy_timeout_sec:
            # 이동 여부와 관계없이 단절 구간마다 최소 한 번 0x81을 보낸다.
            # 이동 중 단절은 재시작 전까지 해제되지 않는 비상정지로 처리한다.
            if self.motion_active and not self.emergency_stop:
                self.activate_emergency_stop(
                    f'/joy timeout: {elapsed:.3f} sec'
                )

            elif not self.joy_timeout_stop_sent:
                self.send_motor_stop()
                self.joy_timeout_stop_sent = True

            if now - self.last_timeout_warning > 2.0:
                if self.joy_received:
                    prefix = '/joy timed out'
                else:
                    prefix = 'No /joy message received'
                self.get_logger().warn(
                    f'{prefix} for {elapsed:.2f} sec; motor stop enforced'
                )
                self.last_timeout_warning = now

    # =============================================================
    # 종료 처리
    # =============================================================

    def shutdown_safely(self) -> None:
        self.get_logger().info(
            'Shutting down: sending motor stop'
        )

        for _ in range(3):
            self.send_motor_stop()
            time.sleep(0.02)

        try:
            self.bus.shutdown()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = JoyRmdIncrementTest()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f'Fatal error: {exc}')

    finally:
        if node is not None:
            node.shutdown_safely()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
