# vision_proj/vision_proj/open_manipulator_x_kinematics.py

import numpy as np
from scipy.optimize import minimize
import yaml
import os

class OpenManipulatorXKinematics:
    def __init__(self, config_file=None):
        # 로봇 링크 길이 (미터 단위) - OpenManipulator-X 4DOF 기준 (그리퍼 제외)
        # 실제 로봇의 정확한 길이를 측정하여 입력해야 합니다.
        self.L1 = 0.077   # Base to Shoulder (Center of Rotation)
        self.L2 = 0.126   # Shoulder to Elbow (Center of Rotation)
        self.L3 = 0.124   # Elbow to Wrist (Center of Rotation)
        self.L4 = 0.130   # Wrist to End-effector (Tool plate or attachment point, excluding gripper)

        # 조인트 제한 (라디안 단위) - AX-12A는 +/- 150도 범위 (약 +/- 2.618 라디안)
        # OpenManipulator-X AX-12A 버전의 실제 물리적 제한을 확인하여 설정
        self.joint_limits = {
            'joint1': [-2.618, 2.618], # Base (0~1023, 512가 0도)
            'joint2': [-2.618, 2.618], # Shoulder
            'joint3': [-2.618, 2.618], # Elbow
            'joint4': [-2.618, 2.618], # Wrist
        }

        # Dynamixel AX-12A 위치값 (0-1023)과 라디안 각도 간의 변환 계수
        # AX-12A는 300도 (5.23599 rad) 범위를 1024 스텝으로 표현
        # 0 ~ 1023 (300도)
        # 512가 중심 (150도)
        self.DXL_MAX_POS = 1023
        self.DXL_MIN_POS = 0
        self.DXL_MID_POS = 511.5 # 512가 0도 중심이라고 할 때
        self.DXL_RANGE_RAD = np.radians(300) # 300도 = 5.23599 라디안

        if config_file and os.path.exists(config_file):
            self._load_config(config_file)
        else:
            print("Warning: Robot config file not provided or not found. Using default kinematics parameters.")

    def _load_config(self, filepath):
        """YAML 파일에서 로봇 설정 (링크 길이, 조인트 제한 등)을 로드합니다."""
        try:
            with open(filepath, 'r') as file:
                config = yaml.safe_load(file)
                if 'link_lengths' in config:
                    lengths = config['link_lengths']
                    self.L1 = lengths.get('L1', self.L1)
                    self.L2 = lengths.get('L2', self.L2)
                    self.L3 = lengths.get('L3', self.L3)
                    self.L4 = lengths.get('L4', self.L4)
                if 'joint_limits' in config:
                    for joint, limits in config['joint_limits'].items():
                        if joint in self.joint_limits:
                            self.joint_limits[joint] = [np.radians(limits[0]), np.radians(limits[1])]
                print(f"Successfully loaded robot config from {filepath}")
        except Exception as e:
            print(f"Error loading robot config from {filepath}: {e}. Using default parameters.")

    def radian_to_dxl(self, angle_rad, motor_id=None):
        """라디안 각도를 Dynamixel AX-12A의 0-1023 위치 값으로 변환합니다."""
        # 이 변환은 각 모터의 설치 방향과 "0도"의 정의에 따라 달라질 수 있습니다.
        # OpenManipulator-X의 경우, 조인트 2, 3, 4는 초기 자세가 보통 0도를 바라보도록 설정됩니다.
        # AX-12A 512가 150도이므로, 0도를 512로 매핑

        # 라디안 -> 0~300도 범위의 비율로 변환
        # 예시: -150도 ~ +150도 (라디안) -> 0 ~ 1023
        # 0 라디안 -> 512
        # -2.618 rad (-150 deg) -> 0
        # +2.618 rad (+150 deg) -> 1023

        # angle_rad + self.DXL_RANGE_RAD / 2  --> 0 ~ DXL_RANGE_RAD 범위로 변환
        # (angle_rad + 2.618) / 5.236 * 1023

        # 일반적인 변환: dxl_pos = (angle_rad / (2 * np.pi)) * 1024 + 512
        # AX-12A 특화: angle_deg = np.degrees(angle_rad)
        # dxl_pos = int(angle_deg / 300.0 * 1023 + self.DXL_MID_POS) # 이 매핑은 로봇의 기구학에 맞게 조정 필요

        # OpenManipulator-X의 경우, 각 조인트의 초기 위치와 물리적 방향에 따라 변환이 다를 수 있습니다.
        # 여기서는 간단하게 AX-12A의 0~1023 범위에 맞춰 맵핑하는 방법을 제안합니다.
        # 즉, -150도 ~ +150도를 0~1023에 맵핑하는 경우:
        # pos = (angle_rad - (-np.radians(150))) / np.radians(300) * 1023

        # 가장 일반적인 Robotis DXL SDK의 mapping:
        # rad = (dxl_value - 512) * (300/1024) * (pi/180)
        # dxl_value = rad * (180/pi) * (1024/300) + 512

        dxl_value = int(angle_rad * (1024 / self.DXL_RANGE_RAD) + self.DXL_MID_POS)

        # 범위 벗어나면 클리핑
        dxl_value = max(self.DXL_MIN_POS, min(self.DXL_MAX_POS, dxl_value))
        return dxl_value

    def dxl_to_radian(self, dxl_pos, motor_id=None):
        """Dynamixel AX-12A의 0-1023 위치 값을 라디안 각도로 변환합니다."""
        # rad = (dxl_pos - self.DXL_MID_POS) * (self.DXL_RANGE_RAD / 1024)
        rad = (dxl_pos - self.DXL_MID_POS) * (self.DXL_RANGE_RAD / 1024.0)
        return rad

    def forward_kinematics(self, joint_angles):
        """
        4자유도 OpenManipulator-X의 순기구학을 계산합니다.
        Base, Shoulder, Elbow, Wrist 조인트 각도 (라디안)를 입력받아
        End-effector의 XYZ 좌표를 반환합니다. (Roll, Pitch, Yaw는 4DOF에서 한정적)

        Args:
            joint_angles (list/np.array): [q1, q2, q3, q4] 라디안 각도.
                                          q1: Base (Yaw), q2: Shoulder (Pitch),
                                          q3: Elbow (Pitch), q4: Wrist (Pitch)

        Returns:
            np.array: [x, y, z] End-effector의 XYZ 좌표 (미터).
        """
        q1, q2, q3, q4 = joint_angles

        # DH 파라미터 또는 회전 변환 행렬을 이용한 방법이 일반적입니다.
        # 여기서는 OpenManipulator-X의 기구학적 구조를 바탕으로 직접 계산하는 예시를 들겠습니다.
        # OpenManipulator-X는 2,3,4번 조인트가 같은 평면상에서 동작하는 평면형 로봇 팔로 볼 수 있습니다.

        # 링크 길이
        L1, L2, L3, L4 = self.L1, self.L2, self.L3, self.L4

        # 1번 조인트 (Base)는 Z축 회전
        x0 = 0
        y0 = 0
        z0 = L1 # Base까지의 높이

        # X-Y 평면에서의 팔의 길이 (L2, L3, L4의 투영)
        # q2는 Shoulder, q3는 Elbow, q4는 Wrist
        # 각도는 로봇의 0도 기준으로부터의 각도입니다.
        # OpenManipulator-X의 경우, 0도는 팔이 앞으로 뻗은 자세입니다.
        # q2 + q3는 Elbow의 절대 각도, q2+q3+q4는 Wrist의 절대 각도

        # Shoulder 조인트까지의 X, Z 좌표
        x_shoulder = 0
        z_shoulder = L1

        # End-effector의 Z 좌표 (수직 방향)
        # L2 * sin(q2) + L3 * sin(q2+q3) + L4 * sin(q2+q3+q4)
        z_eff = z_shoulder + L2 * np.sin(q2) + L3 * np.sin(q2 + q3) + L4 * np.sin(q2 + q3 + q4)

        # End-effector의 X-Y 평면상의 팔의 길이 (수평 방향)
        # L2 * cos(q2) + L3 * cos(q2+q3) + L4 * cos(q2+q3+q4)
        r_eff = L2 * np.cos(q2) + L3 * np.cos(q2 + q3) + L4 * np.cos(q2 + q3 + q4)

        # Base 조인트 (q1) 회전 고려
        x_eff = r_eff * np.cos(q1)
        y_eff = r_eff * np.sin(q1)

        # OpenManipulator-X는 z축이 위, x축이 앞, y축이 좌 (오른손 법칙)
        # 로봇 베이스를 기준으로 End-effector의 최종 XYZ 좌표
        return np.array([x_eff, y_eff, z_eff])


    def inverse_kinematics(self, target_x, target_y, target_z, initial_joint_angles=None):
        """
        4자유도 OpenManipulator-X의 역기구학을 계산합니다 (수치적 방법).
        목표 End-effector XYZ 좌표를 입력받아 조인트 각도 [q1, q2, q3, q4]를 반환합니다.

        Args:
            target_x, target_y, target_z (float): 목표 End-effector XYZ 좌표 (미터).
            initial_joint_angles (list/np.array, optional): 최적화 시작을 위한 초기 조인트 각도.
                                                             제공되지 않으면 0으로 시작.

        Returns:
            np.array: [q1, q2, q3, q4] 라디안 각도, 또는 해를 찾지 못하면 None.
        """
        if initial_joint_angles is None:
            # 초기 추정치 (중립 자세 또는 0 라디안)
            initial_joint_angles = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            initial_joint_angles = np.array(initial_joint_angles)

        # 조인트 제한 설정
        bounds = [
            self.joint_limits['joint1'], # q1
            self.joint_limits['joint2'], # q2
            self.joint_limits['joint3'], # q3
            self.joint_limits['joint4'], # q4
        ]

        def objective_function(joint_angles):
            """현재 조인트 각도에서 End-effector 위치와 목표 위치 간의 거리를 최소화하는 목적 함수."""
            current_x, current_y, current_z = self.forward_kinematics(joint_angles)

            # 유클리드 거리의 제곱을 최소화
            return (current_x - target_x)**2 + \
                   (current_y - target_y)**2 + \
                   (current_z - target_z)**2

        # 최적화 수행
        # method='SLSQP'는 경계 제약 조건을 지원하는 시퀀셜 최소 제곱 프로그래밍 알고리즘
        result = minimize(objective_function, initial_joint_angles, bounds=bounds, method='SLSQP')

        if result.success and result.fun < 1e-4: # 목적 함수 값이 충분히 작으면 성공으로 간주
            # 최적화된 조인트 각도 반환
            return result.x
        else:
            print(f"Inverse Kinematics failed to find a solution. Result: {result.message}, Function value: {result.fun}")
            return None