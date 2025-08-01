import numpy as np
import yaml
import os

class OpenManipulatorXKinematics:
    def __init__(self, config_file=None):
        self.L1 = 0.077
        self.L2 = 0.130
        self.L3 = 0.124
        self.L4 = 0.120

        self.joint_limits = {
            'joint1': [-2.618, 2.618],
            'joint2': [-2.618, 2.618],
            'joint3': [-1.047, 4.188],
            'joint4': [-2.618, 2.618],
        }

        if config_file and os.path.exists(config_file):
            self._load_config(config_file)
        else:
            print("Warning: Robot config file not provided or not found. Using default kinematics parameters.")

    # def _load_config(self, filepath):
    #     try:
    #         with open(filepath, 'r') as file:
    #             config = yaml.safe_load(file)
    #             if 'link_lengths' in config:
    #                 lengths = config['link_lengths']
    #                 self.L1 = lengths.get('L1', self.L1)
    #                 self.L2 = lengths.get('L2', self.L2)
    #                 self.L3 = lengths.get('L3', self.L3)
    #                 self.L4 = lengths.get('L4', self.L4)
    #             if 'joint_limits' in config:
    #                 for joint, limits in config['joint_limits'].items():
    #                     if joint in self.joint_limits:
    #                         self.joint_limits[joint] = [np.radians(limits[0]), np.radians(limits[1])]
    #             print(f"Successfully loaded robot config from {filepath}")
    #     except Exception as e:
    #         print(f"Error loading robot config from {filepath}: {e}. Using default parameters.")

    def inverse_kinematics(self, target_x, target_y, target_z):
        L1, L2, L3, L4 = self.L1, self.L2, self.L3, self.L4
        # 1. q1 (Base Yaw)
        q1 = np.arctan2(target_y, target_x)

        # 2. 평면 거리 및 높이
        r = np.sqrt(target_x**2 + target_y**2)
        z_prime = target_z - L1
        z_prime = -z_prime

        # 3. Wrist까지의 직선 거리
        d = np.sqrt(r**2 + z_prime**2)
        # 마지막 링크는 End-effector 방향으로 가정(L4)
        l23 = L3 + L4

        # 4. q3 (Elbow)
        # Cosine Law
        cos_q3 = (d**2 - L2**2 - l23**2) / (2 * L2 * l23)
        # 수치 오차로 인한 도메인 에러 방지
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        q3 = np.arccos(cos_q3)

        # 5. q2 (Shoulder)
        theta = np.arctan2(z_prime, r)
        phi = np.arctan2(l23 * np.sin(q3), L2 + l23 * np.cos(q3))
        q2 = theta - phi

        # 6. q4 (Wrist)
        q4 = -q2 - q3

        # joint limits 적용
        q_all = np.array([q1, q2, q3, q4])
        for idx, (q, (low, high)) in enumerate(zip(q_all, self.joint_limits.values())):
            if not (low <= q <= high):
                print(f"Joint {idx+1} out of limit: {np.degrees(q):.2f} deg (limit: {np.degrees(low):.2f} ~ {np.degrees(high):.2f})")
                return None

        return q_all
