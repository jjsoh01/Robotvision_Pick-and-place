# vision_proj/vision_proj/transform_utils.py

import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo

def get_3d_point_from_depth(depth_image, u, v, camera_info):
    """
    깊이 이미지, 2D 픽셀 좌표 (u, v), 카메라 내부 파라미터 (CameraInfo)를 사용하여
    카메라 좌표계에서의 3D 점 (X, Y, Z)를 계산합니다.

    Args:
        depth_image (np.array): 16UC1 (unsigned short) 또는 CV_16U 형식의 깊이 이미지. 단위는 mm.
        u (int): 픽셀의 x 좌표 (열).
        v (int): 픽셀의 y 좌표 (행).
        camera_info (CameraInfo): 카메라 내부 파라미터를 포함하는 ROS CameraInfo 메시지.

    Returns:
        np.array: [X, Y, Z] 형태의 3D 점 (단위: 미터), 또는 유효한 깊이를 찾을 수 없으면 None.
    """
    if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
        print(f"Error: Pixel ({u}, {v}) is out of depth image bounds ({depth_image.shape[1]}, {depth_image.shape[0]}).")
        return None

    # 깊이 값 (mm 단위)
    depth_mm = depth_image[v, u]

    if depth_mm == 0: # 깊이 값이 0이면 유효하지 않음 (대부분의 카메라에서 배경, 측정 불가 의미)
        # print(f"Warning: Depth value at pixel ({u}, {v}) is zero.")
        return None

    # mm를 미터로 변환
    depth_m = float(depth_mm) / 1000.0

    # 카메라 내부 파라미터 추출
    # CameraInfo.k 배열은 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    # 3D 점 계산
    # X = (u - cx) * Z / fx
    # Y = (v - cy) * Z / fy
    # Z = depth_m (카메라의 Z축이 깊이 방향)

    point_x = (u - cx) * depth_m / fx
    point_y = (v - cy) * depth_m / fy
    point_z = depth_m

    return np.array([point_x, point_y, point_z])

def load_camera_robot_transform(filepath):
    """
    YAML 파일에서 카메라-로봇 베이스 변환 행렬 (4x4)을 로드합니다.
    이 변환은 외부 캘리브레이션 결과를 나타냅니다.

    Args:
        filepath (str): 변환 행렬이 저장된 YAML 파일 경로.
                        예시:
                        transformation_matrix:
                          - [1.0, 0.0, 0.0, 0.1]
                          - [0.0, 1.0, 0.0, 0.2]
                          - [0.0, 0.0, 1.0, 0.3]
                          - [0.0, 0.0, 0.0, 1.0]

    Returns:
        np.array: 4x4 변환 행렬 (numpy 배열), 또는 로드 실패 시 Identity Matrix.
    """
    try:
        with open(filepath, 'r') as file:
            config = yaml.safe_load(file)
            if 'transformation_matrix' in config:
                transform_matrix = np.array(config['transformation_matrix'])
                if transform_matrix.shape == (4, 4):
                    print(f"Successfully loaded transform from {filepath}")
                    return transform_matrix
                else:
                    print(f"Warning: Transform matrix in {filepath} is not 4x4. Using identity.")
            else:
                print(f"Warning: 'transformation_matrix' key not found in {filepath}. Using identity.")
    except Exception as e:
        print(f"Error loading transform from {filepath}: {e}. Using identity matrix.")

    return np.identity(4) # 로드 실패 시 기본 Identity Matrix 반환

# (선택 사항) 로봇 팔의 End-effector로부터 Grasp Point까지의 변환 (간단한 예시)
def get_grasp_point_from_end_effector(end_effector_pose):
    """
    로봇 팔 End-effector의 Pose에서 그리핑 포인트를 계산합니다.
    이는 End-effector 기준으로 그리퍼가 얼마나 떨어져 있는지에 따라 달라집니다.

    Args:
        end_effector_pose (np.array): [x, y, z, roll, pitch, yaw] 또는 4x4 Homogeneous Transform Matrix
                                     여기서는 간단히 [x, y, z]만 있다고 가정.

    Returns:
        np.array: 그리핑 포인트의 [x, y, z] 좌표.
    """
    # 예시: End-effector에서 -Z 방향으로 5cm 떨어진 지점이 그리핑 포인트라고 가정
    # 실제로는 그리퍼의 기구학적 특성에 따라 더 복잡한 변환이 필요할 수 있습니다.
    grasp_offset_z = -0.05 # 미터 단위 (5cm 아래)

    if len(end_effector_pose) == 3: # [x, y, z]
        return np.array([end_effector_pose[0], end_effector_pose[1], end_effector_pose[2] + grasp_offset_z])
    elif end_effector_pose.shape == (4, 4): # Homogeneous Transform
        # End-effector Transform에서 Z축을 따라 이동
        grasp_point = np.dot(end_effector_pose, np.array([0, 0, grasp_offset_z, 1.0]))
        return grasp_point[:3]
    else:
        print("Unsupported end_effector_pose format.")
        return None