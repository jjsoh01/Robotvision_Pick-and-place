# # check_realsense.py

# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import time

# def check_realsense_camera():
#     print("RealSense 카메라 초기화 중...")
#     pipeline = rs.pipeline()
#     config = rs.config()

#     try:
#         # 컬러 스트림 활성화 (640x480, BGR8 포맷, 30fps)
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
#         # 파이프라인 시작
#         profile = pipeline.start(config)
#         print("RealSense 파이프라인 시작 성공.")
        
#         # 카메라가 안정화될 시간 부여
#         print("카메라 안정화 대기 중 (2초)...")
#         time.sleep(2)

#         # 프레임 가져오기
#         frames = pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()

#         if not color_frame:
#             print("오류: 컬러 프레임을 가져오는 데 실패했습니다.")
#             return False

#         # 프레임을 NumPy 배열로 변환
#         color_image = np.asanyarray(color_frame.get_data())

#         if color_image is not None and color_image.shape[0] > 0 and color_image.shape[1] > 0:
#             print(f"성공: 컬러 이미지 획득! 해상도: {color_image.shape[1]}x{color_image.shape[0]}")
#             print(f"이미지 데이터 타입: {color_image.dtype}")
            
#             # (선택 사항) 이미지 보여주기
#             cv2.imshow("RealSense Color Stream", color_image)
#             print("이미지 창이 나타나면 아무 키나 눌러 닫으세요.")
#             cv2.waitKey(0)
#             cv2.destroyAllWindows()
            
#             return True
#         else:
#             print("오류: 유효한 컬러 이미지를 가져왔으나, 이미지 데이터가 비어 있거나 손상되었습니다.")
#             return False

#     except Exception as e:
#         print(f"오류 발생: {e}")
#         print("pyrealsense2 또는 RealSense SDK 설치에 문제가 있을 수 있습니다.")
#         return False
#     finally:
#         # 파이프라인 정지 (리소스 해제)
#         if 'pipeline' in locals() and pipeline is not None:
#             print("RealSense 파이프라인 정지 중...")
#             pipeline.stop()
#         print("RealSense 확인 종료.")

# if __name__ == "__main__":
#     if check_realsense_camera():
#         print("\nRealSense D435i 카메라와 pyrealsense2가 정상적으로 작동합니다!")
#     else:
#         print("\nRealSense D435i 카메라 또는 pyrealsense2 작동에 문제가 있습니다. 이전에 안내해 드린 설치 단계를 다시 확인해 주세요.")