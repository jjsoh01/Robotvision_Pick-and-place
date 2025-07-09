# vision_proj/vision_proj/opencm_controller.py

import serial
import time

class OpenCmDxlController:
    """
    OpenCM 펌웨어(#<ID>P<POS> 프로토콜) 에 맞춰
    모터 제어 명령을 ASCII 로 전송하는 래퍼 클래스
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        port:  OpenCM 이 붙은 USB 포트 (예: '/dev/ttyACM0' 또는 'COM5')
        baudrate: 펌웨어 쪽 Serial.begin(115200) 과 일치해야 함
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(0.5)  # 인터페이스 초기화 대기
            # 워밍업 리드
            self.ser.reset_input_buffer()
            print(f"[OpenCM] Connected on {port}@{baudrate}")
        except Exception as e:
            raise RuntimeError(f"Cannot open OpenCM port {port}: {e}")

    def set_position(self, motor_id: int, dxl_pos: int) -> bool:
        """
        AX-12A 모터 ID 에 dxl_pos(0~1023) 값 전송
        리턴: ACK 를 받았으면 True, 아니면 False
        """
        cmd = f"#{motor_id}P{dxl_pos}\n"
        self.ser.write(cmd.encode('ascii'))
        # 짧게 ACK 대기
        start = time.time()
        while time.time() - start < self.ser.timeout:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("OK"):
                return True
        print(f"[OpenCM] No ACK for {cmd.strip()}")
        return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OpenCM] Serial closed")
