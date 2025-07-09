# vision_proj/vision_proj/dynamixel_utils.py

import serial
import time
import struct # 데이터를 바이너리로 패킹/언패킹

# Dynamixel AX-12A 제어를 위한 프로토콜 1.0 상수
# Instruction Set
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_RESET = 0x06
INST_SYNC_WRITE = 0x83
INST_BULK_READ = 0x92

# Control Table Addresses (AX-12A)
ADDR_GOAL_POSITION = 30 # 0x1E
ADDR_MOVING_SPEED = 32 # 0x20
ADDR_TORQUE_LIMIT = 34 # 0x22
ADDR_PRESENT_POSITION = 36 # 0x24
ADDR_PRESENT_SPEED = 38 # 0x26
ADDR_PRESENT_LOAD = 40 # 0x28
ADDR_PUNCH = 48 # 0x30 (AX-12A에서는 Torque Limit으로 사용될 수 있음)

class DynamixelAX12A:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        try:
            # 시리얼 포트 설정
            self.ser = serial.Serial(port, baudrate, timeout=0.1) # timeout 설정 중요
            print(f"Successfully connected to serial port {port} at {baudrate} baudrate.")
        except serial.SerialException as e:
            print(f"Error opening serial port {port}: {e}")
            raise

    def _calculate_checksum(self, packet_bytes):
        """패킷의 체크섬을 계산합니다."""
        checksum = 0
        for i in range(2, len(packet_bytes)): # ID부터 시작
            checksum += packet_bytes[i]
        checksum = (255 - (checksum % 256)) & 0xFF
        return checksum

    def _make_packet(self, dxl_id, instruction, parameters):
        """Dynamixel 프로토콜 1.0 패킷을 구성합니다."""
        length = len(parameters) + 2 # LENGTH = Parameter Length + Instruction (1 byte) + Checksum (1 byte)

        # 헤더 (0xFF 0xFF) + ID + LENGTH + INSTRUCTION + PARAMETERS
        packet = bytearray([0xFF, 0xFF, dxl_id, length, instruction]) + bytearray(parameters)

        checksum = self._calculate_checksum(packet)
        packet.append(checksum)
        return packet

    def _read_response(self, expected_len):
        """Dynamixel 응답 패킷을 읽습니다."""
        # 응답 패킷 구조: 0xFF 0xFF ID LENGTH ERROR_CODE [DATA...] CHECKSUM
        # 최소 6바이트 (헤더 2 + ID 1 + 길이 1 + 에러 1 + 체크섬 1)

        header_found = False
        response_buffer = bytearray()

        start_time = time.time()
        while time.time() - start_time < self.ser.timeout * 5: # 타임아웃보다 길게 대기
            byte = self.ser.read(1)
            if not byte:
                continue

            response_buffer.extend(byte)

            if not header_found:
                if len(response_buffer) >= 2 and response_buffer[-2:] == b'\xFF\xFF':
                    header_found = True
                    response_buffer = response_buffer[-2:] # 헤더만 남김

            if header_found and len(response_buffer) >= 4: # ID, LENGTH까지 읽었을 때
                packet_length = response_buffer[3]
                if len(response_buffer) >= packet_length + 4: # 전체 패킷 길이 (헤더 제외)
                    # 전체 패킷을 읽었는지 확인
                    full_packet = response_buffer[:packet_length + 4]
                    # 체크섬 검증
                    received_checksum = full_packet[-1]
                    calculated_checksum = self._calculate_checksum(full_packet[:-1])

                    if received_checksum == calculated_checksum:
                        return full_packet
                    else:
                        print(f"Checksum mismatch. Received: {received_checksum}, Calculated: {calculated_checksum}")
                        response_buffer = response_buffer[1:] # 1바이트씩 앞으로 이동하여 재탐색
                        header_found = False # 다시 헤더 탐색
        return None # 타임아웃 또는 유효한 패킷을 찾지 못함

    def ping(self, dxl_id):
        """Dynamixel 모터와 통신 가능한지 확인합니다."""
        packet = self._make_packet(dxl_id, INST_PING, [])
        self.ser.write(packet)
        response = self._read_response(6) # PING 응답은 6바이트 (FF FF ID LENGTH ERROR CHECKSUM)
        if response and response[4] == 0: # ERROR_CODE가 0이면 성공
            return True
        return False

    def set_position(self, dxl_id, position):
        """
        Dynamixel AX-12A의 목표 위치를 설정합니다.
        position: 0 ~ 1023 (10비트)
        """
        if not (0 <= position <= 1023):
            print(f"Warning: Position {position} out of valid range (0-1023).")
            return False

        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF

        parameters = [ADDR_GOAL_POSITION, pos_l, pos_h] # Address, Low Byte, High Byte
        packet = self._make_packet(dxl_id, INST_WRITE, parameters)
        self.ser.write(packet)
        # WRITE 명령은 응답을 기다리지 않아도 됨 (PING이나 READ만 응답 필요)
        # 하지만 통신 오류 확인을 위해 응답을 확인하는 것이 좋음
        response = self._read_response(6)
        if response and response[4] == 0: # ERROR_CODE가 0이면 성공
            return True
        print(f"Failed to set position for DXL ID {dxl_id}. Error code: {response[4] if response else 'No response'}")
        return False

    def get_position(self, dxl_id):
        """
        Dynamixel AX-12A의 현재 위치를 읽습니다.
        반환 값: 0 ~ 1023
        """
        parameters = [ADDR_PRESENT_POSITION, 2] # Address, Length (2 bytes for position)
        packet = self._make_packet(dxl_id, INST_READ, parameters)
        self.ser.write(packet)
        response = self._read_response(8) # READ 응답은 8바이트 (FF FF ID LENGTH ERROR DATA1 DATA2 CHECKSUM)
        if response and response[4] == 0 and len(response) == 8:
            pos_l = response[5]
            pos_h = response[6]
            return (pos_h << 8) | pos_l
        print(f"Failed to get position for DXL ID {dxl_id}. Error code: {response[4] if response else 'No response'}")
        return None

    def set_moving_speed(self, dxl_id, speed):
        """
        Dynamixel AX-12A의 이동 속도를 설정합니다.
        speed: 0 ~ 1023 (0은 무제한)
        """
        if not (0 <= speed <= 1023):
            print(f"Warning: Speed {speed} out of valid range (0-1023).")
            return False

        speed_l = speed & 0xFF
        speed_h = (speed >> 8) & 0xFF

        parameters = [ADDR_MOVING_SPEED, speed_l, speed_h]
        packet = self._make_packet(dxl_id, INST_WRITE, parameters)
        self.ser.write(packet)
        response = self._read_response(6)
        if response and response[4] == 0:
            return True
        return False

    def close(self):
        """시리얼 포트를 닫습니다."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")