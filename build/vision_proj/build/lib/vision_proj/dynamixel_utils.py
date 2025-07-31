import serial
import numpy as np
import time

def radian_to_dxl(rad):
    pos = int((rad * 180 / np.pi) / 300 * 1023)
    pos = max(0, min(1023, pos))
    return pos

def send_dxl_position(ser, dxl_id, position):
    
    # AX-12A로 목표 위치(0~1023) 전송
    # ser: serial.Serial 객체
    # dxl_id: 모터 ID (int)
    # position: 목표 위치 (0~1023, int)
    
    # AX-12A Goal Position 주소: 0x1E (30), 2바이트
    addr = 0x1E
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF
    length = 5

    packet = [0xFF, 0xFF, dxl_id, length, 0x03, addr, pos_l, pos_h]
    checksum = ~(dxl_id + length + 0x03 + addr + pos_l + pos_h) & 0xFF
    packet.append(checksum)

    ser.write(bytearray(packet))

    print(f"[TX] ID:{dxl_id} → {position} ({[hex(x) for x in packet]})")

def send_joint_positions(ser, joint_rads, gripper=None):
    
    # joint_rads: 각 조인트별 라디안 리스트 [rad, rad, ...]
    # gripper: 그리퍼 값 (0~1023), 없으면 None
    
    for i, rad in enumerate(joint_rads, 1):
        pos = radian_to_dxl(rad)
        send_dxl_position(ser, dxl_id=i, position=pos)
        time.sleep(0.01)

    if gripper is not None:
        send_dxl_position(ser, dxl_id=len(joint_rads)+1, position=gripper)