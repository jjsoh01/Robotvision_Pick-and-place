import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dynamixel_sdk import *  

ADDR_TORQUE_ENABLE  = 24
ADDR_GOAL_POSITION  = 30
ADDR_PRESENT_POSITION = 36
ADDR_MX_MOVING              = 46

ADDR_MX_MOVING_SPEED = 32

# Data Byte 크기
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_POSITION     = 2
LEN_MX_MOVING               = 1
LEN_MX_MOVING_SPEED  = 2


# Protocol version
PROTOCOL_VERSION            = 1.0

# Default setting
DXL1_ID                     = 1                 
DXL2_ID                     = 2                 
DXL3_ID                     = 3                 
DXL4_ID                     = 4                 
DXL5_ID                     = 5                 

DXL_IDS = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID]

BAUDRATE                    = 1000000             
DEVICENAME                  = '/dev/ttyACM0'    # 각자 포트 이름으로 수정.

TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0              

DXL_MOVING_STATUS_THRESHOLD = 20                # 보정값

def dxl_speed(portHandler, packetHandler, dxl_id, speed_val):
    """
    Dynamixel 속도 설정 함수 (AX-12A 기준)
    speed_val: 0 ~ 1023 (0=무제한, 1023≈114 rpm)
    """
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, dxl_id, ADDR_MX_MOVING_SPEED, int(speed_val)
    )

    if dxl_comm_result != COMM_SUCCESS:
        print(f"[ID:{dxl_id}] Comm Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"[ID:{dxl_id}] DXL Error: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print(f"[ID:{dxl_id}] 속도 설정 : {speed_val}")


class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        # 포트 & 패킷 핸들러 초기화
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("Port open 실패")
            quit()
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Baudrate 설정 실패")
            quit()

        # 모든 모터 Torque Enable
        for dxl_id in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel#{dxl_id} successfully connected.")

            speed_tf = 70  
            dxl_speed(self.portHandler, self.packetHandler, dxl_id, speed_tf)

        # 목표 각도 구독
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'dynamixel/joint_goals',   # pick-and-place node가 퍼블리시할 토픽 이름
            self.goal_callback,
            10
        )

        self.get_logger().info("DynamixelControlNode 준비 완료.")

    def goal_callback(self, msg: Float64MultiArray):
        """목표 각도(rad 또는 tick)를 받아 모터에 전달"""
        positions = msg.data

        if len(positions) != len(DXL_IDS):
            self.get_logger().error("잘못된 joint positions.")
            return

        for dxl_id, goal_pos in zip(DXL_IDS, positions):
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, int(goal_pos)
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"[ID:{dxl_id}] Goal Position -> {goal_pos}")

    def destroy_node(self):
        # 종료 시 Torque Disable
        for dxl_id in DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



# # Initialize PortHandler instance
# # Set the port path
# # Get methods and members of PortHandlerLinux or PortHandlerWindows
# portHandler = PortHandler(DEVICENAME)

# # Initialize PacketHandler instance
# # Set the protocol version
# # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
# packetHandler = PacketHandler(PROTOCOL_VERSION)

# # Initialize GroupBulkRead instace for Present Position
# groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# # Open port
# if portHandler.openPort():
#     print("Succeeded to open the port")
# else:
#     print("Failed to open the port")
#     print("Press any key to terminate...")
#     getch()
#     quit()


# # Set port baudrate
# if portHandler.setBaudRate(BAUDRATE):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")
#     print("Press any key to terminate...")
#     getch()
#     quit()

# for dxl_id in DXL_IDS:
#     # Enable Dynamixel Torque
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         print("Dynamixel#%d has been successfully connected" % dxl_id)

#     # Add parameter storage for Dynamixel#1 present position
#     dxl_addparam_result = groupBulkRead.addParam(dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
#     if dxl_addparam_result != True:
#         print("[ID:%03d] groupBulkRead addparam failed" % dxl_id)
#         quit()

# while 1:
#     print("Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break

#     for dxl_id in DXL_IDS:
#         # Write Dynamixel#1 goal position
#         dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
#         if dxl_comm_result != COMM_SUCCESS:
#             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#         elif dxl_error != 0:
#             print("%s" % packetHandler.getRxPacketError(dxl_error))

#     while 1:
#         all_arrived = True

#         for dxl_id in DXL_IDS:
#             # (present, comm_result, error) 를 반환
#             dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
#                 portHandler, dxl_id, ADDR_MX_PRESENT_POSITION
#             )

#             if dxl_comm_result != COMM_SUCCESS:
#                 print(packetHandler.getTxRxResult(dxl_comm_result))
#                 all_arrived = False
#                 continue

#             if dxl_error != 0:
#                 print(packetHandler.getRxPacketError(dxl_error))
#                 all_arrived = False
#                 continue

#             print(f"[ID:{dxl_id:03d}] GoalPos:{dxl_goal_position[index]:03d}  PresPos:{dxl_present_position:03d}")

#             if abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
#                 all_arrived = False

#         if all_arrived:
#             break


#         # # Bulkread present position and moving status
#         # dxl_comm_result = groupBulkRead.txRxPacket()
#         # if dxl_comm_result != COMM_SUCCESS:
#         #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

#         # for dxl_id in DXL_IDS:
#         #     # Check if groupbulkread data of Dynamixel#1 is available
#         #     dxl_getdata_result = groupBulkRead.isAvailable(dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
#         #     if dxl_getdata_result != True:
#         #         print("[ID:%03d] groupBulkRead getdata failed" % dxl_id)
#         #         quit()
#         #     # Get Dynamixel#1 present position value
#         #     dxl_present_position = groupBulkRead.getData(dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
#         #     if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
#         #         break

#         # dxl2_moving_value = groupBulkRead.getData(DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING)

#         # print("[ID:%03d] Present Position : %d \t [ID:%03d] Is Moving: %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_moving_value))

        

#     # Change goal position
#     if index == 0:
#         index = 1
#     else:
#         index = 0

# # Clear bulkread parameter storage
# groupBulkRead.clearParam()

# for dxl_id in DXL_IDS:
#     # Disable Dynamixel#1 Torque
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Close port
# portHandler.closePort()