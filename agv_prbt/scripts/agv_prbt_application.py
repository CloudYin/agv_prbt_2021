#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from take_picture import take_picutre, undistort_pic
from get_marker_pose import get_blue_marker_pose
from std_msgs.msg import Int32MultiArray as HoldingRegister


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"

# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [math.radians(-90), math.radians(30), math.radians(-120), 0, math.radians(-30), math.radians(-45)]    # 起始关节角度
GRIPPER_ORIENTATION = from_euler(0, math.radians(180),  math.radians(45))         # 夹爪方向
SAFETY_HEIGHT =  0.12


# 照片存放路径
table_pic_file_path = "/home/pilz/Pictures/agv_prbt/table.png" 
table_calibrated_pic_file_path = "/home/pilz/Pictures/agv_prbt/table_calibrated.png"


# 速度常量
LIN_SCALE = 0.1         # 直线移动速度
PnP_SCALE = 0.1         # 拾取与放置速度比例

# 初始化变量
agv_at_robotCell = False
agv_prbt_exchanging_box_plate = False
agv_prbt_exchanging_pen_plate = False
agv_prbt_at_home = False
robotCell_box_missing = False
robotCell_pen_missing = False
robotCell_prbt_picking_box = False
robotCell_prbt_picking_pen = False
robotCell_prbt_handing_out_box = False
robotCell_prbt_handing_out_pen = False
robotCell_prbt_at_home = False

# 发送至PSS Modbus寄存器地址
pss_modbus_write_dic = {
    }


def pss4000_modbus_write(start_idx, values):
    rospy.wait_for_service('/pilz_modbus_client_node/modbus_write')
    try:
        modbus_write_client = rospy.ServiceProxy('/pilz_modbus_client_node/modbus_write', WriteModbusRegister)
        modbus_write_client(ModbusRegisterBlock(start_idx, values))
    except rospy.ServiceException as e:
        print("Modbus write service call failed: %s" %e)


def pss4000_modbus_read_callback(data):
    external_start = data.holding_registers.data[1]
    external_stop = data.holding_registers.data[2]
    external_reset = data.holding_registers.data[3]
    robot_run_permission = data.holding_registers.data[4]

    if not robot_run_permission or external_stop:
        r.pause()
        
    if external_start:
        rospy.sleep(1)
        r.resume()


def pss4000_modbus_read():
    rospy.Subscriber("/pilz_modbus_client_node/modbus_read", ModbusMsgInStamped, pss4000_modbus_read_callback, queue_size=1)


def table_cap_and_analyze():
    """
    拍照并获取上料台相对位置
    """
    global table_x
    global table_y
    global table_angle
    take_picutre(table_pic_file_path)
    undistort_pic(table_pic_file_path, table_calibrated_pic_file_path)
    table_x, table_y, table_angle = get_blue_marker_pose(table_calibrated_pic_file_path)


def modclient_send_data(modclient_publisher):
    global agv_at_robotCell
    global agv_prbt_exchanging_box_plate 
    global agv_prbt_exchanging_pen_plate 
    global agv_prbt_at_home
    modbus_client_output = HoldingRegister()
    modbus_client_output.data = [agv_at_robotCell, agv_prbt_exchanging_box_plate, agv_prbt_exchanging_pen_plate, agv_prbt_at_home]
    modclient_publisher.publish(modbus_client_output)


def showUpdatedRegisters(msg):
    global robotCell_box_missing
    global robotCell_pen_missing
    global robotCell_prbt_picking_box
    global robotCell_prbt_picking_pen
    global robotCell_prbt_handing_out_box
    global robotCell_prbt_handing_out_pen
    global robotCell_prbt_at_home
    robotCell_box_missing = msg.data[0]
    robotCell_pen_missing = msg.data[1]
    robotCell_prbt_picking_box = msg.data[2]
    robotCell_prbt_picking_pen = msg.data[3]
    robotCell_prbt_handing_out_box = msg.data[4]
    robotCell_prbt_handing_out_box = msg.data[5]
    robotCell_prbt_at_home = msg.data[6]

# 主程序
def start_program():
    global table_x
    global table_y
    global table_angle
    global agv_at_robotCell
    global agv_prbt_exchanging_box_plate 
    global agv_prbt_exchanging_pen_plate 
    global agv_prbt_at_home
    global robotCell_box_missing
    global robotCell_pen_missing
    global robotCell_prbt_picking_box
    global robotCell_prbt_picking_pen
    global robotCell_prbt_handing_out_box
    global robotCell_prbt_handing_out_pen
    global robotCell_prbt_at_home


    rospy.loginfo("Program started")  # log
    modbus_client_pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=500)
    modclient_send_data(modbus_client_pub)

    """
    工艺安全设置
    获取当前机器人位置
    若当前Z位置小于安全高度，则反向提升50mm
    """
    current_pose = r.get_current_pose()
    if current_pose.position.z < SAFETY_HEIGHT:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.05)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    r.move(Ptp(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
    agv_prbt_at_home = True
    modclient_send_data(modbus_client_pub)
    

if __name__ == "__main__":
    # 创建节点
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # 创建化机器人实例

    # modbus客户端接收数据
    modbus_client_sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=500)    

    # 启动程序
    start_program()

    rospy.spin()
