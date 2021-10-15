#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import random 
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from take_picture import take_picutre, undistort_pic
from get_marker_pose import get_blue_marker_pose
from modbus_wrapper_client import ModbusWrapperClient 
from agv_communication import agv_communication_deploy, agv_communication_lookup


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"

# 位置常量（因涉及到实际机械位置，因此不要修改）
HOME_POSE = [math.radians(-90), math.radians(0), math.radians(-90), 0, math.radians(-90), math.radians(135)]    # 起始关节角度
FEED_TABLE_PICTURE_POSE = [math.radians(90), math.radians(0), math.radians(-90), 0, math.radians(-90), math.radians(135)]    # 起始关节角度
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
robotCell_prbt_picking_box = False
robotCell_prbt_picking_pen = False
robotCell_prbt_handing_out_box = False
robotCell_prbt_handing_out_pen = False
robotCell_prbt_at_home = False
pen_plate_pick_height = 0
pen_plate_place_height = 0
box_plate_pick_height = 0
box_plate_place_height = 0

# 发送至PSS Modbus寄存器地址
pss_modbus_write_dic = {
    'agv_ros_program_run': 1000
    }


def pss_modbus_write(start_idx, values):
    rospy.wait_for_service('/pilz_modbus_client_node/modbus_write')
    try:
        modbus_write_client = rospy.ServiceProxy('/pilz_modbus_client_node/modbus_write', WriteModbusRegister)
        modbus_write_client(ModbusRegisterBlock(start_idx, values))
    except rospy.ServiceException as e:
        print("Modbus write service call failed: %s" %e)


def pss_modbus_read_callback(data):
    global pen_plate_pick_height
    global pen_plate_place_height
    global box_plate_pick_height
    global box_plate_place_height
    external_start = data.holding_registers.data[1]
    external_stop = data.holding_registers.data[2]
    external_reset = data.holding_registers.data[3]
    robot_run_permission = data.holding_registers.data[4]
    pen_plate_pick_height = data.holding_registers.data[10]
    pen_plate_place_height = data.holding_registers.data[11]
    box_plate_pick_height = data.holding_registers.data[12]
    box_plate_place_height = data.holding_registers.data[13]


    # if not robot_run_permission or external_stop:
    if external_stop:
        r.pause()
        
    if external_start:
        rospy.sleep(1)
        r.resume()


def pss_modbus_read():
    rospy.Subscriber("/pilz_modbus_client_node/modbus_read", ModbusMsgInStamped, pss_modbus_read_callback, queue_size=1)


def pymodbus_client():
    NUM_REGISTERS = 20
    ADDRESS_READ_START = 40000
    ADDRESS_WRITE_START = 40020
    host = "192.168.251.112"
    port = 1234
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False)
    modclient.setReadingRegisters(ADDRESS_READ_START,NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    while not (modclient.client.connect() or rospy.is_shutdown()):
        rospy.logwarn("Could not get a modbus connection to the host %s, reconnecting...", host)
        modclient.closeConnection()
        rospy.sleep(3)
        modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False)
        modclient.setReadingRegisters(ADDRESS_READ_START,NUM_REGISTERS)
        modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    modclient.startListening()
    return modclient

def init_modbus():
    pss_modbus_read()
    pss_modbus_write(pss_modbus_write_dic['agv_ros_program_run'], [0])
    rospy.sleep(1)


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
    
    

if __name__ == "__main__":
    # 创建节点
    rospy.init_node('robot_program_node')
    
    # 初始化
    r = Robot(REQUIRED_API_VERSION)  # 创建化机器人实例

    agv_task_id_deploy = random.randint(1, 100)
    agv_task_id_lookup = agv_task_id_deploy

    init_modbus()
    modclient = pymodbus_client()

    # 启动程序
    rospy.loginfo("Program started")  # log
    pss_modbus_write(pss_modbus_write_dic['agv_ros_program_run'], [1])

    """
    工艺安全设置
    获取当前机器人位置
    若当前Z位置小于安全高度，则反向提升50mm
    """
    current_pose = r.get_current_pose()
    if current_pose.position.z < SAFETY_HEIGHT:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.05)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    r.move(Ptp(goal=HOME_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
    modclient.client.write_register(40023, 1)

    # agv运行至1号起始位（充电位）
    # agv_task_id_current, agv_task_deploy_result = agv_communication_deploy(agv_task_id_deploy, 1)
    # while agv_task_deploy_result != 1:
    #     print("Task deploy failed.")
    #     rospy.sleep(1)
    #     agv_task_id_current, agv_task_lookup_result = agv_communication_lookup(agv_task_id_lookup, 1)
    # agv_task_id_deploy = agv_task_id_current + 1
    # agv_task_id_lookup = agv_task_id_current
    
    robotCell_box_missing = modclient.client.read_holding_registers(40006, 1).registers[0]
    if robotCell_box_missing:
        # agv_task_id_current, agv_task_lookup_result = agv_communication_lookup(agv_task_id_lookup, 1)
        # while agv_task_lookup_result != 3:
        #     print("Task not finished.")
        #     rospy.sleep(1)
        #     agv_task_id_current, agv_task_lookup_result = agv_communication_lookup(agv_task_id_lookup, 1)
        r.move(Ptp(goal=FEED_TABLE_PICTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
        r.get_current_pose()


    rospy.spin()
    modclient.stopListening()
