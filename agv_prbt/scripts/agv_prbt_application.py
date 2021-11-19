#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import random 
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, Gripper, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from modbus_wrapper_client import ModbusWrapperClient 
from table_cap_and_analyze import table_cap_and_analyze
from agv_task_deploy_wrapper import agv_task_deply_wrapper


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"

# 位置常量（因涉及到实际机械位置，因此不要修改）
HOME_POSE = [math.radians(-90), math.radians(0), math.radians(-90), 0, math.radians(-90), math.radians(135)]    # 起始关节角度
CAPTURE_POSE = [math.radians(90), math.radians(0), math.radians(-90), 0, math.radians(-90), math.radians(135)]  # 相机拍照角度
GRIPPER_ORIENTATION = from_euler(0, math.radians(180),  math.radians(180))         # 夹爪方向
AGV_PLATE_FULL_POSE = Pose(position=Point(-0.126, -0.36, 0.14), orientation=GRIPPER_ORIENTATION)    # AGV满料盘位置
AGV_PLATE_EMPTY_POSE = Pose(position=Point(0.126, -0.36, 0.14), orientation=GRIPPER_ORIENTATION)    # AGV空料盘位置
SAFETY_HEIGHT =  0.32
FEED_TABLE_BOX_FULL_OFFSET_X = 0.106
FEED_TABLE_BOX_EMPTY_OFFSET_X = 0.106 + 0.215
FEED_TABLE_PEN_FULL_OFFSET_X = 0.106 - 0.43
FEED_TABLE_PEN_EMPTY_OFFSET_X = 0.106 - 0.215
FEED_TABLE_OFFSET_Y = 0.113
FEED_TABLE_PnP_OFFSET_Z = 0.072
SMF_TABLE_BOX_OFFSET_X = 0.263
SMF_TABLE_PEN_OFFSET_X = 0.263 - 0.215
SMF_TABLE_OFFSET_Y = 0.144
CAMERA_GRIPPER_OFFSET = 0.066
PLATE_HEIGHT = 0.025


# 照片存放路径
table_pic_file_path = "/home/pilz/Pictures/agv_prbt/table.png" 
table_calibrated_pic_file_path = "/home/pilz/Pictures/agv_prbt/table_calibrated.png"


# 速度常量
PTP_SCALE = 0.3         # 点到点移动速度比例
LIN_SCALE = 0.1         # 直线移动速度
PnP_SCALE = 0.03         # 拾取与放置速度

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
pen_plate_pick_number = 0
pen_plate_place_number = 0
box_plate_pick_number = 0
box_plate_place_number = 0

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
    global pen_plate_pick_number
    global pen_plate_place_number
    global box_plate_pick_number
    global box_plate_place_number
    external_start = data.holding_registers.data[1]
    external_stop = data.holding_registers.data[2]
    external_reset = data.holding_registers.data[3]
    robot_run_permission = data.holding_registers.data[4]
    pen_plate_pick_number = data.holding_registers.data[10]
    pen_plate_place_number = data.holding_registers.data[11]
    box_plate_pick_number = data.holding_registers.data[12]
    box_plate_place_number = data.holding_registers.data[13]


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
    rospy.sleep(3)


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
    while r.get_current_pose().position.z < SAFETY_HEIGHT:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.05)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    if r.get_current_pose().position.y > 0:
        r.move(Ptp(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
    r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    modclient.client.write_register(40023, 1)
    
    robotCell_box_missing = modclient.client.read_holding_registers(40006, 1).registers[0]
    if robotCell_box_missing:
        if 0 < box_plate_pick_number <= 5:
            # 机器人从上料台拾取满料盘
            r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            _, _, _, _, table_angle = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            r.move(Lin(goal=Pose(orientation=from_euler(0, 0, math.radians(-table_angle))), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            feed_table_x, feed_table_y, _, _, _ = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            r.move(Lin(goal=Pose(position=Point(-feed_table_x - FEED_TABLE_BOX_FULL_OFFSET_X, feed_table_y + FEED_TABLE_OFFSET_Y + CAMERA_GRIPPER_OFFSET, 0.15)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            pick_start_pose = r.get_current_pose()
            pick_height = FEED_TABLE_PnP_OFFSET_Z + (5 - box_plate_pick_number) * 0.025
            r.move(Gripper(goal=0.030))
            r.move(Lin(goal=Pose(position=Point(0, 0, pick_height)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=pick_start_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            
            # 将满料盘放到小车
            r.move(Lin(goal=AGV_PLATE_FULL_POSE, 
                    reference_frame="prbt_base_link", 
                    vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Lin(goal=Pose(position=Point(0, 0, 0.05)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            # r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=AGV_PLATE_FULL_POSE, 
                    reference_frame="prbt_base_link", 
                    vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))

            # AGV去工作站位置
            agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 5)

            # 机器人从工作站拾取空料盘
            r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            _, _, _, _, table_angle = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            r.move(Lin(goal=Pose(orientation=from_euler(0, 0, math.radians(-table_angle))), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            _, _, smf_table_x, smf_table_y, _ = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            r.move(Lin(goal=Pose(position=Point(-smf_table_x - SMF_TABLE_BOX_OFFSET_X, smf_table_y + SMF_TABLE_OFFSET_Y + CAMERA_GRIPPER_OFFSET, 0.12)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            pick_start_pose = r.get_current_pose()
            pick_height = 0.05
        #     r.move(Gripper(goal=0.030))
            r.move(Lin(goal=Pose(position=Point(0, 0, pick_height)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
        #     r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=pick_start_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            
            # 将空料盘放到小车
            r.move(Ptp(goal=AGV_PLATE_EMPTY_POSE, 
                        reference_frame="prbt_base_link",
                        vel_scale=PTP_SCALE, acc_scale=0.2))
            r.move(Lin(goal=Pose(position=Point(0, 0, 0.05)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
        #     r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=AGV_PLATE_EMPTY_POSE, 
                        reference_frame="prbt_base_link",
                        vel_scale=PnP_SCALE, acc_scale=0.1))

            # 从小车拾取满料盘放到工作站
            r.move(Lin(goal=AGV_PLATE_FULL_POSE, 
                        reference_frame="prbt_base_link",
                        vel_scale=PnP_SCALE, acc_scale=0.1))
        #     r.move(Gripper(goal=0.030))
            r.move(Lin(goal=Pose(position=Point(0, 0, 0.05)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
        #     r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=AGV_PLATE_FULL_POSE, 
                        reference_frame="prbt_base_link", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            r.move(Lin(goal=pick_start_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Lin(goal=Pose(position=Point(0, 0, pick_height)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
        #     r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=pick_start_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))

        #     # AGV去上料台位置
            agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 1)

        if 0 <= box_plate_place_number < 5:
            r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            _, _, _, _, table_angle = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            r.move(Lin(goal=Pose(orientation=from_euler(0, 0, math.radians(-table_angle))), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            feed_table_x, feed_table_y, _, _, _ = table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path)
            place_middle_waypoint = r.get_current_pose()
            r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.2))
            r.move(Ptp(goal=AGV_PLATE_EMPTY_POSE, 
                        reference_frame="prbt_base_link", 
                        vel_scale=PTP_SCALE, acc_scale=0.2))
            # r.move(Gripper(goal=0.030))
            r.move(Lin(goal=Pose(position=Point(0, 0, 0.05)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            # r.move(Gripper(goal=0.027))
            rospy.sleep(.5)
            r.move(Lin(goal=AGV_PLATE_EMPTY_POSE, 
                        reference_frame="prbt_base_link", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
            r.move(Lin(goal=place_middle_waypoint, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.2))
            r.move(Lin(goal=Pose(position=Point(-feed_table_x - FEED_TABLE_BOX_EMPTY_OFFSET_X, feed_table_y + FEED_TABLE_OFFSET_Y + CAMERA_GRIPPER_OFFSET, 0.15)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=LIN_SCALE, acc_scale=0.1))
            place_start_pose = r.get_current_pose()
            place_height = FEED_TABLE_PnP_OFFSET_Z + (5 - box_plate_place_number) * 0.025 - 0.023
            r.move(Lin(goal=Pose(position=Point(0, 0, place_height)), 
                        reference_frame="prbt_tcp", 
                        vel_scale=PnP_SCALE, acc_scale=0.1))
            # r.move(Gripper(goal=0.030))
            rospy.sleep(.5)
            r.move(Lin(goal=place_start_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
    
    rospy.spin()
