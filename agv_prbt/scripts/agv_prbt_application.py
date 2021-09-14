#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Lin, Robot, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from take_picture import take_picutre, undistort_pic
from get_marker_pose import get_blue_marker_pose


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"

# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [math.radians(0), 0, math.radians(-90), 0, math.radians(-90), math.radians(0)]    # 起始关节角度
GRIPPER_ORIENTATION = from_euler(0, math.radians(180),  math.radians(45))         # 夹爪方向
SAFETY_HEIGHT =  0.12


# 照片存放路径
feed_table_pic_file_path = "/home/pilz/Pictures/agv_prbt/feed_table.png" 
feed_table_calibrated_pic_file_path = "/home/pilz/Pictures/agv_prbt/feed_table_calibrated.png"
robot_table_pic_file_path = "/home/pilz/Pictures/agv_prbt/robot_table.png" 
robot_table_calibrated_pic_file_path = "/home/pilz/Pictures/agv_prbt/robot_table_calibrated.png"

# 速度常量
LIN_SCALE = 0.1         # 直线移动速度
PnP_SCALE = 0.1         # 拾取与放置速度比例

# 初始化变量
feed_table_x, feed_table_y, feed_table_angle = 0
robot_table_x, robot_table_y, robot_table_angle = 0

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
    
    robot_run_permission = data.holding_registers.data[4]
    external_start = data.holding_registers.data[25]
    external_stop = data.holding_registers.data[26]
    external_reset = data.holding_registers.data[27]

    if not robot_run_permission or external_stop:
        r.pause()
        
    if external_start:
        rospy.sleep(1)
        r.resume()


def pss4000_modbus_read():
    rospy.Subscriber("/pilz_modbus_client_node/modbus_read", ModbusMsgInStamped, pss4000_modbus_read_callback, queue_size=1)


def feed_table_cap_and_analyze():
    """
    拍照并获取上料台相对位置
    """
    global feed_table_x
    global feed_table_y
    global feed_table_angle
    take_picutre(feed_table_pic_file_path)
    undistort_pic(feed_table_pic_file_path, feed_table_calibrated_pic_file_path)
    feed_table_x, feed_table_y, feed_table_angle = get_blue_marker_pose(feed_table_calibrated_pic_file_path)


def robot_table_cap_and_analyze():
    """
    拍照并获取机器人台相对位置
    """
    global robot_table_x
    global robot_table_y
    global robot_table_angle
    take_picutre(robot_table_pic_file_path)
    undistort_pic(robot_table_pic_file_path, robot_table_calibrated_pic_file_path)
    robot_table_x, robot_table_y, robot_table_angle = get_blue_marker_pose(robot_table_calibrated_pic_file_path)


def init_modbus():
    """
    初始化部分通讯状态
    """    


# 主程序
def start_program():
    global feed_table_x
    global feed_table_y
    global feed_table_angle
    global robot_table_x
    global robot_table_y
    global robot_table_angle

    rospy.loginfo("Program started")  # log

    """
    工艺安全设置
    获取当前机器人位置
    若当前Z位置小于安全高度，则反向提升50mm
    """
    current_pose = r.get_current_pose()
    if current_pose.position.z < SAFETY_HEIGHT:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.05)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
    

if __name__ == "__main__":
    # 创建节点
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # 创建化机器人实例

    # 启动通讯
    init_modbus()

    # 启动程序
    start_program()

    rospy.spin()
