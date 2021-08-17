#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, from_euler, Sequence
from agv_communication import agv_communication_deploy, agv_communication_lookup


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"

# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [math.radians(-90), 0, math.radians(-90), 0, math.radians(-90), math.radians(-45)]    # 起始关节角度
TEST_POSE = [math.radians(-90), 0, math.radians(-90), 0, math.radians(0), math.radians(-45)]    # Test Pose


# 速度常量
LIN_SCALE = 0.2         # 直线移动速度
PTP_SCALE = 0.2         # PTP Scale

task_id_deploy = 1
task_id_lookup = 1

# 主程序
def start_program():
    global task_id_deploy, task_id_lookup
    rospy.loginfo("Program started")  # log

    r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    task_id_current, task_deploy_result = agv_communication_deploy(task_id_deploy, 2)
    print("Task id current: " + str(task_id_current))
    print("Task id deploy result: " + str(task_deploy_result))
    while task_deploy_result != 1:
        print("Task deploy failed.")
        rospy.sleep(1)
        task_id_current, task_lookup_result = agv_communication_lookup(task_id_lookup, 2)
    task_id_deploy = task_id_current + 1
    task_id_lookup = task_id_current
    print("Task id next deploy: " + str(task_id_deploy))

    rospy.sleep(10)

    print("Task id lookup: " + str(task_id_lookup))
    task_id_current, task_lookup_result = agv_communication_lookup(task_id_lookup, 2)
    print("Task id lookup result: " + str(task_lookup_result))
    while task_lookup_result != 3:
        print("Task not finished.")
        rospy.sleep(1)
        task_id_current, task_lookup_result = agv_communication_lookup(task_id_lookup, 2)
    r.move(Ptp(goal=TEST_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    print("Task id current: " + str(task_id_current))
    print("Task lookup result: " + str(task_lookup_result))



if __name__ == "__main__":
    # 创建节点
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # 创建化机器人实例

    # 启动程序
    start_program()

    rospy.spin()
