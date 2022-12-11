#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2022-10-18 16:08:35
# @Author  : BrightSoul (653538096@qq.com)

import rospy
import rospkg


import os
import json
import time

from Px4Controller import Px4Controller
from rgbd_proc import rgbd_img_proc

from task_cross import TaskCross

from bs_debuger import Debuger


# from yolo_detector import YoloDetector

file_pwd = os.path.dirname(os.path.abspath(__file__))


if __name__=="__main__":
    rospy.init_node("offboard_node")

    px4_control = Px4Controller()
    # yolo_detector = YoloDetector("{}/bs_real.yaml".format(file_pwd))
    yolo_detector = None
    img_proc = rgbd_img_proc(yolo_detector=yolo_detector)
    
    task_debuger = Debuger()

    zzfly_task = TaskCross(px4_control,img_proc,task_debuger)

    print("Px4 Controller Initialized!")

    # 开始推位置
    px4_control.start_pub()
    
    # 先检查是否能进入offboard
    # 切换至position模式
    # ch7为高则进入任务模式
    while not px4_control.task_ready:
        time.sleep(0.1)
    print("==============================")
    print("Begin Task!")
    

    # 先起飞然后再执行其他的任务
    # 起飞函数会先解锁然后切换至offboard模式
    px4_control.takeoff(vz=0.5,h=1.5)

    zzfly_task.task_two_circle()
 

    # 降落
    px4_control.moveByVelocityYawrateBodyFrame(0, 0, -0.5, 0.)
    