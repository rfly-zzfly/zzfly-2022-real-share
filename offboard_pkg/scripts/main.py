#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2022-10-18 16:08:35
# @Author  : BrightSoul (653538096@qq.com)

import rospy
import rospkg


import os
import time

from Px4Controller import Px4Controller


from task_base import TaskBase






if __name__=="__main__":
    rospy.init_node("offboard_node")
    
    
    px4_control = Px4Controller()



    bs_task = TaskBase(px4_control)

    print("Px4 Controller Initialized!")

    # 开始推位置
    px4_control.start_pub()
    
    # 先检查是否能进入offboard
    # 切换至offboard模式并且
    # ch7为高则进入任务模式
    while not px4_control.task_ready:
        time.sleep(0.1)
    print("==============================")
    print("Begin Task!")
    

    # 先起飞然后再执行其他的任务
    # 起飞函数会先解锁然后切换至offboard模式
    px4_control.takeoff(vz=1.25,h=2.0)



    # 执行一些奇奇怪怪的任务 
    # ==============================================
    
    # 测试rpy方向
    # bs_task.task_print_rpy()
 
    # 测试速度接口
    bs_task.task_test_vel()
    # bs_task.task_test_velENU()
    
    # 测试位置接口
    # 别用位置接口，会变的不幸
    # bs_task.task_test_pos()
    
    # 测试速度极限
    # bs_task.task_test_vel_fast()


    # ==============================================


    # 降落
    px4_control.moveByVelocityYawrateBodyFrame(0, 0, -0.5, 0.)