#!/usr/bin/env python
# -*- coding: utf-8 -*-

import io
import os
import time
import numpy as np

import json5


from visual_servo_bs import visual_servo_bs
from utils import constrain_rad

file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")
task_json = "{}/config.json".format(file_pwd)




with io.open(task_json,"r",encoding="utf-8") as fp:
    dataD = json5.load(fp)

wpL = dataD["wpL"]


TASK_ACCESS     = 0
TASK_VISUAL     = 1
TASK_THROUGH    = 2







class TaskCross:
    def __init__(self,px4_control,img_proc,debuger):
        self.px4_control = px4_control
        self.img_proc = img_proc
        self.debuger = debuger

        self.visual_servo = visual_servo_bs()




    
    def task_two_circle(self):
        px4_control = self.px4_control
        img_proc = self.img_proc
        visual_servo = self.visual_servo
        debuger = self.debuger



        for wp in wpL:
            # wp参数设置
            # —————————————————————————————
            task_name = wp["name"]
            method = wp["method"]
            tgt_yaw = np.deg2rad(wp["tgt_yaw"])
            tgt_vel = wp["tgt_vel"]
            # —————————————————————————————


            # 状态重置
            # —————————————————————————————
            task_state = TASK_ACCESS
            img_proc.reset()
            visual_servo.reset()
            # —————————————————————————————



            if "vs_dict" in wp:
                visual_servo.__dict__.update(wp["vs_dict"])

            
            while True:
                yaw = px4_control.mav_yaw
                R_be = px4_control.mav_Rbe

                img_proc.update_ring_info(method=method)



                if task_state == TASK_ACCESS:
                    v_cmd = [tgt_vel,0,0]
                # 视觉伺服控制
                elif task_state == TASK_VISUAL:
                    visual_servo.update_kinematics(R_be,yaw)
                    v_cmd, _ = visual_servo.visual_servo_real_update(img_proc.circle_xyr,img_proc.circle_FRD,tgt_yaw)
                



                if task_state == TASK_ACCESS:
                    # 开始视觉伺服
                    if visual_servo.switch_to(img_proc.circle_xyr[2]):
                        task_state = TASK_VISUAL
                elif task_state == TASK_VISUAL:
                    # 结束视觉伺服
                    if visual_servo.next_task_flag:
                        print("visual_servo done")
                        break
                
                
                debug_info_dict = {
                    "task_state": "n:{} s:{} vs:{}\nxyr:{:.2f} {:.2f} {:.2f}\nfrd:{:.2f} {:.2f} {:.2f}".format(
                        task_name,task_state,visual_servo.vs_state,
                        img_proc.circle_xyr[0],img_proc.circle_xyr[1],img_proc.circle_xyr[2],
                        img_proc.circle_FRD[0],img_proc.circle_FRD[1],img_proc.circle_FRD[2]
                    ),
                    "cmdv_flu"  : "v_flu:{:.2f} {:.2f} {:.2f}".format(v_cmd[0],v_cmd[1],v_cmd[2]),
                }



                debuger.update(debug_info_dict)

                # 发送指令
                px4_control.moveByVelocityYawrateBodyFrame(v_cmd[0],v_cmd[1],v_cmd[2],0)
                time.sleep(0.01)
            print("{} done".format(task_name))
