#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np

from utils import saturation,minAngleDiff,constrain_rad


XYR_CTL = 10



class visual_servo_bs:
    def __init__(self):
        # 旋转的p控制
        self.k_yaw = 0.55

        # 相机参数
        self.cx = 312.11
        self.cy = 240.94
        self.fx = 610.96
        self.fy = 610.08



        # 动力学参数
        self.R_be = np.identity(3)
        self.yaw_rad = 0

        # 参数配置
        self.reset_config()

    def reset(self):
        self.reset_config()
        self.reset_state()

    def reset_config(self):
        # 会根据配置文件更新的参数
        self.vx = 1.0
     
    
    def reset_state(self):
        self.vs_state = XYR_CTL
        self.next_task_flag = False

        self.loss_count = 0
        self.through_count = 0
        self.frd_count = 0
        

        self.last_F = 0
        self.last_R = 0
        self.last_v_cmd = np.zeros(3)
        self.last_yaw_rate_cmd = 0
        
        self.state_time = time.time()
    
    def switch_to(self,r):
        result = False
        if self.xyr_R_min < r < self.xyr_R_max:
            self.vs_start_time = time.time()
            self.state_time = time.time()
            result = True

        return result


    def update_kinematics(self,R_be,yaw_rad):
        self.R_be = R_be
        self.yaw_rad = yaw_rad

    def visual_servo_real_update(self,circle_xyr,circle_FRD,tgt_yaw):
        # TODO
        yaw_rate_cmd = self.k_yaw * constrain_rad(tgt_yaw - self.yaw_rad)
        v_cmd = np.zeros(3)
        return v_cmd, yaw_rate_cmd


    

