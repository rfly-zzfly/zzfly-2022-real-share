#!/usr/bin/env python
# -*- coding: utf-8 -*-


import time
import numpy as np



class TaskBase:
    def __init__(self,px4_control):
        self.px4_control = px4_control


    def task_print_rpy(self):
        px4_control = self.px4_control
        while True:
            yaw_offset = np.rad2deg(px4_control.mav_yaw_offset)
            yaw = np.rad2deg(px4_control.mav_yaw)
            roll = np.rad2deg(px4_control.mav_roll)
            pitch = np.rad2deg(px4_control.mav_pitch) 
            print("rpy: {:.2f} {:.2f} {:.2f} {:.2f}".format(roll,pitch,yaw,yaw_offset))
            time.sleep(1)


    def task_test_vel(self):
        '''
        速度指令FLU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByVelocityYawrateBodyFrame(vx=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(vy=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(yaw_rate=np.deg2rad(45))
        time.sleep(2)

        px4_control.moveByVelocityYawrateBodyFrame(vx=-0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(vy=0.5)
        time.sleep(5)
    
    def task_test_velENU(self):
        '''
        速度指令ENU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByVelocityYawrateENU(vx=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(vy=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(yaw_rate=np.deg2rad(45))
        time.sleep(2)

        px4_control.moveByVelocityYawrateENU(vx=-0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(vy=-0.5)
        time.sleep(5)
    

    
    def task_test_pos(self):
        '''
        位置指令ENU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByPosENU(x=2)
        time.sleep(5)
        px4_control.moveByPosENU(y=2)
        time.sleep(5)
        px4_control.moveByPosENU(z=3)
        time.sleep(5)

    
