#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# 饱和
def saturation(a, maxv):
    n = np.linalg.norm(a) if type(a) is np.ndarray else a
    if n > maxv:
        return a / n * maxv
    else:
        return a


# 角度差计算
def minAngleDiff( a, b):
    diff = a - b
    if diff < 0:
        diff += 2*np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2*np.pi

# 限制角度在-pi~pi
def constrain_rad(ori_rad):
    delta_rad = 0
    if ori_rad > np.pi:
        delta_rad = -2*np.pi
    if ori_rad < -np.pi:
        delta_rad = 2*np.pi
    return ori_rad + delta_rad



def eulerAngleToMatrix(rpy):
    # 实际上全都用FLU系的
    # Z-Y-X
    R_x = np.array([
            [1,                  0,                   0],
            [0, np.cos(rpy[0]), -np.sin(rpy[0])],
            [0,  np.sin(rpy[0]), np.cos(rpy[0]) ]
        ] ,dtype=float
    )
                        
    R_y = np.array([
            [np.cos(rpy[1]), 0, np.sin(rpy[1])],
            [0,                  1,                  0],
            [-np.sin(rpy[1]), 0, np.cos(rpy[1])]
        ],dtype=float
    )
                
    R_z = np.array([
            [np.cos(rpy[2]), -np.sin(rpy[2]), 0],
            [np.sin(rpy[2]), np.cos(rpy[2]),  0],
            [0,                  0,                   1]
        ],dtype=float
    )
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R


if __name__=="__main__":
    R = eulerAngleToMatrix([np.deg2rad(-0.69),np.deg2rad(1.84),np.deg2rad(3.14)])
    print(R)
    print(R.T)
    print(saturation(12,10))
    print(saturation(np.array([1,2,3]),10))
