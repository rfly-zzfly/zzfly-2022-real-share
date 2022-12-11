#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def color_red_proc(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    chH,chS,chV = cv2.split(hsv_img)
    chH[chH < 60] += 180
    hsv_img = cv2.merge([chH,chS,chV])

    # 上海红
    low_L = [160, 115, 0]
    up_L  = [179, 255, 255]
    img_bin = cv2.inRange(hsv_img, np.array(low_L), np.array(up_L))
    return img_bin

def get_maxArea_id(contours):
    areaL = []
    for contour in contours:
        area = cv2.contourArea(contour)
        areaL.append(area)

    max_id = np.argmax(areaL)
    return max_id

# 计算mask内的平均深度
def depth_proc(img,mask,dis_bias = 1.25):
    img_res =  img.copy()
    hist,bin_edges  = np.histogram(img[mask],bins=20,range=(0,10))
    id_max = np.argmax(hist[1:])+1
    dis_mid = bin_edges[id_max]+0.25
    # print(dis_mid)
    depth_mask1 = (img > dis_mid+dis_bias)
    img_res[depth_mask1] = 0
    if dis_mid > 2:
        depth_mask2 = (img < dis_mid-dis_bias)
        img_res[depth_mask2] = 0
    return img_res


def find_contour_max(contourL, hierarchyL):
    hierarchyL = np.squeeze(hierarchyL,0)
    if len(contourL) == 0:
        return None,None


    innerL = []
    outerL = []

    # 寻找最大的内边缘，或者最大的外边缘
    for i,[contour,hierarchy] in enumerate(zip(contourL,hierarchyL)):
        area = cv2.contourArea(contour)
        if area < 900:
            continue

        is_inner = (hierarchy[-1] != -1)
        res = [area,i]
        if is_inner:
            innerL.append(res)
        else:
            outerL.append(res)

    if len(innerL) > 0:
        innerL.sort(key=lambda x:x[0], reverse=True)
        area_max,idx_max = innerL[0]
    elif len(outerL) > 0:
        outerL.sort(key=lambda x:x[0], reverse=True)
        area_max,idx_max = outerL[0]
    else:
        area_max,idx_max = None,None
    
    return area_max,idx_max

def ring_normal_dect(color_img,depth_img):
    # 红色处理
    color_bin = color_red_proc(color_img)
    color_mask = (color_bin > 0)
    
    # 深度图像预处理
    img_depth_mf = cv2.medianBlur(depth_img, 5)
    depth_beyond_mask = (img_depth_mf > 10000)
    img_depth_float = img_depth_mf/1000
    img_depth_float[depth_beyond_mask] = 0

    # 在颜色图像的基础上处理深度图
    img_depth_res = depth_proc(img_depth_float,color_mask)
    depth_mask = (img_depth_res > 0)

    # 计算一下两者的IOU
    img_mask = np.bitwise_and(depth_mask,color_mask)
    A1 = np.sum(color_mask)
    A3 = np.sum(img_mask)
    IOU_color = (A3)/(A1)
    if IOU_color < 0.3:
        img_mask = color_mask

    # 深度计算
    real_dis = np.average(img_depth_float[img_mask])

    img_bin = np.zeros_like(depth_img,dtype=np.uint8)
    img_bin[img_mask] = 1


    # 形态学操作
    kernel_size_unit = np.clip(np.sqrt(np.sum(img_mask))/480*1.5, 0.5, 2)
    kernel = np.ones(shape=[int(kernel_size_unit*3), int(kernel_size_unit*4)], dtype=np.uint8)
    open_result = cv2.morphologyEx(img_bin, op=cv2.MORPH_OPEN, kernel=kernel, iterations=2)
    kernel = np.ones(shape=[int(kernel_size_unit*3*2), int(kernel_size_unit*4*2)], dtype=np.uint8)
    close_result = cv2.morphologyEx(open_result, op=cv2.MORPH_CLOSE, kernel=kernel, iterations=4)


    mm = cv2.moments(open_result)
    if mm['m00'] < 400:
        return [-1]*4

    yxL = np.where(close_result == 1)
    cx = mm['m10'] / mm['m00']
    cy = mm['m01'] / mm['m00']
    radiusL = [ (cx-x)**2 + (cy-y)**2 for y,x in zip(yxL[0],yxL[1])]
    r = np.sqrt(np.average(radiusL))
    
    img_res = color_img
    cv2.circle(img_res, (int(cx), int(cy)), 3, (255, 255, 255), -1)
    cv2.circle(img_res, (int(cx), int(cy)), int(r), (255, 255, 255), 2)

    return cx,cy,r,real_dis



def ring_special_dect(color_img,depth_img):
    color_bin = color_red_proc(color_img)
    color_mask = (color_bin > 0)

    # 深度图像预处理
    img_depth_mf = cv2.medianBlur(depth_img, 5)
    depth_beyond_mask = (img_depth_mf > 10000)
    img_depth_float = img_depth_mf/1000
    img_depth_float[depth_beyond_mask] = 0

    img_depth_res = depth_proc(img_depth_float,color_mask)
    depth_mask = (img_depth_res > 0)

    img_mask = np.bitwise_and(depth_mask,color_mask)
    real_dis = np.average(img_depth_res[img_mask])

    # color_img上面寻找最大内边缘
    # 形态学操作
    kernel = np.ones(shape=[int(2), int(2)], dtype=np.uint8)
    open_result = cv2.morphologyEx(color_bin, op=cv2.MORPH_OPEN, kernel=kernel, iterations=2)
    kernel = np.ones(shape=[int(2*2), int(2*2)], dtype=np.uint8)
    close_result = cv2.morphologyEx(open_result, op=cv2.MORPH_CLOSE, kernel=kernel, iterations=10)

    contourL, hierarchyL = cv2.findContours(close_result, cv2.RETR_CCOMP , cv2.CHAIN_APPROX_SIMPLE)
    area_max,idx_max = find_contour_max(contourL, hierarchyL)

    if None in [area_max,idx_max] or area_max < 400:
        return [-1]*4
    
    mm = cv2.moments(contourL[idx_max])
    cx = mm['m10'] / mm['m00']
    cy = mm['m01'] / mm['m00']

    r = np.sqrt(area_max)
    img_res = color_img
    cv2.circle(img_res, (int(cx), int(cy)), 3, (255, 255, 255), -1)
    cv2.drawContours(img_res, contourL, idx_max, (0, 255, 0), 2)
    return cx,cy,r,real_dis



def ring_rect_proc(color_img,depth_img,method="ring_normal"):
    # ring_normal
    # ring_special
    
    if method == "ring_normal":
        cx,cy,r,real_dis = ring_normal_dect(color_img,depth_img)
        return cx,cy,r,real_dis
    elif method == "ring_special":
        cx,cy,r,real_dis = ring_special_dect(color_img,depth_img)
        return cx,cy,r,real_dis     
    else:
        return [-1]*4
