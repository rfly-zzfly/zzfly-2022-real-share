#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import time
import cv2
import numpy as np

import rospy
import threading


from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError



from ring_rect_proc import ring_rect_proc








img_w,img_h = 640,480
depth_w,depth_h = 640,480
# 深度图算距离，RGB图算坐标
colorK = np.mat(
    [
        [610.96,0,312.11],
        [0,610.08,240.94],
        [0,0,1],
    ],
    dtype=float
)



def bbox2ltrb(cx,cy,w,h):
    l = int(round(cx - (w / 2)))
    r = int(round(cx + (w / 2)))
    t = int(round(cy - (h / 2)))
    b = int(round(cy + (h / 2)))

    l = max(l,0)
    t = max(t,0)
    r = min(r,img_w-1)
    b = min(b,img_h-1)

    return l, t, r, b



class rgbd_img_proc:
    def __init__(self,yolo_detector=None):
        self.yolo_detector = yolo_detector
        
        self.img_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.res_bridge = CvBridge()
        
        self.circle_xyr = [-1]*3
        self.circle_FRD = [-1.0]*3
        self.circle_FRD_px = [-1.0]*3

        self.color_done = False
        self.depth_done = False

        self.color_img = None
        self.depth_img = None
        self.depth_img_uint8 = None

        self.color_lock = threading.Lock()
        self.depth_lock = threading.Lock()

        
        self.img_cnt = 0
        self.circle_info_time = time.time()

        
        self.img_sub = rospy.Subscriber("/d435i/color/image_raw", ImageMsg, self.img_cb)
        self.depth_sub = rospy.Subscriber("/d435i/aligned_depth_to_color/image_raw", ImageMsg, self.depth_cb)

        self.img_res_pub = rospy.Publisher("/bs_debuger/image_res", ImageMsg, queue_size=1)
    
    def reset(self):
        self.circle_xyr = [-1]*3
        self.circle_FRD = [-1.0]*3
        self.circle_FRD_px = [-1.0]*3


    # def init_done(self):
    #     return self.color_done and self.depth_done

    def img_cb(self,msg):
        if not self.color_done:
            self.color_done = True
        self.color_lock.acquire()
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.color_img = cv2.rotate(cv_image, cv2.ROTATE_180)
        self.color_lock.release()
    
    def depth_cb(self,msg):
        if not self.depth_done:
            self.depth_done = True
        self.depth_lock.acquire()
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.depth_img = cv2.rotate(cv_image, cv2.ROTATE_180)
        self.depth_lock.release()
    
    def update_yolo_xyr(self):
        result = False
        if not self.color_done:
            return result
        self.color_lock.acquire()
        cx,cy,w,h = self.yolo_detector.detector(self.color_img)
        if -1 in [cx,cy]:
            self.circle_xyr = [-1]*3
        else:
            self.circle_xyr = [cx,cy,np.sqrt(w*h)/2]
        self.color_lock.release()
    

    
    def update_ring_info(self,method):
        result = False
        if not self.color_done or not self.depth_done:
            return result

        self.depth_lock.acquire()
        self.color_lock.acquire()

        img_result = ring_rect_proc(self.color_img,self.depth_img,method)
        if method in ["ring_normal", "ring_special"]:
            cx,cy,r,real_dis = img_result
            if -1 in [cx,cy]:
                self.circle_xyr = [-1]*3
                self.circle_FRD = [-1.0]*3
            else:
                p_rd = np.array([[cx,cy] + [1]])
                p_out1 = colorK.I*p_rd.T
                R,D,F = (p_out1.T*real_dis).tolist()[0]
                self.circle_xyr = [cx,cy,r]
                self.circle_FRD = [F,R,D]



        img_msg = self.res_bridge.cv2_to_imgmsg(self.color_img, "bgr8")
        self.img_res_pub.publish(img_msg)

        self.color_lock.release()
        self.depth_lock.release()
        self.color_done = False
        self.depth_done = False
        self.circle_info_time = time.time()
        return result





if __name__ == "__main__":
    import os
    from yolo_detector import YoloDetector
    file_pwd = os.path.dirname(os.path.abspath(__file__))
    
    rospy.init_node('rgbd_img_proc_node')
    yolo_detector = YoloDetector("{}/bs_real.yaml".format(file_pwd),yolo_view=True)

    img_proc = rgbd_img_proc(yolo_detector)
    # rospy.spin()
    while True:
        img_proc.update_yolo_xyr()

