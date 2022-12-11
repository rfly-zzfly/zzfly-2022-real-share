#!/usr/bin/env python

import rospy
from std_msgs.msg import String


debug_nameL = [
    "task_state",
    "yaw_state" ,
    "yolo_px"   ,
    "yolo_frd"  ,
    "cmdv_frd"  ,
    "vs_loss"   ,
    "circle_position",
]



class Debuger:
    def __init__(self):
        self.pub_dict = {}
        for i in debug_nameL:
            self.pub_dict[i] = rospy.Publisher('/bs_debuger/{}'.format(i), String, queue_size=1)

    
    def update(self,debug_info_dict):
        for k,v in debug_info_dict.items():
            self.pub_dict[k].publish(v)







