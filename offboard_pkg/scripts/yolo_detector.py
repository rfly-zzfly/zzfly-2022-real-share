#!/usr/bin/python3

import os
import yaml

import cv2
import darknet


class YoloCfg():
    def __init__(self,yaml_path):
        with open(yaml_path, "r") as fp:
            yaml_cfg = yaml.load(fp,Loader=yaml.FullLoader)
        file_pwd = os.path.dirname(os.path.abspath(__file__))

        self.weights = os.path.join(file_pwd,yaml_cfg["weights"])
        self.config_file = os.path.join(file_pwd,yaml_cfg["config_file"])
        self.name_file = os.path.join(file_pwd,yaml_cfg["name_file"])
        self.thresh = 0.25


class YoloDetector():
    def __init__(self, yaml_path,view=False):
        self.view = view


        self.args = YoloCfg(yaml_path)

        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.args.config_file,
            self.args.name_file,
            self.args.weights,
            batch_size=1
        )
        self.darknet_width = darknet.network_width(self.network)
        self.darknet_height = darknet.network_height(self.network)

    def detector(self, img_bgr):
        frame_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        #frame_rgb = img_bgr
        frame_resized = cv2.resize(frame_rgb, (self.darknet_width, self.darknet_height), interpolation=cv2.INTER_LINEAR)
        img_for_detect = darknet.make_image(self.darknet_width, self.darknet_height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        detections = darknet.detect_image(self.network, self.class_names, img_for_detect, thresh=self.args.thresh)

        darknet.free_image(img_for_detect)
        detections_adjusted = []
        for label, confidence, bbox in detections:
            bbox_adjusted = self.convert2original(img_bgr, bbox)
            detections_adjusted.append((str(label), confidence, bbox_adjusted))
        
        # sort by area of bbox
        detections_adjusted.sort(key=lambda x: (-x[2][2]*x[2][3]))   

        if self.view:
            # print(detections_adjusted)
            img_bgr = darknet.draw_boxes(detections_adjusted, img_bgr, self.class_colors)
            cv2.imshow('detector', img_bgr)
            cv2.waitKey(1)

        if detections_adjusted == []:
            return -1,-1,-1,-1
        else:
            return detections_adjusted[0][2]

    def convert2relative(self, bbox):
        """
        YOLO format use relative coordinates for annotation
        """
        x, y, w, h  = bbox
        _height     = self.darknet_height
        _width      = self.darknet_width
        return x/_width, y/_height, w/_width, h/_height

    def convert2original(self, image, bbox):
        x, y, w, h = self.convert2relative(bbox)
        image_h, image_w, __ = image.shape

        orig_x       = int(x * image_w)
        orig_y       = int(y * image_h)
        orig_width   = int(w * image_w)
        orig_height  = int(h * image_h)

        bbox_converted = (orig_x, orig_y, orig_width, orig_height)
        return bbox_converted

    def test(self,dataset_path):
        import numpy as np
        import time

        file_names = os.listdir(dataset_path)
        file_names.sort()
        start_time = time.time()
        l = len(file_names)
        for img_file in file_names:
            img_name = os.path.basename(img_file)
            img_bgr = cv2.imread(os.path.join(dataset_path, img_name))
            bbox = self.detector(img_bgr)
            cv2.circle(img_bgr, (bbox[0], bbox[1]), int(np.sqrt(bbox[2]*bbox[3])/2), (0,255,0), 2)
            cv2.circle(img_bgr, (bbox[0], bbox[1]), 2, (0,255,255), 3)
            cv2.imshow("circle", img_bgr)
            cv2.waitKey(1)
        print("FPS:", l/(time.time()-start_time))






if __name__ == "__main__":
    dataset_path = "img_test"
    d = YoloDetector("./bs_stage1.yaml")
    d.test(dataset_path)
