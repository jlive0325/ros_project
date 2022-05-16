#!/usr/bin/python3
# -*- coding: utf-8 -*-

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
from time import time
import numpy as np
import rospy
import cv2
import os

accel = rs.stream.accel
any = rs.stream.any
color = rs.stream.color
confidence = rs.stream.confidence
depth = rs.stream.depth
fisheye = rs.stream.fisheye
gpio = rs.stream.gpio
gyro = rs.stream.gyro
infrared = rs.stream.infrared
pose = rs.stream.pose

def talker():

    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16)

    pipeline.start(config)

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        coverage = [0]*32
        image_all = []
        add_num = 0
        for y in range(240):
            for x in range(320):
                dist = depth.get_distance(x, y)
                if 0 < dist and dist < 2:
                    coverage[x//10] += 1
            if y%20 is 19:
                line = ""
                for c in coverage:
                    line += "012345678"[c//25]
                coverage = [0]*32
                image_all.append(line)
                for a in range(1,32):
                    if line[a] == " " : add_num = add_num
                    else : add_num = add_num + int(line[a])

        for y in range(12):
            print(image_all[y])
        print(add_num)
        
        key = cv2.waitKey(25)
        if key == 27: #ESC
            break


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

