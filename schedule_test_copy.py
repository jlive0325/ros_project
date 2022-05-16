#!/usr/bin/python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Twist
import pyzbar.pyzbar as pyzbar
import pyrealsense2 as rs
import numpy as np
import schedule
import rospy
import time
import cv2
from datetime import datetime

time_count = 0
speed = 0
num = 0
left_angle = 0.8
right_angle = -0.8


def cam_lidar_read():
    global pipeline
    pipeline = rs.pipeline()

    # 스트림 구성
    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, )

    # 스트리밍 시작
    pipeline.start(config)

def cam_lidar_use():
    global add_num, image_all
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()

    # 이미지를 10x20 픽셀 영역으로 나누고 1m 이내의 픽셀 범위를 근사하여 간단한 텍스트 기반 이미지 인쇄
    coverage = [0]*32
    image_all = []
    add_num = 0

    for y in range(240):
        for x in range(320):
            dist = depth.get_distance(x, y)
            if 0 < dist and dist < 1:
                coverage[x//10] += 1
        
        if y%20 is 17:
            line = ""
            for c in coverage:
                line += " 12345678"[c//25]

            coverage = [0]*32
            image_all.append(line)
            for a in range(32):
                if line[a] == " " : add_num = add_num
                else : add_num = add_num + int(line[a])


time_1 = datetime.strptime('00','%S')
time_2 = datetime.strptime('05','%S')
time_interver = time_2 - time_1
print(time_interver)



def talker():
    global speed, num, time_interver
    
    rospy.init_node("schedule")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()

    # cam_lidar_read()
    
    while not rospy.is_shutdown():
        for _ in range(time_interver):
            pub.publish(msg)
            # cam_lidar_use()

    
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

