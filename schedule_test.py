#!/usr/bin/python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Twist
import pyzbar.pyzbar as pyzbar
from datetime import datetime
import pyrealsense2 as rs
import numpy as np
import schedule
import rospy
import time
import cv2
import os

time_count = 0
speed = 0
angle = 0
num = 0
view_box = 0

## 시간제어 변수
prev_time = 0
next_time = 0
count_second = 0 

def cam_lidar_read():
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, )
    pipeline.start(config)

def cam_lidar_use():
    global image_all, add_num, add_edge_num, add_edge_remain_num
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    coverage = [0]*32
    image_all = []
    add_num = 0
    add_edge_num = 0
    add_edge_remain_num = 0
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
            for a in range(1,32):
                if line[a] == " " : add_num = add_num
                else : add_num = add_num + int(line[a])
            for a in range(1,2):
                if line[a] == " " : add_edge_num = add_edge_num
                else : add_edge_num = add_edge_num + int(line[a])
            for a in range(2,30):
                if line[a] == " " : add_edge_remain_num = add_edge_remain_num
                else : add_edge_remain_num = add_edge_remain_num + int(line[a])

def count_time():
    global count_second, prev_time, next_time
    now = datetime.now()
    prev_time = next_time
    next_time = now.second
    if next_time != prev_time : count_second = count_second + 1


def talker():
    global speed, angle, num, count_second, image_all, view_box
    
    rospy.init_node("schedule")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()

    cam_lidar_read()

    while True:

        msg.linear.x = speed
        msg.angular.z = angle

        cam_lidar_use()

        ## 출력 내용
        os.system("clear")
        for _ in range(12):
            print(image_all[_])
        print("전체 값          : ", add_num)
        print("가장자리 값      : ", add_edge_num)
        print("가장자리 이외 값 : ", add_edge_remain_num)

        ## 
        
        if add_num != 0 : 
            if add_num != 0 : 
                if add_edge_remain_num == 0 and add_edge_num > 0 : speed = 0.1   
                if add_edge_remain_num == 0 and add_edge_num > 0 : angle = 0   

                if add_edge_remain_num >  0 and add_edge_num > 0 : speed = 0   
                if add_edge_remain_num >  0 and add_edge_num > 0 : angle = -0.2  
            if add_num == 0 : angle = 0.2 

        pub.publish(msg)     

        # if next_time != prev_time and 1 <= count_second and count_second <= 2 : print("앵글값 변경")
        # if next_time != prev_time and 3 <= count_second and count_second <= 4 : print("스피드 변경")
        # if next_time != prev_time and 5 <= count_second and count_second <= 6 : print("정지")
        # if next_time != prev_time and count_second == 6 : count_second = 0
        
        key = cv2.waitKey(25)
        if key == 27: #ESC
            break




    
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

