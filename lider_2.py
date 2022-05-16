# -*- coding: utf-8 -*-

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#!/usr/bin/python
#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import os

try:
    # 컨텍스트 개체를 만듭니다. 이 개체는 연결된 모든 실제 감지 장치의 핸들을 소유합니다.
    pipeline = rs.pipeline()

    # 스트림 구성
    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, )

    # 스트리밍 시작
    pipeline.start(config)

    while True:
        # 이 호출은 장치에서 새로운 일관된 프레임 집합을 사용할 수 있을 때까지 기다립니다.
        # 디바이스에서 get_frame_data(...) 및 get_frame_timestamp(...)를 호출하면 wait_for_frames(...)가 호출될 때까지 안정적인 값이 반환됩니다.
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth: continue

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
                    # line += " .:nhBXWW"[c//25]
                    line += " 12345678"[c//25]

                coverage = [0]*32
                # print(line)
                image_all.append(line)
                for a in range(32):
                    if line[a] == " " : add_num = add_num
                    else : add_num = add_num + int(line[a])
                    

        os.system("clear")

        for _ in range(12):
            print(image_all[_])
        print(add_num)


    exit(0)
#except rs.error as e:
#    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
#    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
#    print("    %s\n", e.what())
#    exit(1)
except Exception as e:
    print(e)
    pass