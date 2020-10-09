#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from landing.msg import center # 自定义消息类型
import os
import cv2
import numpy as np
import time

# is_sim : 0 is real,1 is sim 
is_sim = 1
is_plot = 1

center_publish=rospy.Publisher('/center', center, queue_size=1) # 发布矩形中心
# 获得摄像头图像的回调函数
def callback(Image):
    img = np.fromstring(Image.data, np.uint8)
    img = img.reshape(240,320,3)
    track(img, Image.width, Image.height) # 寻找矩形

# 订阅获得摄像头图像
def listener():
    rospy.init_node('track')
    if is_sim:
        rospy.Subscriber('/iris/usb_cam/image_raw', Image, callback)
    else:
        rospy.Subscriber('/usb_cam/image_rect_color', Image, callback)
    rospy.spin()

# 寻找矩形中心
def track(frame, width, height):
    img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # _, img = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY)
    ret2, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # 轮廓提取
    rects = [] # 存放四边形
    centers = [] # 存放中心点
    for contour in contours[1]:
        if cv2.contourArea(contour) < 100: # 过滤掉矩形面积小的
            continue
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if approx.shape[0] == 4 and cv2.isContourConvex(approx): # 过滤掉非四边形与非凸形
            rects.append(approx)
            centers.append((approx[0]+approx[1]+approx[2]+approx[3]).squeeze()/4)

	# 以下部分为聚类算法
    center_iter = list(range(len(centers)))
    result = []
    threshold = 20
    while len(center_iter) is not 0:
        j = 1
        resultnow = []
        while j < len(center_iter):
            if np.sum((centers[center_iter[0]] - centers[center_iter[j]])**2) < threshold:
                resultnow.append(center_iter[j])
                center_iter.pop(j)
                j = j-1
            j = j+1
        resultnow.append(center_iter[0])
        center_iter.pop(0)
        if len(result) < len(resultnow):
            result = resultnow
    rects = np.array(rects)[result]
    # 如果嵌套的矩形数量大于2才算提取成功
    if len(result) > 2:
        centers = np.sum(np.array(centers)[result], axis=0).astype(np.double) / len(result)
        publish(centers, width, height) # 发布消息
    else:
        center_temp = center()
        center_temp.iffind = False
        center_publish.publish(center_temp)
    
    # 下面注释掉的部分将画出提取出的轮廓
    if is_plot:
        cv2.polylines(frame, rects, True, (0,0,255), 2)
        cv2.imshow('w',frame)
        cv2.waitKey(1)

# 发布中心点消息
def publish(centers, width, height):
    center_temp = center()
    center_temp.width = width
    center_temp.height = height
    center_temp.x = centers[1]
    center_temp.y = centers[0]
    center_temp.iffind = True
    center_publish.publish(center_temp)

if __name__ == '__main__':
    print("Tracking start")
    listener()
