#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 2020-06-06

Updated on 2020-06-06

@author: 小小磊

@requirements: Ubuntu 16.04.6 LTS, Python 3.5, ROS Version: kinetic 1.12.13

@decription: 删除地图中无意义的未知区域

@ref: https://www.cnblogs.com/silence-cho/p/10926248.html
"""

import numpy as np
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
from collections import defaultdict


class Reducing(object):
    def __init__(self):
        rospy.init_node('map_reducing', anonymous=False)

        self.last_map = OccupancyGrid()
        self.map_adj_pub = rospy.Publisher('/map_reducing', OccupancyGrid, queue_size=10,
                                           latch=True)  # 定义发布剔除冗余数据后的地图数据
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=10)  # 订阅地图数据

    def map_callback(self, msg):
        map_adj = OccupancyGrid()
        map_adj.header.frame_id = '/map'
        map_adj.header.stamp = rospy.Time.now()
        map_adj.info = msg.info
        original = msg.data
        print("原始地图信息：")
        print(len(original), map_adj.info.width * map_adj.info.height, map_adj.info.width, map_adj.info.height,
              msg.info.origin.position.x, msg.info.origin.position.y)

        # 原始地图转为灰度图后获取最大外接矩形
        img_tmp = self.map2grayscale(original)
        img_gray = np.array(img_tmp).reshape(map_adj.info.height, map_adj.info.width)
        img_gray = img_gray.astype(np.uint8)  # 转换为灰度图的格式
        # cv2.imwrite('copy_empire.jpg', img_gray)  # 保存图片
        # print(type(img_gray), img_gray.shape, img_gray.dtype)
        ret, thresh = cv2.threshold(img_gray, 180, 255, cv2.THRESH_BINARY)
        binary, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = contours[0]
        x, y, w, h = cv2.boundingRect(cnt)  # (x, y) 为矩形左上角的坐标, (w, h) 是矩形的宽和高
        print("原始地图转为灰度图后获取的最大外接矩形的信息：")
        print(x, y, w, h)

        # 最大外接矩形对应于地图数据的范围
        h_min = y
        h_max = y + h
        w_min = x
        w_max = x + w
        print("最大外接矩形对应于地图数据的范围:")
        print(h_min, h_max, w_min, w_max)

        # 取出地图数据的范围
        arr = np.array(original).reshape(map_adj.info.height, map_adj.info.width)
        # print(type(arr), arr.shape, arr.dtype)
        arr_a = arr[h_min:h_max]  # 先取出想要的行数据
        new = arr_a[:, w_min:w_max]  # 再取出要求的列数据
        new = new.flatten()  # 对数组进行降维，返回折叠后的一维数组，原数组不变

        # 计算新地图原点的世界坐标
        origin_index = h_min * map_adj.info.width + w_min  # 新地图原点在原始地图中的索引值
        origin_point = point_of_index(map_adj.info, origin_index)

        # 修改数据
        map_adj.info.width = w_max - w_min
        map_adj.info.height = h_max - h_min
        map_adj.info.origin.position.x = origin_point[0]
        map_adj.info.origin.position.y = origin_point[1]
        map_adj.data = new
        print("剔除冗余数据后的地图信息：")
        print(len(map_adj.data), map_adj.info.width * map_adj.info.height, map_adj.info.width, map_adj.info.height,
              map_adj.info.origin.position.x, map_adj.info.origin.position.y)

        # # 发布剔除冗余数据后的地图数据
        # self.map_adj_pub.publish(map_adj)  # 只发布一次

        """
        外接矩形：原点坐标不变
        """
        # 外接矩形：原点坐标不变，宽不变，高不变
        if map_adj.info.origin.position.x == self.last_map.info.origin.position.x and \
                map_adj.info.origin.position.y == self.last_map.info.origin.position.y and \
                map_adj.info.width == self.last_map.info.width and \
                map_adj.info.height == self.last_map.info.height and \
                self.last_map is not None:  # 首次无数据

            # 提取发生变化的数据的下标和变化后的值
            dd = defaultdict(list)
            for i, v in enumerate(map_adj.data):
                if map_adj.data[i] != self.last_map.data[i]:
                    dd[i] = v
            print("外接矩形：原点坐标不变，宽不变，高不变，输出差异数据的位置和差异值：")
            print(len(dd))

        # 外接矩形：原点坐标不变，宽不变，高变
        if map_adj.info.origin.position.x == self.last_map.info.origin.position.x and \
                map_adj.info.origin.position.y == self.last_map.info.origin.position.y and \
                map_adj.info.width == self.last_map.info.width and \
                map_adj.info.height != self.last_map.info.height and \
                self.last_map is not None:

            # 提取发生变化的数据的下标和变化后的值
            dd = defaultdict(list)
            for i, v in enumerate(map_adj.data):
                if len(map_adj.data) > len(self.last_map.data):  # 高变大
                    if i < len(self.last_map.data):  # 已有数据
                        if map_adj.data[i] != self.last_map.data[i]:
                            dd[i] = v
                    else:  # 新增数据
                        dd[i] = v
                else:  # 高变小
                    if i < len(map_adj.data):
                        if map_adj.data[i] != self.last_map.data[i]:
                            dd[i] = v
            print("外接矩形：原点坐标不变，宽不变，高变，输出差异数据的位置和差异值：")
            print(len(dd))

        # 外接矩形：原点坐标不变，宽变，高不变
        if map_adj.info.origin.position.x == self.last_map.info.origin.position.x and \
                map_adj.info.origin.position.y == self.last_map.info.origin.position.y and \
                map_adj.info.width != self.last_map.info.width and \
                map_adj.info.height == self.last_map.info.height and \
                self.last_map is not None:

            # 提取发生变化的数据的下标和变化后的值
            dd = defaultdict(list)
            for i, v in enumerate(map_adj.data):
                if map_adj.info.width > self.last_map.info.width:  # 宽变大
                    if i < self.last_map.info.width:  # 第一行已有数据
                        if map_adj.data[i] != self.last_map.data[i]:
                            dd[i] = v
                    elif self.last_map.info.width <= i < map_adj.info.width:  # 第一行新增数据
                        dd[i] = v
                    else:  # 其余行数据
                        if i % map_adj.info.width >= map_adj.info.width-self.last_map.info.width:  # 其余行已有数据
                            j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                                ((i % map_adj.info.width)-(
                                        map_adj.info.width-self.last_map.info.width))  # 对应的前一时刻数据的下标
                            if map_adj.data[i] != self.last_map.data[j]:
                                dd[i] = v
                        else:  # 其余行新增数据
                            dd[i] = v
                else:  # 宽变小
                    if i < map_adj.info.width:  # 第一行数据
                        if map_adj.data[i] != self.last_map.data[i]:
                            dd[i] = v
                    else:  # 其余行数据
                        j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                            ((i % map_adj.info.width)+(
                                    self.last_map.info.width-map_adj.info.width))  # 对应的前一时刻数据的下标
                        if map_adj.data[i] != self.last_map.data[j]:
                            dd[i] = v
            print("外接矩形：原点坐标不变，宽变，高不变，输出差异数据的位置和差异值：")
            print(len(dd))

        # 外接矩形：原点坐标不变，宽变，高变
        if map_adj.info.origin.position.x == self.last_map.info.origin.position.x and \
                map_adj.info.origin.position.y == self.last_map.info.origin.position.y and \
                map_adj.info.width != self.last_map.info.width and \
                map_adj.info.height != self.last_map.info.height and \
                self.last_map is not None:

            # 提取发生变化的数据的下标和变化后的值
            dd = defaultdict(list)
            for i, v in enumerate(map_adj.data):
                if map_adj.info.width > self.last_map.info.width:  # 宽变大
                    if map_adj.info.height > self.last_map.info.height:  # 高变大
                        if i < self.last_map.info.height*map_adj.info.width:  # 原有高
                            if i < self.last_map.info.width:  # 第一行已有数据
                                if map_adj.data[i] != self.last_map.data[i]:
                                    dd[i] = v
                            elif self.last_map.info.width <= i < map_adj.info.width:  # 第一行新增数据
                                dd[i] = v
                            else:  # 其余行数据
                                if i % map_adj.info.width >= map_adj.info.width-self.last_map.info.width:  # 其余行已有数据
                                    j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                                        ((i % map_adj.info.width)-(
                                                map_adj.info.width-self.last_map.info.width))  # 对应的前一时刻数据的下标
                                    if map_adj.data[i] != self.last_map.data[j]:
                                        dd[i] = v
                                else:  # 其余行新增数据
                                    dd[i] = v
                        else:  # 新增高
                            dd[i] = v
                    else:  # 高变小
                        if i < self.last_map.info.width:  # 第一行已有数据
                            if map_adj.data[i] != self.last_map.data[i]:
                                dd[i] = v
                        elif self.last_map.info.width <= i < map_adj.info.width:  # 第一行新增数据
                            dd[i] = v
                        else:  # 其余行数据
                            if i % map_adj.info.width >= map_adj.info.width-self.last_map.info.width:  # 其余行已有数据
                                j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                                    ((i % map_adj.info.width)-(
                                            map_adj.info.width-self.last_map.info.width))  # 对应的前一时刻数据的下标
                                if map_adj.data[i] != self.last_map.data[j]:
                                    dd[i] = v
                            else:  # 其余行新增数据
                                dd[i] = v

                else:  # 宽变小
                    if map_adj.info.height > self.last_map.info.height:  # 高变大
                        if i < map_adj.info.height * map_adj.info.width:  # 当前高
                            if i < map_adj.info.width:  # 第一行数据
                                if map_adj.data[i] != self.last_map.data[i]:
                                    dd[i] = v
                            else:  # 其余行数据
                                j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                                    ((i % map_adj.info.width)+(
                                            self.last_map.info.width-map_adj.info.width))  # 对应的前一时刻数据的下标
                                if map_adj.data[i] != self.last_map.data[j]:
                                    dd[i] = v
                        else:  # 新增高
                            dd[i] = v
                    else:  # 高变小
                        if i < map_adj.info.width:  # 第一行数据
                            if map_adj.data[i] != self.last_map.data[i]:
                                dd[i] = v
                        else:  # 其余行数据
                            j = (int(i/map_adj.info.width))*self.last_map.info.width + \
                                ((i % map_adj.info.width)+(
                                            self.last_map.info.width-map_adj.info.width))  # 对应的前一时刻数据的下标
                            if map_adj.data[i] != self.last_map.data[j]:
                                dd[i] = v
            print("外接矩形：原点坐标不变，宽变，高变，输出差异数据的位置和差异值：")
            print(len(dd))

        """
        外接矩形：原点坐标变（外接矩形不匹配，输出变化后的外接矩形内的全部数据）
        """
        # 外接矩形：原点坐标不变，宽变，高变
        if self.last_map is not None:
            if map_adj.info.origin.position.x != self.last_map.info.origin.position.x or \
                    map_adj.info.origin.position.y != self.last_map.info.origin.position.y:
                print("外接矩形：原点坐标变，输出变化后的外接矩形内的全部数据：")
                print(len(map_adj.data))

        # 保存最新时刻的数据
        self.last_map = map_adj
        print("========= end =========")

    @staticmethod
    def map2grayscale(data):
        # 对于只有黑白颜色的灰度图，为单通道，一个像素块对应矩阵中一个数字，数值为 0 到 255 ，其中 0 表示最暗（黑色）， 255 表示最亮（白色）
        # map data:    100 occupied    -1 unknown    0 free
        # costmap data:    100 occupied    -1 unknown    0 free    99 inflation layer
        data_new = copy.deepcopy(list(data))
        for idx, value in enumerate(data):
            if value == -1:  # 未知区域
                data_new[idx] = 0
            else:
                data_new[idx] = 255
        return data_new


def point_of_index(map_info, i):
    # 计算地图栅格的索引值对应的世界坐标点
    y = map_info.origin.position.y + (i / map_info.width) * map_info.resolution
    x = map_info.origin.position.x + (i - (i / map_info.width) * map_info.width) * map_info.resolution
    return [x, y]


if __name__ == '__main__':
    try:
        Reducing()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Reducing /map redundant data terminated.')
