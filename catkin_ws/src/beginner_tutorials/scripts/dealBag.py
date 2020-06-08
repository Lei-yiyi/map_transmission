#!/usr/bin/env python
# coding=utf-8


import rospy
from nav_msgs.msg import OccupancyGrid  # map 数据


class TestMap(object):
    def __init__(self):
        # Give the node a name
        rospy.init_node('test_map', anonymous=False)

        rospy.Subscriber("/map", OccupancyGrid, self.get_map_data, queue_size=10)

    def get_map_data(self, msg):
        height = msg.info.height  # pgm 图片属性的像素值（栅格地图初始化大小中的高）
        width = msg.info.width  # pgm 图片属性的像素值（栅格地图初始化大小中的宽）
        origin_x = msg.info.origin.position.x  # ROS 建图的 origin
        origin_y = msg.info.origin.position.y  # ROS 建图的 origin
        resolution = msg.info.resolution  # ROS 建图的分辨率 resolution（栅格地图分辨率对应栅格地图中一小格的长和宽）
        data_size = len(msg.data)
        print(height, width, origin_x, origin_y, resolution, data_size)


if __name__ == "__main__":
    try:
        TestMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Test Map terminated.')
