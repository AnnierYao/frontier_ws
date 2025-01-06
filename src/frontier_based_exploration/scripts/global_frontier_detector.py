#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from frontier_based_exploration.msg import PointArray
from geometry_msgs.msg import Point
from getfrontier import getfrontier

class GlobalFrontierDetector:
    def __init__(self):
        rospy.init_node('global_frontier_detector')
        
        # 参数
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 数据存储
        self.global_map = None
        
        # 发布器
        self.global_frontier_pub = rospy.Publisher('/global_frontiers', OccupancyGrid, queue_size=10)
        self.global_frontier_points_pub = rospy.Publisher('/global_frontier_points', PointArray, queue_size=10)
        
        # 订阅器
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        rospy.loginfo("Global frontier detector initialized")
    
    def map_callback(self, msg):
        """处理全局地图数据"""
        self.global_map = msg
    
    def detect_global_frontiers(self):
        """检测全局地图中的前沿点"""
        if self.global_map is None:
            return
        
        # 使用getfrontier函数检测前沿点
        frontiers = getfrontier(self.global_map)
        
        if frontiers is not None and len(frontiers) > 0:
            # 创建前沿点地图
            frontier_map = OccupancyGrid()
            frontier_map.header = self.global_map.header
            frontier_map.info = self.global_map.info
            frontier_map.data = [0] * len(self.global_map.data)
            
            # 创建前沿点数组消息
            frontier_points = PointArray()
            frontier_points.points = []
            
            # 在地图上标记前沿点
            for frontier in frontiers:
                # 添加到PointArray消息
                point = Point()
                point.x = frontier[0]
                point.y = frontier[1]
                point.z = 0.0
                frontier_points.points.append(point)
                
                # 在栅格地图上标记
                mx = int((frontier[0] - self.global_map.info.origin.position.x) / 
                        self.global_map.info.resolution)
                my = int((frontier[1] - self.global_map.info.origin.position.y) / 
                        self.global_map.info.resolution)
                index = my * self.global_map.info.width + mx
                if 0 <= index < len(frontier_map.data):
                    frontier_map.data[index] = 100
            
            # 发布消息
            self.global_frontier_pub.publish(frontier_map)
            self.global_frontier_points_pub.publish(frontier_points)
            
            # rospy.loginfo(f"Published {len(frontiers)} global frontier points")
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            self.detect_global_frontiers()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        detector = GlobalFrontierDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass 