#!/usr/bin/env python

from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from frontier_based_exploration.msg import PointArray, RobotState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA
import tf
from nav_msgs.msg import Odometry
from getfrontier import getfrontier
import numpy as np

# 全局变量
mapData = OccupancyGrid()
globalMapData = OccupancyGrid()  # 添加全局地图数据
map_topic = None
robot_pose = [0, 0, 0]  # [x, y, theta]
robot_velocity = [0, 0]  # [linear_velocity, angular_velocity]

def mapCallback(data):
    global mapData, map_topic
    mapData = data
    # if map_topic:  # 检查 map_topic 是否有值
    #     rospy.loginfo(f"{rospy.get_caller_id()} // {map_topic} map data is currently received")
    # else:
    #     rospy.loginfo(f"{rospy.get_caller_id()} // map data is currently received")
    if mapData is None:
        rospy.logwarn(f"{rospy.get_caller_id()} // map data is None")

def globalMapCallback(data):
    """处理全局地图数据"""
    global globalMapData
    globalMapData = data
    # rospy.loginfo_once("Global map data received")
    if globalMapData is None:
        rospy.logwarn(f"{rospy.get_caller_id()} // global map data is None")

def odomCallback(data):
    global robot_velocity
    robot_velocity[0] = data.twist.twist.linear.x
    robot_velocity[1] = data.twist.twist.angular.z

def get_robot_pose(listener, map_topic, base_link_topic):
    try:
        (trans, rot) = listener.lookupTransform(map_topic, base_link_topic, rospy.Time(0))
        global robot_pose
        robot_pose[0] = trans[0]
        robot_pose[1] = trans[1]
        _, _, robot_pose[2] = tf.transformations.euler_from_quaternion(rot)
        # rospy.loginfo(f" robot pose: {robot_pose}")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def get_local_grid_map(mapData, robot_pose, radius=5.0):
    """提取机器人附近局部网格地图，但保持全局地图尺寸"""
    resolution = mapData.info.resolution
    origin = mapData.info.origin.position
    width = mapData.info.width
    height = mapData.info.height
    
    # 打印调试信息
    # rospy.loginfo(f"Map size: {width}x{height}, resolution: {resolution}")
    # rospy.loginfo(f"Map origin: x={origin.x}, y={origin.y}")
    
    # 计算半径对应的网格数
    grid_radius = int(radius / resolution)
    
    # 机器人在全局地图中的网格坐标
    robot_grid_x = int((robot_pose[0] - origin.x) / resolution)
    robot_grid_y = int((robot_pose[1] - origin.y) / resolution)
    
    # 计算局部区域范围
    min_x = max(0, robot_grid_x - grid_radius)
    max_x = min(width, robot_grid_x + grid_radius)
    min_y = max(0, robot_grid_y - grid_radius)
    max_y = min(height, robot_grid_y + grid_radius)
    
    # 创建与全局地图相同大小的局部地图
    local_map = np.full((height, width), -1, dtype=np.int8)  # 用-1填充未知区域
    
    # 只更新机器人周围的区域
    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            idx = y * width + x
            if 0 <= idx < len(mapData.data):
                local_map[y, x] = mapData.data[idx]
    
    # rospy.loginfo(f"Robot at grid coordinates: ({robot_grid_x}, {robot_grid_y})")
    # rospy.loginfo(f"Local area: ({min_x}, {min_y}) to ({max_x}, {max_y})")
    
    return local_map, (0, 0)  # 返回(0,0)因为我们使用全局坐标系

def detector_node():
    # 初始化节点
    rospy.init_node('detector', anonymous=False)
    global mapData, map_topic  # 确保声明 map_topic 为全局变量
    
    # 获取命名空间参数
    namespace = rospy.get_param('~namespace', '/robot_1')  # 默认为 robot_1
    
    # 获取其他参数，使用命名空间作为默认值的一部分
    map_topic = rospy.get_param('~map_topic', f'{namespace}/map')
    odom_topic = rospy.get_param('~odom_topic', f'{namespace}/odom')
    base_link_topic = rospy.get_param('~base_link_topic', f'{namespace}/base_link')
    global_map_topic = rospy.get_param('~global_map_topic', '/map')
    local_map_size = rospy.get_param('~local_map_size', 256)
    
    # rospy.loginfo(f"Namespace: {namespace}")
    # rospy.loginfo(f"Map topic: {map_topic}")
    # rospy.loginfo(f"Odom topic: {odom_topic}")
    # rospy.loginfo(f"Base link topic: {base_link_topic}")
    # rospy.loginfo(f"Global map topic: {global_map_topic}")
    
    # 发布器
    marker_pub = rospy.Publisher(f'{namespace}/visualization_marker', Marker, queue_size=10)
    state_pub = rospy.Publisher(f'{namespace}/robot_state', RobotState, queue_size=10)
    local_map_pub = rospy.Publisher(f'{namespace}/local_map', OccupancyGrid, queue_size=10)
    state_pub = rospy.Publisher(f'{namespace}/robot_state', RobotState, queue_size=10)
    frontiers_pub = rospy.Publisher(f'{namespace}/frontiers', PointArray, queue_size=10)
    frontier_map_pub = rospy.Publisher(f'{namespace}/frontier_map', OccupancyGrid, queue_size=10)  # 新增栅格地图发布器
    
    # 订阅器
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallback)  # 局部地图
    rospy.Subscriber(global_map_topic, OccupancyGrid, globalMapCallback)  # 全局地图
    rospy.Subscriber(odom_topic, Odometry, odomCallback)
    
    # TF监听器
    listener = tf.TransformListener()
    
    rate = rospy.Rate(100)
    
    # 等待第一次地图数据
    # rospy.loginfo("Waiting for map data...")
    
    while mapData.info.resolution == 0:
        rospy.loginfo("Waiting for map data...")
        rospy.sleep(0.1)
        if rospy.is_shutdown():
            return
    # rospy.loginfo_once("----- Requested map topic is " + map_topic + " -----")
    #------------------------------

    p = Point()
    

    while not rospy.is_shutdown():
        # 获取机器人位置
        get_robot_pose(listener, map_topic, base_link_topic)
        
        # 发布机器人状态
        state_msg = RobotState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = f"{namespace}/base_link"  # 使用正确的frame_id
        state_msg.x = robot_pose[0]
        state_msg.y = robot_pose[1]
        state_msg.theta = robot_pose[2]
        state_msg.linear_velocity = robot_velocity[0]
        state_msg.angular_velocity = robot_velocity[1]
        
        state_pub.publish(state_msg)
        
        # 发布局部地图
        if mapData is not None:
            local_map_msg = OccupancyGrid()
            local_map_msg.header.stamp = rospy.Time.now()
            local_map_msg.header.frame_id = f"{namespace}/base_link"
            
            # 获取局部地图
            local_map, _ = get_local_grid_map(mapData, robot_pose, radius=5.0)
            
            # 设置地图信息（与全局地图相同）
            local_map_msg.info = mapData.info  # 直接复制全局地图的信息
            local_map_msg.data = local_map.flatten().tolist()
            
            local_map_pub.publish(local_map_msg)
            # rospy.loginfo(f"Local map published for {namespace}")
        
        #----- rviz visualization -----
        frontier_points = Marker()
        # print("new : ", frontier_points.points)

        frontier_points.header.frame_id = mapData.header.frame_id
        frontier_points.header.stamp=rospy.Time.now()
        frontier_points.ns = "points"

        if(mapData.header.frame_id == 'robot_1/map'):
            frontier_points.id = 0
        if(mapData.header.frame_id == 'robot_2/map'):
            frontier_points.id = 1
        if(mapData.header.frame_id == 'robot_3/map'):
            frontier_points.id = 2
        
        frontier_points.type = Marker.POINTS
        frontier_points.action = Marker.ADD

        frontier_points.pose.orientation.w = 1.0
        frontier_points.scale.x = 0.15
        frontier_points.scale.y = 0.15
        frontier_points.color = ColorRGBA(1, 1, 0, 1)
        frontier_points.lifetime = rospy.Duration()
    #------------------------------

        # getfrontier.py Node
        frontiers = getfrontier(mapData)
        if frontiers is not None and len(frontiers) > 0:
            # 创建前沿点消息
            frontier_msg = PointArray()
            frontier_msg.points = []
            
            # 创建栅格地图格式的前沿信息
            frontier_map = OccupancyGrid()
            frontier_map.header = mapData.header
            frontier_map.info = mapData.info
            frontier_map.data = [0] * len(mapData.data)
            
            for frontier in frontiers:
                # 添加到 Marker 用于可视化
                frontier_points.points.append(Point(frontier[0], frontier[1], 0))
                
                # 添加到 PointArray 用于训练
                p = Point()
                p.x = frontier[0]
                p.y = frontier[1]
                p.z = 0.0
                frontier_msg.points.append(p)
                
                # 添加到栅格地图
                mx = int((frontier[0] - mapData.info.origin.position.x) / mapData.info.resolution)
                my = int((frontier[1] - mapData.info.origin.position.y) / mapData.info.resolution)
                index = my * mapData.info.width + mx
                if 0 <= index < len(frontier_map.data):
                    frontier_map.data[index] = 100
            
            # 发布所有消息
            marker_pub.publish(frontier_points)      # 可视化
            frontiers_pub.publish(frontier_msg)      # 点数组格式
            frontier_map_pub.publish(frontier_map)   # 栅格地图格式
            
            # rospy.loginfo(f"Published {len(frontier_msg.points)} frontiers for {namespace}")
        else:
            # 即使没有前沿点，也发布空的消息
            # rospy.loginfo("No frontiers found")
            marker_pub.publish(frontier_points)
            frontiers_pub.publish(PointArray())
            
            # 发布空的栅格地图
            empty_map = OccupancyGrid()
            empty_map.header = mapData.header
            empty_map.info = mapData.info
            empty_map.data = [0] * len(mapData.data)
            frontier_map_pub.publish(empty_map)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        detector_node()
    except rospy.ROSInterruptException:
        pass