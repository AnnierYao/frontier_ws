#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf
from numpy import array, vstack, delete
from functions import gridValue, informationGain
from sklearn.cluster import MeanShift
from frontier_based_exploration.msg import PointArray
import random

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    rospy.loginfo(f"Received frontier point: {x}")
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x
    rospy.loginfo(f"Current frontiers length: {len(frontiers)}")


def mapCallBack(data):
    global mapData
    # 确保地图数据使用全局坐标系
    if 'robot_' in data.header.frame_id:
        # 从机器人局部地图转换到全局地图
        mapData = data
        mapData.header.frame_id = 'map'
    else:
        mapData = data


def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data

# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, namespace_init_count
    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 50)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 2.0)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 3)
    namespace = rospy.get_param('~namespace', '/robot_')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    rateHz = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param(
        '~global_costmap_topic', '/move_base/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = rospy.Rate(rateHz)
# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)


# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rospy.Subscriber(namespace+str(i+namespace_init_count) +
                             global_costmap_topic, OccupancyGrid, globalMap)
    elif len(namespace) == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)
# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass
# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rospy.loginfo('Waiting for the global costmap')
            rospy.sleep(0.1)
            pass

    global_frame = "/"+mapData.header.frame_id

    tfLisn = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tfLisn.waitForTransform(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tfLisn.waitForTransform(
            global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

    rospy.loginfo("the map and global costmaps are received")
    rospy.loginfo(f"frontiers length: {len(frontiers)}")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    points.lifetime = rospy.Duration()

    p = Point()

    p.z = 0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = []
        front = copy(frontiers)
        rospy.loginfo(f"Processing frontiers, length: {len(front)}")
        
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_
            rospy.loginfo(f"After clustering, centroids length: {len(centroids)}")
            
            # 随机选择一个中心点
            if len(centroids) > 0:
                selected_idx = random.randint(0, len(centroids)-1)
                selected_point = centroids[selected_idx]
                rospy.loginfo(f"Randomly selected frontier point {selected_idx}: ({selected_point[0]:.2f}, {selected_point[1]:.2f})")
                
                # 只保留选中的点
                centroids = [selected_point]
        
        elif len(front) == 1:
            centroids = front
            rospy.loginfo("Single frontier point, using it directly")
        
        # 发布选中的点
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        rospy.loginfo(f"publishing arraypoints length: {len(arraypoints.points)}")
        filterpub.publish(arraypoints)
        
        # 可视化
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        pub2.publish(points_clust)
        
        rate.sleep()


def simple_information_gain(mapData, point, radius):
    """
    计算点周围的信息增益，考虑距离中心的权重
    """
    resolution = mapData.info.resolution
    x = int((point[0] - mapData.info.origin.position.x) / resolution)
    y = int((point[1] - mapData.info.origin.position.y) / resolution)
    r = int(radius / resolution)
    
    # 检查坐标是否在地图范围内
    if x < 0 or x >= mapData.info.width or y < 0 or y >= mapData.info.height:
        rospy.logwarn(f"Point ({point[0]}, {point[1]}) is outside map bounds")
        return 0
    
    rospy.loginfo(f"World point ({point[0]:.2f}, {point[1]:.2f}) -> Grid point ({x}, {y})")
    
    unknown = 0  # 未知区域
    free = 0    # 空闲区域
    occupied = 0  # 占用区域
    weighted_unknown = 0  # 加权未知区域
    r_squared = r * r
    
    # 遍历圆形区域
    for i in range(max(0, x-r), min(mapData.info.width, x+r+1)):
        for j in range(max(0, y-r), min(mapData.info.height, y+r+1)):
            # 计算到中心点的距离（平方）
            dist_squared = (i-x)**2 + (j-y)**2
            if dist_squared <= r_squared:  # 在圆内
                idx = j * mapData.info.width + i
                if 0 <= idx < len(mapData.data):
                    cell_value = mapData.data[idx]
                    
                    # 计算距离权重（越近权重越大）
                    dist_weight = 1.0 - (dist_squared / r_squared) ** 0.5
                    
                    if cell_value == -1:  # 未知区域
                        unknown += 1
                        weighted_unknown += dist_weight
                    elif cell_value == 0:  # 空闲区域
                        free += 1
                    else:  # 占用区域
                        occupied += 1
    
    total = unknown + free + occupied
    if total == 0:
        rospy.logwarn(f"No valid cells found around grid point ({x}, {y})")
        return 0
    
    # 计算各种比例
    unknown_ratio = unknown / total
    free_ratio = free / total
    occupied_ratio = occupied / total
    
    # 计算得分
    score = 0
    if unknown_ratio >= 0.2 and occupied_ratio < 0.4:  # 基本条件
        # 基础分数是加权未知区域
        base_score = weighted_unknown * (resolution ** 2)
        
        # 根据未知区域比例增加权重
        unknown_weight = 1.0
        if unknown_ratio > 0.6:
            unknown_weight = 1.2
        elif unknown_ratio > 0.4:
            unknown_weight = 1.1
        
        # 根据占用区域比例减少权重
        occupied_penalty = 1.0
        if occupied_ratio > 0.2:
            occupied_penalty = 0.8
        
        # 计算最终得分
        score = base_score * unknown_weight * occupied_penalty
        
        # 添加一个小的随机因子，避免完全相同的得分
        score *= (0.95 + 0.1 * random.random())
    
    rospy.loginfo(f"Cell counts - Unknown: {unknown}, Free: {free}, Occupied: {occupied}")
    rospy.loginfo(f"Weighted unknown: {weighted_unknown:.2f}")
    rospy.loginfo(f"Ratios - Unknown: {unknown_ratio:.2%}, Free: {free_ratio:.2%}, Occupied: {occupied_ratio:.2%}")
    rospy.loginfo(f"Final score: {score:.3f} m²")
    
    return score


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass