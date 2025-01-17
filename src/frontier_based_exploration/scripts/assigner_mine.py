#!/usr/bin/env python

from copy import copy
from turtle import position
import rospy
from nav_msgs.msg import OccupancyGrid
from frontier_based_exploration.msg import PointArray
from numpy import array
from numpy.linalg import norm
from nav_msgs.msg import Odometry
from numpy import floor
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import actionlib

mapData=OccupancyGrid()
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()

frontiers=[]
globalmaps=[]
pose_1 = None
pose_2 = None
pose_3 = None

def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data

def pose_callback_1(msg):
	global pose_1
	pose_1 = msg.pose.pose.position

def pose_callback_2(msg):
	global pose_2
	pose_2 = msg.pose.pose.position

def pose_callback_3(msg):
	global pose_3
	pose_3 = msg.pose.pose.position

def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain*(mapData.info.resolution**2)

def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int(	(floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index

def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    return array([x, y])

def assign_node():
	global frontiers,mapData,global1,global2,global3,globalmaps,pose_1,pose_2,pose_3
	rospy.init_node('assigner_mine', anonymous=False)
	rospy.loginfo("assigner_mine node is loaded")

	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')
	namespace = rospy.get_param('~namespace','')
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)

	rate = rospy.Rate(rateHz)

	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
	rospy.Subscriber('robot_1/odom', Odometry, pose_callback_1) # for current position
	rospy.Subscriber('robot_2/odom', Odometry, pose_callback_2) # for current position
	rospy.Subscriber('robot_3/odom', Odometry, pose_callback_3) # for current position

	goal = MoveBaseGoal()
	goal.target_pose = PoseStamped()
	goal.target_pose.header.frame_id = "map"
	client_1 = actionlib.SimpleActionClient('robot_1'+'/move_base', MoveBaseAction)
	client_1.wait_for_server()
	rospy.loginfo("client_1 is ready")
	client_2 = actionlib.SimpleActionClient('robot_2'+'/move_base', MoveBaseAction)
	client_2.wait_for_server()
	rospy.loginfo("client_2 is ready")
	client_3 = actionlib.SimpleActionClient('robot_3'+'/move_base', MoveBaseAction)
	client_3.wait_for_server()
	rospy.loginfo("client_3 is ready")

	# wait if no frontier is received yet 
	while len(frontiers)<1:
		rospy.loginfo("No frontier is received yet")
		pass
	#wait if map is not received yet
	while (len(mapData.data)<1):
		rospy.loginfo("No map is received yet")
		pass
	
	# 等待所有机器人的位置信息
	while None in [pose_1, pose_2, pose_3]:
		rospy.loginfo("Waiting for all robots' positions...")
		rospy.sleep(0.1)
	
	while not rospy.is_shutdown():
		centroids = copy(frontiers)
		if len(centroids) == 0:
			rospy.loginfo("No frontiers available")
			rate.sleep()
			continue

		# 获取机器人位置
		position = []
		try:
			position.append(array([pose_1.x, pose_1.y]))
			position.append(array([pose_2.x, pose_2.y]))
			position.append(array([pose_3.x, pose_3.y]))
		except AttributeError as e:
			rospy.logwarn(f"Error getting robot positions: {e}")
			rate.sleep()
			continue

		# 计算信息增益
		infoGain = []
		for ip in range(0, len(centroids)):
			infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))

		revenue_record = []
		centroid_record = []
		id_record = []

		# 计算每个机器人对每个前沿点的收益
		for i in range(3):
			for j in range(0, len(centroids)):
				try:
					cost = norm(position[i] - centroids[j])
					information_gain = infoGain[j]

					if norm(position[i] - centroids[j]) <= hysteresis_radius:
						information_gain *= hysteresis_gain

					revenue = information_gain * info_multiplier - cost

					revenue_record.append(revenue)
					centroid_record.append(centroids[j])
					id_record.append(i)
				except Exception as e:
					rospy.logwarn(f"Error calculating revenue: {e}")
					continue

		if len(revenue_record) == 0:
			rospy.logwarn("No valid revenue calculations")
			rate.sleep()
			continue

		# 打印调试信息
		rospy.loginfo("revenue record: " + str(revenue_record))
		rospy.loginfo("centroid record: " + str(centroid_record))
		rospy.loginfo("robot IDs record: " + str(id_record))

		temp1 = [];temp2 = [];temp3 = []
		cen_temp1 = [];cen_temp2 = [];cen_temp3 = []


		for k in range(len(id_record)):
			if(id_record[k] == 0):
				temp1.append(revenue_record[k])
				cen_temp1.append(centroid_record[k])
				winner_id=temp1.index(max(temp1))

				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose.position.x = cen_temp1[winner_id][0]
				goal.target_pose.pose.position.y = cen_temp1[winner_id][1]
				
				goal.target_pose.pose.orientation.w = 1.0

				rospy.loginfo(f"Sending client_1 goal: {goal.target_pose.pose.position.x}, {goal.target_pose.pose.position.y}")
				client_1.send_goal(goal)
				rospy.loginfo("robot_1 got a new plan")

			elif(id_record[k] == 1):
				temp2.append(revenue_record[k])
				cen_temp2.append(centroid_record[k])
				winner_id=temp2.index(max(temp2))

				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose.position.x = cen_temp2[winner_id][0]
				goal.target_pose.pose.position.y = cen_temp2[winner_id][1]
				
				goal.target_pose.pose.orientation.w = 1.0

				rospy.loginfo(f"Sending client_2 goal: {goal.target_pose.pose.position.x}, {goal.target_pose.pose.position.y}")
				client_2.send_goal(goal)
				rospy.loginfo("robot_2 got a new plan")

			elif(id_record[k] == 2):
				temp3.append(revenue_record[k])
				cen_temp3.append(centroid_record[k])
				winner_id=temp3.index(max(temp3))

				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose.position.x = cen_temp3[winner_id][0]
				goal.target_pose.pose.position.y = cen_temp3[winner_id][1]
				
				goal.target_pose.pose.orientation.w = 1.0

				rospy.loginfo(f"Sending client_3 goal: {goal.target_pose.pose.position.x}, {goal.target_pose.pose.position.y}")
				client_3.send_goal(goal)
				rospy.loginfo("robot_3 got a new plan")
		

		rate.sleep()

if __name__ == '__main__':
    try:
        assign_node()
    except rospy.ROSInterruptException:
        pass