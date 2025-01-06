#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from frontier_based_exploration.msg import RobotState
from geometry_msgs.msg import Point
import torch
from torch import nn
import cv2

class MultiRobotEnv:
    def __init__(self, n_robots, use_frontier=True):
        self.n_robots = n_robots
        self.use_frontier = use_frontier
        
        # 状态空间维度
        self.robot_state_dim = 5  # x, y, theta, linear_vel, angular_vel
        self.global_map_feature_dim = 128  # 全局地图特征
        self.global_frontier_feature_dim = 64  # 全局前沿特征
        self.exploration_rate = 0.0  # 探索率
        
        # 初始化地图和状态
        self.global_map = None
        self.local_maps = [None] * n_robots
        self.robot_states = [None] * n_robots
        self.local_frontiers = [None] * n_robots
        self.global_frontiers = None
        
        # 检查是否有可用的 GPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Using device: {self.device}")
        
        # 创建地图编码器（固定输入尺寸 384x384）
        self.map_encoder = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=8, stride=4, padding=2),  # -> 32x96x96
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1),  # -> 64x48x48
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),  # -> 64x24x24
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((8, 8)),  # -> 64x8x8 = 4096
            nn.Flatten(),  # -> 4096
            nn.Linear(4096, self.global_map_feature_dim)  # -> 64
        ).to(self.device)
        
        # 创建前沿地图编码器（与地图编码器相同的结构）
        self.frontier_encoder = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=8, stride=4, padding=2),  # -> 32x96x96
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1),  # -> 64x48x48
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),  # -> 64x24x24
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((8, 8)),  # -> 64x8x8 = 4096
            nn.Flatten(),  # -> 4096
            nn.Linear(4096, self.global_frontier_feature_dim)  # -> 32
        ).to(self.device)
        
        # 修改激光雷达数据存储
        self.n_laser_sectors = 36  # 将360度分成36个扇区，每个扇区10度
        self.laser_sectors = [np.full(self.n_laser_sectors, float('inf'))] * n_robots
        self.min_obstacle_distance = [float('inf')] * n_robots
        
        # 碰撞检测参数
        self.collision_threshold = 0.3    # 与其他机器人的碰撞阈值
        self.obstacle_threshold = 0.25    # 与障碍物的碰撞阈值
        self.warning_threshold = 0.5      # 警告距离阈值
        
    
    def _encode_map(self, map_data):
        """编码地图数据"""
        if map_data is None:
            return torch.zeros(self.global_map_feature_dim).to(self.device)
        
        # 确保地图数据是正确的形状
        if isinstance(map_data, np.ndarray):
            # 归一化到 [-1, 1]
            map_data = map_data.astype(np.float32) / 100.0
        else:
            # 如果是 OccupancyGrid 消息
            map_data = np.array(map_data.data).reshape(
                map_data.info.height, map_data.info.width).astype(np.float32) / 100.0
        
        # 添加批次和通道维度
        map_tensor = torch.FloatTensor(map_data).unsqueeze(0).unsqueeze(0).to(self.device)
        
        # 编码并分离梯度
        with torch.no_grad():
            return self.map_encoder(map_tensor).detach()
    
    def _encode_frontier(self, frontier_data):
        """编码前沿地图数据"""
        if frontier_data is None:
            return torch.zeros(self.global_frontier_feature_dim).to(self.device)
        
        # 确保前沿数据是正确的形状
        if isinstance(frontier_data, np.ndarray):
            # 二值化
            frontier_data = (frontier_data > 0).astype(np.float32)
        else:
            # 如果是 OccupancyGrid 消息
            frontier_data = (np.array(frontier_data.data).reshape(
                frontier_data.info.height, frontier_data.info.width) > 0).astype(np.float32)
        
        # 添加批次和通道维度
        frontier_tensor = torch.FloatTensor(frontier_data).unsqueeze(0).unsqueeze(0).to(self.device)
        
        # 编码并分离梯度
        with torch.no_grad():
            return self.frontier_encoder(frontier_tensor).detach()
    
    def encode_global_map(self, global_map):
        """编码全局地图"""
        if global_map is None:
            return torch.zeros(self.global_map_feature_dim).to(self.device)
        
        # 使用与局部地图相同的编码器
        return self._encode_map(global_map)
    
    def encode_local_map(self, local_map):
        """编码局部地图"""
        if local_map is None:
            return torch.zeros(self.global_map_feature_dim).to(self.device)
        
        # 使用统一的编码方法
        return self._encode_map(local_map)
    
    def encode_frontier(self, frontier_map):
        """编码前沿地图"""
        if frontier_map is None:
            return torch.zeros(self.global_frontier_feature_dim).to(self.device)
        
        # 使用统一的前沿编码方法
        return self._encode_frontier(frontier_map)
    
    # def encode_local_frontier(self, frontier_map):
    #     """将局部前沿地图编码为特征向量"""
    #     if frontier_map is None:
    #         return np.zeros(32)  # 返回固定维度
        
    #     # 如果输入是 OccupancyGrid 消息
    #     if hasattr(frontier_map, 'info'):
    #         width = frontier_map.info.width
    #         height = frontier_map.info.height
    #         data = np.array(frontier_map.data).reshape(height, width)
    #     else:
    #         # 如果输入已经是 numpy 数组
    #         data = frontier_map
        
    #     # 直接使用原始大小的地图，让网络自适应处理
    #     x = torch.FloatTensor(data).unsqueeze(0).unsqueeze(0).to(self.device)
        
    #     # 使用编码器
    #     with torch.no_grad():
    #         return self.frontier_encoder(x).detach().cpu().numpy().flatten()
    
    # def encode_global_frontier(self, frontier_map):
    #     """使用CNN编码全局前沿地图"""
    #     if frontier_map is None:
    #         return np.zeros(self.global_frontier_feature_dim)
        
    #     # 如果输入是 OccupancyGrid 消息
    #     if hasattr(frontier_map, 'info'):
    #         width = frontier_map.info.width
    #         height = frontier_map.info.height
    #         data = np.array(frontier_map.data).reshape(height, width)
    #     else:
    #         # 如果输入已经是 numpy 数组
    #         data = frontier_map
        
    #     # 直接使用原始大小的地图，让网络自适应处理
    #     x = torch.FloatTensor(data).unsqueeze(0).unsqueeze(0).to(self.device)
    #     return self.frontier_encoder(x).detach().cpu().numpy().flatten()
    
    def get_state(self, robot_id):
        """获取单个机器人的状态"""
        # 获取机器人状态
        robot_state = self.robot_states[robot_id]
        if robot_state is None:
            return np.zeros(self.get_state_dim())
        
        # 基本状态：x, y, theta, linear_vel, angular_vel
        basic_state = np.array([
            robot_state.x,
            robot_state.y,
            robot_state.theta,
            robot_state.linear_velocity,
            robot_state.angular_velocity
        ], dtype=np.float32)
        
        # 编码地图特征并确保是一维的
        global_map_feature = self.encode_global_map(self.global_map).cpu().numpy().flatten()
        local_map_feature = self.encode_local_map(self.local_maps[robot_id]).cpu().numpy().flatten()
        
        # 如果使用前沿信息
        if self.use_frontier:
            frontier_feature = self.encode_frontier(self.local_frontiers[robot_id]).cpu().numpy().flatten()
            # 获取激光雷达状态
            laser_state = self.get_laser_state(robot_id)
            
            # 确保所有特征都是一维的
            features = [
                basic_state,
                global_map_feature,
                local_map_feature,
                frontier_feature,
                laser_state
            ]
            
            # 打印调试信息
            # for i, f in enumerate(features):
            #     rospy.loginfo(f"Feature {i} shape: {f.shape}")
            
            # 组合所有特征
            return np.concatenate(features)
        else:
            # 不使用前沿信息时的状态
            laser_state = self.get_laser_state(robot_id)
            
            # 确保所有特征都是一维的
            features = [
                basic_state,
                global_map_feature,
                local_map_feature,
                laser_state
            ]
            
            # 打印调试信息
            # for i, f in enumerate(features):
            #     rospy.loginfo(f"Feature {i} shape: {f.shape}")
            
            return np.concatenate(features)
    
    def get_all_states(self):
        """获取所有机器人的状态向量"""
        return [self.get_state(i) for i in range(self.n_robots)]
    
    def step(self, actions):
        """执行动作并返回奖励"""
        rewards = []
        info = {
            'collision': False,
            'exploration_rates': [],
            'frontier_counts': [],
            'collisions': [],
            'robot_positions': [],
            'distances': []
        }
        
        for i, action in enumerate(actions):
            # 获取当前机器人状态
            robot_state = self.robot_states[i]
            if robot_state is None:
                continue
            
            # 记录机器人位置
            current_pos = np.array([robot_state.x, robot_state.y])
            info['robot_positions'].append(current_pos)
            
            # 计算与其他机器人的距离
            robot_distances = []
            for j in range(self.n_robots):
                if i != j and self.robot_states[j] is not None:
                    other_pos = np.array([self.robot_states[j].x, self.robot_states[j].y])
                    distance = np.linalg.norm(current_pos - other_pos)
                    robot_distances.append(distance)
                    if distance < 0.3:  # 碰撞检测
                        info['collision'] = True
                        info['collisions'].append((i, j))
            info['distances'].append(robot_distances)
            
            # 计算探索率
            if self.local_maps[i] is not None:
                explored_cells = np.sum(self.local_maps[i] != -1)
                total_cells = self.global_map.shape[0] * self.global_map.shape[1]
                exploration_rate = explored_cells / total_cells
                info['exploration_rates'].append(exploration_rate)
                rospy.loginfo(f"Explored cells for robot {i}: {explored_cells}")
                rospy.loginfo(f"total_cells: {total_cells}")
            
            # 计算前沿点数量
            if self.local_frontiers[i] is not None:
                # 将OccupancyGrid消息转换为numpy数组
                frontier_data = np.array(self.local_frontiers[i].data)
                frontier_count = np.sum(frontier_data > 0)
                info['frontier_counts'].append(frontier_count)
            
            # 计算奖励
            exploration_reward = self._calculate_exploration_reward(i)
            collision_penalty = self._calculate_collision_penalty(i)
            energy_penalty = self._calculate_energy_penalty(action)

            rospy.loginfo(f"Reward for robot {i}: exploration_reward={exploration_reward}, collision_penalty={collision_penalty}, energy_penalty={energy_penalty}")
            
            reward = exploration_reward - collision_penalty - energy_penalty
            rewards.append(reward)
        
        # 检查是否完成探索任务
        done = self._check_exploration_complete()
        
        return self.get_all_states(), rewards, done, info
    
    def _calculate_exploration_reward(self, robot_id):
        """计算探索奖励，基于新发现的未知区域大小"""
        if self.local_maps[robot_id] is None:
            return 0.0
        
        # 获取当前局部地图
        current_map = self.local_maps[robot_id]
        
        # 计算不同类型的格子数量
        unknown_cells = np.sum(current_map == -1)  # 未知区域
        free_cells = np.sum(current_map == 0)      # 空闲区域
        occupied_cells = np.sum(current_map == 100) # 障碍物区域
        
        # 计算探索覆盖率变化（只考虑已知区域）
        total_cells = self.global_map.shape[0] * self.global_map.shape[1]
        explored_cells = free_cells + occupied_cells  # 已探索的格子 = 空闲 + 占用
        
        previous_exploration_rate = self.exploration_rate
        current_exploration_rate = explored_cells / total_cells
        exploration_rate_change = current_exploration_rate - previous_exploration_rate
        
        # 更新探索覆盖率
        self.exploration_rate = current_exploration_rate
        
        # 计算奖励
        base_reward = exploration_rate_change * 100  # 放大奖励
        
        return base_reward
    
    def _calculate_collision_penalty(self, robot_id):
        """使用扇区化的激光雷达数据计算碰撞惩罚"""
        if self.robot_states[robot_id] is None:
            return 0.0
        
        collision_penalty = 0.0
        current_robot = self.robot_states[robot_id]
        current_pos = np.array([current_robot.x, current_robot.y])
        
        # 1. 检测与其他机器人的碰撞
        for i in range(self.n_robots):
            if i != robot_id and self.robot_states[i] is not None:
                other_pos = np.array([self.robot_states[i].x, self.robot_states[i].y])
                distance = np.linalg.norm(current_pos - other_pos)
                
                if distance < self.collision_threshold:
                    collision_penalty = -200.0
                    # rospy.logwarn(f"Robot {robot_id} collision with Robot {i}! Distance: {distance:.2f}")
                    break
                elif distance < self.warning_threshold:
                    collision_penalty = min(collision_penalty, 
                                         -100.0 * (self.warning_threshold/distance))
        
        # 2. 使用扇区化的激光雷达数据检测障碍物
        if self.laser_sectors[robot_id] is not None:
            sectors = self.laser_sectors[robot_id]
            # 找出最危险的扇区
            min_distance = np.min(sectors)
            if min_distance < self.obstacle_threshold:
                collision_penalty = -150.0
                danger_sector = np.argmin(sectors)
                danger_angle = danger_sector * (360.0 / self.n_laser_sectors)
                # rospy.logwarn(f"Robot {robot_id} collision risk! Distance: {min_distance:.2f}m, "f"Angle: {danger_angle:.1f}°")
            elif min_distance < self.warning_threshold:
                collision_penalty = min(collision_penalty,
                                     -75.0 * (self.warning_threshold/min_distance))
        
        # # 3. 考虑运动状态
        # if collision_penalty < 0:
        #     speed = np.sqrt(current_robot.linear_velocity**2 + 
        #                    current_robot.angular_velocity**2)
        #     collision_penalty *= (1 + speed)
        
        return -collision_penalty
    
    def _calculate_energy_penalty(self, action):
        """计算能量消耗惩罚，鼓励高线速度，惩罚大角速度"""
        linear_vel, angular_vel = action
        
        # 线速度奖励：速度越大奖励越大，但有上限
        linear_reward = 0.1 * (1.0 - np.exp(-2.0 * abs(linear_vel)))  # 使用指数函数使奖励平滑
        
        # 角速度惩罚：角速度越大惩罚越大
        # 使用二次函数使小角速度惩罚较小，大角速度惩罚迅速增加
        angular_penalty = 0.05 * (angular_vel ** 2)
        
        # 当线速度很小而角速度很大时，增加额外惩罚（避免原地打转）
        if abs(linear_vel) < 0.1 and abs(angular_vel) > 0.5:
            spinning_penalty = 0.1
        else:
            spinning_penalty = 0.0
        
        # 总的能量消耗评估
        energy_value = -linear_reward + angular_penalty + spinning_penalty
        
        return energy_value
    
    def get_state_dim(self):
        """获取状态空间维度"""
        # 基本状态维度
        dim = self.robot_state_dim  # x, y, theta, linear_vel, angular_vel
        
        # 地图特征维度
        dim += self.global_map_feature_dim  # 全局地图特征
        dim += self.global_map_feature_dim  # 局部地图特征
        
        # 前沿特征维度（如果使用）
        if self.use_frontier:
            dim += self.global_frontier_feature_dim
        
        # 激光雷达状态维度
        dim += self.n_laser_sectors
        
        rospy.loginfo(f"State dimension: {dim}")
        return dim
    
    def get_robot_exploration_rate(self, robot_id):
        """获取单个机器人的探索覆盖率"""
        if self.local_maps[robot_id] is None:
            return 0.0
        
        # 计算已知区域（空闲 + 占用）的数量
        free_cells = np.sum(self.local_maps[robot_id] == 0)
        occupied_cells = np.sum(self.local_maps[robot_id] == 100)
        explored_cells = free_cells + occupied_cells
        
        total_cells = self.global_map.shape[0] * self.global_map.shape[1]
        return explored_cells / total_cells
    
    def _check_exploration_complete(self):
        """检查是否完成探索任务"""
        if self.global_map is None:
            return False
        
        # 计算全局地图中已知区域的比例
        free_cells = np.sum(self.global_map == 0)
        occupied_cells = np.sum(self.global_map == 100)
        explored_cells = free_cells + occupied_cells
        
        total_cells = self.global_map.shape[0] * self.global_map.shape[1]
        exploration_rate = explored_cells / total_cells

        rospy.loginfo(f"Exploration rate: {exploration_rate:.2f}")
        # 如果探索率超过95%，认为任务完成
        return exploration_rate > 0.4
    
    def laser_scan_callback(self, msg, robot_id):
        """处理激光雷达数据，将360度分成n_laser_sectors个扇区"""
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # 计算每个扇区的角度范围
        sector_size = 2 * np.pi / self.n_laser_sectors
        sectors = np.full(self.n_laser_sectors, float('inf'))
        
        for i, r in enumerate(ranges):
            if not np.isfinite(r):
                continue
            
            # 计算当前激光束的角度
            angle = angle_min + i * angle_increment
            # 将角度归一化到[0, 2π]
            angle = angle % (2 * np.pi)
            # 确定扇区索引
            sector_idx = int(angle / sector_size)
            # 更新扇区的最小距离
            sectors[sector_idx] = min(sectors[sector_idx], r)
        
        self.laser_sectors[robot_id] = sectors
        self.min_obstacle_distance[robot_id] = np.min(sectors)
        
        # 检测是否有危险的障碍物
        dangerous_sectors = np.where(sectors < self.obstacle_threshold)[0]
        if len(dangerous_sectors) > 0:
            angles = dangerous_sectors * sector_size * 180 / np.pi  # 转换为角度
            # rospy.logwarn(f"Robot {robot_id} detecting obstacles at angles: {angles}")
    
    def get_laser_state(self, robot_id):
        """获取简化的激光雷达状态表示"""
        if self.laser_sectors[robot_id] is None:
            return np.zeros(self.n_laser_sectors)
        
        # 将距离值归一化到[0,1]范围
        normalized_sectors = np.clip(self.laser_sectors[robot_id] / 5.0, 0, 1)
        return normalized_sectors 