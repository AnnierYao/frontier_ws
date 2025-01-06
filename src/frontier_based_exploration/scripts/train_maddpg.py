#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from frontier_based_exploration.msg import RobotState, PointArray
from sensor_msgs.msg import LaserScan
from multi_robot_env import MultiRobotEnv
from geometry_msgs.msg import Twist
from MADDPG import MADDPG
import torch
import os
from torch.utils.tensorboard import SummaryWriter
from datetime import datetime
import matplotlib.pyplot as plt

class Trainer:
    def __init__(self):
        rospy.init_node('maddpg_trainer')
        
        # 从参数服务器获取参数
        self.n_robots = rospy.get_param('~n_robots', 3)
        
        # 添加数据缓存用于可视化
        self.viz_data = {
            'local_map': None,
            'local_frontier': None,
            'global_map': None,
            'global_frontier': None
        }
        
        # 添加可视化更新标志
        self.need_viz_update = False
        
        # 添加matplotlib图形初始化
        plt.ion()  # 开启交互模式
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 10))
        plt.show(block=False)
        
        # 训练参数
        self.max_episodes = rospy.get_param('~max_episodes', 10000)
        self.max_steps = rospy.get_param('~max_steps', 1000)
        self.batch_size = rospy.get_param('~batch_size', 64)
        self.memory_capacity = rospy.get_param('~memory_capacity', 1000000)
        self.episodes_before_train = rospy.get_param('~episodes_before_train', 0)
        self.update_interval = rospy.get_param('~update_interval', 1)
        self.save_interval = rospy.get_param('~save_interval', 100)
        
        # 模型保存路径
        self.model_path = rospy.get_param('~model_path', 'trained_models/')
        
        # 速度限制
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', 0.5)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 1.0)
        
        # 获取机器人命名空间列表
        self.robot_namespaces = rospy.get_param('~robot_namespaces', 
                                              [f'robot_{i+1}' for i in range(self.n_robots)])
        
        # 话题前缀
        self.global_map_topic = rospy.get_param('~global_map_topic', '/map')
        self.cmd_vel_prefix = rospy.get_param('~cmd_vel_prefix', 'cmd_vel')
        self.local_map_prefix = rospy.get_param('~local_map_prefix', 'local_map')
        self.robot_state_prefix = rospy.get_param('~robot_state_prefix', 'robot_state')
        self.frontiers_prefix = rospy.get_param('~frontiers_prefix', 'frontiers')
        
        # 添加前沿点使用配置
        self.use_frontier = rospy.get_param('~use_frontier', True)
        rospy.loginfo(f"Using frontier in state: {self.use_frontier}")
        
        # 环境参数
        self.env = MultiRobotEnv(self.n_robots, self.use_frontier)
        
        # 为每个机器人创建数据缓存
        self.robot_data = {i: {
            'local_map': None,
            'state': None,
            'frontiers': None
        } for i in range(self.n_robots)}
        
        # 等待数据就绪
        self.setup_subscribers()
        
        # 创建定时器，每100ms更新一次图形
        self.viz_timer = rospy.Timer(rospy.Duration(0.1), self.update_visualization)
        
        self.wait_for_data()

        rospy.loginfo(f"self.local_maps.shape: {self.env.local_maps[0].shape}")
        
        # 初始化MADDPG
        self.setup_maddpg()
        
        # 创建发布器用于发送动作命令
        self.cmd_vel_pubs = []
        for i in range(self.n_robots):
            pub = rospy.Publisher(f'/robot_{i+1}/cmd_vel', Twist, queue_size=10)
            self.cmd_vel_pubs.append(pub)
        
        # 创建 TensorBoard writer
        current_time = datetime.now().strftime('%Y%m%d-%H%M%S')
        self.log_dir = os.path.join(self.model_path, 'runs', current_time)
        self.writer = SummaryWriter(log_dir=self.log_dir)
        rospy.loginfo(f"TensorBoard logs will be saved to: {self.log_dir}")
    
    def setup_maddpg(self):
        """初始化MADDPG模型"""
        # 获取状态和动作空间维度
        state_dim = self.env.get_state_dim()  # 使用新方法获取状态维度
        action_dim = 2  # 线速度和角速度
        
        # rospy.loginfo(f"State dimension: {state_dim}")
        # rospy.loginfo(f"Action dimension: {action_dim}")
        
        # 初始化MADDPG
        self.maddpg = MADDPG(
            n_agents=self.n_robots,
            dim_obs=state_dim,
            dim_act=action_dim,
            batch_size=self.batch_size,
            capacity=self.memory_capacity,
            episodes_before_train=self.episodes_before_train
        )
    
    def setup_subscribers(self):
        """设置所有订阅器"""
        for i in range(self.n_robots):
            robot_ns = f'robot_{i+1}'
            
            # 订阅局部地图
            rospy.Subscriber(
                f'/{robot_ns}/local_map', 
                OccupancyGrid, 
                self.local_map_callback,
                callback_args=i
            )
            
            # 订阅机器人状态
            rospy.Subscriber(
                f'/{robot_ns}/robot_state', 
                RobotState, 
                self.robot_state_callback,
                callback_args=i
            )
            
            # 订阅局部前沿地图
            rospy.Subscriber(
                f'/{robot_ns}/frontier_map', 
                OccupancyGrid, 
                self.frontier_callback,
                callback_args=i
            )
            
            # 添加激光雷达订阅器
            rospy.Subscriber(
                f'/{robot_ns}/scan',
                LaserScan,
                self.env.laser_scan_callback,
                callback_args=i
            )
        
        # 订阅全局地图和全局前沿地图
        rospy.Subscriber('/map', OccupancyGrid, self.global_map_callback)
        rospy.Subscriber('/global_frontiers', OccupancyGrid, self.global_frontier_callback)
    
    def execute_action(self, robot_id, action):
        """执行动作并发送到机器人"""
        cmd = Twist()
        cmd.linear.x = float(action[0])  # 线速度
        cmd.angular.z = float(action[1])  # 角速度
        
        # 限制速度范围
        cmd.linear.x = np.clip(cmd.linear.x, -0.5, 0.5)
        cmd.angular.z = np.clip(cmd.angular.z, -1.0, 1.0)
        
        self.cmd_vel_pubs[robot_id].publish(cmd)
    
    def wait_for_data(self):
        """等待所有必要的数据都收到"""
        rospy.loginfo("Waiting for robot data...")
        rate = rospy.Rate(1)  # 1Hz检查
        
        while not rospy.is_shutdown():
            all_ready = True
            for i in range(self.n_robots):
                data = self.robot_data[i]
                # 分别检查每个数据
                if (data['local_map'] is None or 
                    data['state'] is None or 
                    data['frontiers'] is None):
                    all_ready = False
                    rospy.loginfo(f"Waiting for robot {i} data...")
                    break
            
            if all_ready:
                rospy.loginfo("All robot data received!")
                break
            rate.sleep()
    
    def local_map_callback(self, msg, robot_id):
        """处理局部地图数据"""
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        rospy.loginfo(f"Robot {robot_id} local map shape: {map_data.shape}, resolution: {msg.info.resolution}")
        rospy.loginfo(f"Local map origin: x={msg.info.origin.position.x}, y={msg.info.origin.position.y}")
        self.robot_data[robot_id]['local_map'] = map_data
        self.env.local_maps[robot_id] = map_data
        
        # 更新可视化数据（只为第一个机器人）
        if robot_id == 0:
            self.viz_data['local_map'] = map_data
            self.viz_data['local_frontier'] = self.robot_data[robot_id]['frontiers']
            self.viz_data['global_map'] = self.env.global_map
            self.viz_data['global_frontier'] = self.env.global_frontiers
            self.need_viz_update = True
    
    def robot_state_callback(self, msg, robot_id):
        """处理机器人状态数据"""
        self.robot_data[robot_id]['state'] = msg
        self.env.robot_states[robot_id] = msg
        # rospy.loginfo(f"Received state for robot {robot_id}")
    
    def frontier_callback(self, msg, robot_id):
        """处理局部前沿地图数据"""
        if robot_id in self.robot_data:
            if hasattr(msg, 'data') and hasattr(msg, 'info'):
                frontier_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
                rospy.loginfo(f"Robot {robot_id} frontier map shape: {frontier_data.shape}, resolution: {msg.info.resolution}")
                rospy.loginfo(f"Frontier map origin: x={msg.info.origin.position.x}, y={msg.info.origin.position.y}")
            else:
                rospy.logwarn(f"Robot {robot_id} received frontier message without expected attributes")
            
            self.robot_data[robot_id]['frontiers'] = msg
            self.env.local_frontiers[robot_id] = msg
            
            # 更新可视化数据（只为第一个机器人）
            if robot_id == 0:
                self.viz_data['local_map'] = self.robot_data[robot_id]['local_map']
                self.viz_data['local_frontier'] = msg
                self.viz_data['global_map'] = self.env.global_map
                self.viz_data['global_frontier'] = self.env.global_frontiers
                self.need_viz_update = True
    
    def global_map_callback(self, msg):
        """处理全局地图数据"""
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        rospy.loginfo(f"global map shape: {map_data.shape}")
        self.env.global_map = map_data
        
        # 更新可视化数据
        if 0 in self.robot_data:
            self.viz_data['local_map'] = self.robot_data[0]['local_map']
            self.viz_data['local_frontier'] = self.robot_data[0]['frontiers']
            self.viz_data['global_map'] = map_data
            self.viz_data['global_frontier'] = self.env.global_frontiers
            self.need_viz_update = True
    
    def global_frontier_callback(self, msg):
        """处理全局前沿地图数据"""
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        rospy.loginfo(f"global frontier map shape: {map_data.shape}")
        self.env.global_frontiers = map_data
        
        # 更新可视化数据
        if 0 in self.robot_data:
            self.viz_data['local_map_1'] = self.robot_data[0]['local_map']
            self.viz_data['local_frontier_1'] = self.robot_data[0]['frontiers']
            self.viz_data['local_map_2'] = self.robot_data[1]['local_map']
            self.viz_data['local_frontier_2'] = self.robot_data[1]['frontiers']
            self.viz_data['local_map_3'] = self.robot_data[2]['local_map']
            self.viz_data['local_frontier_3'] = self.robot_data[2]['frontiers']
            self.viz_data['global_map'] = self.env.global_map
            self.viz_data['global_frontier'] = map_data
            self.need_viz_update = True
    
    # def create_frontier_map(self, frontier_points):
    #     """将前沿点转换为二值地图"""
    #     frontier_map = np.zeros((self.local_map_size, self.local_map_size))
    #     for point in frontier_points:
    #         x = int(point.x)
    #         y = int(point.y)
    #         if 0 <= x < self.local_map_size and 0 <= y < self.local_map_size:
    #             frontier_map[y, x] = 1
    #     return frontier_map
    
    def train(self):
        """训练MADDPG"""
        rospy.loginfo("Starting training...")
        total_steps = 0
        FloatTensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
        
        # 创建可视化更新的Rate对象
        viz_rate = rospy.Rate(10)  # 10Hz
        
        # 记录训练开始的超参数
        self.writer.add_text('Hyperparameters/n_robots', str(self.n_robots), 0)
        self.writer.add_text('Hyperparameters/batch_size', str(self.batch_size), 0)
        self.writer.add_text('Hyperparameters/max_episodes', str(self.max_episodes), 0)
        self.writer.add_text('Hyperparameters/max_steps', str(self.max_steps), 0)
        
        for episode in range(self.max_episodes):
            episode_reward = 0
            episode_critic_loss = []
            episode_actor_loss = []
            exploration_rates = []
            collision_counts = 0
            frontier_counts = []
            
            # 获取所有机器人的状态并堆叠
            states = self.env.get_all_states()
            states = np.stack(states)
            states = torch.from_numpy(states).float().to(self.maddpg.device)
            
            for step in range(self.max_steps):
                # 联合选择动作
                actions = self.maddpg.select_action(states).data.cpu()
                
                # 记录动作值
                for i in range(self.n_robots):
                    self.writer.add_scalar(f'Actions/Robot_{i}/Linear_Velocity', 
                                         actions[i][0], total_steps)
                    self.writer.add_scalar(f'Actions/Robot_{i}/Angular_Velocity', 
                                         actions[i][1], total_steps)
                
                # 执行动作
                for i in range(self.n_robots):
                    self.execute_action(i, actions[i].numpy())
                
                # 等待执行和状态更新
                rospy.sleep(0.1)
                
                # 获取新状态和奖励
                next_states, rewards, done, info = self.env.step(actions.numpy())
                # rospy.loginfo(f"Reward: {rewards}")

                for i in range(self.n_robots):
                    self.writer.add_scalar(f'Rewards/Robot_{i}', rewards[i], total_steps)
                
                # 记录探索率和前沿点数量
                for i in range(self.n_robots):
                    exploration_rates.append(self.env.exploration_rate)
                    self.writer.add_scalar(f'Exploration/Robot_{i}', self.env.exploration_rate, total_steps)
                    if self.env.local_frontiers[i] is not None:
                        # 将OccupancyGrid消息转换为numpy数组
                        frontier_data = np.array(self.env.local_frontiers[i].data)
                        frontier_counts.append(np.sum(frontier_data > 0))
                
                # 检测碰撞
                if 'collision' in info and info['collision']:
                    collision_counts += 1
                
                # 转换数据格式
                next_states = np.stack(next_states)
                next_states = torch.from_numpy(next_states).float().to(self.maddpg.device)
                rewards = torch.FloatTensor(rewards).to(self.maddpg.device)
                
                # 存储经验
                if step != self.max_steps - 1:
                    self.maddpg.memory.push(states.data, actions, next_states, rewards)
                else:
                    self.maddpg.memory.push(states.data, actions, None, rewards)
                
                # rospy.loginfo(f"memory length: {self.maddpg.memory.__len__()}")
                episode_reward += rewards.sum().item()
                states = next_states
                
                # rospy.loginfo(f"memory length: {self.maddpg.memory.__len__()}")
                # rospy.loginfo(f"batch size: {self.maddpg.batch_size}")
                # 更新策略
                if self.maddpg.memory.__len__() > self.maddpg.batch_size:
                    if total_steps % self.update_interval == 0:
                        critic_loss, actor_loss = self.maddpg.update_policy()
                        rospy.loginfo(f"critic_loss: {critic_loss}")
                        rospy.loginfo(f"actor_loss: {actor_loss}")
                        if critic_loss is not None and actor_loss is not None:
                            episode_critic_loss.extend(critic_loss)
                            episode_actor_loss.extend(actor_loss)
                            
                            # 记录损失
                            for i in range(self.n_robots):
                                self.writer.add_scalar(f'Loss/Critic_Robot_{i}', 
                                                     critic_loss[i], total_steps)
                                self.writer.add_scalar(f'Loss/Actor_Robot_{i}', 
                                                     actor_loss[i], total_steps)
                
                total_steps += 1
                
                # 在主循环中更新可视化
                if self.need_viz_update:
                    self.update_visualization()
                    self.need_viz_update = False
                
                viz_rate.sleep()
                
                if done:
                    break
            
            # 记录每个episode的结果
            self.maddpg.episode_done += 1
            
            # 计算并记录平均值
            if episode_critic_loss:
                # 将列表转换为 PyTorch 张量并计算平均值
                critic_losses = torch.tensor(episode_critic_loss)
                actor_losses = torch.tensor(episode_actor_loss)
                avg_critic_loss = critic_losses.mean().item()
                avg_actor_loss = actor_losses.mean().item()
            else:
                avg_critic_loss = 0
                avg_actor_loss = 0
            
            avg_exploration_rate = np.mean(exploration_rates) if exploration_rates else 0
            avg_frontier_count = np.mean(frontier_counts) if frontier_counts else 0
            
            # 记录到 TensorBoard
            self.writer.add_scalar('Rewards/Episode_Reward', episode_reward, episode)
            self.writer.add_scalar('Metrics/Average_Critic_Loss', avg_critic_loss, episode)
            self.writer.add_scalar('Metrics/Average_Actor_Loss', avg_actor_loss, episode)
            self.writer.add_scalar('Metrics/Exploration_Rate', avg_exploration_rate, episode)
            self.writer.add_scalar('Metrics/Frontier_Count', avg_frontier_count, episode)
            self.writer.add_scalar('Metrics/Collision_Count', collision_counts, episode)
            
            # 记录每个机器人的探索覆盖率
            for i in range(self.n_robots):
                robot_exploration = self.env.get_robot_exploration_rate(i)
                self.writer.add_scalar(f'Exploration/Robot_{i}', robot_exploration, episode)
            
            rospy.loginfo(f"Episode {episode} finished with reward {episode_reward}")
            
            # 保存模型
            if episode % self.save_interval == 0:
                self.save_models(episode)
            
            # 开始训练的提示
            if self.maddpg.episode_done == self.maddpg.episodes_before_train:
                rospy.loginfo("Training now begins...")
        
        # 关闭 TensorBoard writer
        self.writer.close()
    
    def save_models(self, episode):
        """保存模型"""
        try:
            os.makedirs(self.model_path, exist_ok=True)
            for i, actor in enumerate(self.maddpg.actors):
                torch.save(
                    actor.state_dict(), 
                    f"{self.model_path}/actor_{i}_episode_{episode}.pth"
                )
            for i, critic in enumerate(self.maddpg.critics):
                torch.save(
                    critic.state_dict(), 
                    f"{self.model_path}/critic_{i}_episode_{episode}.pth"
                )
            rospy.loginfo(f"Saved models at episode {episode}")
        except Exception as e:
            rospy.logerr(f"Error saving models: {e}")
    
    def update_visualization(self):
        """更新可视化"""
        try:
            # 清除之前的图像
            for ax in self.axes.flat:
                ax.clear()
            
            # 绘制局部地图
            if self.viz_data['local_map_1'] is not None:
                self.axes[0,0].imshow(self.viz_data['local_map_1'], cmap='gray', origin='lower')
                self.axes[0,0].set_title('Local Map')
            
            if self.viz_data['local_map_2'] is not None:
                self.axes[0,1].imshow(self.viz_data['local_map_2'], cmap='gray', origin='lower')
                self.axes[0,1].set_title('Local Map')
            
            if self.viz_data['local_map_3'] is not None:
                self.axes[0,2].imshow(self.viz_data['local_map_3'], cmap='gray', origin='lower')
                self.axes[0,2].set_title('Local Map')
            
            # 绘制局部前沿地图
            if self.viz_data['local_frontier'] is not None:
                if hasattr(self.viz_data['local_frontier_1'], 'data') and hasattr(self.viz_data['local_frontier_1'], 'info'):
                    frontier_data = np.array(self.viz_data['local_frontier_1'].data).reshape(
                        self.viz_data['local_frontier_1'].info.height, 
                        self.viz_data['local_frontier_1'].info.width)
                else:
                    frontier_data = self.viz_data['local_frontier']
                self.axes[1,0].imshow(frontier_data, cmap='hot', origin='lower')
                self.axes[1,0].set_title('Local Frontier')
            
            if self.viz_data['local_frontier_2'] is not None:
                self.axes[1,1].imshow(self.viz_data['local_frontier_2'], cmap='hot', origin='lower')
                self.axes[1,1].set_title('Local Frontier')
            
            if self.viz_data['local_frontier_3'] is not None:
                self.axes[1,2].imshow(self.viz_data['local_frontier_3'], cmap='hot', origin='lower')
                self.axes[1,2].set_title('Local Frontier')
            
            # 绘制全局地图
            if self.viz_data['global_map'] is not None:
                self.axes[2,0].imshow(self.viz_data['global_map'], cmap='gray', origin='lower')
                self.axes[2,0].set_title('Global Map')
            
            # 绘制全局前沿地图
            if self.viz_data['global_frontier'] is not None:
                if hasattr(self.viz_data['global_frontier'], 'data') and hasattr(self.viz_data['global_frontier'], 'info'):
                    frontier_data = np.array(self.viz_data['global_frontier'].data).reshape(
                        self.viz_data['global_frontier'].info.height,
                        self.viz_data['global_frontier'].info.width)
                else:
                    frontier_data = self.viz_data['global_frontier']
                self.axes[2,1].imshow(frontier_data, cmap='hot', origin='lower')
                self.axes[2,1].set_title('Global Frontier')
            
            plt.tight_layout()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            rospy.logwarn(f"Visualization update failed: {e}")
    
    def __del__(self):
        """析构函数，确保正确关闭"""
        if hasattr(self, 'writer'):
            self.writer.close()
        plt.close('all')

if __name__ == '__main__':
    try:
        trainer = Trainer()
        trainer.train()
    except rospy.ROSInterruptException:
        pass