#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/usr/lib/python3/dist-packages')
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading

# --- 辅助函数 ---
def wrap_angle(theta):
    """
    将角度归一化到 [-pi, pi] 范围
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi

def get_vector_angle(v):
    """
    计算二维向量的角度 (atan2)
    """
    if np.linalg.norm(v) < 0.01: return 0.0
    return np.arctan2(v[1], v[0])

# --- 类: 目标运动模拟器 (模拟被围捕的飞机) ---
class TargetMover:
    def __init__(self, drone_id, motion_mode, speed):
        self.drone_id = drone_id
        self.mode = motion_mode
        self.speed = speed
        self.start_time = rospy.Time.now().to_sec()
        self.last_pub_time = 0.0
        self.pub_interval = 1.0 # 增加间隔到 2秒，避免频繁重置规划器
        
        # 发布目标的期望位置
        self.pub = rospy.Publisher(f"/drone_{drone_id}_planning/move_base_simple/goal", PoseStamped, queue_size=1)
        
    def step(self):
        """
        计算下一时刻的目标位置并发布
        """
        now = rospy.Time.now().to_sec()
        if now - self.last_pub_time < self.pub_interval:
            return

        self.last_pub_time = now
        # 预测未来时刻的位置，让规划器有一段长轨迹可飞
        # 预瞄时间设置为发布间隔的 1.5 倍，保证连续性
        lookahead = 2
        t = (now - self.start_time) + lookahead
        
        x, y = 0.0, 0.0
        
        # 根据设定模式生成轨迹
        if self.mode == 'linear': # 匀速直线
            x = -15.0 + self.speed * t
            y = 0.0
        elif self.mode == 'circle': # 圆形轨迹
            R = 10.0
            omega = self.speed / R
            x = R * np.cos(omega * t)
            y = R * np.sin(omega * t)
        elif self.mode == 'parabola': # 抛物线
            vx = self.speed * 0.8
            x = -15.0 + vx * t
            y = 0.04 * x**2 - 5.0
        elif self.mode == 'sine': # 正弦曲线
            vx = self.speed
            x = -15.0 + vx * t
            y = 4.0 * np.sin(0.2 * x)
            
        # 构造并发布 Goal 消息
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 1.0 # 固定高度
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)
        # rospy.loginfo(f"Target Goal Published: [{x:.2f}, {y:.2f}]")

# --- 类: AICG 拦截群 (核心围捕逻辑) ---
class AICGSwarm:
    def __init__(self, interceptor_ids, target_id):
        self.ids = interceptor_ids # 拦截机 ID 列表 [1, 2, 3]
        self.target_id = target_id # 目标 ID 0
        
        # AICG 算法参数
        self.k_ratio = 0.8  # 速度比 V_target / V_interceptor (假设拦截者更快)
        self.capture_radius = 3.0 # 增大围捕半径到 5.0米，减少拥挤和求解失败
        self.comm_topology = 'full' # 通信拓扑 (论文假设动态环形拓扑)
        
        # 状态存储器 {id: {'pos': [x,y], 'vel': [vx,vy]}}
        self.states = {} 
        for i in self.ids + [self.target_id]:
            self.states[i] = None
            
        # 期望角度 (AICG 算法维护的虚拟状态)
        # 初始分布: 0度, 120度, -120度
        self.desired_angles = {
            1: 0.0,
            2: 2.0 * np.pi / 3.0,
            3: -2.0 * np.pi / 3.0
        }
        
        # 上一次发布的 Goal 位置，用于去重
        self.last_goals = {i: np.array([999.0, 999.0]) for i in self.ids}

        # ROS 订阅与发布
        self.subs = []
        self.pubs = {}
        for i in self.ids + [self.target_id]:
            # 订阅里程计信息 (位置/速度)
            self.subs.append(rospy.Subscriber(f"/drone_{i}_visual_slam/odom", Odometry, self.odom_cb, callback_args=i))
            # 仅为拦截机创建 Goal 发布者
            if i in self.ids:
                self.pubs[i] = rospy.Publisher(f"/drone_{i}_planning/move_base_simple/goal", PoseStamped, queue_size=1)

    def odom_cb(self, msg, drone_id):
        """ 里程计回调函数: 更新位置和速度 """
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        self.states[drone_id] = {'pos': pos, 'vel': vel}

    def calculate_escape_zone(self, v_target):
        """
        根据论文公式 (12) 计算逃逸区 U_theta
        返回: 逃逸方向中心 (heading), 逃逸区半宽 (alpha)
        """
        heading = get_vector_angle(v_target)
        
        # 根据阿波罗尼奥斯圆原理，最大逃逸角取决于速度比 k
        # sin(alpha) = k
        k = min(self.k_ratio, 0.99) # 限制 k < 1 保证有解
        alpha = np.arcsin(k)
        
        return heading, alpha 

    def step(self):
        """
        核心控制循环: 计算 AICG 策略并发布指令
        """
        # 1. 检查数据是否就绪
        if any(self.states[i] is None for i in self.ids + [self.target_id]):
            return

        # 2. 获取目标状态
        t_pos = self.states[self.target_id]['pos']
        t_vel = self.states[self.target_id]['vel']
        
        # 3. AICG 核心: 更新期望角度 (自适应区域分配)
        theta_t, alpha = self.calculate_escape_zone(t_vel)
        
        # 根据当前的期望角度对拦截机进行排序，确定动态的“左邻右舍”
        sorted_ids = sorted(self.ids, key=lambda i: self.desired_angles[i])
        N = len(sorted_ids)
        
        updates = {} # 存储角度更新量
        
        for idx in range(N):
            curr_id = sorted_ids[idx]
            left_id = sorted_ids[(idx - 1) % N]
            right_id = sorted_ids[(idx + 1) % N]
            
            # 获取三者的当前期望角度
            q = self.desired_angles[curr_id]
            q_left = self.desired_angles[left_id]
            q_right = self.desired_angles[right_id]
            
            # 计算物理角度间隙 (Physical Gap)
            gap_left = wrap_angle(q - q_left)
            if gap_left < 0: gap_left += 2*np.pi
            
            gap_right = wrap_angle(q_right - q)
            if gap_right < 0: gap_right += 2*np.pi
            
            # --- 自适应加权逻辑 ---
            gap_center_right = wrap_angle(q + gap_right/2.0)
            dist_right = abs(wrap_angle(gap_center_right - theta_t))
            
            gap_center_left = wrap_angle(q - gap_left/2.0)
            dist_left = abs(wrap_angle(gap_center_left - theta_t))
            
            # 降低增益以避免剧烈机动
            K_w = 2.0 # 降低逃逸区吸引力，使变化更平缓
            sigma = alpha if alpha > 0.1 else 0.5 
            
            w_right = 1.0 + K_w * np.exp(-(dist_right**2) / (sigma**2))
            w_left  = 1.0 + K_w * np.exp(-(dist_left**2) / (sigma**2))
            
            err = (w_right * gap_right) - (w_left * gap_left)

            updates[curr_id] = 0.02 * err # 降低步长增益，使轨迹更平滑

        # 应用角度更新
        for i in self.ids:
            self.desired_angles[i] = wrap_angle(self.desired_angles[i] + updates[i])
            
        # 4. 发布最终的拦截目标点
        for i in self.ids:
            ang = self.desired_angles[i]
            
            # 计算拦截点
            gx = t_pos[0] + self.capture_radius * np.cos(ang)
            gy = t_pos[1] + self.capture_radius * np.sin(ang)
            new_goal = np.array([gx, gy])
            
            # 只有当新目标点距离上次目标点足够远时才发布，避免震荡
            dist = np.linalg.norm(new_goal - self.last_goals[i])
            if dist > 0.5: # 0.5米的移动阈值
                self.last_goals[i] = new_goal
                
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "world"
                goal.pose.position.x = gx
                goal.pose.position.y = gy
                goal.pose.position.z = 1.0 
                goal.pose.orientation.w = 1.0
                
                self.pubs[i].publish(goal)


# --- 主节点入口 ---
if __name__ == '__main__':
    try:
        rospy.init_node('aicg_coordinator')
        
        mode = rospy.get_param('~motion_mode', 'circle')
        target_speed = rospy.get_param('~target_speed', 2.0)
        target = TargetMover(drone_id=0, motion_mode=mode, speed=target_speed)
        swarm = AICGSwarm(interceptor_ids=[1, 2, 3], target_id=0)
        
        rospy.loginfo(f"AICG 演示已启动。模式: {mode}")
        
        rate = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            target.step()
            swarm.step()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass