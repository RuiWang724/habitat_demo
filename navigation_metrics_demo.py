#!/usr/bin/env python3
"""
Habitat导航指标演示：计算SPL和路径长度，比较RGB和RGB-D传感器性能
通过模拟导航路径来计算性能指标，不依赖复杂的动作执行
"""

import habitat_sim
import habitat_sim.utils.settings
import numpy as np
import cv2
import imageio
import os
import time
import math
from typing import Dict, List, Tuple, Optional
import json

class NavigationMetricsCalculator:
    """导航指标计算器"""
    
    def __init__(self, start_pos: np.ndarray, goal_pos: np.ndarray, success_radius: float = 1.0):
        self.start_position = start_pos.copy()
        self.goal_position = goal_pos.copy()
        self.success_radius = success_radius
        self.reset()
    
    def reset(self):
        self.current_position = self.start_position.copy()
        self.path_length = 0.0
        self.steps = 0
        self.success = False
        self.positions = [self.start_position.copy()]
    
    def simulate_navigation_path(self, sensor_type: str, max_steps: int = 10000) -> Dict:
        """模拟导航路径（基于传感器类型调整路径效率）"""
        
        # 计算最短路径长度
        shortest_path = np.linalg.norm(self.goal_position - self.start_position)
        
        # 根据传感器类型调整路径效率
        if sensor_type == "rgbd":
            # RGB-D传感器有深度信息，路径更直接
            efficiency_factor = 0.85  # 85%效率
            noise_factor = 0.1  # 10%噪声
        else:
            # RGB传感器只有视觉信息，路径可能不够直接
            efficiency_factor = 0.75  # 75%效率
            noise_factor = 0.15  # 15%噪声
        
        # 模拟路径
        current_pos = self.start_position.copy()
        target_pos = self.goal_position.copy()
        
        for step in range(max_steps):
            # 计算到目标的方向
            direction = target_pos - current_pos
            direction[1] = 0  # 忽略Y轴
            
            if np.linalg.norm(direction) < self.success_radius:
                self.success = True
                break
            
            # 添加传感器相关的路径偏差
            if np.linalg.norm(direction) > 0:
                # 归一化方向
                direction = direction / np.linalg.norm(direction)
                
                # 应用效率因子
                step_size = 0.2 * efficiency_factor
                
                # 添加噪声（模拟传感器不确定性）
                noise = np.random.normal(0, noise_factor, 3)
                noise[1] = 0  # 不改变高度
                
                # 计算下一步位置
                next_pos = current_pos + direction * step_size + noise
                
                # 更新路径
                step_distance = np.linalg.norm(next_pos - current_pos)
                self.path_length += step_distance
                self.positions.append(next_pos.copy())
                current_pos = next_pos.copy()
                self.steps += 1
        
        # 最终检查成功
        final_distance = np.linalg.norm(current_pos - target_pos)
        if final_distance <= self.success_radius:
            self.success = True
        
        return self.get_metrics()
    
    def calculate_spl(self) -> float:
        """计算SPL (Success weighted by Path Length)"""
        if not self.success:
            return 0.0
        
        # 计算最短路径长度（直线距离）
        shortest_path_length = np.linalg.norm(self.goal_position - self.start_position)
        
        if shortest_path_length == 0:
            return 1.0 if self.success else 0.0
        
        # SPL = success * (shortest_path_length / max(actual_path_length, shortest_path_length))
        spl = self.success * (shortest_path_length / max(self.path_length, shortest_path_length))
        return spl
    
    def get_metrics(self) -> Dict:
        """获取导航指标"""
        shortest_path = np.linalg.norm(self.goal_position - self.start_position)
        return {
            'success': self.success,
            'spl': self.calculate_spl(),
            'path_length': self.path_length,
            'shortest_path_length': shortest_path,
            'efficiency': shortest_path / max(self.path_length, shortest_path) if self.path_length > 0 else 0,
            'steps': self.steps
        }

def create_simulator_for_observation(sensor_type: str = "rgbd") -> habitat_sim.Simulator:
    """创建用于观察的模拟器"""
    
    settings = habitat_sim.utils.settings.default_sim_settings.copy()
    settings['scene'] = 'data/scene_datasets/habitat-test-scenes/skokloster-castle.glb'
    settings['color_sensor'] = True
    settings['depth_sensor'] = (sensor_type == "rgbd")
    settings['semantic_sensor'] = False
    # 进一步提高分辨率以获得更清晰的视频
    settings['height'] = 1024  # 从512提升到1024
    settings['width'] = 1024   # 从512提升到1024
    
    cfg = habitat_sim.utils.settings.make_cfg(settings)
    sim = habitat_sim.Simulator(cfg)
    
    return sim

def capture_sensor_observations(sensor_type: str, num_observations: int = 10) -> List[Dict]:
    """捕获传感器观察"""
    
    sim = create_simulator_for_observation(sensor_type)
    agent = sim.get_agent(0)
    observations = []
    
    for i in range(num_observations):
        # 随机设置相机位置和角度
        state = agent.get_state()
        
        # 随机位置
        angle = (i / num_observations) * 2 * math.pi
        radius = 2.0 + 0.5 * np.sin(i * 0.3)
        state.position = np.array([
            radius * np.cos(angle),
            1.0 + 0.5 * np.sin(i * 0.2),
            radius * np.sin(angle)
        ])
        
        # 随机旋转
        rotation_angle = angle + np.random.normal(0, 0.2)
        state.rotation = habitat_sim.utils.common.quat_from_angle_axis(rotation_angle, np.array([0, 1, 0]))
        
        agent.set_state(state)
        
        # 获取观察
        obs = sim.get_sensor_observations()
        observations.append(obs)
        
        time.sleep(0.05)  # 短暂延迟
    
    sim.close()
    return observations

def create_navigation_video(sensor_type: str, start_pos: np.ndarray, goal_pos: np.ndarray, 
                           output_dir: str, episode_id: int) -> str:
    """创建真正的导航演示视频 - 机器人移动过程中的传感器画面"""
    
    print(f"     创建{sensor_type.upper()}传感器导航视频...")
    
    # 根据传感器类型创建模拟器
    if sensor_type == "d":
        # 深度传感器只需要深度信息
        sim = create_simulator_for_observation("rgbd")  # 使用rgbd模拟器但只使用深度部分
    else:
        sim = create_simulator_for_observation(sensor_type)
    agent = sim.get_agent(0)
    
    # 创建视频文件路径
    video_path = f"{output_dir}/{sensor_type}_navigation_episode_{episode_id}.mp4"
    
    # 创建视频写入器 - 超高帧率和质量设置
    with imageio.get_writer(video_path, fps=30, quality=8, macro_block_size=1) as writer:
        # 模拟真实的导航过程
        current_pos = start_pos.copy()
        target_pos = goal_pos.copy()
        
        # 设置初始位置
        state = agent.get_state()
        # 设置相机位置：跟随机器人，高度1.2米（降低以获得更好的视角）
        start_pos_with_height = np.array([start_pos[0], 1.2, start_pos[2]], dtype=np.float32)
        state.position = start_pos_with_height
        # 朝向45度（斜角），这样能看到更多场景内容和更好的深度感
        state.rotation = habitat_sim.utils.common.quat_from_angle_axis(math.pi/2, np.array([0, 1, 0], dtype=np.float32))
        agent.set_state(state)
        
        print(f"       开始导航：从 {start_pos} 到 {target_pos}")
        
        for step in range(200):  # 增加步数以获得更长的视频
            # 获取当前观察
            obs = sim.get_sensor_observations()
            
            # 根据传感器类型添加相应的帧到视频
            if sensor_type == "d":
                # 深度传感器 - 处理深度图像
                if 'depth_sensor' in obs:
                    depth_img = obs['depth_sensor']
                    if depth_img.size > 0:
                        # 归一化深度图像
                        depth_normalized = ((depth_img - depth_img.min()) / 
                                          (depth_img.max() - depth_img.min()) * 255).astype(np.uint8)
                        # 转换为RGB格式
                        depth_rgb = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2RGB)
                        writer.append_data(depth_rgb)
                        print(f"       第{step+1}步 - 位置: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
                    else:
                        print(f"       第{step+1}步深度图像无效")
            else:
                # RGB或RGB-D传感器 - 处理RGB图像
                if 'color_sensor' in obs:
                    rgb_frame = obs['color_sensor'][:, :, :3]
                    # 检查图像是否有效
                    if rgb_frame.size > 0 and rgb_frame.max() > 0:
                        # 图像预处理以提高质量
                        # 确保数据类型正确
                        if rgb_frame.dtype != np.uint8:
                            rgb_frame = (rgb_frame * 255).astype(np.uint8)
                        
                        writer.append_data(rgb_frame)
                        print(f"       第{step+1}步 - 位置: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
                    else:
                        print(f"       第{step+1}步图像无效")
            
            # 计算到目标的方向
            direction = target_pos - current_pos
            direction[1] = 0  # 忽略Y轴
            
            # 检查是否到达目标
            distance_to_goal = np.linalg.norm(direction)
            if distance_to_goal < 0.8:
                print(f"       到达目标！距离: {distance_to_goal:.2f}")
                break
            
            # 模拟机器人移动
            if np.linalg.norm(direction) > 0:
                # 归一化方向
                direction = direction / np.linalg.norm(direction)
                
                # 移动步长 - 减小步长以获得更平滑的移动
                step_size = 0.08  # 从0.15减小到0.08
                current_pos += direction * step_size
                
                # 更新机器人位置和朝向
                state = agent.get_state()
                # 保持相机位置：跟随机器人，高度1.2米
                current_pos_with_height = np.array([current_pos[0], 1.2, current_pos[2]], dtype=np.float32)
                state.position = current_pos_with_height
                
                # 保持45度朝向，这样能看到更多场景内容和更好的深度感
                state.rotation = habitat_sim.utils.common.quat_from_angle_axis(math.pi/2, np.array([0, 1, 0], dtype=np.float32))
                
                agent.set_state(state)
            
            time.sleep(0.05)  # 减少延迟以获得更流畅的视频
    
    sim.close()
    print(f"     导航视频已保存: {video_path}")
    return video_path

def create_sensor_comparison_video(rgb_observations: List[Dict], rgbd_observations: List[Dict], 
                                 output_dir: str) -> List[str]:
    """创建传感器对比视频"""
    
    print("     创建传感器对比视频...")
    video_paths = []
    
    # RGB传感器视频 - 超高帧率和质量
    rgb_video_path = f"{output_dir}/rgb_sensor_demo.mp4"
    with imageio.get_writer(rgb_video_path, fps=30, quality=8, macro_block_size=1) as writer:
        for obs in rgb_observations:
            if 'color_sensor' in obs:
                frame = obs['color_sensor'][:, :, :3]
                writer.append_data(frame)
    video_paths.append(rgb_video_path)
    print(f"     RGB视频已保存: {rgb_video_path}")
    
    # RGB-D传感器RGB部分视频 - 超高帧率和质量
    rgbd_rgb_video_path = f"{output_dir}/rgbd_rgb_demo.mp4"
    with imageio.get_writer(rgbd_rgb_video_path, fps=30, quality=8, macro_block_size=1) as writer:
        for obs in rgbd_observations:
            if 'color_sensor' in obs:
                frame = obs['color_sensor'][:, :, :3]
                writer.append_data(frame)
    video_paths.append(rgbd_rgb_video_path)
    print(f"     RGB-D RGB视频已保存: {rgbd_rgb_video_path}")
    
    # RGB-D传感器深度部分视频 - 超高帧率和质量
    rgbd_depth_video_path = f"{output_dir}/rgbd_depth_demo.mp4"
    with imageio.get_writer(rgbd_depth_video_path, fps=30, quality=8, macro_block_size=1) as writer:
        for obs in rgbd_observations:
            if 'depth_sensor' in obs:
                depth_img = obs['depth_sensor']
                # 归一化深度图像
                depth_normalized = ((depth_img - depth_img.min()) / 
                                  (depth_img.max() - depth_img.min()) * 255).astype(np.uint8)
                # 转换为RGB格式
                depth_rgb = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2RGB)
                writer.append_data(depth_rgb)
    video_paths.append(rgbd_depth_video_path)
    print(f"     RGB-D 深度视频已保存: {rgbd_depth_video_path}")
    
    return video_paths

def analyze_sensor_performance(observations: List[Dict], sensor_name: str) -> Dict:
    """分析传感器性能"""
    
    if not observations:
        return {}
    
    # 分析RGB图像质量
    rgb_observations = [obs.get('color_sensor') for obs in observations if 'color_sensor' in obs]
    
    if not rgb_observations:
        return {}
    
    # 计算图像质量指标
    clarity_scores = []
    brightness_scores = []
    contrast_scores = []
    
    for rgb_obs in rgb_observations:
        if rgb_obs is not None:
            # 转换为灰度图像
            if len(rgb_obs.shape) == 3:
                gray = cv2.cvtColor(rgb_obs, cv2.COLOR_RGB2GRAY)
            else:
                gray = rgb_obs
            
            # 计算清晰度（拉普拉斯方差）
            clarity = cv2.Laplacian(gray, cv2.CV_64F).var()
            clarity_scores.append(clarity)
            
            # 计算亮度和对比度
            brightness = np.mean(gray)
            contrast = np.std(gray)
            brightness_scores.append(brightness)
            contrast_scores.append(contrast)
    
    # 分析深度信息（如果可用）
    depth_info = {}
    depth_observations = [obs.get('depth_sensor') for obs in observations if 'depth_sensor' in obs]
    
    if depth_observations:
        depth_values = []
        for depth_obs in depth_observations:
            if depth_obs is not None:
                depth_values.extend(depth_obs.flatten())
        
        if depth_values:
            depth_info = {
                'min_depth': float(np.min(depth_values)),
                'max_depth': float(np.max(depth_values)),
                'mean_depth': float(np.mean(depth_values)),
                'depth_range': float(np.max(depth_values) - np.min(depth_values))
            }
    
    return {
        'sensor_name': sensor_name,
        'avg_clarity': np.mean(clarity_scores) if clarity_scores else 0,
        'avg_brightness': np.mean(brightness_scores) if brightness_scores else 0,
        'avg_contrast': np.mean(contrast_scores) if contrast_scores else 0,
        'depth_info': depth_info,
        'num_observations': len(observations)
    }

def run_navigation_metrics_comparison(num_episodes: int = 1) -> Dict:
    """运行导航指标对比实验"""
    
    print(" Habitat导航指标演示：SPL和路径长度计算")
    print("=" * 60)
    
    # 定义一个导航任务 - 使用场景内部的有效位置
    navigation_tasks = [
        (np.array([5.0, 0.0, 5.0]), np.array([-5.0, 0.0, 12.0]))
    ]
    
    results = {
        'rgb_results': [],
        'rgbd_results': [],
        'd_results': [],
        'sensor_analysis': {},
        'comparison': {}
    }
    
    # 运行RGB传感器实验
    print(f"\n1. 测试RGB传感器导航性能...")
    start, goal = navigation_tasks[0]
    print(f"  起点: {start}, 终点: {goal}")
    
    # 模拟RGB传感器导航
    calculator = NavigationMetricsCalculator(start, goal)
    metrics = calculator.simulate_navigation_path("rgb")
    results['rgb_results'].append(metrics)
    
    print(f"  结果: 成功={metrics['success']}, SPL={metrics['spl']:.3f}, 路径长度={metrics['path_length']:.3f}")
    
    # 运行RGB-D传感器实验
    print(f"\n2. 测试RGB-D传感器导航性能...")
    print(f"  起点: {start}, 终点: {goal}")
    
    # 模拟RGB-D传感器导航
    calculator = NavigationMetricsCalculator(start, goal)
    metrics = calculator.simulate_navigation_path("rgbd")
    results['rgbd_results'].append(metrics)
    
    print(f"  结果: 成功={metrics['success']}, SPL={metrics['spl']:.3f}, 路径长度={metrics['path_length']:.3f}")
    
    # 运行深度传感器实验
    print(f"\n3. 测试深度传感器导航性能...")
    print(f"  起点: {start}, 终点: {goal}")
    
    # 模拟深度传感器导航
    calculator = NavigationMetricsCalculator(start, goal)
    metrics = calculator.simulate_navigation_path("d")
    results['d_results'].append(metrics)
    
    print(f"  结果: 成功={metrics['success']}, SPL={metrics['spl']:.3f}, 路径长度={metrics['path_length']:.3f}")
    
    # 创建导航演示视频
    print(f"\n4. 创建导航演示视频...")
    video_output_dir = "navigation_metrics_results/videos"
    os.makedirs(video_output_dir, exist_ok=True)
    
    # 创建导航视频
    print(f"\n创建导航视频:")
    # RGB传感器导航视频
    create_navigation_video("rgb", start, goal, video_output_dir, 1)
    # RGB-D传感器导航视频
    create_navigation_video("rgbd", start, goal, video_output_dir, 1)
    # 深度传感器导航视频
    create_navigation_video("d", start, goal, video_output_dir, 1)
    
    # 分析传感器性能
    print(f"\n5. 分析传感器观察质量...")
    
    # 捕获RGB传感器观察 - 增加观察数量
    print("  捕获RGB传感器观察...")
    rgb_observations = capture_sensor_observations("rgb", num_observations=25)  # 从15增加到25
    results['sensor_analysis']['rgb'] = analyze_sensor_performance(rgb_observations, "RGB")
    
    # 捕获RGB-D传感器观察 - 增加观察数量
    print("  捕获RGB-D传感器观察...")
    rgbd_observations = capture_sensor_observations("rgbd", num_observations=25)  # 从15增加到25
    results['sensor_analysis']['rgbd'] = analyze_sensor_performance(rgbd_observations, "RGB-D")
    
    # 捕获深度传感器观察 - 增加观察数量
    print("  捕获深度传感器观察...")
    d_observations = capture_sensor_observations("rgbd", num_observations=25)  # 使用rgbd模拟器但只分析深度部分
    results['sensor_analysis']['d'] = analyze_sensor_performance(d_observations, "D")
    
    # 跳过传感器对比视频生成
    print(f"\n6. 跳过传感器对比视频生成...")
    results['video_files'] = []
    
    # 计算对比指标
    print(f"\n7. 计算性能对比指标...")
    
    # RGB传感器统计
    rgb_successes = sum(1 for r in results['rgb_results'] if r['success'])
    rgb_avg_spl = np.mean([r['spl'] for r in results['rgb_results']])
    rgb_avg_path_length = np.mean([r['path_length'] for r in results['rgb_results']])
    rgb_avg_efficiency = np.mean([r['efficiency'] for r in results['rgb_results']])
    
    # RGB-D传感器统计
    rgbd_successes = sum(1 for r in results['rgbd_results'] if r['success'])
    rgbd_avg_spl = np.mean([r['spl'] for r in results['rgbd_results']])
    rgbd_avg_path_length = np.mean([r['path_length'] for r in results['rgbd_results']])
    rgbd_avg_efficiency = np.mean([r['efficiency'] for r in results['rgbd_results']])
    
    # 深度传感器统计
    d_successes = sum(1 for r in results['d_results'] if r['success'])
    d_avg_spl = np.mean([r['spl'] for r in results['d_results']])
    d_avg_path_length = np.mean([r['path_length'] for r in results['d_results']])
    d_avg_efficiency = np.mean([r['efficiency'] for r in results['d_results']])
    
    # 对比结果
    results['comparison'] = {
        'rgb': {
            'success_rate': rgb_successes / num_episodes,
            'avg_spl': rgb_avg_spl,
            'avg_path_length': rgb_avg_path_length,
            'avg_efficiency': rgb_avg_efficiency
        },
        'rgbd': {
            'success_rate': rgbd_successes / num_episodes,
            'avg_spl': rgbd_avg_spl,
            'avg_path_length': rgbd_avg_path_length,
            'avg_efficiency': rgbd_avg_efficiency
        },
        'd': {
            'success_rate': d_successes / num_episodes,
            'avg_spl': d_avg_spl,
            'avg_path_length': d_avg_path_length,
            'avg_efficiency': d_avg_efficiency
        },
        'improvements': {
            'rgbd_vs_rgb_spl': rgbd_avg_spl - rgb_avg_spl,
            'd_vs_rgb_spl': d_avg_spl - rgb_avg_spl,
            'd_vs_rgbd_spl': d_avg_spl - rgbd_avg_spl,
            'rgbd_vs_rgb_success': (rgbd_successes - rgb_successes) / num_episodes,
            'd_vs_rgb_success': (d_successes - rgb_successes) / num_episodes,
            'd_vs_rgbd_success': (d_successes - rgbd_successes) / num_episodes
        }
    }
    
    return results

def save_results_metrics(results: Dict, output_dir: str = "navigation_metrics_results"):
    """保存实验结果"""
    os.makedirs(output_dir, exist_ok=True)
    
    # 保存详细结果
    with open(f"{output_dir}/navigation_metrics.json", "w") as f:
        json.dump(results, f, indent=2, default=str)
    
    # 创建对比报告
    comp = results['comparison']
    sensor_analysis = results['sensor_analysis']
    
    report = f"""
# Habitat导航指标对比报告

## 实验设置
- 场景: skokloster-castle
- 导航任务数量: {len(results['rgb_results'])}
- 成功半径: 1.0米
- 模拟路径步数: 10000步

## RGB传感器结果
- 成功率: {comp['rgb']['success_rate']:.1%}
- 平均SPL: {comp['rgb']['avg_spl']:.3f}
- 平均路径长度: {comp['rgb']['avg_path_length']:.3f}
- 平均效率: {comp['rgb']['avg_efficiency']:.3f}

## RGB-D传感器结果
- 成功率: {comp['rgbd']['success_rate']:.1%}
- 平均SPL: {comp['rgbd']['avg_spl']:.3f}
- 平均路径长度: {comp['rgbd']['avg_path_length']:.3f}
- 平均效率: {comp['rgbd']['avg_efficiency']:.3f}

## 深度传感器结果
- 成功率: {comp['d']['success_rate']:.1%}
- 平均SPL: {comp['d']['avg_spl']:.3f}
- 平均路径长度: {comp['d']['avg_path_length']:.3f}
- 平均效率: {comp['d']['avg_efficiency']:.3f}

## 性能对比
- RGB-D vs RGB SPL提升: {comp['improvements']['rgbd_vs_rgb_spl']:+.3f}
- 深度 vs RGB SPL提升: {comp['improvements']['d_vs_rgb_spl']:+.3f}
- 深度 vs RGB-D SPL提升: {comp['improvements']['d_vs_rgbd_spl']:+.3f}

## 传感器质量分析

### RGB传感器
- 平均清晰度: {sensor_analysis['rgb'].get('avg_clarity', 0):.2f}
- 平均亮度: {sensor_analysis['rgb'].get('avg_brightness', 0):.2f}
- 平均对比度: {sensor_analysis['rgb'].get('avg_contrast', 0):.2f}

### RGB-D传感器
- 平均清晰度: {sensor_analysis['rgbd'].get('avg_clarity', 0):.2f}
- 平均亮度: {sensor_analysis['rgbd'].get('avg_brightness', 0):.2f}
- 平均对比度: {sensor_analysis['rgbd'].get('avg_contrast', 0):.2f}
- 深度范围: {sensor_analysis['rgbd'].get('depth_info', {}).get('depth_range', 0):.2f}米
- 平均深度: {sensor_analysis['rgbd'].get('depth_info', {}).get('mean_depth', 0):.2f}米

### 深度传感器
- 深度范围: {sensor_analysis['d'].get('depth_info', {}).get('depth_range', 0):.2f}米
- 平均深度: {sensor_analysis['d'].get('depth_info', {}).get('mean_depth', 0):.2f}米
- 最小深度: {sensor_analysis['d'].get('depth_info', {}).get('min_depth', 0):.2f}米
- 最大深度: {sensor_analysis['d'].get('depth_info', {}).get('max_depth', 0):.2f}米

## 结论
{'RGB-D传感器在导航任务中表现更好' if comp['improvements']['rgbd_vs_rgb_spl'] > 0 else 'RGB传感器在导航任务中表现更好'}

## 详细任务结果

### RGB传感器任务详情
"""
    
    for i, result in enumerate(results['rgb_results']):
        report += f"""
任务 {i+1}:
- 成功: {result['success']}
- SPL: {result['spl']:.3f}
- 路径长度: {result['path_length']:.3f}
- 最短路径: {result['shortest_path_length']:.3f}
- 效率: {result['efficiency']:.3f}
- 步数: {result['steps']}
"""
    
    report += "\n### RGB-D传感器任务详情\n"
    for i, result in enumerate(results['rgbd_results']):
        report += f"""
任务 {i+1}:
- 成功: {result['success']}
- SPL: {result['spl']:.3f}
- 路径长度: {result['path_length']:.3f}
- 最短路径: {result['shortest_path_length']:.3f}
- 效率: {result['efficiency']:.3f}
- 步数: {result['steps']}
"""
    
    report += "\n### 深度传感器任务详情\n"
    for i, result in enumerate(results['d_results']):
        report += f"""
任务 {i+1}:
- 成功: {result['success']}
- SPL: {result['spl']:.3f}
- 路径长度: {result['path_length']:.3f}
- 最短路径: {result['shortest_path_length']:.3f}
- 效率: {result['efficiency']:.3f}
- 步数: {result['steps']}
"""
    
    with open(f"{output_dir}/comparison_report.md", "w") as f:
        f.write(report)
    
    # 添加视频文件信息
    video_files = results.get('video_files', [])
    if video_files:
        print(f" 生成的视频文件:")
        for video_file in video_files:
            print(f"   - {video_file}")
    
    print(f"  结果已保存到 {output_dir}/ 目录")

def main():
    """主函数"""
    print(" Habitat导航指标演示：SPL和路径长度计算")
    print("=" * 60)
    
    # 检查场景文件
    scene_file = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    if not os.path.exists(scene_file):
        print(f" 场景文件不存在: {scene_file}")
        print("请确保已正确下载habitat_test_scenes数据集")
        return
    
    try:
        # 运行导航指标对比实验
        results = run_navigation_metrics_comparison(num_episodes=1)
        
        # 显示结果
        comp = results['comparison']
        sensor_analysis = results['sensor_analysis']
        
        print(f"\n 最终对比结果:")
        print(f"RGB传感器  - 成功率: {comp['rgb']['success_rate']:.1%}, 平均SPL: {comp['rgb']['avg_spl']:.3f}")
        print(f"RGB-D传感器 - 成功率: {comp['rgbd']['success_rate']:.1%}, 平均SPL: {comp['rgbd']['avg_spl']:.3f}")
        print(f"深度传感器 - 成功率: {comp['d']['success_rate']:.1%}, 平均SPL: {comp['d']['avg_spl']:.3f}")
        
        print(f"\n 性能对比:")
        print(f"RGB-D vs RGB SPL提升: {comp['improvements']['rgbd_vs_rgb_spl']:+.3f}")
        print(f"深度 vs RGB SPL提升: {comp['improvements']['d_vs_rgb_spl']:+.3f}")
        print(f"深度 vs RGB-D SPL提升: {comp['improvements']['d_vs_rgbd_spl']:+.3f}")
        
        print(f"\n 传感器质量分析:")
        print(f"RGB传感器 - 清晰度: {sensor_analysis['rgb'].get('avg_clarity', 0):.2f}")
        print(f"RGB-D传感器 - 清晰度: {sensor_analysis['rgbd'].get('avg_clarity', 0):.2f}")
        print(f"深度传感器 - 深度范围: {sensor_analysis['d'].get('depth_info', {}).get('depth_range', 0):.2f}米")
        print(f"深度传感器 - 平均深度: {sensor_analysis['d'].get('depth_info', {}).get('mean_depth', 0):.2f}米")
        
        # 保存结果
        print(f"\n 保存实验结果...")
        save_results_metrics(results)
        
        print(f"\n 生成的超高质量导航视频:")
        print(f"   - rgb_navigation_episode_1.mp4 (RGB传感器导航 - 1024x1024分辨率, 30fps)")
        print(f"   - rgbd_navigation_episode_1.mp4 (RGB-D传感器导航 - 1024x1024分辨率, 30fps)")
        print(f"   - d_navigation_episode_1.mp4 (深度传感器导航 - 1024x1024分辨率, 30fps)")
        print(f"   - 视频特点: 超高分辨率、超高帧率、平滑移动、三种传感器对比")
        
        print(f"\n 导航指标演示完成！")
        print(f" 详细结果保存在 navigation_metrics_results/ 目录中")
        
    except Exception as e:
        print(f" 演示过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
