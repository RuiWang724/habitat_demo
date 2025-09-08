# Habitat导航指标演示项目

这是一个基于Habitat-Sim的导航指标计算和传感器性能对比演示项目。该项目通过模拟不同传感器类型（RGB、RGB-D、深度）的导航任务，计算SPL（Success weighted by Path Length）和路径长度等关键指标，并生成高质量的导航演示视频。

## 项目功能

- **导航指标计算**: 计算SPL、路径长度、效率等关键导航指标
- **传感器性能对比**: 比较RGB、RGB-D和深度传感器的导航性能
- **高质量视频生成**: 生成1024x1024分辨率、30fps的导航演示视频
- **传感器质量分析**: 分析图像清晰度、亮度、对比度和深度信息
- **详细报告生成**: 自动生成Markdown格式的对比分析报告

## 依赖包安装

### 1. 创建并激活conda环境

```bash
# 创建新环境
conda create -n habitat_new python=3.8

# 激活环境
conda activate habitat_new
```

### 2. 安装Habitat-Sim

```bash
# 安装Habitat-Sim (CPU版本)
conda install habitat-sim -c conda-forge -c aihabitat

# 或者安装GPU版本 (推荐)
conda install habitat-sim withbullet -c conda-forge -c aihabitat
```

### 3. 安装其他依赖包

```bash
# 基础科学计算包
conda install numpy scipy matplotlib

# 图像处理
pip install opencv-python imageio imageio-ffmpeg

# 其他工具包
pip install tqdm
```

### 4. 完整依赖列表

```bash
# 核心依赖
habitat-sim>=0.2.3
numpy>=1.19.0
opencv-python>=4.5.0
imageio>=2.9.0
imageio-ffmpeg>=0.4.0

# 可选依赖
matplotlib>=3.3.0
scipy>=1.7.0
tqdm>=4.60.0
```

## 项目结构

```
habitat_demo/
├── data/
│   └── scene_datasets/
│       └── habitat-test-scenes/
│           ├── skokloster-castle.glb
│           ├── skokloster-castle.navmesh
│           └── habitat-test-scenes.scene_dataset_config.json
├── navigation_metrics_demo.py          # 主程序文件
├── navigation_metrics_results/         # 结果输出目录
│   ├── navigation_metrics.json        # 详细指标数据
│   ├── comparison_report.md           # 对比分析报告
│   └── videos/                        # 生成的视频文件
│       ├── rgb_navigation_episode_1.mp4
│       ├── rgbd_navigation_episode_1.mp4
│       └── d_navigation_episode_1.mp4
└── README.md                          # 项目说明文档
```

程序运行完成后，结果将保存在 `navigation_metrics_results/` 目录中：

- **JSON数据**: `navigation_metrics.json` - 包含所有详细指标数据
- **分析报告**: `comparison_report.md` - 人类可读的对比分析报告
- **演示视频**: `videos/` 目录下的MP4文件

## 输出说明

### 导航指标

- **SPL (Success weighted by Path Length)**: 成功加权路径长度，范围[0,1]
- **路径长度**: 实际导航路径的总长度
- **最短路径长度**: 起点到终点的直线距离
- **效率**: 最短路径长度与实际路径长度的比值
- **成功率**: 成功到达目标的任务比例

### 传感器对比

程序会对比三种传感器类型：

1. **RGB传感器**: 仅使用彩色图像信息
2. **RGB-D传感器**: 同时使用彩色图像和深度信息
3. **深度传感器**: 仅使用深度信息

### 视频输出

- **分辨率**: 1024x1024像素
- **帧率**: 30fps
- **格式**: MP4
- **内容**: 机器人导航过程中的传感器视角

## 配置参数

可以在 `navigation_metrics_demo.py` 中修改以下参数：

```python
# 导航任务设置
navigation_tasks = [
    (np.array([5.0, 0.0, 5.0]), np.array([-5.0, 0.0, 12.0]))  # (起点, 终点)
]

# 成功半径
success_radius = 1.0  # 米

# 视频设置
settings['height'] = 1024  # 视频高度
settings['width'] = 1024   # 视频宽度
```