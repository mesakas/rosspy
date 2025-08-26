# ROSELF

Roself 是一个基于 Python 的 交互式 ROS1 / ROS2 Topic 浏览 + Bag 播放工具，在终端（TUI）中即可方便地查看、筛选、钻取和实时监控话题数据，同时支持类似媒体播放器的 rosbag 播放与控制。

Roself is a Python-based interactive ROS1/ROS2 Topic Browser and Bag Player, allowing you to conveniently explore, filter, drill down, and monitor topic data in real-time, as well as control rosbag playback directly from the terminal (TUI).




## 运行环境 / Requirements

- Ubuntu 20.04+ / 22.04+
- ROS1 (Noetic) 或 ROS2 (Foxy+, Humble 等)
- Python 3.8+




## 下载与使用 / Download And Use
### Pip（推荐）:
```shell
pip install --user roself
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

#### 使用pip安装的运行方式 / Run(use Pip)


```shell
# ROS1 环境
source /opt/ros/noetic/setup.bash
source <your_custom_msg>/devel/setup.bash   # 若有自定义消息
roself

# ROS2 环境
source /opt/ros/humble/setup.bash
roself

# 播放 rosbag
roself -b your_rosbag.bag        # ROS1
roself -b your_ros2_bag_folder   # ROS2
```

### 从源码运行
```shell
git clone https://github.com/mesakas/roself.git
cd roself
python3 roself.py -b your_rosbag.bag
```


## 预览
### 首页
<img width="768" height="202" alt="image" src="https://github.com/user-attachments/assets/18d91a03-5d50-4371-9fb6-5fb03c4ddc4c" />

### 消息内容页
<img width="1106" height="559" alt="image" src="https://github.com/user-attachments/assets/35d81da9-f810-46c9-840a-cd6169bec1fe" />

### 分析图表页
<img width="1098" height="557" alt="image" src="https://github.com/user-attachments/assets/dbd1bc93-45a8-4477-8fcc-41b728c6f2a7" />

---

### Ros Bag 支持
新增对播放rosbag的支持：
<img width="1175" height="540" alt="image" src="https://github.com/user-attachments/assets/d2cc96e1-3171-4872-8a86-5234f550889f" />









## 功能 / Features
### 话题浏览 / Topic Browser
- 自动列出所有话题，支持 ROS1 与 ROS2
- 支持上下选择、左右翻页、关键字筛选
- 显示 话题名 | 类型 | Hz
- 显示当前运行的 ROS 版本 与 运行时信息

### 消息详情 / Message Viewer
- 订阅任意话题，实时查看最新消息
- 支持 嵌套字段递归展开
- 每个字段显示 Name | Type | Value
- 自动统计并显示 Hz

### 实时数值曲线 / Realtime Numeric Plot
- 任意选中数值字段 → 按 Enter 打开实时曲线
- 支持多种渲染模式：bars, blocks, envelope, line, braille
- 显示 min/max/avg/cur，右下角保留高精度当前值
- 支持平滑、锁定 Y 轴、缩放窗口

### Rosbag 播放 / Bag Playback
- 支持 ROS1 .bag 与 ROS2 rosbag2 (sqlite3/mcap)
- 完整的播放控制：
- SPACE 播放/暂停
- ← / → 按步长后退/前进
- \+ / - / 0 调整/重置步长
- Shift+←/→ 快速跳转 10%
- L 开关循环（全局或区间）
- Shift+1..9 设置书签；1..9 跳转书签
- C 区间模式：
  - Off → Modify（输入起止点） → On（区间生效）
  - 在 On/Modify 再按一次 C → 清除区间，回到 Off
  - SegMode 状态显示：Off / Modify / On
  - R 回到起始点（若有区间则回到区间起点）

### 日志捕获 / Log Capture
- 内置日志窗，捕获程序输出与错误
- F2 切换底部/右侧
- F3/F4 调整大小
- PgUp/PgDn/Home/End 滚动查看
- o 开关捕获，l 清空

### 完全命令行 / Fully CLI-based
- 无需 GUI / rqt / matplotlib
- 所有功能在终端完成，支持远程 SSH / 服务器环境




