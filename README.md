# ROSELF

**Roself** 是一个基于 Python 的 **交互式 ROS1 Topic 浏览工具**，支持在命令行界面（TUI）中方便地查看、筛选、钻取和实时监控话题数据。

Roself is a Python-based interactive ROS1 Topic Browser that allows you to conveniently explore, filter, drill down, and monitor topic data in real-time from the terminal (TUI).




## 运行环境 / Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+




## 下载与使用 / Download And Use
### Pip（推荐）:
```shell
pip install --user roself
```

#### 使用pip安装的运行方式 / Run(use Pip)


```shell
source /opt/ros/noetic/setup.bash
source <your_custom_message_package>/devel/setup.bash    # custom message.
roself

# rosbag play：
roself -b your_rosbag.bag
```



### 代码安装
```shell
git clone https://github.com/mesakas/roself.git
```



#### 使用代码的运行方式 / Run(clone code)


```shell
source /opt/ros/noetic/setup.bash
source <your_custom_message_package>/devel/setup.bash    # custom message.
python3 roself.py

# rosbag play：
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




## 特性 / Features

- **话题列表浏览**
  - 自动列出 ROS master 中所有话题，支持自定义消息（需要source <你的消息>/setup.bash）
  - 支持上下选择、左右翻页、关键词筛选
  - 显示话题名称与类型
- **支持对rosbag的播放和控制**
  - 可以播放任意自定义消息的rosbag包，只需要source消息后运行即可
  - 可以运行时控制播放的位置，就像音乐播放器那样
- **消息详情浏览**
  - 任意进入话题，实时显示最新消息
  - 支持**嵌套字段钻取**（递归展开）
  - 每个字段显示 **Name | Type | Value**
  - 自动计算并显示消息频率 Hz
- **实时数值曲线**
  - 选中数值字段，按 **Enter** 预览单条曲线
  - 曲线采用 ASCII/Unicode 火花线实时渲染，直接在命令行显示
  - 显示数值范围 `[min, max]`
- **日志/输出捕获**
  - 内置日志窗口，捕获程序输出和错误
  - 支持底部/右侧布局切换（F2）、大小调整（+/-）、滚动查看（PgUp/PgDn）、清空（l）
- **完全命令行**
  - 无需 GUI / rqt / matplotlib 窗口，只需要一个命令行窗口，即可使用所有功能
  - 适合服务器、远程 SSH 环境







- **Topic List Browser**
  - Automatically list all topics from the ROS master, including custom message types (requires `source <your_msgs>/setup.bash`)
  - Navigate with ↑/↓, paginate with ←/→, filter by keywords
  - Display topic name and type
- **Bag Playback and Control**
  - Play any rosbag file, including those containing custom message types (just `source` your message package before running)
  - Full playback control at runtime, just like a media player: play/pause, step forward/backward, seek by percentage, and adjust step size
- **Message Detail Viewer**
  - Subscribe to any topic and view the latest message in real-time
  - Support for **nested field drill-down** (recursive expansion)
  - Display each field as **Name | Type | Value**
  - Automatically estimate and display message frequency (Hz)
- **Realtime Numeric Plots**
  - Select a numeric field and press **Enter** to preview a single curve
  - Rendered directly in the terminal using ASCII/Unicode sparklines
  - Shows numeric range `[min, max]`
- **Log / Output Capture**
  - Built-in log panel to capture program output and errors
  - Toggle layout (bottom/right) with **F2**
  - Resize with (+/-), scroll with (PgUp/PgDn), clear with (l)
- **Fully Command-Line Based**
  - No GUI / rqt / matplotlib windows required
  - Everything works directly in the terminal
  - Ideal for servers and remote SSH sessions
















