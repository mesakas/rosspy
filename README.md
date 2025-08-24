# ROSSPY

**rosspy** 是一个基于 Python 的 **交互式 ROS1 Topic 浏览工具**，支持在命令行界面（TUI）中方便地查看、筛选、钻取和实时监控话题数据。

rosspy is a Python-based interactive ROS1 Topic Browser that allows you to conveniently explore, filter, drill down, and monitor topic data in real-time from the terminal (TUI).




## 运行环境 / Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+




## 下载 / Download
```shell
git clone https://github.com/mesakas/rosspy.git
```



## 运行 / Run


```shell
source /opt/ros/noetic/setup.bash
source <your_custom_message_package>/devel/setup.bash    # custom message.
python3 rosspy.py

```




## 特性 / Features

- **话题列表浏览**
  - 自动列出 ROS master 中所有话题，支持自定义消息（需要source <你的消息>/setup.bash）
  - 支持上下选择、左右翻页、关键词筛选
  - 显示话题名称与类型
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









