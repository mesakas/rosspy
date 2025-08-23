#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_tui_watch.py - ROS Noetic 交互式 TUI 话题浏览（支持自定义消息、嵌套钻取、实时Hz）
- 列表页：↑↓选择、←→翻页、输入筛选、Enter进入、q退出
- 详情页：树形浏览（Enter进入子消息/数组，ESC返回上一层），显示 Name | Type | Value 与 Hz
- 日志窗：F2切换底/右，+/-大小，PgUp/PgDn/Home/End滚动，o开关拦截，l清空
"""

import os
import curses
import curses.ascii
import time
import threading
import traceback
import sys
from collections import deque
from typing import List, Tuple, Optional, Any

import rospy
import rostopic
import roslib.message
import rosgraph              # <<< 新增：使用 rosgraph.Master 访问 rosmaster
from roslib.message import strify_message


def safe_addstr(win, y, x, text, attr=0):
    """向窗口安全写字符串：自动裁剪，越界直接忽略，不抛错。"""
    try:
        if text is None:
            text = ""
        H, W = win.getmaxyx()
        if H <= 0 or W <= 0:
            return
        if y < 0 or y >= H or x < 0 or x >= W:
            return
        maxlen = max(0, W - x)
        if maxlen <= 0:
            return
        win.addnstr(y, x, text, maxlen, attr)
    except curses.error:
        pass

def safe_hline(win, y, x, ch, n):
    """安全画水平线。"""
    try:
        H, W = win.getmaxyx()
        if H <= 0 or W <= 0:
            return
        if y < 0 or y >= H or x < 0 or x >= W:
            return
        n = max(0, min(n, W - x))
        if n <= 0:
            return
        win.hline(y, x, ch, n)
    except curses.error:
        pass



def ensure_ros_node():
    if not rospy.core.is_initialized():
        try:
            rospy.init_node("ros_tui_watch", anonymous=True, disable_signals=True)
        except Exception as e:
            # 没连上 master 也别阻塞，记录日志，后面会重试
            GLOBAL_LOG.append(f"[WARN] init_node failed: {e}")

def get_master_uri() -> str:
    return os.environ.get("ROS_MASTER_URI", "(unset)")

# ====================== 日志收集（分屏） ======================
class LogBuffer:
    def __init__(self, max_lines=5000):
        self.lines = deque(maxlen=max_lines)
        self.lock = threading.Lock()
        self.scroll = 0
        self.capture_enabled = True

    def append(self, text: str):
        if not text:
            return
        with self.lock:
            for ln in text.splitlines():
                self.lines.append(ln)

    def clear(self):
        with self.lock:
            self.lines.clear()
            self.scroll = 0

    def toggle_capture(self):
        with self.lock:
            self.capture_enabled = not self.capture_enabled
            return self.capture_enabled

    def get_view(self, height: int) -> List[str]:
        with self.lock:
            n = len(self.lines)
            if n == 0:
                return []
            start = max(0, n - height - self.scroll)
            end = max(0, n - self.scroll)
            return list(list(self.lines)[start:end])

    def scroll_up(self, n: int = 5):
        with self.lock:
            self.scroll = min(len(self.lines), self.scroll + n)

    def scroll_down(self, n: int = 5):
        with self.lock:
            self.scroll = max(0, self.scroll - n)

    def scroll_home(self):
        with self.lock:
            self.scroll = len(self.lines)

    def scroll_end(self):
        with self.lock:
            self.scroll = 0

GLOBAL_LOG = LogBuffer()

class _StreamToLog:
    def __init__(self, name):
        self.name = name
        self.buf = ""
    def write(self, s):
        if not isinstance(s, str):
            s = str(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            if GLOBAL_LOG.capture_enabled:
                GLOBAL_LOG.append(line)
    def flush(self):
        if self.buf and GLOBAL_LOG.capture_enabled:
            GLOBAL_LOG.append(self.buf)
        self.buf = ""

_ORIG_STDOUT = sys.stdout
_ORIG_STDERR = sys.stderr
sys.stdout = _StreamToLog("stdout")
sys.stderr = _StreamToLog("stderr")

# ====================== ROS 工具 ======================
def master_get_topics() -> List[Tuple[str, str]]:
    """非阻塞查询 master：短超时，失败就返回空列表并记录到日志。"""
    import socket
    old_to = socket.getdefaulttimeout()
    try:
        socket.setdefaulttimeout(0.5)  # 500ms 软超时，避免“卡住”
        try:
            master = rosgraph.Master('/ros_tui_watch')
            topics = master.getTopicTypes()  # List[(name, type)]
            topics.sort(key=lambda x: x[0])
            return topics
        except Exception as e:
            GLOBAL_LOG.append(f"[WARN] master not reachable: {e}")
            return []
    finally:
        socket.setdefaulttimeout(old_to)

def resolve_msg_class(topic: str):
    ttype, _, _ = rostopic.get_topic_type(topic, blocking=False)
    if ttype is None:
        raise RuntimeError(f"Cannot resolve type for topic: {topic}")
    cls = roslib.message.get_message_class(ttype)
    if cls is None:
        raise RuntimeError(f"Cannot load message class for type: {ttype}")
    return cls, ttype

def is_ros_message(obj: Any) -> bool:
    return hasattr(obj, "__slots__") and hasattr(obj, "_slot_types")

def strip_array_suffix(type_str: str) -> Tuple[str, Optional[str]]:
    s = type_str.strip()
    if "[" in s and s.endswith("]"):
        base = s[:s.index("[")]
        arr = s[s.index("["):]
        return base, arr
    return s, None

def preview_value(v: Any, max_len: int = 64) -> str:
    try:
        if is_ros_message(v):
            return "<msg>"
        if isinstance(v, (list, tuple)):
            return f"<seq:{len(v)}>"
        if isinstance(v, bytes):
            text = f"<bytes:{len(v)}>"
        else:
            text = str(v)
        if len(text) > max_len:
            return text[:max_len-3] + "..."
        return text
    except Exception:
        return "<unprintable>"

# ====================== TUI：日志窗 ======================
class LogPane:
    """
    日志窗：
      - 位置：F2 切换 底部/右侧
      - 尺寸：F3 增大 / F4 减小
      - 默认底部固定 3 行（避免随终端高度变大）
    """
    def __init__(self):
        self.at_bottom = True
        self.min_lines = 3
        self.fixed_lines = 3      # 底部模式下，固定显示行数（默认 3）
        self.min_cols = 16
        self.fixed_cols = 24      # 右侧模式下，固定宽度列数
        self.capture_enabled = True  # 仅用于 UI 显示
        self.last_raw = None              # 最近一次原始值（未平滑）
        self.last_raw_ts = 0.0            # 该值的时间戳


    def toggle_side(self):
        self.at_bottom = not self.at_bottom

    def inc(self):
        if self.at_bottom:
            self.fixed_lines = min(20, self.fixed_lines + 1)
        else:
            self.fixed_cols = min(80, self.fixed_cols + 2)

    def dec(self):
        if self.at_bottom:
            self.fixed_lines = max(self.min_lines, self.fixed_lines - 1)
        else:
            self.fixed_cols = max(self.min_cols, self.fixed_cols - 2)

    def layout(self, H, W):
        if self.at_bottom:
            log_h = min(max(self.min_lines, self.fixed_lines), max(1, H-1))
            main_h = max(1, H - log_h)
            return (0, 0, main_h, W), (main_h, 0, log_h, W)
        else:
            log_w = min(max(self.min_cols, self.fixed_cols), max(1, W-1))
            main_w = max(1, W - log_w)
            return (0, 0, H, main_w), (0, main_w, H, log_w)

    def draw(self, stdscr, rect):
        y, x, h, w = rect
        try:
            win = stdscr.derwin(h, w, y, x)
            win.erase()
            pos = 'BOTTOM' if self.at_bottom else 'RIGHT'
            title = f" Logs [{pos} h={h if self.at_bottom else '-'} w={w if not self.at_bottom else '-'}]  F2切换  F3增大  F4减小  PgUp/PgDn滚动  o拦截={GLOBAL_LOG.capture_enabled}  l清空 "
            win.addnstr(0, 0, title.ljust(w-1), w-1, curses.A_REVERSE)

            view_h = max(1, h - 1)
            lines = GLOBAL_LOG.get_view(view_h)
            row = 1
            for ln in lines[-view_h:]:
                for i in range(0, len(ln), max(1, w-1)):
                    if row >= h: break
                    win.addnstr(row, 0, ln[i:i+max(1, w-1)], w-1)
                    row += 1
                if row >= h: break
            win.noutrefresh()
        except Exception:
            pass






class TopicHzMonitor:
    """
    只对“当前页可见的 topics”订阅并统计到达时间，实时估算 Hz。
    - set_active(visible_topic_types): 传入 [(topic, type)]，内部会按需增/减订阅
    - get_hz(topic): 返回估计 Hz（float）
    """
    def __init__(self, maxlen: int = 200):
        ensure_ros_node()
        self.maxlen = maxlen
        self.lock = threading.Lock()
        self.arrivals = {}   # topic -> deque[monotonic_time]
        self.subs = {}       # topic -> rospy.Subscriber

    def _cb_for(self, topic: str):
        dq = self.arrivals.setdefault(topic, deque(maxlen=self.maxlen))
        def _cb(_msg):
            now = time.monotonic()
            dq.append(now)
        return _cb

    def set_active(self, topic_types: List[Tuple[str, str]]):
        """只保持对给定集合的订阅；其余的自动注销。"""
        want = {t for t, _ in topic_types}
        with self.lock:
            # 取消不需要的订阅
            for t in list(self.subs.keys()):
                if t not in want:
                    try: self.subs[t].unregister()
                    except Exception: pass
                    self.subs.pop(t, None)
            # 新增需要的订阅
            for t, ty in topic_types:
                if t in self.subs:
                    continue
                try:
                    cls = roslib.message.get_message_class(ty)
                    if cls is None:
                        continue
                    self.subs[t] = rospy.Subscriber(t, cls, self._cb_for(t), queue_size=1)
                except Exception as e:
                    GLOBAL_LOG.append(f"[WARN] subscribe {t} for Hz failed: {e}")

    def get_hz(self, topic: str) -> float:
        dq = self.arrivals.get(topic)
        if not dq or len(dq) < 2:
            return 0.0
        span = dq[-1] - dq[0]
        if span <= 0:
            return 0.0
        return (len(dq) - 1) / span







# ====================== TUI：列表视图 ======================
class TopicListUI:
    def __init__(self, win):
        self.win = win
        self.filter_text = ""
        self.all_topics = []
        self.filtered = []
        self.sel_index = 0
        self.page = 0
        self.filter_mode = False      # 是否在筛选模式
        self.page_size = 1
        self.last_refresh_time = 0.0
        self.refresh_interval = 1.5


        # 列表页 Hz 监控器
        self.hzmon = TopicHzMonitor()
        self._last_visible_key = None   # 防抖，避免每帧都切订阅


    def refresh_topics(self, force=False):
        now = time.time()
        if force or (now - self.last_refresh_time) >= self.refresh_interval:
            self.all_topics = master_get_topics()
            self.apply_filter()
            self.last_refresh_time = now

    def apply_filter(self):
        if not self.filter_text:
            self.filtered = list(self.all_topics)
        else:
            ft = self.filter_text.lower()
            self.filtered = [(t, ty) for (t, ty) in self.all_topics if ft in t.lower()]
        if self.sel_index >= len(self.filtered):
            self.sel_index = max(0, len(self.filtered) - 1)
        self.fix_page()

    def fix_page(self):
        if self.page_size <= 0: self.page_size = 1
        self.page = self.sel_index // self.page_size

    def handle_key(self, ch) -> Optional[str]:
        # 进入/退出筛选模式
        if ch == ord('/'):
            self.filter_mode = True
            return None

        if self.filter_mode:
            # 在筛选模式内的按键处理
            if ch == 27:  # ESC 取消筛选并退出筛选模式
                self.filter_mode = False
                if self.filter_text:
                    self.filter_text = ""
                    self.apply_filter()
                return None
            elif ch in (curses.KEY_BACKSPACE, 127, curses.ascii.DEL):
                if self.filter_text:
                    self.filter_text = self.filter_text[:-1]
                    self.apply_filter()
                return None
            elif 32 <= ch <= 126:
                self.filter_text += chr(ch)
                self.apply_filter()
                return None
            # 其它按键（上下/翻页/回车）在筛选模式下也允许继续使用，落到下面通用逻辑

        # 通用导航逻辑
        if ch in (curses.KEY_UP, ord('k')) and self.sel_index > 0:
            self.sel_index -= 1; self.fix_page()
        elif ch in (curses.KEY_DOWN, ord('j')) and self.sel_index + 1 < len(self.filtered):
            self.sel_index += 1; self.fix_page()
        elif ch == curses.KEY_LEFT and self.page > 0:
            self.page -= 1; self.sel_index = self.page * self.page_size
        elif ch == curses.KEY_RIGHT:
            max_page = max(0, (len(self.filtered) - 1) // self.page_size)
            if self.page < max_page:
                self.page += 1
                self.sel_index = min(len(self.filtered) - 1, self.page * self.page_size)
        elif ch in (curses.KEY_BACKSPACE, 127, curses.ascii.DEL):
            # 非筛选模式下，Backspace 不做事
            pass
        elif ch in (10, 13, curses.KEY_ENTER):
            if 0 <= self.sel_index < len(self.filtered):
                return self.filtered[self.sel_index]   # (topic, type)
        # 非筛选模式下，直接输入字符不触发筛选
        return None


    def draw(self):
        self.win.erase()
        H, W = self.win.getmaxyx()
        header = "ROS Topic Browser  |  ↑ ↓ 移动  ← → 翻页  /搜索  Enter查看  q退出 "
        self.win.addnstr(0, 0, header, W-1, curses.A_REVERSE)
        self.page_size = max(1, H - 4)   # 留出表头/过滤/页脚各一行
        total = len(self.filtered)
        max_page = max(0, (total - 1) // self.page_size)
        self.page = min(self.page, max_page)
        start = self.page * self.page_size
        end = min(total, start + self.page_size)

        # 过滤提示
        if self.filter_mode:
            filter_line = f"Filter (/ active, ESC to cancel): {self.filter_text}"
        else:
            filter_line = f"Press / to filter   |   Master: {get_master_uri()}"
        self.win.addnstr(1, 0, filter_line, W-1)

        # 列宽
        name_w = max(20, min(48, W - 22 - 10))  # 给 TYPE/HZ 留空间
        type_w =  max(12, min(22, W - name_w - 10))
        hz_w   = 8

        # 表头
        self.win.addnstr(2, 0, "TOPIC".ljust(name_w) + "TYPE".ljust(type_w) + "HZ".rjust(hz_w), W-1, curses.A_BOLD)

        # —— 只为“当前页可见”项采样 Hz（轻量）——
        visible = self.filtered[start:end]
        # 生成一个 key 做防抖（页/筛选变化时才切订阅）
        vis_key = tuple(visible)
        if vis_key != self._last_visible_key:
            self.hzmon.set_active(visible)
            self._last_visible_key = vis_key

        # 行渲染
        row = 3
        for i in range(start, end):
            t, ty = self.filtered[i]
            attr = curses.A_REVERSE if i == self.sel_index else curses.A_NORMAL

            # Hz 估计
            hz = self.hzmon.get_hz(t)
            hz_str = f"{hz:6.2f}" if hz > 0 else "  -   "

            line = ("> " if i == self.sel_index else "  ") + t
            self.win.addnstr(row, 0, line.ljust(name_w) + ty.ljust(type_w) + hz_str.rjust(hz_w), W-1, attr)
            row += 1
            if row >= H - 1:
                break

        footer = f"Total: {total}  Page: {self.page+1}/{max_page+1}"
        self.win.addnstr(H-1, 0, footer.ljust(W-1), W-1, curses.A_REVERSE)
        self.win.noutrefresh()




# ====================== 详情视图：树形浏览 ======================
class TopicViewUI:
    def __init__(self, win, topic: str, type_name: str):
        self.win = win
        self.topic = topic
        self.type_name = type_name  # 直接保存类型名
        self.msg_cls = None
        self.sub = None
        self.msg_lock = threading.Lock()
        self.last_msg = None

        self.arrivals = deque(maxlen=200)  # 用于 Hz 估计
        self.hz = 0.0

        self.path = []   # [("slot", name, type_str) | ("idx", index, elem_type)]
        self.sel_index = 0
        self.page = 0
        self.page_size = 1


    def _make_leaf_reader(self, leaf_name: str):
        """
        返回一个 read_value() 回调：
        - 若 leaf 是 slot：从当前层 getattr(obj, leaf_name)
        - 若 leaf 是索引：[i]
        - 只返回数值（int/float/bool -> float），否则 None
        """
        is_index = leaf_name.startswith('[') and leaf_name.endswith(']')
        idx = int(leaf_name[1:-1]) if is_index else None

        def reader():
            with self.msg_lock:
                msg = self.last_msg
                if msg is None:
                    return None
                obj, _ = self._navigate(msg)
                try:
                    v = obj[idx] if is_index else getattr(obj, leaf_name)
                except Exception:
                    return None
            if isinstance(v, bool):
                return 1.0 if v else 0.0
            if isinstance(v, (int, float)):
                return float(v)
            return None
        return reader


    def start_sub(self):
        ensure_ros_node()  # ✅ 惰性初始化，不阻塞主界面
        self.msg_cls = roslib.message.get_message_class(self.type_name)
        if self.msg_cls is None:
            raise RuntimeError(f"Cannot load message class for type: {self.type_name}")

        def cb(msg):
            now = time.monotonic()
            self.arrivals.append(now)
            if len(self.arrivals) >= 2:
                span = self.arrivals[-1] - self.arrivals[0]
                if span > 0:
                    self.hz = (len(self.arrivals) - 1) / span
            with self.msg_lock:
                self.last_msg = msg

        self.sub = rospy.Subscriber(self.topic, self.msg_cls, cb, queue_size=5)


    def stop_sub(self):
        if self.sub is not None:
            try: self.sub.unregister()
            except Exception: pass
            self.sub = None

    def _root_type_str(self) -> str:
        return self.type_name

    def _navigate(self, msg):
        obj = msg
        tstr = self._root_type_str()
        for kind, key, elem_t in self.path:
            if kind == "slot":
                obj = getattr(obj, key)
                tstr = elem_t
            elif kind == "idx":
                obj = obj[key]
                tstr = elem_t
        return obj, tstr

    def _children(self, obj, type_str) -> List[Tuple[str, str, Any, bool]]:
        out = []
        if is_ros_message(obj):
            slots = getattr(obj, "__slots__", [])
            types = getattr(obj, "_slot_types", [])
            for s, t in zip(slots, types):
                v = getattr(obj, s)
                is_cont = is_ros_message(v) or isinstance(v, (list, tuple))
                out.append((s, t, v, is_cont))
        elif isinstance(obj, (list, tuple)):
            base, arr = strip_array_suffix(type_str) if type_str else (None, None)
            elem_type = base if arr is not None else None
            for i, v in enumerate(obj):
                vt = elem_type or (v.__class__.__name__)
                is_cont = is_ros_message(v) or isinstance(v, (list, tuple))
                out.append((f"[{i}]", vt, v, is_cont))
        return out

    def handle_key(self, ch) -> bool:
        if ch == 27:  # ESC
            if self.path:
                self.path.pop(); self.sel_index = 0; self.page = 0
            else:
                return True
        elif ch in (curses.KEY_UP, ord('k')):
            self.sel_index = max(0, self.sel_index - 1); self._fix_page()
        elif ch in (curses.KEY_DOWN, ord('j')):
            self.sel_index = min(self.sel_index + 1, self._total_children() - 1); self._fix_page()
        elif ch == curses.KEY_LEFT:
            if self.page > 0:
                self.page -= 1; self.sel_index = self.page * self.page_size
        elif ch == curses.KEY_RIGHT:
            max_page = max(0, (self._total_children() - 1) // self.page_size)
            if self.page < max_page:
                self.page += 1
                self.sel_index = min(self._total_children() - 1, self.page * self.page_size)
        elif ch in (10, 13, curses.KEY_ENTER):
            with self.msg_lock:
                msg = self.last_msg
            if msg is None: return False
            cur_obj, cur_t = self._navigate(msg)
            items = self._children(cur_obj, cur_t)
            if 0 <= self.sel_index < len(items):
                name, tstr, val, is_cont = items[self.sel_index]
                if is_cont:
                    if name.startswith("[") and name.endswith("]"):
                        idx = int(name[1:-1])
                        self.path.append(("idx", idx, tstr))
                    else:
                        self.path.append(("slot", name, tstr))
                    self.sel_index = 0; self.page = 0
                else:
                    # 叶子：请求打开实时曲线图 —— 返回一个信号给主循环
                    title = f"{self.topic}  {self.type_name}  |  Path: " + \
                            ("/" + "/".join([f"{k}:{v}" if kind=='slot' else f'[{k}]:{v}'
                              for (kind,k,v) in [(p[0], p[1], p[2]) for p in self.path]]) or "/")
                    reader = self._make_leaf_reader(name)
                    # 在日志里提示一下非数值的情况
                    test_v = reader()
                    if test_v is None:
                        GLOBAL_LOG.append(f"[INFO] '{name}' 不是数值（或暂无值），图表页会等待样本...")
                    # 用一个特殊 tuple 让主循环知道要打开图表
                    return ("chart", title, reader)
        return False

    def _fix_page(self):
        if self.page_size <= 0: self.page_size = 1
        self.page = self.sel_index // self.page_size

    def _total_children(self) -> int:
        with self.msg_lock:
            msg = self.last_msg
        if msg is None: return 0
        cur_obj, cur_t = self._navigate(msg)
        return len(self._children(cur_obj, cur_t))

    # def draw(self):
    #     self.win.erase()
    #     H, W = self.win.getmaxyx()
    #     path_str = "/" + "/".join(
    #         [f"{k}:{v}" if kind=="slot" else f"[{k}]:{v}"
    #          for (kind,k,v) in [(p[0], p[1], p[2]) for p in self.path]]
    #     )
    #     # header = f"{self.topic}  ({self.type_name})  |  Path: {path_str if path_str!='/' else '/'}  |  Hz: {self.hz:.2f}"

    #     header = (
    #         f"{self.topic}  ({self.type_name})"
    #         f"  |  Path: {path_str if path_str!='/' else '/'}"
    #         f"  |  Hz: {self.hz:.2f}"
    #         f"  |  Master: {get_master_uri()}"
    #     )

    #     self.win.addnstr(0, 0, header, W-1, curses.A_REVERSE)

    #     name_w = max(12, int(W*0.28))
    #     type_w = max(14, int(W*0.30))
    #     val_w  = max(10, W - 2 - name_w - type_w)
    #     self.win.addnstr(1, 0, f"{'Name'.ljust(name_w)}{'Type'.ljust(type_w)}Value", W-1, curses.A_BOLD)
    #     self.win.hline(2, 0, ord('-'), W-1)

    #     with self.msg_lock:
    #         msg = self.last_msg
    #     if msg is None:
    #         self.win.addnstr(3, 0, "(waiting for message...)", W-1)
    #         self.win.noutrefresh()
    #         return

    #     cur_obj, cur_t = self._navigate(msg)
    #     items = self._children(cur_obj, cur_t)

    #     avail_rows = max(1, H - 4)
    #     self.page_size = avail_rows
    #     total = len(items)
    #     max_page = max(0, (total - 1) // self.page_size)
    #     self.page = min(self.page, max_page)
    #     start = self.page * self.page_size
    #     end = min(total, start + self.page_size)

    #     row = 3
    #     for i in range(start, end):
    #         name, tstr, val, is_cont = items[i]
    #         val_txt = preview_value(val, max_len=val_w)
    #         mark = "▶ " if is_cont else "  "
    #         line_name = (mark + name).ljust(name_w)
    #         line_type = tstr.ljust(type_w)
    #         attr = curses.A_REVERSE if i == self.sel_index else curses.A_NORMAL
    #         self.win.addnstr(row, 0, line_name + line_type + val_txt, W-1, attr)
    #         row += 1

    #     footer = f"Items: {total}  Page: {self.page+1}/{max_page+1}   Enter进入(若可)  ESC返回"
    #     self.win.addnstr(H-1, 0, footer.ljust(W-1), W-1, curses.A_REVERSE)
    #     self.win.noutrefresh()


    def draw(self):
        self.win.erase()
        H, W = self.win.getmaxyx()

        # 过小窗口直接给提示，避免后续越界
        if H < 4 or W < 20:
            safe_addstr(self.win, 0, 0, "Terminal too small; enlarge this pane.")
            self.win.noutrefresh()
            return

        # 计算路径字符串
        path_str = "/" + "/".join(
            [f"{k}:{v}" if kind == "slot" else f"[{k}]:{v}"
            for (kind, k, v) in [(p[0], p[1], p[2]) for p in self.path]]
        )

        header = (
            f"{self.topic}  ({self.type_name})"
            f"  |  Path: {path_str if path_str != '/' else '/'}"
            f"  |  Hz: {self.hz:.2f}"
            f"  |  Master: {get_master_uri()}"
        )
        safe_addstr(self.win, 0, 0, header, curses.A_REVERSE)

        # 表头
        name_w = max(12, int(W * 0.28))
        type_w = max(14, int(W * 0.30))
        val_w  = max(10, W - 2 - name_w - type_w)
        safe_addstr(self.win, 1, 0, f"{'Name'.ljust(name_w)}{'Type'.ljust(type_w)}Value", curses.A_BOLD)
        safe_hline(self.win, 2, 0, ord('-'), W - 1)

        # 数据
        with self.msg_lock:
            msg = self.last_msg
        if msg is None:
            safe_addstr(self.win, 3, 0, "(waiting for message...)")
            self.win.noutrefresh()
            return

        cur_obj, cur_t = self._navigate(msg)
        items = self._children(cur_obj, cur_t)

        avail_rows = max(1, H - 4)
        self.page_size = avail_rows
        total = len(items)
        max_page = max(0, (total - 1) // self.page_size)
        self.page = min(self.page, max_page)
        start = self.page * self.page_size
        end = min(total, start + self.page_size)

        row = 3
        for i in range(start, end):
            name, tstr, val, is_cont = items[i]
            val_txt = preview_value(val, max_len=val_w)
            mark = "▶ " if is_cont else "  "
            line_name = (mark + name).ljust(name_w)
            line_type = tstr.ljust(type_w)
            attr = curses.A_REVERSE if i == self.sel_index else curses.A_NORMAL
            safe_addstr(self.win, row, 0, line_name + line_type + val_txt, attr)
            row += 1
            if row >= H - 1:
                break

        footer = f"Items: {total}  Page: {self.page+1}/{max_page+1}   Enter进入(若可)  ESC返回"
        safe_addstr(self.win, H - 1, 0, footer, curses.A_REVERSE)
        self.win.noutrefresh()








def lab_len(s: str) -> int:
    try:
        return len(s)
    except Exception:
        return 0


class ChartViewUI:
    """
    实时曲线图（增强版：支持 Braille 像素化渲染）
      - 模式切换： m -> envelope / line / braille
      - 其它快捷键：Space暂停，+/-时间窗，g网格，r清空，s平滑，y锁Y轴，ESC返回
    """
    def __init__(self, win, title: str, read_value_fn, time_window: float = 10.0, max_points: int = 20000):
        self.win = win
        self.title = title
        self.read_value = read_value_fn
        self.time_window = max(1.0, float(time_window))
        self.max_points = max_points
        self.paused = False
        self.show_grid = True
        self.samples = deque()  # [(t, v)]
        self.last_sample_t = 0.0

        # 视觉/算法
        self.use_smoothing = False
        self.smooth_alpha = 0.35
        self.last_smooth = None

        self.lock_y = False
        self.locked_vmin = None
        self.locked_vmax = None
        self.vmargin_ratio = 0.08

        # self.mode = "braille"  # 默认直接给你更细腻的 braille
        # self.mode = "blocks"  # 默认音量条方块风格
        self.mode = "bars"

    # ---------- 交互 ----------
    def handle_key(self, ch) -> bool:
        if ch == 27:  # ESC
            return True
        elif ch in (ord(' '),):
            self.paused = not self.paused
        elif ch in (ord('+'),):
            self.time_window = min(300.0, self.time_window * 1.25)
        elif ch in (ord('-'),):
            self.time_window = max(1.0, self.time_window / 1.25)
        elif ch in (ord('r'), ord('R')):
            self.samples.clear(); self.last_smooth = None
        elif ch in (ord('g'), ord('G')):
            self.show_grid = not self.show_grid
        elif ch in (ord('s'), ord('S')):
            self.use_smoothing = not self.use_smoothing; self.last_smooth = None
        elif ch in (ord('y'), ord('Y')):
            self.lock_y = not self.lock_y
            if self.lock_y:
                vmin, vmax = self._current_vrange()
                if vmin is not None:
                    self.locked_vmin, self.locked_vmax = vmin, vmax
            else:
                self.locked_vmin = self.locked_vmax = None
        elif ch in (ord('m'), ord('M')):
            order = ["bars", "blocks", "envelope", "line", "braille"]
            self.mode = order[(order.index(self.mode) + 1) % len(order)]
        return False

    # ---------- 采样 ----------
    def _sample(self):
        if self.paused:
            return
        now = time.monotonic()
        if now - self.last_sample_t < 0.02:  # 最高约 50Hz
            return
        self.last_sample_t = now

        v = self.read_value()
        if v is None:
            return
        try:
            v = float(v)
        except Exception:
            return
        
        # ... try/except 后，v 已经是 float
        self.last_raw = v
        self.last_raw_ts = now

        if self.use_smoothing:
            self.last_smooth = v if self.last_smooth is None else (self.smooth_alpha * v + (1 - self.smooth_alpha) * self.last_smooth)
            v_plot = self.last_smooth
        else:
            v_plot = v

        self.samples.append((now, v_plot))
        cutoff = now - self.time_window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        while len(self.samples) > self.max_points:
            self.samples.popleft()

    def _current_vrange(self):
        if not self.samples:
            return (None, None)
        vs = [v for _, v in self.samples]
        vmin = min(vs); vmax = max(vs)
        if vmin == vmax:
            pad = 1.0 if vmax == 0 else abs(vmax) * 0.1 + 1e-9
            vmin -= pad; vmax += pad
        vr = vmax - vmin
        return (vmin - vr*self.vmargin_ratio, vmax + vr*self.vmargin_ratio)

    def _stats(self):
        if not self.samples:
            return None
        vs = [v for _, v in self.samples]
        return dict(cur=vs[-1], min=min(vs), max=max(vs), avg=sum(vs)/len(vs))

    # ---------- 绘制 ----------
    def draw(self):
        self._sample()

        self.win.erase()
        H, W = self.win.getmaxyx()
        if H < 8 or W < 32:
            safe_addstr(self.win, 0, 0, "Terminal too small; enlarge this pane.")
            self.win.noutrefresh()
            return

        header = (
            f"{self.title}  |  window={self.time_window:.1f}s  "
            f"{'PAUSED' if self.paused else 'LIVE'}  |  "
            f"mode:{self.mode}  smooth:{'on' if self.use_smoothing else 'off'}  "
            f"Y:{'lock' if self.lock_y else 'auto'}  "
            f"+/-缩放 空格暂停 g网格 r清空 s平滑 y锁Y m模式 ESC返回"
        )
        safe_addstr(self.win, 0, 0, header[:W-1], curses.A_REVERSE)

        # 布局
        top = 1
        bottom = H - 3
        left_pad = 10
        plot_x0 = left_pad
        plot_w = max(10, W - plot_x0 - 1)
        plot_h = max(4, bottom - top)
        axis_y = bottom

        # 边框/轴
        safe_hline(self.win, top, 0, ord('─'), W-1)
        safe_hline(self.win, axis_y, 0, ord('─'), W-1)
        for yy in range(top+1, axis_y):
            safe_addstr(self.win, yy, 0, "│")

        if not self.samples:
            safe_addstr(self.win, top+1, plot_x0, "(waiting for numeric samples...)")
            self.win.noutrefresh()
            return

        # 纵向范围
        if self.lock_y and self.locked_vmin is not None:
            vmin, vmax = self.locked_vmin, self.locked_vmax
        else:
            vmin, vmax = self._current_vrange()
        yr = (vmax - vmin) if (vmin is not None) else 1.0

        # y 轴刻度与网格
        ticks = [vmax, vmin + yr*0.5, vmin]
        tick_rows = [top+1, top + 1 + plot_h//2, axis_y-1]
        for row, tv in zip(tick_rows, ticks):
            if 0 <= row < H:
                label = f"{tv:.3g}"
                safe_addstr(self.win, row, 1, label.rjust(left_pad-2))
                if self.show_grid:
                    for xx in range(plot_x0, min(W-1, plot_x0 + plot_w)):
                        safe_addstr(self.win, row, xx, '┈')
        # 0 线
        if vmin < 0 < vmax:
            zero_row = axis_y - 1 - int((0 - vmin) / yr * (plot_h - 1))
            zero_row = max(top+1, min(axis_y-1, zero_row))
            for xx in range(plot_x0, min(W-1, plot_x0 + plot_w)):
                safe_addstr(self.win, zero_row, xx, '╌')

        # x 轴刻度
        safe_addstr(self.win, axis_y, plot_x0, f"{-self.time_window:.0f}s")
        rlab = "0s"
        safe_addstr(self.win, axis_y, min(W-1-lab_len(rlab), plot_x0 + plot_w - lab_len(rlab)), rlab)

        # 渲染方式
        if self.mode == "bars":
            self._draw_bars(top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax)
        elif self.mode == "blocks":
            self._draw_blocks(top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax)
        elif self.mode == "envelope":
            self._draw_envelope(top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax)
        elif self.mode == "line":
            self._draw_line(top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax)
        else:  # braille
            self._draw_braille(top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax)

        # 统计信息
        st = self._stats()
        if st:
            info = f"cur={st['cur']:.4g}  min={st['min']:.4g}  max={st['max']:.4g}  avg={st['avg']:.4g}"
            safe_addstr(self.win, axis_y+1, 0, info[:W-1])

        # 右下角：高精度“当前值”
        if self.last_raw is not None:
            # 17 有效位，尽量还原 IEEE-754 双精度
            cur_str = f"cur_raw={format(self.last_raw, '.17g')}"
            # 也可带时间： time.strftime('%H:%M:%S', time.localtime(time.time()))
            y = axis_y + 1
            x = max(0, W - 1 - len(cur_str))
            safe_addstr(self.win, y, x, cur_str)  # 右对齐到最右

        self.win.noutrefresh()

    # ---------- 基础映射 ----------
    def _time_to_col(self, t, start_t, width):
        col = int((t - start_t) / self.time_window * (width - 1))
        return max(0, min(width - 1, col))

    def _val_to_row(self, v, vmin, vmax, top, axis_y, plot_h):
        r = axis_y - 1 - int((v - vmin)/(vmax - vmin) * (plot_h - 1))
        return max(top+1, min(axis_y-1, r))

    # ---------- 竖条包络 ----------
    def _draw_envelope(self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax):
        now = self.samples[-1][0]
        start_t = now - self.time_window
        buckets = [{"min": None, "max": None, "last": None} for _ in range(plot_w)]
        for (t, v) in self.samples:
            if t < start_t: continue
            c = self._time_to_col(t, start_t, plot_w)
            b = buckets[c]
            b["min"] = v if b["min"] is None else min(b["min"], v)
            b["max"] = v if b["max"] is None else max(b["max"], v)
            b["last"] = v
        for i, b in enumerate(buckets):
            if b["last"] is None: continue
            x = plot_x0 + i
            lo = self._val_to_row(b["min"], vmin, vmax, top, axis_y, plot_h)
            hi = self._val_to_row(b["max"], vmin, vmax, top, axis_y, plot_h)
            if hi > lo: lo, hi = hi, lo
            for rr in range(hi, lo+1):
                safe_addstr(self.win, rr, x, '│')
            safe_addstr(self.win, self._val_to_row(b["last"], vmin, vmax, top, axis_y, plot_h), x, '•')

    # ---------- 普通线条 ----------
    def _draw_line(self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax):
        now = self.samples[-1][0]
        start_t = now - self.time_window
        last_row = None
        last_x = None
        for (t, v) in self.samples:
            if t < start_t: continue
            c = self._time_to_col(t, start_t, plot_w)
            x = plot_x0 + c
            y = self._val_to_row(v, vmin, vmax, top, axis_y, plot_h)
            safe_addstr(self.win, y, x, '•')
            if last_row is not None and x > last_x:
                # 竖向连线填充
                step = 1 if y < last_row else -1
                for rr in range(last_row, y, -step):
                    safe_addstr(self.win, rr, x-1, '│')
            last_row, last_x = y, x

    # ---------- Braille 像素化 ----------
    def _draw_braille(self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax):
        """
        将绘图区映射到虚拟像素网格：
          真实字符网格 plot_w × plot_h
          虚拟像素网格 (plot_w*2) × (plot_h*4)
          先在虚拟网格上用 Bresenham 划线，再打包回每字符的 2×4 点阵（Braille）。
        """
        # 虚拟画布尺寸
        vW = plot_w * 2
        vH = plot_h * 4

        # 虚拟画布起点（上/左）对应真实网格 top+1 ~ axis_y-1, plot_x0 ~ plot_x0+plot_w-1
        def vrow_from_val(val):
            # 把值映射到 [0, vH-1]，0 在顶部
            ratio = (val - vmin) / (vmax - vmin + 1e-18)
            ratio = max(0.0, min(1.0, ratio))
            return int((1.0 - ratio) * (vH - 1))

        def vcol_from_time(t, start_t):
            # 把时间映射到 [0, vW-1]
            ratio = (t - start_t) / (self.time_window + 1e-18)
            ratio = max(0.0, min(1.0, ratio))
            return int(ratio * (vW - 1))

        now = self.samples[-1][0]
        start_t = now - self.time_window

        # 点收集：[(vc, vr)]
        pts = []
        for (t, v) in self.samples:
            if t < start_t: continue
            vc = vcol_from_time(t, start_t)
            vr = vrow_from_val(v)
            pts.append((vc, vr))
        if not pts:
            return

        # 在虚拟网格上做线段连接（Bresenham）
        on_pixels = set()
        def plot(vx, vy):
            if 0 <= vx < vW and 0 <= vy < vH:
                on_pixels.add((vx, vy))

        def line(x0, y0, x1, y1):
            dx = abs(x1 - x0); dy = -abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx + dy
            x, y = x0, y0
            while True:
                plot(x, y)
                if x == x1 and y == y1: break
                e2 = 2 * err
                if e2 >= dy:
                    err += dy; x += sx
                if e2 <= dx:
                    err += dx; y += sy

        px, py = pts[0]
        plot(px, py)
        for (cx, cy) in pts[1:]:
            line(px, py, cx, cy)
            px, py = cx, cy

        # 将虚拟 2×4 子像素打包到字符网格
        # Braille 点位（2列×4行）→ bit：
        # (0,0)=dot1(0x01) (0,1)=dot2(0x02) (0,2)=dot3(0x04) (0,3)=dot7(0x40)
        # (1,0)=dot4(0x08) (1,1)=dot5(0x10) (1,2)=dot6(0x20) (1,3)=dot8(0x80)
        DOT_BITS = {(0,0):0x01,(0,1):0x02,(0,2):0x04,(0,3):0x40,
                    (1,0):0x08,(1,1):0x10,(1,2):0x20,(1,3):0x80}
        BRAILLE_BASE = 0x2800

        # 映射到字符网格（宽 plot_w， 高 plot_h）
        for cell_y in range(plot_h):
            for cell_x in range(plot_w):
                vx0 = cell_x * 2
                vy0 = cell_y * 4
                mask = 0
                # 采样该字符内的 2×4 子像素
                for sx in range(2):
                    for sy in range(4):
                        if (vx0+sx, vy0+sy) in on_pixels:
                            mask |= DOT_BITS[(sx, sy)]
                ch = chr(BRAILLE_BASE + mask) if mask else ' '
                # 把字符画到真实终端行列：
                term_y = (plot_h - 1 - cell_y) + (top+1)  # 注意：虚拟 0 顶部 -> 终端 top+1
                term_x = plot_x0 + cell_x
                if 0 < term_y < axis_y and plot_x0 <= term_x < plot_x0+plot_w:
                    safe_addstr(self.win, term_y, term_x, ch)


    # ---------- Blocks（音量条方块） ----------
    def _draw_blocks(self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax):
        """
        使用 U+2581..U+2588 ▂▃▄▅▆▇█ 作为“不同高度方块”，每个终端列一个方块。
        做法：把时间窗口切成 plot_w 个桶，桶内取最后值（或均值），
        然后把该值映射成 1~8 级方块字符。
        """
        now = self.samples[-1][0]
        start_t = now - self.time_window

        # 划分时间桶
        buckets = [None] * plot_w
        for (t, v) in self.samples:
            if t < start_t: continue
            ratio = (t - start_t) / (self.time_window + 1e-12)
            c = int(ratio * (plot_w - 1))
            c = max(0, min(plot_w - 1, c))
            buckets[c] = v if buckets[c] is None else v  # 这里取“最后值”；如需均值可自己累加/计数

        # 方块字符表（从矮到高）
        blocks = [" ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"]

        for i, v in enumerate(buckets):
            x = plot_x0 + i
            if v is None:
                # 无样本，留空
                continue

            # 映射到 0..1
            level = (v - vmin) / (vmax - vmin + 1e-18)
            level = 0.0 if level < 0 else (1.0 if level > 1 else level)

            # 1..8 级（0 级空白）
            idx = int(round(level * 8))
            idx = max(1, min(8, idx))

            ch = blocks[idx]

            # 底线位置
            y = axis_y - 1
            safe_addstr(self.win, y, x, ch)

            # 可选：用浅网格补充“柱体”感（非必要）
            # 若想显示整根柱子，可改为从顶部下落的“倒置”，这会改变视觉风格：
            # for fy in range(idx): 
            #     safe_addstr(self.win, y - fy, x, '█')


        # ---------- Bars（整列填充，半行精度） ----------
    
    
    def _draw_bars(self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax):
        """
        用整列柱子铺满绘图区：
          - 每列按当前值从底部“堆”到目标高度
          - 使用半行精度：plot_h*2 个“半行”，满半行用 '█'，顶端剩余半行用 '▄'
            （'▄' = 下半块，表示该字符格的下半被填充）
        """
        if vmax <= vmin:
            return
        now = self.samples[-1][0]
        start_t = now - self.time_window

        # 每列取最后值（也可改成均值/最大值）
        buckets = [None] * plot_w
        for (t, v) in self.samples:
            if t < start_t:
                continue
            col = int((t - start_t) / self.time_window * (plot_w - 1) + 1e-9)
            col = max(0, min(plot_w - 1, col))
            buckets[col] = v

        # 画每一列
        total_half_rows = (axis_y - (top + 1)) * 2  # 可用的“半行”总数
        for i, v in enumerate(buckets):
            if v is None:
                continue
            # 归一化到 0..total_half_rows
            level = (v - vmin) / (vmax - vmin)
            level = 0.0 if level < 0 else (1.0 if level > 1 else level)
            fill_half = int(round(level * total_half_rows))

            x = plot_x0 + i
            # 从底部开始往上填：每 2 个半行 = 1 个整行（'█'）
            full_rows = fill_half // 2
            half_top  = (fill_half % 2) == 1

            # 先填满的整行（用 '█'）
            for r in range(full_rows):
                y = (axis_y - 1) - r
                if y <= top:  # 防守
                    break
                safe_addstr(self.win, y, x, '█')

            # 顶端剩半行（用 '▄' 表示“底半块”）
            if half_top:
                y = (axis_y - 1) - full_rows
                if y > top:
                    safe_addstr(self.win, y, x, '▄')




# ====================== 主循环 ======================
def main_curses(stdscr):
    # 某些终端不支持隐藏光标，做个保护
    try:
        curses.curs_set(0)
    except curses.error:
        pass

    stdscr.nodelay(True)
    stdscr.keypad(True)

    logpane = LogPane()

    chart_ui = None              # ChartViewUI 或 None
    list_ui: Optional[TopicListUI] = None
    view_ui: Optional[TopicViewUI] = None

    def draw_all():
        stdscr.erase()
        H, W = stdscr.getmaxyx()
        main_rect, log_rect = logpane.layout(H, W)
        main_win = stdscr.derwin(main_rect[2], main_rect[3], main_rect[0], main_rect[1])

        # 只在一个地方决定画什么：chart 优先，其次 view，最后 list
        if chart_ui is not None:
            chart_ui.win = main_win
            chart_ui.draw()
        elif view_ui is not None:
            view_ui.win = main_win
            view_ui.draw()
        else:
            nonlocal list_ui
            if list_ui is None:
                list_ui = TopicListUI(main_win)
                list_ui.refresh_topics(force=True)
            else:
                list_ui.win = main_win
            list_ui.draw()

        logpane.draw(stdscr, log_rect)
        curses.doupdate()

    # 渲染与刷新控制
    fps = 30.0
    interval = 1.0 / fps
    last_draw = 0.0
    last_topic_refresh = 0.0

    while True:
        try:
            ch = stdscr.getch()

            # 终端尺寸变化
            if ch == curses.KEY_RESIZE:
                stdscr.erase()
                try:
                    new_h, new_w = stdscr.getmaxyx()
                    if hasattr(curses, "is_term_resized") and curses.is_term_resized(new_h, new_w):
                        try:
                            curses.resizeterm(new_h, new_w)
                        except curses.error:
                            pass
                except Exception:
                    pass
                last_draw = 0
                # 不要在这里继续处理其他逻辑，直接进入下一轮
                continue

            # 全局日志窗快捷键（不依赖当前视图）
            # if ch == curses.KEY_F2:       logpane.toggle_side()
            # elif ch in (ord('+'),):       logpane.inc()
            # elif ch in (ord('-'),):       logpane.dec()
            # elif ch == curses.KEY_PPAGE:  GLOBAL_LOG.scroll_up(10)
            # elif ch == curses.KEY_NPAGE:  GLOBAL_LOG.scroll_down(10)
            # elif ch == curses.KEY_HOME:   GLOBAL_LOG.scroll_home()
            # elif ch == curses.KEY_END:    GLOBAL_LOG.scroll_end()
            # elif ch in (ord('l'), ord('L')): GLOBAL_LOG.clear()
            # elif ch in (ord('o'), ord('O')):
            #     on = GLOBAL_LOG.toggle_capture()
            #     GLOBAL_LOG.append(f"[LOG] output capture -> {on}")



            # === 全局日志窗快捷键（避免与其它页冲突；在筛选模式下禁用） ===
            allow_log_hotkeys = True
            if isinstance(list_ui, TopicListUI) and list_ui.filter_mode:
                allow_log_hotkeys = False  # 筛选时屏蔽日志窗快捷键

            if allow_log_hotkeys:
                if ch == curses.KEY_F2:       logpane.toggle_side()
                elif ch == curses.KEY_F3:     logpane.inc()
                elif ch == curses.KEY_F4:     logpane.dec()
                elif ch == curses.KEY_PPAGE:  GLOBAL_LOG.scroll_up(10)
                elif ch == curses.KEY_NPAGE:  GLOBAL_LOG.scroll_down(10)
                elif ch == curses.KEY_HOME:   GLOBAL_LOG.scroll_home()
                elif ch == curses.KEY_END:    GLOBAL_LOG.scroll_end()
                elif ch in (ord('l'), ord('L')): GLOBAL_LOG.clear()
                elif ch in (ord('o'), ord('O')):
                    on = GLOBAL_LOG.toggle_capture()
                    GLOBAL_LOG.append(f"[LOG] output capture -> {on}")




            # —— 页面调度（唯一的一处）——
            if chart_ui is not None:
                # 图表页优先处理
                if ch != -1 and chart_ui.handle_key(ch):
                    chart_ui = None

            elif view_ui is not None:
                # 详情页
                ret = None
                if ch != -1:
                    ret = view_ui.handle_key(ch)
                if ret is True:
                    # 退出详情页
                    view_ui.stop_sub()
                    view_ui = None
                elif isinstance(ret, tuple) and len(ret) == 3 and ret[0] == "chart":
                    # 打开实时曲线图
                    _, title, reader = ret
                    chart_ui = ChartViewUI(None, title, reader)

            else:
                # 列表页
                if ch in (ord('q'), ord('Q')):
                    break
                if list_ui is not None and ch != -1:
                    sel = list_ui.handle_key(ch)
                    if sel is not None:
                        topic, ttype = sel
                        view_ui = TopicViewUI(None, topic, ttype)
                        try:
                            view_ui.start_sub()
                        except Exception as e:
                            if view_ui: view_ui.stop_sub()
                            view_ui = None
                            GLOBAL_LOG.append(f"[ERR] 订阅失败: {e}")

                # 仅在列表页定时刷新 topic 列表
                now = time.time()
                if list_ui and (now - last_topic_refresh > 1.2):
                    list_ui.refresh_topics()
                    last_topic_refresh = now

            # 统一绘制
            now = time.time()
            if now - last_draw >= interval:
                draw_all()
                last_draw = now

            time.sleep(0.005)

        except KeyboardInterrupt:
            break
        except SystemExit:
            break
        except Exception:
            GLOBAL_LOG.append("[EXC] " + "".join(traceback.format_exc()))
            time.sleep(0.05)




def main():
    # rospy.init_node("ros_tui_watch", anonymous=True, disable_signals=True)

    curses.wrapper(main_curses)




if __name__ == "__main__":
    main()
