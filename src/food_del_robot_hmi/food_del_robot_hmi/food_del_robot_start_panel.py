import os
import json
import math
import subprocess
import threading
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QScrollArea, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from food_del_robot_animations import fade_in, stagger_fade_in, PulseAnimation
from food_del_robot_datas import record_trip
from PySide6.QtGui import QCursor, QPainter, QColor, QPen, QFont, QBrush

BASE_DIR       = os.path.dirname(os.path.abspath(__file__))
POSITIONS_FILE = os.path.join(BASE_DIR, "saved_positions.json")

ROS_AVAILABLE = True  # checked at runtime


# --- helpers -----------------------------------------------------------------

def load_positions():
    if os.path.exists(POSITIONS_FILE):
        with open(POSITIONS_FILE, "r") as f:
            return json.load(f)
    return []


# --- state constants ---------------------------------------------------------

STATE_IDLE    = "IDLE"
STATE_RUNNING = "RUNNING"
STATE_STOPPED = "STOPPED"


# --- ROS2 Nav2 bridge --------------------------------------------------------

class Nav2Bridge(QObject):
    """
    Uses a MultiThreadedExecutor so spin and goal callbacks
    never block each other — fixes the freeze on send_goal.
    """
    goal_accepted  = Signal()
    goal_succeeded = Signal()
    goal_failed    = Signal(str)

    def __init__(self):
        super().__init__()
        self._node         = None
        self._executor     = None
        self._action_client = None
        self._goal_handle  = None
        self._thread       = None
        self._running      = False

    def start(self):
        if not ROS_AVAILABLE:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            import rclpy
        except ImportError:
            print("[ROS] rclpy not available")
            return
        # only init if not already initialized
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception:
                pass
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        self._node     = rclpy.create_node("start_panel_nav2_bridge")
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._action_client = ActionClient(
            self._node, NavigateToPose, "navigate_to_pose"
        )
        self._node.get_logger().info("Nav2Bridge: ready (MultiThreadedExecutor)")
        while self._running:
            self._executor.spin_once(timeout_sec=0.05)
        self._executor.shutdown()
        try:
            self._node.destroy_node()
        except Exception:
            pass
        # do NOT call rclpy.shutdown() — other nodes may still be running

    def send_goal(self, x, y, yaw):
        if self._action_client is None:
            self.goal_failed.emit("ROS not available - node not ready")
            return
        threading.Thread(
            target=self._send_goal_thread,
            args=(x, y, yaw),
            daemon=True
        ).start()

    def _send_goal_thread(self, x, y, yaw):
        from nav2_msgs.action import NavigateToPose
        if not self._action_client.wait_for_server(timeout_sec=8.0):
            self.goal_failed.emit("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = "map"
        goal_msg.pose.header.stamp       = self._node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = float(x)
        goal_msg.pose.pose.position.y    = float(y)
        goal_msg.pose.pose.position.z    = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_send_done)

    def _on_send_done(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.goal_failed.emit("Goal rejected by Nav2")
            return
        self.goal_accepted.emit()
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result_done)

    def _on_result_done(self, future):
        try:
            from action_msgs.msg import GoalStatus
            result = future.result()
            # Nav2 result: result.status is the action status code
            # STATUS_SUCCEEDED = 4
            status = result.status if hasattr(result, 'status') else -1
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.goal_succeeded.emit()
            elif status == GoalStatus.STATUS_ABORTED:
                self.goal_failed.emit("Goal aborted by Nav2")
            elif status == GoalStatus.STATUS_CANCELED:
                self.goal_failed.emit("Goal cancelled")
            else:
                # unknown status — treat as succeeded if no error
                self.goal_succeeded.emit()
        except Exception as ex:
            print(f"[Nav2Bridge] result callback error: {ex}")
            self.goal_succeeded.emit()  # assume success if we can't read status

    def cancel(self):
        if self._goal_handle is None:
            return
        threading.Thread(target=self._cancel_thread, daemon=True).start()

    def _cancel_thread(self):
        if self._goal_handle:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: setattr(self, "_goal_handle", None)
            )

    def stop(self):
        self._running = False


# --- rviz launcher -----------------------------------------------------------

class RVizLauncher:
    def __init__(self, rviz_config=None):
        self._proc        = None
        self._rviz_config = rviz_config

    def start(self, hmi_x, hmi_y, hmi_w, hmi_h):
        cmd = ["rviz2"]
        if self._rviz_config and os.path.exists(self._rviz_config):
            cmd += ["-d", self._rviz_config]
        env = os.environ.copy()
        env["DISPLAY"] = os.environ.get("DISPLAY", ":0")
        self._proc = subprocess.Popen(
            cmd, env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        rviz_x = hmi_x + hmi_w
        threading.Thread(
            target=self._reposition,
            args=(rviz_x, hmi_y, hmi_w, hmi_h),
            daemon=True
        ).start()

    def _reposition(self, x, y, w, h):
        time.sleep(3)
        if self._proc and self._proc.poll() is None:
            try:
                result = subprocess.run(
                    ["xdotool", "search", "--pid", str(self._proc.pid)],
                    capture_output=True, text=True, timeout=3
                )
                wids = result.stdout.strip().split()
                if not wids:
                    result = subprocess.run(
                        ["xdotool", "search", "--class", "rviz2"],
                        capture_output=True, text=True, timeout=3
                    )
                    wids = result.stdout.strip().split()
                if wids:
                    wid = wids[-1]
                    subprocess.run(["xdotool", "windowmove", wid, str(x), str(y)], timeout=3)
                    subprocess.run(["xdotool", "windowsize", wid, str(w), str(h)], timeout=3)
            except Exception as e:
                print(f"RViz reposition failed: {e}")

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None


# --- control button ----------------------------------------------------------

class ControlButton(QFrame):
    def __init__(self, text, accent="#000000", callback=None):
        super().__init__()
        self._callback = callback
        self._accent   = accent
        self._enabled  = True
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setFixedHeight(52)
        self.setFixedWidth(130)
        lv = QVBoxLayout(self)
        lv.setContentsMargins(0, 0, 0, 0)
        self.lbl = QLabel(text.upper())
        self.lbl.setAlignment(Qt.AlignCenter)
        lv.addWidget(self.lbl)
        self._set_style(False)

    def set_text(self, text):
        self.lbl.setText(text.upper())

    def _set_style(self, hovered):
        if not self._enabled:
            self.setStyleSheet("QFrame{background:#e0e0e0;border:2px solid #cccccc;}")
            self.lbl.setStyleSheet("color:#aaaaaa;font-size:16px;font-family:'Courier New';letter-spacing:3px;font-weight:700;background:transparent;border:none;")
            return
        h = self._accent.lstrip("#")
        r, g, b = int(h[0:2],16), int(h[2:4],16), int(h[4:6],16)
        if hovered:
            r, g, b = max(0,r-40), max(0,g-40), max(0,b-40)
        bg = f"#{r:02x}{g:02x}{b:02x}"
        self.setStyleSheet(f"QFrame{{background:{bg};border:2px solid {self._accent};}}")
        self.lbl.setStyleSheet("color:#ffffff;font-size:16px;font-family:'Courier New';letter-spacing:3px;font-weight:700;background:transparent;border:none;")

    def set_enabled(self, val):
        self._enabled = val
        self.setCursor(QCursor(Qt.PointingHandCursor if val else Qt.ForbiddenCursor))
        self._set_style(False)

    def enterEvent(self, e):
        if self._enabled: self._set_style(True)
    def leaveEvent(self, e): self._set_style(False)
    def mousePressEvent(self, e):
        if self._enabled and self._callback:
            self._callback()


# --- position button ---------------------------------------------------------

class PositionButton(QFrame):
    def __init__(self, name, x, y, yaw_deg, callback=None):
        super().__init__()
        self._callback = callback
        self._selected = False
        self.pos_name  = name
        self.pos_x     = x
        self.pos_y     = y
        self.pos_yaw   = math.radians(yaw_deg)
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setMinimumHeight(90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(4)

        self.name_lbl  = QLabel(name)
        self.name_lbl.setAlignment(Qt.AlignCenter)
        self.name_lbl.setWordWrap(True)

        self.coord_lbl = QLabel(f"X: {x}   Y: {y}   Yaw: {yaw_deg}°")
        self.coord_lbl.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.name_lbl)
        layout.addWidget(self.coord_lbl)
        self._apply_normal()

    def _apply_normal(self):
        self.setStyleSheet("QFrame{background:#ffffff;border:2px solid #000000;}")
        self.name_lbl.setStyleSheet("color:#000000;font-size:20px;font-family:'Courier New';font-weight:700;background:transparent;border:none;letter-spacing:1px;")
        self.coord_lbl.setStyleSheet("color:#888888;font-size:15px;font-family:'Courier New';background:transparent;border:none;")

    def _apply_selected(self):
        self.setStyleSheet("QFrame{background:#000000;border:2px solid #000000;}")
        self.name_lbl.setStyleSheet("color:#ffffff;font-size:20px;font-family:'Courier New';font-weight:700;background:transparent;border:none;letter-spacing:1px;")
        self.coord_lbl.setStyleSheet("color:#aaaaaa;font-size:15px;font-family:'Courier New';background:transparent;border:none;")

    def _apply_hover(self):
        if self._selected: return
        self.setStyleSheet("QFrame{background:#f0f0f0;border:2px solid #000000;}")
        self.name_lbl.setStyleSheet("color:#000000;font-size:20px;font-family:'Courier New';font-weight:700;background:transparent;border:none;letter-spacing:1px;")
        self.coord_lbl.setStyleSheet("color:#888888;font-size:15px;font-family:'Courier New';background:transparent;border:none;")

    def select(self):
        self._selected = True
        self._apply_selected()

    def deselect(self):
        self._selected = False
        self._apply_normal()

    def enterEvent(self, e):   self._apply_hover()
    def leaveEvent(self, e):
        self._apply_selected() if self._selected else self._apply_normal()
    def mousePressEvent(self, e):
        if self._callback: self._callback(self)


# --- status indicator --------------------------------------------------------

class StatusIndicator(QWidget):
    def __init__(self):
        super().__init__()
        self._state = STATE_IDLE
        self.setFixedHeight(48)
        self.setStyleSheet("background:#ffffff;")
        hl = QHBoxLayout(self)
        hl.setContentsMargins(0, 0, 0, 0)
        hl.setSpacing(12)
        hl.setAlignment(Qt.AlignCenter)

        self._dot = QFrame()
        self._dot.setFixedSize(14, 14)
        self._dot.setStyleSheet("background:#cccccc;border-radius:7px;border:none;")
        self._pulse = None

        self._lbl     = QLabel("IDLE — SELECT A DESTINATION")
        self._lbl.setStyleSheet("font-size:16px;font-family:'Courier New';color:#888888;letter-spacing:3px;font-weight:700;")

        self._pos_lbl = QLabel("")
        self._pos_lbl.setStyleSheet("font-size:16px;font-family:'Courier New';color:#000000;letter-spacing:2px;")

        hl.addWidget(self._dot)
        hl.addWidget(self._lbl)
        hl.addWidget(self._pos_lbl)

    def update_state(self, state, position_name=""):
        colors = {
            STATE_IDLE:    ("#cccccc", "#888888", "IDLE — SELECT A DESTINATION"),
            STATE_RUNNING: ("#22c55e", "#22c55e", "RUNNING"),
            STATE_STOPPED: ("#ef4444", "#ef4444", "STOPPED"),
        }
        dot_c, txt_c, label = colors.get(state, colors[STATE_IDLE])
        self._dot.setStyleSheet(f"background:{dot_c};border-radius:7px;border:none;")
        self._lbl.setStyleSheet(f"font-size:16px;font-family:'Courier New';color:{txt_c};letter-spacing:3px;font-weight:700;")
        self._lbl.setText(label)
        self._pos_lbl.setText(f"→  {position_name}" if position_name else "")


# --- main start panel --------------------------------------------------------

class StartPanel(QWidget):
    def __init__(self, on_back=None, rviz_config=None, parent_window=None):
        super().__init__()
        self.on_back         = on_back
        self._selected_btn   = None
        self._state          = STATE_IDLE
        self._position_btns  = []
        self._parent_window  = parent_window
        self.setStyleSheet("background:#ffffff;")
        fade_in(self, duration=300)

        self._rviz = RVizLauncher(rviz_config=rviz_config)
        self._nav  = Nav2Bridge()
        self._nav.goal_accepted.connect(self._on_goal_accepted)
        self._nav.goal_succeeded.connect(self._on_goal_succeeded)
        self._nav.goal_failed.connect(self._on_goal_failed)
        self._nav.start()

        self._build_ui()
        QTimer.singleShot(800, self._launch_rviz)

    def _launch_rviz(self):
        from PySide6.QtWidgets import QApplication
        if self._parent_window:
            geo = self._parent_window.geometry()
            hx, hy, hw, hh = geo.x(), geo.y(), geo.width(), geo.height()
        else:
            screen = QApplication.primaryScreen().geometry()
            hx, hy, hw, hh = 0, 0, screen.width() // 2, screen.height()
        self._rviz.start(hx, hy, hw, hh)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        # title row
        title_row = QHBoxLayout()
        title = QLabel("ចាប់ផ្តើមមុខងារ  /  Start Mission")
        title.setStyleSheet("font-size:24px;font-family:'Noto Sans Khmer','Courier New';font-weight:700;color:#000;letter-spacing:1px;")
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self._flat_btn("← ត្រឡប់ / Return", inverted=False, width=160, height=40, callback=self._go_back))
        root.addLayout(title_row)

        sub = QLabel("ជ្រើសរើសទិសដៅ រួចចុច Start ដើម្បីចាក់ផ្តើម  /  Select a destination then press Start.")
        sub.setStyleSheet("font-size:16px;font-family:'Noto Sans Khmer','Courier New';color:#888;")
        sub.setWordWrap(True)
        root.addWidget(sub)

        # status
        self._status = StatusIndicator()
        root.addWidget(self._status)

        div = QFrame(); div.setFrameShape(QFrame.HLine); div.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div)

        # controls
        ctrl_lbl = QLabel("ប៊ូតុងបញ្ជា  /  CONTROLS")
        ctrl_lbl.setStyleSheet("font-size:14px;font-family:'Courier New';letter-spacing:3px;color:#aaa;")
        root.addWidget(ctrl_lbl)

        ctrl_row = QHBoxLayout()
        ctrl_row.setSpacing(12)
        ctrl_row.setAlignment(Qt.AlignCenter)

        self._btn_start  = ControlButton("▶  Start",   accent="#000000", callback=self._on_start)
        self._btn_stop   = ControlButton("⏹  Stop",    accent="#ef4444", callback=self._on_stop)

        for b in [self._btn_start, self._btn_stop]:
            ctrl_row.addWidget(b)

        root.addLayout(ctrl_row)
        self._update_controls()

        div2 = QFrame(); div2.setFrameShape(QFrame.HLine); div2.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div2)

        # destinations
        dest_lbl = QLabel("ជ្រើសទិសដៅ  /  SELECT DESTINATION")
        dest_lbl.setStyleSheet("font-size:14px;font-family:'Courier New';letter-spacing:3px;color:#aaa;")
        root.addWidget(dest_lbl)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea{border:2px solid #000;}")
        scroll_content = QWidget()
        scroll_content.setStyleSheet("background:#ffffff;")
        self._grid = QGridLayout(scroll_content)
        self._grid.setSpacing(8)
        self._grid.setContentsMargins(12, 12, 12, 12)
        scroll.setWidget(scroll_content)
        root.addWidget(scroll, stretch=1)

        self._load_positions()


    def _load_positions(self):
        while self._grid.count():
            item = self._grid.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self._position_btns.clear()
        self._selected_btn = None

        positions = load_positions()
        if not positions:
            empty = QLabel("មិនទាន់មានទីតាំងទេ\nNo positions saved yet. Go to Setup Desire Positions first.")
            empty.setAlignment(Qt.AlignCenter)
            empty.setStyleSheet("font-size:17px;font-family:'Noto Sans Khmer','Courier New';color:#aaa;")
            self._grid.addWidget(empty, 0, 0)
            return

        cols = 3
        for i, pos in enumerate(positions):
            btn = PositionButton(
                name    = pos["name"],
                x       = pos["x"],
                y       = pos["y"],
                yaw_deg = pos.get("yaw_deg", 0.0),
                callback= self._on_position_selected
            )
            self._grid.addWidget(btn, i // cols, i % cols)
            self._position_btns.append(btn)
        # stagger fade in position buttons
        stagger_fade_in(self._position_btns, base_delay=100, step=80)

    def _on_position_selected(self, btn):
        if self._state == STATE_RUNNING:
            return
        for b in self._position_btns:
            b.deselect()
        btn.select()
        self._selected_btn = btn
        self._update_controls()
        self._status.update_state(STATE_IDLE, btn.pos_name)

    def _on_start(self):
        if not self._selected_btn:
            return
        self._state = STATE_RUNNING
        self._status.update_state(STATE_RUNNING, self._selected_btn.pos_name)
        self._update_controls()
        # send goal to Nav2
        self._nav.send_goal(
            self._selected_btn.pos_x,
            self._selected_btn.pos_y,
            self._selected_btn.pos_yaw
        )

    def _on_stop(self):
        self._nav.cancel()
        self._state  = STATE_STOPPED
        pos_name     = self._selected_btn.pos_name if self._selected_btn else ""
        if pos_name:
            record_trip(pos_name, "cancelled")
        self._status.update_state(STATE_STOPPED, pos_name)
        self._update_controls()

    # --- Nav2 callbacks ------------------------------------------------------

    def _on_goal_accepted(self):
        self._status.update_state(STATE_RUNNING, self._selected_btn.pos_name if self._selected_btn else "")

    def _on_goal_succeeded(self):
        self._state  = STATE_IDLE
        pos_name     = self._selected_btn.pos_name if self._selected_btn else ""
        record_trip(pos_name, "success")
        self._status.update_state(STATE_IDLE, f"{pos_name} — Arrived!")
        self._update_controls()

    def _on_goal_failed(self, reason):
        self._state  = STATE_STOPPED
        pos_name     = self._selected_btn.pos_name if self._selected_btn else ""
        record_trip(pos_name, "failed")
        self._status.update_state(STATE_STOPPED, reason)
        self._update_controls()

    # --- helpers -------------------------------------------------------------

    def _update_controls(self):
        has_selection = self._selected_btn is not None
        is_running    = self._state == STATE_RUNNING
        self._btn_start.set_enabled(has_selection and self._state == STATE_IDLE or self._state == STATE_STOPPED)
        self._btn_stop.set_enabled(is_running)

    def _go_back(self):
        self._nav.cancel()
        self._nav.stop()
        self._rviz.stop()
        if self.on_back:
            self.on_back()

    def _flat_btn(self, text, inverted=True, callback=None, width=120, height=40):
        f = QFrame()
        f.setCursor(QCursor(Qt.PointingHandCursor))
        f.setFixedSize(width, height)
        lv = QVBoxLayout(f)
        lv.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        if inverted:
            f.setStyleSheet("QFrame{background:#000;border:2px solid #000;} QFrame:hover{background:#333;}")
            lbl.setStyleSheet("color:#fff;font-size:15px;font-family:'Courier New';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        else:
            f.setStyleSheet("QFrame{background:#fff;border:2px solid #000;} QFrame:hover{background:#f0f0f0;}")
            lbl.setStyleSheet("color:#000;font-size:15px;font-family:'Courier New';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        lv.addWidget(lbl)
        if callback:
            f.mousePressEvent = lambda e: callback()
        return f

    def closeEvent(self, e):
        self._nav.cancel()
        self._nav.stop()
        self._rviz.stop()
        super().closeEvent(e)