import math
import json
import os
import subprocess
import threading
import time
# numpy imported lazily in set_map

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QScrollArea, QFrame, QLineEdit, QSizePolicy,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QMessageBox, QApplication,
    QStyledItemDelegate
)
from PySide6.QtCore import Qt, Signal, QObject, QTimer
from PySide6.QtGui import QCursor, QWindow

try:
    import rclpy
    from rclpy.qos import (
        QoSProfile, QoSDurabilityPolicy,
        QoSReliabilityPolicy, QoSHistoryPolicy
    )
    from geometry_msgs.msg import PoseStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
POSITIONS_FILE = os.path.join(BASE_DIR, 'saved_positions.json')


# --- style helpers -----------------------------------------------------------

def flat_btn(text, inverted=True, callback=None, width=120, height=40):
    f = QFrame()
    f.setCursor(QCursor(Qt.PointingHandCursor))
    f.setFixedSize(width, height)
    lv = QVBoxLayout(f)
    lv.setContentsMargins(0, 0, 0, 0)
    lbl = QLabel(text.upper())
    lbl.setAlignment(Qt.AlignCenter)
    if inverted:
        f.setStyleSheet("QFrame{background:#000;border:2px solid #000;} QFrame:hover{background:#333;border:2px solid #333;}")
        lbl.setStyleSheet("color:#fff;font-size:14px;font-family:'Courier New';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
    else:
        f.setStyleSheet("QFrame{background:#fff;border:2px solid #000;} QFrame:hover{background:#f0f0f0;border:2px solid #000;}")
        lbl.setStyleSheet("color:#000;font-size:14px;font-family:'Courier New';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
    lv.addWidget(lbl)
    if callback:
        f.mousePressEvent = lambda e: callback()
    return f


# --- RViz launcher -----------------------------------------------------------

class RVizLauncher:
    """
    Launches rviz2 on the RIGHT half of the screen.
    Killed cleanly when stop() is called.
    """
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

        # position rviz2 right next to the HMI window after it loads
        rviz_x = hmi_x + hmi_w
        threading.Thread(
            target=self._reposition,
            args=(rviz_x, hmi_y, hmi_w, hmi_h),
            daemon=True
        ).start()

    def _reposition(self, x, y, w, h):
        time.sleep(3)  # wait for rviz2 window to appear
        if self._proc and self._proc.poll() is None:
            try:
                # find rviz2 window by PID and move+resize it
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
                    subprocess.run(
                        ["xdotool", "windowmove", wid, str(x), str(y)],
                        timeout=3
                    )
                    subprocess.run(
                        ["xdotool", "windowsize", wid, str(w), str(h)],
                        timeout=3
                    )
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


# --- ROS2 bridge -------------------------------------------------------------

class RosBridge(QObject):
    """
    Subscribes to /setup_goal_marker (NOT /goal_pose).
    Robot never moves — we just record the pose.
    """
    goal_received = Signal(float, float, float)  # x, y, yaw

    def __init__(self):
        super().__init__()
        self._node    = None
        self._thread  = None
        self._running = False

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
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception:
                pass

        self._node = rclpy.create_node("setup_panel_bridge")
        self._node.create_subscription(
            PoseStamped, "/setup_goal_marker", self._on_goal, 10
        )
        self._node.get_logger().info(
            "SetupBridge: listening on /setup_goal_marker (robot will NOT move)"
        )
        while self._running:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        # do NOT call rclpy.shutdown() — other nodes may still be running

    def _on_goal(self, msg):
        x   = msg.pose.position.x
        y   = msg.pose.position.y
        qz  = msg.pose.orientation.z
        qw  = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.goal_received.emit(round(float(x), 3), round(float(y), 3), float(yaw))

    def stop(self):
        self._running = False




# --- positions persistence ---------------------------------------------------

def load_positions():
    if os.path.exists(POSITIONS_FILE):
        with open(POSITIONS_FILE, 'r') as f:
            return json.load(f)
    return []

def save_positions(positions):
    with open(POSITIONS_FILE, 'w') as f:
        json.dump(positions, f, indent=2)


# --- custom delegate to force black text while editing --------------------

class BlackTextDelegate(QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        editor = super().createEditor(parent, option, index)
        if editor:
            editor.setStyleSheet(
                "color: #000000; background: #fffde7;"
                "font-family: 'Courier New'; font-size: 12px;"
                "border: 2px solid #000000; padding: 2px;"
            )
        return editor


# --- positions table ---------------------------------------------------------

class PositionsTable(QTableWidget):
    def __init__(self):
        super().__init__(0, 5)
        self.setHorizontalHeaderLabels(["Name", "X", "Y", "Yaw (deg)", "Actions"])
        self.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for col in [1, 2, 3, 4]:
            self.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.verticalHeader().setVisible(False)
        self.setStyleSheet("""
            QTableWidget {
                border: 2px solid #000;
                font-family: 'Courier New';
                font-size: 12px;
                gridline-color: #e0e0e0;
                background: #fff;
            }
            QHeaderView::section {
                background: #000;
                color: #fff;
                font-family: 'Courier New';
                font-size: 10px;
                letter-spacing: 2px;
                padding: 6px;
                border: none;
            }
            QTableWidget::item { padding: 6px; color: #000000; }
            QTableWidget::item:selected { background: #f0f0f0; color: #000000; }
            QTableWidget::item:focus { color: #000000; background: #fffde7; }
        """)
        self._positions = []
        self.setItemDelegate(BlackTextDelegate(self))

    def add_position(self, name, x, y, yaw_rad):
        yaw_deg = round(math.degrees(yaw_rad), 1)
        self._positions.append({"name": name, "x": x, "y": y, "yaw": yaw_rad})
        row = self.rowCount()
        self.insertRow(row)
        from PySide6.QtGui import QColor
        for col, val in enumerate([name, str(x), str(y), str(yaw_deg)]):
            item = QTableWidgetItem(val)
            item.setForeground(QColor("#000000"))
            self.setItem(row, col, item)
        self._set_action_btns(row)
        self.setRowHeight(row, 40)

    def _set_action_btns(self, row, editing=False):
        cell = QWidget()
        hl   = QHBoxLayout(cell)
        hl.setContentsMargins(4, 2, 4, 2)
        hl.setSpacing(6)
        if not editing:
            hl.addWidget(flat_btn("Edit",   inverted=False, width=60, height=28, callback=lambda r=row: self._edit_row(r)))
            hl.addWidget(flat_btn("Delete", inverted=True,  width=60, height=28, callback=lambda r=row: self._delete_row(r)))
        else:
            hl.addWidget(flat_btn("Save",   inverted=True,  width=60, height=28, callback=lambda r=row: self._save_row(r)))
            hl.addWidget(flat_btn("Cancel", inverted=False, width=60, height=28, callback=lambda r=row: self._cancel_row(r)))
        self.setCellWidget(row, 4, cell)

    def _edit_row(self, row):
        from PySide6.QtGui import QColor
        self._editing_row = row
        for col in range(4):
            item = self.item(row, col)
            item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable)
            item.setForeground(QColor("#000000"))
        self.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.SelectedClicked | QAbstractItemView.AnyKeyPressed)
        self.setCurrentCell(row, 1)
        self.editItem(self.item(row, 1))
        self._set_action_btns(row, editing=True)

    def _save_row(self, row):
        from PySide6.QtGui import QColor
        for col in range(4):
            item = self.item(row, col)
            item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            item.setForeground(QColor("#000000"))
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        name    = self.item(row, 0).text()
        x       = float(self.item(row, 1).text())
        y       = float(self.item(row, 2).text())
        yaw_deg = float(self.item(row, 3).text())
        self._positions[row] = {"name": name, "x": x, "y": y, "yaw": math.radians(yaw_deg)}
        self._set_action_btns(row, editing=False)

    def _cancel_row(self, row):
        from PySide6.QtGui import QColor
        # restore original values first
        pos = self._positions[row]
        for col, val in enumerate([pos["name"], str(pos["x"]), str(pos["y"]), str(round(math.degrees(pos["yaw"]), 1))]):
            item = self.item(row, col)
            item.setText(val)
            item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            item.setForeground(QColor("#000000"))
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._set_action_btns(row, editing=False)

    def _delete_row(self, row):
        reply = QMessageBox.question(
            self, "Delete", f"Delete '{self.item(row, 0).text()}'?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.removeRow(row)
            self._positions.pop(row)
            for r in range(self.rowCount()):
                self._set_action_btns(r)

    def get_positions(self):
        return list(self._positions)


# --- main panel --------------------------------------------------------------

class SetupDesirePositionsPanel(QWidget):
    def __init__(self, on_back=None, rviz_config=None, parent_window=None):
        super().__init__()
        self.on_back       = on_back
        self._pos_count    = 0
        self._parent_win   = parent_window   # QMainWindow ref for geometry
        self._rviz_config  = rviz_config
        self.setStyleSheet("background:#ffffff;")

        self._rviz    = RVizLauncher(rviz_config=rviz_config)
        self._ros     = RosBridge()
        self._ros.goal_received.connect(self._on_goal_received)
        self._ros.start()

        self._build_ui()

        # launch rviz after a short delay so window is fully shown
        QTimer.singleShot(800, self._launch_rviz)

    def _launch_rviz(self):
        if self._parent_win:
            geo   = self._parent_win.geometry()
            hx, hy, hw, hh = geo.x(), geo.y(), geo.width(), geo.height()
        else:
            screen = QApplication.primaryScreen().geometry()
            hx, hy = 0, 0
            hw     = screen.width() // 2
            hh     = screen.height()
            # also resize HMI to left half
            if self._parent_win:
                self._parent_win.setGeometry(hx, hy, hw, hh)

        self._rviz.start(hx, hy, hw, hh)

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea{border:none;}")
        outer.addWidget(scroll)

        content = QWidget()
        content.setStyleSheet("background:#ffffff;")
        scroll.setWidget(content)

        root = QVBoxLayout(content)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        # title
        title = QLabel("Setup Desire Positions")
        title.setStyleSheet("font-size:26px;font-family:'Courier New';font-weight:700;color:#000;letter-spacing:2px;")
        root.addWidget(title)

        sub = QLabel(
            "នៅក្នុង RViz (បង្អួចខាងស្ដាំ): ជ្រើស '2D Goal Pose' ហើយចុចលើផែនទី — X/Y/Yaw នឹងបំពេញដោយស្វ័យប្រវត្តិ។ "
            "រ៉ូបូតនឹងមិនផ្លាស់ទីទេ។ វាយឈ្មោះ ហើយចុចរក្សាទុក  /  "
            "In RViz (right window): select '2D Goal Pose', click map — auto-fill below. Robot will NOT move. Enter name and Save."
        )
        sub.setStyleSheet("font-size:15px;font-family:'Courier New';color:#888;letter-spacing:1px;")
        sub.setWordWrap(True)
        root.addWidget(sub)

        # status bar showing last received pose
        self._status_bar = QFrame()
        self._status_bar.setStyleSheet("QFrame{background:#f5f5f5;border:2px solid #000;}")
        self._status_bar.setFixedHeight(44)
        sb = QHBoxLayout(self._status_bar)
        sb.setContentsMargins(16, 0, 16, 0)
        self._status_lbl = QLabel("កំពុងរង់ចាំការជ្រើសរើស 2D Goal Pose នៅក្នុង RViz...  /  Waiting for 2D Goal Pose in RViz...")
        self._status_lbl.setStyleSheet("font-size:15px;font-family:'Courier New';color:#888;background:transparent;border:none;")
        sb.addWidget(self._status_lbl)
        root.addWidget(self._status_bar)

        # record position entry
        rec_lbl = QLabel("RECORD POSITION")
        rec_lbl.setStyleSheet("font-size:14px;font-family:'Courier New';letter-spacing:3px;color:#aaa;")
        root.addWidget(rec_lbl)

        entry = QFrame()
        entry.setStyleSheet("QFrame{background:#f5f5f5;border:2px solid #000;}")
        entry.setFixedHeight(64)
        ef = QHBoxLayout(entry)
        ef.setContentsMargins(16, 0, 16, 0)
        ef.setSpacing(16)

        self._inp_x   = self._make_input("X", "—", 90)
        self._inp_y   = self._make_input("Y", "—", 90)
        self._inp_yaw = self._make_input("Yaw (deg)", "—", 100)

        for w in [self._inp_x, self._inp_y, self._inp_yaw]:
            ef.addWidget(w)

        ef.addStretch()

        name_lbl = QLabel("Name:")
        name_lbl.setStyleSheet("font-size:15px;font-family:'Courier New';color:#555;background:transparent;border:none;")
        ef.addWidget(name_lbl)

        self._name_input = QLineEdit()
        self._name_input.setPlaceholderText("ឧ. តុទី ១  /  e.g. Station 1")
        self._name_input.setFixedWidth(150)
        self._name_input.setFixedHeight(36)
        self._name_input.setStyleSheet("border:none;border-bottom:2px solid #000;font-family:'Courier New';font-size:17px;background:transparent;padding:4px;color:#000000;")
        self._name_input.returnPressed.connect(self._save_pose)
        ef.addWidget(self._name_input)

        ef.addWidget(flat_btn("Save", inverted=True, width=80, height=36, callback=self._save_pose))
        root.addWidget(entry)

        # table
        tbl_lbl = QLabel("SAVED POSITIONS")
        tbl_lbl.setStyleSheet("font-size:14px;font-family:'Courier New';letter-spacing:3px;color:#aaa;")
        root.addWidget(tbl_lbl)

        self._table = PositionsTable()
        self._table.setMinimumHeight(200)
        root.addWidget(self._table)
        self._load_saved_positions()

        # return button — bottom right
        bottom = QHBoxLayout()
        bottom.addStretch()
        bottom.addWidget(flat_btn("← Return", inverted=False, width=130, height=44, callback=self._go_back))
        root.addLayout(bottom)

    def _make_input(self, label, default, width):
        w  = QWidget()
        w.setStyleSheet("background:transparent;")
        vl = QVBoxLayout(w)
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(2)
        lbl = QLabel(label)
        lbl.setStyleSheet("font-size:13px;font-family:'Courier New';color:#888;background:transparent;border:none;letter-spacing:1px;")
        inp = QLineEdit(default)
        inp.setFixedWidth(width)
        inp.setFixedHeight(32)
        inp.setStyleSheet("border:none;border-bottom:2px solid #ccc;font-family:'Courier New';font-size:16px;background:transparent;padding:2px;color:#000000;")
        vl.addWidget(lbl)
        vl.addWidget(inp)
        # store the QLineEdit directly on the wrapper for easy access
        w.input = inp
        return w

    def _on_goal_received(self, x, y, yaw):
        """Called when user places 2D Goal Pose in RViz — auto-fills fields."""
        self._inp_x.input.setText(str(x))
        self._inp_y.input.setText(str(y))
        self._inp_yaw.input.setText(str(round(math.degrees(yaw), 1)))
        self._status_bar.setStyleSheet("QFrame{background:#e8f5e9;border:2px solid #000;}")
        self._status_lbl.setText(f"បានទទួលគោលដៅ  →  X: {x}   Y: {y}   Yaw: {round(math.degrees(yaw),1)}°  —  វាយឈ្មោះ ហើយចុចរក្សាទុក  /  Goal received — enter name and Save")
        self._status_lbl.setStyleSheet("font-size:15px;font-family:'Courier New';color:#000;background:transparent;border:none;")
        self._name_input.setFocus()

    def _save_pose(self):
        try:
            x   = float(self._inp_x.input.text())
            y   = float(self._inp_y.input.text())
            yaw = math.radians(float(self._inp_yaw.input.text()))
        except ValueError:
            return

        name = self._name_input.text().strip() or f"Position {self._pos_count + 1}"
        self._table.add_position(name, x, y, yaw)
        self._pos_count += 1
        self._name_input.clear()

        # reset status
        self._status_bar.setStyleSheet("QFrame{background:#f5f5f5;border:2px solid #000;}")
        self._status_lbl.setText("បានរក្សាទុក! កំណត់គោលដៅបន្ទាប់ ឬ ចុចត្រឡប់  /  Saved! Place another goal or click Return.")
        self._status_lbl.setStyleSheet("font-size:15px;font-family:'Courier New';color:#888;background:transparent;border:none;")

    def _load_saved_positions(self):
        for pos in load_positions():
            self._table.add_position(
                pos['name'], pos['x'], pos['y'],
                math.radians(pos['yaw_deg'])
            )
            self._pos_count += 1

    def _go_back(self):
        self._persist_positions()
        self._rviz.stop()
        self._ros.stop()
        if self.on_back:
            self.on_back()

    def _persist_positions(self):
        data = []
        for pos in self._table.get_positions():
            data.append({
                'name':    pos['name'],
                'x':       pos['x'],
                'y':       pos['y'],
                'yaw_deg': round(math.degrees(pos['yaw']), 3)
            })
        save_positions(data)

    def closeEvent(self, e):
        self._persist_positions()
        self._rviz.stop()
        self._ros.stop()
        super().closeEvent(e)