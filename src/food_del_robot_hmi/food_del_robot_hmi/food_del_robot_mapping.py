#!/usr/bin/env python3
import os
import subprocess
import threading
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QLineEdit, QSizePolicy, QFileDialog
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QCursor

try:
    from food_del_robot_animations import fade_in, stagger_fade_in
except ImportError:
    def fade_in(w, **kw): pass
    def stagger_fade_in(ws, **kw): pass

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
ROS2_SETUP  = "/opt/ros/humble/setup.bash"
WS_SETUP    = "/home/panha/Food_Delivery_Robot/install/setup.bash"
LAUNCH_FILE = "/home/panha/Food_Delivery_Robot/src/food_del_robot_bringup/launch/simulation/slam_sim_hmi.launch.py"
MAPS_DIR    = "/home/panha/Food_Delivery_Robot/src/food_del_robot_bringup/maps/Simulation_map"

KHMER = "Noto Sans Khmer"
MONO  = "Courier New"


# --- helpers -----------------------------------------------------------------

def _sourced_cmd(cmd_str):
    """Wrap a command with ROS2 source."""
    prefix = ""
    if os.path.exists(ROS2_SETUP):
        prefix += f"source {ROS2_SETUP} && "
    if os.path.exists(WS_SETUP):
        prefix += f"source {WS_SETUP} && "
    return ["bash", "-c", prefix + cmd_str]


def flat_btn(text, inverted=True, callback=None, width=160, height=52):
    f = QFrame()
    f.setCursor(QCursor(Qt.PointingHandCursor))
    f.setFixedSize(width, height)
    lv = QVBoxLayout(f)
    lv.setContentsMargins(0, 0, 0, 0)
    lbl = QLabel(text)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setWordWrap(True)
    if inverted:
        f.setStyleSheet("QFrame{background:#000;border:2px solid #000;} QFrame:hover{background:#333;border:2px solid #333;}")
        lbl.setStyleSheet(f"color:#fff;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
    else:
        f.setStyleSheet("QFrame{background:#fff;border:2px solid #000;} QFrame:hover{background:#f0f0f0;border:2px solid #000;}")
        lbl.setStyleSheet(f"color:#000;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
    lv.addWidget(lbl)
    if callback:
        f.mousePressEvent = lambda e: callback()
    return f


# --- SLAM launcher -----------------------------------------------------------

class SlamLauncher(QObject):
    started   = Signal()
    stopped   = Signal()
    map_saved = Signal(bool, str)   # success, path

    def __init__(self):
        super().__init__()
        self._proc    = None
        self._running = False

    def start(self):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._launch, daemon=True).start()

    def _launch(self):
        prefix = ""
        if os.path.exists(ROS2_SETUP):
            prefix += f"source {ROS2_SETUP} && "
        if os.path.exists(WS_SETUP):
            prefix += f"source {WS_SETUP} && "

        cmd = prefix + f"ros2 launch {LAUNCH_FILE}"
        self._proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid
        )
        print(f"[SlamLauncher] PID: {self._proc.pid}")
        self.started.emit()

    def stop(self):
        self._running = False
        if self._proc and self._proc.poll() is None:
            # kill entire process group to stop all child nodes
            import signal
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    self._proc.kill()
        self._proc = None
        # kill slam_toolbox, hand_controller, and mapping rviz
        for pattern in ["slam_toolbox", "hand_controller", "food_del_robot_mapping.rviz", "online_async"]:
            subprocess.run(["pkill", "-f", pattern], capture_output=True)
        self.stopped.emit()

    def save_map(self, map_name):
        """Save the current SLAM map to the bringup maps directory."""
        os.makedirs(MAPS_DIR, exist_ok=True)
        map_path = os.path.join(MAPS_DIR, map_name)
        cmd = f"ros2 run nav2_map_server map_saver_cli -f {map_path}"
        def _run():
            print(f"[SlamLauncher] Saving map to: {map_path}")
            result = subprocess.run(_sourced_cmd(cmd), capture_output=True, text=True)
            success = result.returncode == 0
            print(result.stdout if success else result.stderr)
            self.map_saved.emit(success, map_path)
        threading.Thread(target=_run, daemon=True).start()
        return map_path


# --- mapping panel -----------------------------------------------------------

class MappingPanel(QWidget):
    def __init__(self, on_back=None):
        super().__init__()
        self.on_back  = on_back
        self._slam    = SlamLauncher()
        self._slam.started.connect(self._on_slam_started)
        self._slam.map_saved.connect(self._on_map_saved)
        self.setStyleSheet("background:#ffffff;")
        fade_in(self, duration=300)
        self._build_ui()

        # auto-start SLAM on open
        QTimer.singleShot(500, self._slam.start)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(32, 32, 32, 32)
        root.setSpacing(20)

        # title row
        title_row = QHBoxLayout()
        kh = QLabel("បង្កើតផែនទី")
        kh.setStyleSheet(f"font-size:36px;font-family:'{KHMER}';font-weight:700;color:#000;")
        en = QLabel("MAPPING")
        en.setStyleSheet(f"font-size:20px;font-family:'{MONO}';font-weight:700;color:#000;letter-spacing:4px;")
        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        title_col.addWidget(kh)
        title_col.addWidget(en)
        title_row.addLayout(title_col)
        title_row.addStretch()
        title_row.addWidget(flat_btn("← ត្រឡប់\nReturn", inverted=False, width=140, height=56, callback=self._go_back))
        root.addLayout(title_row)

        # status bar
        self._status_frame = QFrame()
        self._status_frame.setFixedHeight(52)
        self._status_frame.setStyleSheet("QFrame{background:#f5f5f5;border:2px solid #000;}")
        sl = QHBoxLayout(self._status_frame)
        sl.setContentsMargins(20, 0, 20, 0)
        sl.setSpacing(12)

        self._dot = QFrame()
        self._dot.setFixedSize(14, 14)
        self._dot.setStyleSheet("background:#f59e0b;border-radius:7px;border:none;")

        self._status_lbl = QLabel("កំពុងចាប់ផ្តើម SLAM...  /  Starting SLAM...")
        self._status_lbl.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#888;background:transparent;border:none;")
        sl.addWidget(self._dot)
        sl.addWidget(self._status_lbl)
        sl.addStretch()
        root.addWidget(self._status_frame)

        # divider
        div = QFrame(); div.setFrameShape(QFrame.HLine); div.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div)

        # instructions
        instr_lbl = QLabel("ការណែនាំ  /  INSTRUCTIONS")
        instr_lbl.setStyleSheet(f"font-size:13px;font-family:'{MONO}';letter-spacing:3px;color:#aaa;")
        root.addWidget(instr_lbl)

        instructions = [
            ("①", "បើក RViz ហើយប្រើ Teleop ឬ Hand Controller ដើម្បីបើកបររ៉ូបូត",
                   "Open RViz and use Teleop or Hand Controller to drive the robot around the map area."),
            ("②", "ផ្លាស់ទីរ៉ូបូតឱ្យបានគ្រប់គ្រាន់ ដើម្បីស្កេនបរិស្ថាន",
                   "Drive the robot around until the entire environment is scanned."),
            ("③", "នៅពេលផែនទីល្អ វាយឈ្មោះ ហើយចុចរក្សាទុក",
                   "When the map looks complete, enter a name below and click Save Map."),
        ]

        for num, kh_txt, en_txt in instructions:
            row = QHBoxLayout()
            row.setSpacing(16)

            num_lbl = QLabel(num)
            num_lbl.setFixedWidth(36)
            num_lbl.setStyleSheet(f"font-size:22px;font-family:'{MONO}';font-weight:700;color:#000;")
            num_lbl.setAlignment(Qt.AlignTop)

            txt_col = QVBoxLayout()
            txt_col.setSpacing(2)
            kl = QLabel(kh_txt)
            kl.setStyleSheet(f"font-size:17px;font-family:'{KHMER}';color:#000;")
            kl.setWordWrap(True)
            el = QLabel(en_txt)
            el.setStyleSheet(f"font-size:13px;font-family:'{MONO}';color:#888;letter-spacing:1px;")
            el.setWordWrap(True)
            txt_col.addWidget(kl)
            txt_col.addWidget(el)

            row.addWidget(num_lbl)
            row.addLayout(txt_col)
            root.addLayout(row)

        root.addStretch()

        # divider
        div2 = QFrame(); div2.setFrameShape(QFrame.HLine); div2.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div2)

        # save map section
        save_lbl = QLabel("រក្សាទុកផែនទី  /  SAVE MAP")
        save_lbl.setStyleSheet(f"font-size:13px;font-family:'{MONO}';letter-spacing:3px;color:#aaa;")
        root.addWidget(save_lbl)

        save_row = QHBoxLayout()
        save_row.setSpacing(12)

        name_lbl = QLabel("ឈ្មោះ / Name:")
        name_lbl.setStyleSheet(f"font-size:16px;font-family:'{KHMER}';color:#000;")
        save_row.addWidget(name_lbl)

        self._map_name = QLineEdit("my_map")
        self._map_name.setFixedHeight(48)
        self._map_name.setStyleSheet(f"""
            QLineEdit {{
                border: none;
                border-bottom: 3px solid #000;
                font-size: 17px;
                font-family: '{MONO}';
                color: #000;
                background: transparent;
                padding: 4px;
            }}
            QLineEdit:focus {{
                border-bottom: 3px solid #000;
            }}
        """)
        save_row.addWidget(self._map_name, stretch=1)

        save_row.addWidget(flat_btn("💾 រក្សាទុក\nSave Map", inverted=True, width=160, height=56, callback=self._save_map))
        root.addLayout(save_row)

        self._save_status = QLabel("")
        self._save_status.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#22c55e;")
        root.addWidget(self._save_status)

    def _on_slam_started(self):
        self._dot.setStyleSheet("background:#22c55e;border-radius:7px;border:none;")
        self._status_lbl.setText("SLAM កំពុងដំណើរការ — RViz បើករួចហើយ  /  SLAM running — RViz is open")
        self._status_lbl.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#000;background:transparent;border:none;")

    def _save_map(self):
        name = self._map_name.text().strip()
        if not name:
            name = "my_map"
        self._save_status.setText("⏳  កំពុងរក្សាទុក...  /  Saving map...")
        self._save_status.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#f59e0b;")
        self._slam.save_map(name)

    def _on_map_saved(self, success, path):
        if success:
            self._save_status.setText(f"✓  បានរក្សាទុក:  {path}.yaml  /  Map saved!")
            self._save_status.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#22c55e;")
        else:
            self._save_status.setText("✕  រក្សាទុកបរាជ័យ  /  Save failed.")
            self._save_status.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#ef4444;")

    def _go_back(self):
        self._slam.stop()
        if self.on_back:
            self.on_back()

    def closeEvent(self, e):
        self._slam.stop()
        super().closeEvent(e)