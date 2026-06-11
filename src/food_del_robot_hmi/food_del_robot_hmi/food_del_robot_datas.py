#!/usr/bin/env python3
import os
import json
import math
import threading
import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QScrollArea, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QCursor, QPainter, QColor, QPen, QBrush, QFont

try:
    from food_del_robot_animations import fade_in
except ImportError:
    def fade_in(w, **kw): pass

ROS_AVAILABLE = True  # checked at runtime

BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
TRIPS_FILE = os.path.join(BASE_DIR, "trip_history.json")
POSITIONS_FILE = os.path.join(BASE_DIR, "saved_positions.json")

KHMER = "Noto Sans Khmer"
MONO  = "Courier New"


# --- data helpers ------------------------------------------------------------

def load_trips():
    if os.path.exists(TRIPS_FILE):
        with open(TRIPS_FILE, "r") as f:
            return json.load(f)
    return []

def save_trips(trips):
    with open(TRIPS_FILE, "w") as f:
        json.dump(trips, f, indent=2)

def load_positions():
    if os.path.exists(POSITIONS_FILE):
        with open(POSITIONS_FILE, "r") as f:
            return json.load(f)
    return []

def record_trip(destination, status):
    """Record a trip entry. Called from StartPanel when goal completes."""
    trips = load_trips()
    trips.append({
        "destination": destination,
        "status":      status,        # "success" | "failed" | "cancelled"
        "timestamp":   datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    })
    save_trips(trips)


# --- ROS bridge for live status ----------------------------------------------

class DatasBridge(QObject):
    status_updated = Signal()

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
        # only init if not already initialized
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception:
                pass
        self._node = rclpy.create_node("datas_panel_bridge")
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self._node)
        while self._running:
            executor.spin_once(timeout_sec=0.1)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        # do NOT call rclpy.shutdown() — other nodes may still be running

    def stop(self):
        self._running = False


# --- stat card widget --------------------------------------------------------

class StatCard(QFrame):
    def __init__(self, value, label_kh, label_en, accent="#000000"):
        super().__init__()
        self.setStyleSheet(f"QFrame{{background:#ffffff;border:2px solid #000000;}}")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(130)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(4)
        layout.setAlignment(Qt.AlignCenter)

        self._val_lbl = QLabel(str(value))
        self._val_lbl.setAlignment(Qt.AlignCenter)
        self._val_lbl.setStyleSheet(f"color:{accent};font-size:44px;font-family:'{MONO}';font-weight:700;background:transparent;border:none;")

        kh = QLabel(label_kh)
        kh.setAlignment(Qt.AlignCenter)
        kh.setStyleSheet(f"color:#000;font-size:17px;font-family:'{KHMER}';font-weight:700;background:transparent;border:none;")

        en = QLabel(label_en.upper())
        en.setAlignment(Qt.AlignCenter)
        en.setStyleSheet(f"color:#888;font-size:12px;font-family:'{MONO}';letter-spacing:2px;background:transparent;border:none;")

        layout.addWidget(self._val_lbl)
        layout.addWidget(kh)
        layout.addWidget(en)

    def set_value(self, v):
        self._val_lbl.setText(str(v))


# --- trip row ----------------------------------------------------------------

def trip_row(trip):
    """Build one row frame for a trip entry."""
    f = QFrame()
    status = trip.get("status", "unknown")
    if status == "success":
        border_color = "#22c55e"
        dot_color    = "#22c55e"
        status_kh    = "ដល់ហើយ"
        status_en    = "ARRIVED"
    elif status == "failed":
        border_color = "#ef4444"
        dot_color    = "#ef4444"
        status_kh    = "បរាជ័យ"
        status_en    = "FAILED"
    else:
        border_color = "#f59e0b"
        dot_color    = "#f59e0b"
        status_kh    = "បោះបង់"
        status_en    = "CANCELLED"

    f.setStyleSheet(f"QFrame{{background:#ffffff;border-left:4px solid {border_color};border-top:1px solid #e0e0e0;border-right:none;border-bottom:none;}}")
    f.setFixedHeight(64)

    hl = QHBoxLayout(f)
    hl.setContentsMargins(16, 0, 16, 0)
    hl.setSpacing(16)

    # dot
    dot = QFrame()
    dot.setFixedSize(12, 12)
    dot.setStyleSheet(f"background:{dot_color};border-radius:6px;border:none;")
    hl.addWidget(dot)

    # destination
    dest = QLabel(trip.get("destination", "—"))
    dest.setStyleSheet(f"font-size:16px;font-family:'{MONO}';font-weight:700;color:#000;background:transparent;border:none;")
    hl.addWidget(dest)
    hl.addStretch()

    # status
    st_col = QVBoxLayout()
    st_col.setSpacing(0)
    st_kh = QLabel(status_kh)
    st_kh.setAlignment(Qt.AlignRight)
    st_kh.setStyleSheet(f"font-size:14px;font-family:'{KHMER}';color:{dot_color};font-weight:700;background:transparent;border:none;")
    st_en = QLabel(status_en)
    st_en.setAlignment(Qt.AlignRight)
    st_en.setStyleSheet(f"font-size:11px;font-family:'{MONO}';color:#aaa;letter-spacing:2px;background:transparent;border:none;")
    st_col.addWidget(st_kh)
    st_col.addWidget(st_en)
    hl.addLayout(st_col)

    # timestamp
    ts = QLabel(trip.get("timestamp", ""))
    ts.setStyleSheet(f"font-size:12px;font-family:'{MONO}';color:#ccc;background:transparent;border:none;")
    ts.setFixedWidth(140)
    ts.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    hl.addWidget(ts)

    return f


# --- main datas panel --------------------------------------------------------

class DatasPanel(QWidget):
    def __init__(self, on_back=None):
        super().__init__()
        self.on_back = on_back
        self.setStyleSheet("background:#ffffff;")
        fade_in(self, duration=300)
        self._build_ui()
        self._refresh()

        # auto-refresh every 8 seconds (only when visible)
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        # timer started in showEvent

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(32, 32, 32, 24)
        root.setSpacing(20)

        # title row
        title_row = QHBoxLayout()
        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        kh = QLabel("ទិន្នន័យ")
        kh.setStyleSheet(f"font-size:40px;font-family:'{KHMER}';font-weight:700;color:#000;")
        en = QLabel("DATAS & STATISTICS")
        en.setStyleSheet(f"font-size:18px;font-family:'{MONO}';font-weight:700;color:#000;letter-spacing:4px;")
        title_col.addWidget(kh)
        title_col.addWidget(en)
        title_row.addLayout(title_col)
        title_row.addStretch()
        title_row.addWidget(self._flat_btn("← ត្រឡប់\nReturn", inverted=False, callback=self._go_back))
        root.addLayout(title_row)

        div = QFrame(); div.setFrameShape(QFrame.HLine); div.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div)

        # stat cards row
        stats_lbl = QLabel("សង្ខេប  /  SUMMARY")
        stats_lbl.setStyleSheet(f"font-size:13px;font-family:'{MONO}';letter-spacing:3px;color:#aaa;")
        root.addWidget(stats_lbl)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(12)

        self._card_total    = StatCard("0", "ដំណើរការសរុប",   "Total Trips",    accent="#000000")
        self._card_success  = StatCard("0", "បានជោគជ័យ",      "Succeeded",      accent="#22c55e")
        self._card_failed   = StatCard("0", "បរាជ័យ",          "Failed",         accent="#ef4444")
        self._card_cancelled= StatCard("0", "បោះបង់",          "Cancelled",      accent="#f59e0b")
        self._card_rate     = StatCard("0%","អត្រាជោគជ័យ",    "Success Rate",   accent="#000000")

        for card in [self._card_total, self._card_success, self._card_failed,
                     self._card_cancelled, self._card_rate]:
            cards_row.addWidget(card)

        root.addLayout(cards_row)

        div2 = QFrame(); div2.setFrameShape(QFrame.HLine); div2.setStyleSheet("color:#e0e0e0;")
        root.addWidget(div2)

        # trip history
        hist_row = QHBoxLayout()
        hist_lbl = QLabel("ប្រវត្តិដំណើរ  /  TRIP HISTORY")
        hist_lbl.setStyleSheet(f"font-size:13px;font-family:'{MONO}';letter-spacing:3px;color:#aaa;")
        hist_row.addWidget(hist_lbl)
        hist_row.addStretch()
        hist_row.addWidget(self._flat_btn("Clear History", inverted=True,
                                          width=140, height=40, callback=self._clear_history))
        root.addLayout(hist_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea{border:2px solid #000;}")

        self._history_widget = QWidget()
        self._history_widget.setStyleSheet("background:#ffffff;")
        self._history_layout = QVBoxLayout(self._history_widget)
        self._history_layout.setContentsMargins(0, 0, 0, 0)
        self._history_layout.setSpacing(0)
        self._history_layout.addStretch()

        scroll.setWidget(self._history_widget)
        root.addWidget(scroll, stretch=1)

    def _refresh(self):
        trips = load_trips()

        total     = len(trips)
        succeeded = sum(1 for t in trips if t.get("status") == "success")
        failed    = sum(1 for t in trips if t.get("status") == "failed")
        cancelled = sum(1 for t in trips if t.get("status") == "cancelled")
        rate      = f"{int(succeeded/total*100)}%" if total > 0 else "0%"

        self._card_total.set_value(total)
        self._card_success.set_value(succeeded)
        self._card_failed.set_value(failed)
        self._card_cancelled.set_value(cancelled)
        self._card_rate.set_value(rate)

        # rebuild history list
        while self._history_layout.count() > 1:
            item = self._history_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if not trips:
            empty = QLabel("មិនទាន់មានទិន្នន័យ  /  No trip data yet.")
            empty.setAlignment(Qt.AlignCenter)
            empty.setStyleSheet(f"font-size:16px;font-family:'{KHMER}';color:#aaa;")
            self._history_layout.insertWidget(0, empty)
        else:
            for i, trip in enumerate(reversed(trips)):   # newest first
                self._history_layout.insertWidget(i, trip_row(trip))

    def _clear_history(self):
        save_trips([])
        self._refresh()

    def showEvent(self, e):
        if hasattr(self, '_timer'):
            self._timer.start(5000)
        super().showEvent(e)

    def hideEvent(self, e):
        if hasattr(self, '_timer'):
            self._timer.stop()
        super().hideEvent(e)

    def _go_back(self):
        self._timer.stop()
        if self.on_back:
            self.on_back()

    def _flat_btn(self, text, inverted=True, callback=None, width=160, height=52):
        f = QFrame()
        f.setCursor(QCursor(Qt.PointingHandCursor))
        f.setFixedSize(width, height)
        lv = QVBoxLayout(f)
        lv.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setWordWrap(True)
        if inverted:
            f.setStyleSheet("QFrame{background:#000;border:2px solid #000;} QFrame:hover{background:#333;}")
            lbl.setStyleSheet(f"color:#fff;font-size:13px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        else:
            f.setStyleSheet("QFrame{background:#fff;border:2px solid #000;} QFrame:hover{background:#f0f0f0;}")
            lbl.setStyleSheet(f"color:#000;font-size:13px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        lv.addWidget(lbl)
        if callback:
            f.mousePressEvent = lambda e: callback()
        return f