import os
from PySide6.QtWidgets import (
    QWidget, QGridLayout, QLabel,
    QVBoxLayout, QFrame, QSizePolicy,
)
from PySide6.QtCore import Qt, QPropertyAnimation, QEasingCurve, QTimer
from PySide6.QtGui import QCursor, QAction, QFont, QPainter, QColor, QPen, QBrush

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

KHMER = "Noto Sans Khmer"
MONO  = "Courier New"


# --- card button -------------------------------------------------------------

class CardButton(QFrame):
    def __init__(self, number, khmer_text, english_text, callback=None):
        super().__init__()
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self._callback = callback

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 22)
        layout.setSpacing(0)

        self.num_label   = QLabel(number)
        self.num_label.setAlignment(Qt.AlignLeft)

        self.khmer_label = QLabel(khmer_text)
        self.khmer_label.setAlignment(Qt.AlignCenter)
        self.khmer_label.setWordWrap(True)

        self.eng_label   = QLabel(english_text.upper())
        self.eng_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.num_label)
        layout.addStretch()
        layout.addWidget(self.khmer_label)
        layout.addSpacing(6)
        layout.addWidget(self.eng_label)
        layout.addStretch()

        self._apply_normal()

    def _apply_normal(self):
        self.setStyleSheet("QFrame{background-color:#ffffff;border:2px solid #000000;}")
        self.num_label.setStyleSheet(f"color:#aaaaaa;font-size:13px;font-family:'{MONO}';letter-spacing:3px;background:transparent;border:none;")
        self.khmer_label.setStyleSheet(f"color:#000000;font-size:32px;font-family:'{KHMER}';font-weight:700;background:transparent;border:none;")
        self.eng_label.setStyleSheet(f"color:#000000;font-size:18px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")

    def _apply_hover(self):
        self.setStyleSheet("QFrame{background-color:#f0f0f0;border:2px solid #000000;}")
        self.num_label.setStyleSheet(f"color:#aaaaaa;font-size:13px;font-family:'{MONO}';letter-spacing:3px;background:transparent;border:none;")
        self.khmer_label.setStyleSheet(f"color:#000000;font-size:32px;font-family:'{KHMER}';font-weight:700;background:transparent;border:none;")
        self.eng_label.setStyleSheet(f"color:#000000;font-size:18px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")

    def _apply_pressed(self):
        self.setStyleSheet("QFrame{background-color:#000000;border:2px solid #000000;}")
        self.num_label.setStyleSheet(f"color:#555;font-size:13px;font-family:'{MONO}';letter-spacing:3px;background:transparent;border:none;")
        self.khmer_label.setStyleSheet(f"color:#ffffff;font-size:32px;font-family:'{KHMER}';font-weight:700;background:transparent;border:none;")
        self.eng_label.setStyleSheet(f"color:#ffffff;font-size:18px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")

    def enterEvent(self, e):       self._apply_hover()
    def leaveEvent(self, e):       self._apply_normal()
    def mousePressEvent(self, e):
        self._apply_pressed()
        # brief scale effect via geometry
        geo = self.geometry()
        shrink = self.geometry().adjusted(4, 4, -4, -4)
        anim = QPropertyAnimation(self, b"geometry", self)
        anim.setDuration(80)
        anim.setStartValue(geo)
        anim.setEndValue(shrink)
        anim.setEasingCurve(QEasingCurve.OutQuad)
        anim.start()
        self._press_anim = anim
    def mouseReleaseEvent(self, e):
        self._apply_hover()
        geo = self.geometry()
        expand = self.geometry().adjusted(-4, -4, 4, 4)
        anim = QPropertyAnimation(self, b"geometry", self)
        anim.setDuration(120)
        anim.setStartValue(geo)
        anim.setEndValue(expand)
        anim.setEasingCurve(QEasingCurve.OutBack)
        anim.finished.connect(lambda: self._callback() if self._callback else None)
        anim.start()
        self._release_anim = anim


# --- center circle START button ----------------------------------------------

class CircleStartButton(QWidget):
    """
    Circle START button with custom paintEvent.
    Safe because animations.py no longer uses QGraphicsOpacityEffect.
    """
    def __init__(self, callback=None):
        super().__init__()
        self._callback = callback
        self._hovered  = False
        self._pressed  = False
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setMinimumSize(180, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, e):
        from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont
        from PySide6.QtCore import QRect
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w  = self.width()
        h  = self.height()
        r  = min(w, h) // 2 - 8
        cx = w // 2
        cy = h // 2

        # shadow
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#e0e0e0")))
        painter.drawEllipse(cx - r - 4, cy - r - 4, (r+4)*2, (r+4)*2)

        # main fill
        if self._pressed:
            fill = QColor("#444444")
        elif self._hovered:
            fill = QColor("#222222")
        else:
            fill = QColor("#000000")

        painter.setBrush(QBrush(fill))
        painter.setPen(QPen(QColor("#000000"), 3))
        painter.drawEllipse(cx - r, cy - r, r*2, r*2)

        # inner ring
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawEllipse(cx - r+10, cy - r+10, (r-10)*2, (r-10)*2)

        # Khmer
        painter.setPen(QPen(QColor("#ffffff")))
        painter.setFont(QFont(KHMER, 18, QFont.Bold))
        painter.drawText(
            QRect(cx - r+12, cy - r+14, (r-12)*2, r-10),
            Qt.AlignHCenter | Qt.AlignBottom, "ចាប់ផ្តើម"
        )

        # English
        painter.setFont(QFont(MONO, 16, QFont.Bold))
        painter.drawText(
            QRect(cx - r+12, cy+4, (r-12)*2, r-10),
            Qt.AlignHCenter | Qt.AlignTop, "START"
        )
        painter.end()

    def enterEvent(self, e):
        self._hovered = True; self.update()
    def leaveEvent(self, e):
        self._hovered = False; self.update()
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._pressed = True; self.update()
    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._pressed = False; self._hovered = True; self.update()
            if self._callback: self._callback()


# --- home panel --------------------------------------------------------------

class HomePanel(QWidget):
    def __init__(self, username, on_logout=None, on_navigate=None, on_start=None):
        super().__init__()
        self.username    = username
        self.on_logout   = on_logout
        self.on_navigate = on_navigate
        self.on_start    = on_start
        self.setStyleSheet("background-color:#ffffff;")
        self._setup_ui()

    def _setup_ui(self):
        layout = QGridLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        layout.setColumnStretch(0, 5)
        layout.setColumnStretch(1, 3)
        layout.setColumnStretch(2, 5)
        layout.setRowStretch(0, 5)
        layout.setRowStretch(1, 3)
        layout.setRowStretch(2, 5)

        cards = [
            ("01", "កំណត់គោលដៅ",      "Setup Desire Positions", "setup",   0, 0),
            ("02", "បង្កើតផែនទី",      "Mapping",                "mapping", 0, 2),
            ("03", "កំណែទម្រង់ផែនទី",  "Edit Map",               "editmap", 2, 0),
            ("04", "ទិន្នន័យ",           "Datas",                  "datas",   2, 2),
        ]

        for num, khmer, eng, key, row, col in cards:
            btn = CardButton(num, khmer, eng, callback=lambda k=key: self._navigate(k))
            layout.addWidget(btn, row, col)

        # connector widgets (white spacers between cards and circle)
        for row, col in [(0,1),(1,0),(1,2),(2,1)]:
            sp = QWidget()
            sp.setStyleSheet("background:#ffffff;")
            layout.addWidget(sp, row, col)

        # center START circle
        self._start_btn = CircleStartButton(callback=self._on_start)
        layout.addWidget(self._start_btn, 1, 1)

    def _navigate(self, key):
        if self.on_navigate: self.on_navigate(key)

    def _on_start(self):
        if self.on_start: self.on_start()

    def setup_menu(self, menubar):
        menubar.clear()
        about_menu = menubar.addMenu("ABOUT US")
        about_act  = QAction("Food Delivery Robot v1.0", self)
        about_act.setEnabled(False)
        about_menu.addAction(about_act)

        user_menu    = menubar.addMenu(f"👤 {self.username.upper()}")
        logout_act   = QAction("ចាកចេញ / Log Out", self)
        logout_act.triggered.connect(self._do_logout)
        user_menu.addAction(logout_act)

    def _do_logout(self):
        if self.on_logout: self.on_logout()