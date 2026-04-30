import json
import os
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QFrame, QScrollArea,
    QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QRect, QPoint
from PySide6.QtGui import (
    QCursor, QPainter, QColor, QPen,
    QBrush, QFont, QLinearGradient
)

BASE_DIR  = os.path.dirname(os.path.abspath(__file__))
USER_FILE = os.path.join(BASE_DIR, "user_data.json")

KHMER = "Noto Sans Khmer"
MONO  = "Courier New"


# --- data helpers ------------------------------------------------------------

def load_users():
    if os.path.exists(USER_FILE):
        with open(USER_FILE, "r") as f:
            data = json.load(f)
            if isinstance(data, dict) and "username" in data and "password" in data:
                return {data["username"]: data["password"]}
            return data
    return {}

def save_users(users):
    with open(USER_FILE, "w") as f:
        json.dump(users, f, indent=2)

def add_user(username, password):
    users = load_users()
    users[username] = password
    save_users(users)


# --- input field -------------------------------------------------------------

class InputField(QLineEdit):
    def __init__(self, placeholder, password=False):
        super().__init__()
        self.setPlaceholderText(placeholder)
        if password:
            self.setEchoMode(QLineEdit.Password)
        self.setFixedHeight(48)
        self.setStyleSheet(f"""
            QLineEdit {{
                border: none;
                border-bottom: 1px solid #cccccc;
                padding: 8px 0px;
                font-size: 15px;
                font-family: '{MONO}';
                background: transparent;
                color: #111111;
            }}
            QLineEdit:focus {{
                border-bottom: 2px solid #111111;
                color: #000000;
            }}
        """)


# --- black button ------------------------------------------------------------

class BlackButton(QFrame):
    def __init__(self, text, callback=None, width=320, height=52, outlined=False):
        super().__init__()
        self._callback = callback
        self._outlined = outlined
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setFixedSize(width, height)

        lv = QVBoxLayout(self)
        lv.setContentsMargins(0, 0, 0, 0)
        self.lbl = QLabel(text)
        self.lbl.setAlignment(Qt.AlignCenter)
        lv.addWidget(self.lbl)
        self._normal()

    def _normal(self):
        if not self._outlined:
            self.setStyleSheet("QFrame{background:#111111;border:none;}")
            self.lbl.setStyleSheet(f"color:#ffffff;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        else:
            self.setStyleSheet("QFrame{background:#ffffff;border:1px solid #cccccc;}")
            self.lbl.setStyleSheet(f"color:#111111;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")

    def _hover(self):
        if not self._outlined:
            self.setStyleSheet("QFrame{background:#333333;border:none;}")
            self.lbl.setStyleSheet(f"color:#ffffff;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")
        else:
            self.setStyleSheet("QFrame{background:#f5f5f5;border:1px solid #999999;}")
            self.lbl.setStyleSheet(f"color:#111111;font-size:14px;font-family:'{MONO}';letter-spacing:2px;font-weight:700;background:transparent;border:none;")

    def enterEvent(self, e): self._hover()
    def leaveEvent(self, e): self._normal()
    def mousePressEvent(self, e):
        if self._callback: self._callback()


# --- error label -------------------------------------------------------------

class ErrorLabel(QLabel):
    def __init__(self):
        super().__init__("")
        self.setFixedHeight(20)
        self.setStyleSheet(f"color:#cc0000;font-size:12px;font-family:'{MONO}';background:transparent;")

    def show_error(self, msg): self.setText("✕  " + msg)
    def clear_msg(self): self.setText("")


# --- text link ---------------------------------------------------------------

def make_link(text, callback, size=13):
    lbl = QLabel(text)
    lbl.setCursor(QCursor(Qt.PointingHandCursor))
    lbl.setStyleSheet(f"color:#555555;font-size:{size}px;font-family:'{MONO}';text-decoration:underline;background:transparent;")
    lbl.mousePressEvent = lambda e: callback()
    return lbl


# --- dark brand panel --------------------------------------------------------

class BrandPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, e):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # dark gradient background
        grad = QLinearGradient(0, 0, 0, self.height())
        grad.setColorAt(0, QColor("#0a0a0a"))
        grad.setColorAt(1, QColor("#1a1a1a"))
        painter.fillRect(self.rect(), grad)

        # subtle grid pattern
        painter.setPen(QPen(QColor(255, 255, 255, 8), 1))
        step = 40
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)

        cx = self.width() // 2
        cy = self.height() // 2

        # large circle
        painter.setPen(QPen(QColor(255, 255, 255, 20), 1))
        painter.setBrush(Qt.NoBrush)
        r = min(self.width(), self.height()) // 3
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)
        painter.drawEllipse(cx - r//2, cy - r//2, r, r)

        # robot icon
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(255, 255, 255, 220)))
        icon_size = 56
        painter.setFont(QFont(MONO, 40))
        painter.setPen(QPen(QColor("#ffffff")))
        painter.drawText(
            QRect(cx - icon_size, cy - 120, icon_size * 2, icon_size + 10),
            Qt.AlignCenter, "◈"
        )

        # Khmer name
        painter.setFont(QFont(KHMER, 28, QFont.Bold))
        painter.setPen(QPen(QColor("#ffffff")))
        painter.drawText(
            QRect(20, cy - 40, self.width() - 40, 60),
            Qt.AlignCenter, "រ៉ូបូតសេវាកម្ម"
        )

        # English name
        painter.setFont(QFont(MONO, 16, QFont.Bold))
        painter.setPen(QPen(QColor("#ffffff")))
        painter.drawText(
            QRect(20, cy + 28, self.width() - 40, 36),
            Qt.AlignCenter, "SERVICE ROBOT"
        )

        # divider line
        painter.setPen(QPen(QColor(255, 255, 255, 60), 1))
        line_w = 80
        painter.drawLine(cx - line_w, cy + 76, cx + line_w, cy + 76)

        # tagline
        painter.setFont(QFont(KHMER, 13))
        painter.setPen(QPen(QColor(255, 255, 255, 120)))
        painter.drawText(
            QRect(20, cy + 86, self.width() - 40, 30),
            Qt.AlignCenter, "ប្រព័ន្ធស្វ័យប្រវត្តិ"
        )

        painter.setFont(QFont(MONO, 10))
        painter.setPen(QPen(QColor(255, 255, 255, 60)))
        painter.drawText(
            QRect(20, cy + 116, self.width() - 40, 24),
            Qt.AlignCenter, "AUTONOMOUS NAVIGATION SYSTEM"
        )

        # version bottom right
        painter.setFont(QFont(MONO, 10))
        painter.setPen(QPen(QColor(255, 255, 255, 40)))
        painter.drawText(
            QRect(0, self.height() - 36, self.width() - 20, 24),
            Qt.AlignRight | Qt.AlignVCenter, "v1.0"
        )

        painter.end()


# --- form container (centered vertically) ------------------------------------

def make_form_left():
    left = QWidget()
    left.setStyleSheet("background:#ffffff;")
    left.setFixedWidth(520)
    return left


# --- Welcome panel -----------------------------------------------------------

class WelcomePanel(QWidget):
    def __init__(self, on_signup, on_login):
        super().__init__()
        self.setStyleSheet("background:#ffffff;")

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left = make_form_left()
        outer = QVBoxLayout(left)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addStretch(2)

        inner = QVBoxLayout()
        inner.setContentsMargins(72, 0, 64, 0)
        inner.setSpacing(0)

        badge = QLabel("SERVICE ROBOT SYSTEM")
        badge.setStyleSheet(f"font-size:11px;font-family:'{MONO}';letter-spacing:4px;color:#aaaaaa;background:transparent;")
        inner.addWidget(badge)
        inner.addSpacing(20)

        title_kh = QLabel("សូមស្វាគមន៍")
        title_kh.setStyleSheet(f"font-size:48px;font-family:'{KHMER}';font-weight:700;color:#111111;background:transparent;")
        inner.addWidget(title_kh)

        title_en = QLabel("WELCOME")
        title_en.setStyleSheet(f"font-size:26px;font-family:'{MONO}';font-weight:700;color:#111111;letter-spacing:6px;background:transparent;")
        inner.addWidget(title_en)
        inner.addSpacing(20)

        sub_kh = QLabel("ចូលគណនី ឬ បង្កើតគណនីថ្មី")
        sub_kh.setStyleSheet(f"font-size:16px;font-family:'{KHMER}';color:#666666;background:transparent;")
        inner.addWidget(sub_kh)

        sub_en = QLabel("Sign in to your account or create a new one")
        sub_en.setStyleSheet(f"font-size:13px;font-family:'{MONO}';color:#aaaaaa;letter-spacing:1px;background:transparent;")
        inner.addWidget(sub_en)
        inner.addSpacing(52)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(16)
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.addWidget(BlackButton("ចូល  /  Log In",       callback=on_login,  width=190, outlined=False))
        btn_row.addWidget(BlackButton("ចុះឈ្មោះ  /  Sign Up", callback=on_signup, width=190, outlined=True))
        btn_row.addStretch()
        inner.addLayout(btn_row)

        outer.addLayout(inner)
        outer.addStretch(3)

        root.addWidget(left)
        root.addWidget(BrandPanel(), stretch=1)


# --- Signup panel ------------------------------------------------------------

class SignupPanel(QWidget):
    def __init__(self, on_success, on_back):
        super().__init__()
        self.on_success = on_success
        self.setStyleSheet("background:#ffffff;")

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left = make_form_left()
        outer = QVBoxLayout(left)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addStretch(1)

        inner = QVBoxLayout()
        inner.setContentsMargins(72, 0, 64, 0)
        inner.setSpacing(0)

        inner.addWidget(make_link("← ត្រឡប់  /  Back", on_back))
        inner.addSpacing(32)

        badge = QLabel("CREATE ACCOUNT")
        badge.setStyleSheet(f"font-size:11px;font-family:'{MONO}';letter-spacing:4px;color:#aaaaaa;background:transparent;")
        inner.addWidget(badge)
        inner.addSpacing(14)

        title_kh = QLabel("ចុះឈ្មោះ")
        title_kh.setStyleSheet(f"font-size:42px;font-family:'{KHMER}';font-weight:700;color:#111111;background:transparent;")
        inner.addWidget(title_kh)

        title_en = QLabel("SIGN UP")
        title_en.setStyleSheet(f"font-size:22px;font-family:'{MONO}';font-weight:700;color:#111111;letter-spacing:5px;background:transparent;")
        inner.addWidget(title_en)
        inner.addSpacing(6)

        sub = QLabel("បំពេញព័ត៌មានខាងក្រោម  /  Fill in your details below")
        sub.setStyleSheet(f"font-size:14px;font-family:'{KHMER}';color:#888888;background:transparent;")
        inner.addWidget(sub)
        inner.addSpacing(36)

        self.user_input = InputField("ឈ្មោះអ្នកប្រើ  /  Username")
        self.pw_input   = InputField("លេខសម្ងាត់  /  Password", password=True)
        self.pw2_input  = InputField("បញ្ជាក់លេខសម្ងាត់  /  Confirm Password", password=True)

        for w in [self.user_input, self.pw_input, self.pw2_input]:
            inner.addWidget(w)
            inner.addSpacing(18)

        self.error = ErrorLabel()
        inner.addWidget(self.error)
        inner.addSpacing(24)

        inner.addWidget(BlackButton("ចុះឈ្មោះ  /  Create Account", callback=self._signup, width=320))

        outer.addLayout(inner)
        outer.addStretch(2)

        root.addWidget(left)
        root.addWidget(BrandPanel(), stretch=1)

    def _signup(self):
        username = self.user_input.text().strip()
        pw       = self.pw_input.text()
        pw2      = self.pw2_input.text()
        if not username:
            self.error.show_error("ឈ្មោះអ្នកប្រើមិនអាចទទេ  /  Username cannot be empty"); return
        users = load_users()
        if username in users:
            self.error.show_error("ឈ្មោះនេះមានរួចហើយ  /  Username already exists"); return
        if len(pw) < 4:
            self.error.show_error("លេខសម្ងាត់យ៉ាងតិច ៤ តួ  /  Min 4 characters"); return
        if pw != pw2:
            self.error.show_error("លេខសម្ងាត់មិនដូចគ្នា  /  Passwords do not match"); return
        add_user(username, pw)
        self.error.clear_msg()
        self.on_success(username)


# --- Account picker ----------------------------------------------------------

class AccountPickerPanel(QWidget):
    def __init__(self, on_pick, on_back, on_signup):
        super().__init__()
        self.on_pick = on_pick
        self.setStyleSheet("background:#ffffff;")

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left = make_form_left()
        outer = QVBoxLayout(left)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addStretch(1)

        inner = QVBoxLayout()
        inner.setContentsMargins(72, 0, 64, 0)
        inner.setSpacing(0)

        inner.addWidget(make_link("← ត្រឡប់  /  Back", on_back))
        inner.addSpacing(32)

        badge = QLabel("SELECT ACCOUNT")
        badge.setStyleSheet(f"font-size:11px;font-family:'{MONO}';letter-spacing:4px;color:#aaaaaa;background:transparent;")
        inner.addWidget(badge)
        inner.addSpacing(14)

        title_kh = QLabel("ជ្រើសគណនី")
        title_kh.setStyleSheet(f"font-size:42px;font-family:'{KHMER}';font-weight:700;color:#111111;background:transparent;")
        inner.addWidget(title_kh)

        title_en = QLabel("CHOOSE ACCOUNT")
        title_en.setStyleSheet(f"font-size:20px;font-family:'{MONO}';font-weight:700;color:#111111;letter-spacing:4px;background:transparent;")
        inner.addWidget(title_en)
        inner.addSpacing(32)

        users = load_users()
        for uname in users:
            inner.addWidget(self._account_row(uname))
            inner.addSpacing(10)

        inner.addSpacing(16)
        inner.addWidget(make_link("+ បន្ថែមគណនីថ្មី  /  Add another account", on_signup))

        outer.addLayout(inner)
        outer.addStretch(2)

        root.addWidget(left)
        root.addWidget(BrandPanel(), stretch=1)

    def _account_row(self, username):
        row = QFrame()
        row.setCursor(QCursor(Qt.PointingHandCursor))
        row.setFixedHeight(60)
        row.setStyleSheet("""
            QFrame{background:#f9f9f9;border:1px solid #eeeeee;}
            QFrame:hover{background:#f0f0f0;border:1px solid #111111;}
        """)
        hl = QHBoxLayout(row)
        hl.setContentsMargins(16, 0, 16, 0)
        hl.setSpacing(14)

        av = QLabel("🤖")
        av.setFixedSize(36, 36)
        av.setAlignment(Qt.AlignCenter)
        av.setStyleSheet("background:#111;color:#fff;font-size:18px;border:none;border-radius:18px;")

        nm = QLabel(username)
        nm.setStyleSheet(f"font-size:15px;font-family:'{MONO}';color:#111111;background:transparent;border:none;letter-spacing:1px;")

        ar = QLabel("→")
        ar.setStyleSheet("font-size:15px;color:#cccccc;background:transparent;border:none;")

        hl.addWidget(av)
        hl.addWidget(nm)
        hl.addStretch()
        hl.addWidget(ar)
        row.mousePressEvent = lambda e, u=username: self.on_pick(u)
        return row


# --- Login panel -------------------------------------------------------------

class LoginPanel(QWidget):
    def __init__(self, username, on_success, on_back):
        super().__init__()
        self.username   = username
        self.on_success = on_success
        self.setStyleSheet("background:#ffffff;")

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left = make_form_left()
        outer = QVBoxLayout(left)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addStretch(1)

        inner = QVBoxLayout()
        inner.setContentsMargins(72, 0, 64, 0)
        inner.setSpacing(0)

        inner.addWidget(make_link("← ត្រឡប់  /  Back", on_back))
        inner.addSpacing(32)

        # avatar
        av = QLabel("🤖")
        av.setFixedSize(56, 56)
        av.setAlignment(Qt.AlignCenter)
        av.setStyleSheet("background:#111;color:#fff;font-size:28px;border:none;border-radius:28px;")
        inner.addWidget(av)
        inner.addSpacing(24)

        badge = QLabel("WELCOME BACK")
        badge.setStyleSheet(f"font-size:11px;font-family:'{MONO}';letter-spacing:4px;color:#aaaaaa;background:transparent;")
        inner.addWidget(badge)
        inner.addSpacing(12)

        title_kh = QLabel("សូមស្វាគមន៍មកវិញ")
        title_kh.setStyleSheet(f"font-size:36px;font-family:'{KHMER}';font-weight:700;color:#111111;background:transparent;")
        inner.addWidget(title_kh)

        title_en = QLabel(f"HELLO,  {username.upper()}")
        title_en.setStyleSheet(f"font-size:18px;font-family:'{MONO}';font-weight:700;color:#111111;letter-spacing:3px;background:transparent;")
        inner.addWidget(title_en)
        inner.addSpacing(8)

        sub_kh = QLabel("បញ្ចូលលេខសម្ងាត់ដើម្បីបន្ត")
        sub_kh.setStyleSheet(f"font-size:15px;font-family:'{KHMER}';color:#888888;background:transparent;")
        inner.addWidget(sub_kh)

        sub_en = QLabel("Enter your password to continue")
        sub_en.setStyleSheet(f"font-size:12px;font-family:'{MONO}';color:#aaaaaa;letter-spacing:1px;background:transparent;")
        inner.addWidget(sub_en)
        inner.addSpacing(32)

        self.pw_input = InputField("លេខសម្ងាត់  /  Password", password=True)
        self.pw_input.returnPressed.connect(self._login)
        inner.addWidget(self.pw_input)
        inner.addSpacing(16)

        self.error = ErrorLabel()
        inner.addWidget(self.error)
        inner.addSpacing(24)

        inner.addWidget(BlackButton("ចូល  /  Log In", callback=self._login, width=320))

        outer.addLayout(inner)
        outer.addStretch(2)

        root.addWidget(left)
        root.addWidget(BrandPanel(), stretch=1)

    def _login(self):
        pw    = self.pw_input.text()
        users = load_users()
        if users.get(self.username) == pw:
            self.error.clear_msg()
            self.on_success(self.username)
        else:
            self.error.show_error("លេខសម្ងាត់មិនត្រឹមត្រូវ  /  Incorrect password")
            self.pw_input.clear()