import sys
import os
import subprocess
import signal
from PySide6.QtWidgets import QApplication, QMainWindow, QStackedWidget
from PySide6.QtCore import QTimer
from food_del_robot_animations import fade_in, fade_out
from PySide6.QtCore import QPropertyAnimation, QEasingCurve

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

SETUP_RVIZ  = "/home/panha/Food_Delivery_Robot/src/food_del_robot_hmi/food_del_robot_hmi/Food_del_robot_setup.rviz"
ACTION_RVIZ = "/home/panha/Food_Delivery_Robot/src/food_del_robot_hmi/food_del_robot_hmi/Food_del_robot_action.rviz"

# panels loaded lazily on first use to keep startup fast
def _import_login():
    from food_del_robot_login_signup_panel import (
        WelcomePanel, SignupPanel, AccountPickerPanel, LoginPanel, load_users
    )
    return WelcomePanel, SignupPanel, AccountPickerPanel, LoginPanel, load_users

def _import_home():
    from food_del_robot_home_panel import HomePanel
    return HomePanel

def _import_setup():
    from food_del_robot_setup_positions_panel import SetupDesirePositionsPanel
    return SetupDesirePositionsPanel

def _import_start():
    from food_del_robot_start_panel import StartPanel
    return StartPanel

def _import_mapping():
    from food_del_robot_mapping import MappingPanel
    return MappingPanel

def _import_datas():
    from food_del_robot_datas import DatasPanel
    return DatasPanel


class MainHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(960, 600)
        self._current_user  = None

        self.setStyleSheet("""
            QMainWindow { background: #ffffff; }
            QMenuBar {
                background-color: #ffffff;
                color: #000000;
                font-size: 15px;
                font-family: 'Noto Sans Khmer', 'Courier New';
                letter-spacing: 2px;
                padding: 8px 12px;
                border-bottom: 2px solid #000000;
            }
            QMenuBar::item:selected { background-color: #000; color: #fff; }
            QMenu {
                background:#fff; color:#000;
                border:1px solid #000;
                font-size: 15px;
                font-family: 'Noto Sans Khmer', 'Courier New';
            }
            QMenu::item:selected { background:#000; color:#fff; }
        """)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        self.menuBar().setVisible(False)
        self._show_welcome()

    # --- helpers -------------------------------------------------------------

    def _push(self, widget, direction="right"):
        old = self.stack.currentWidget()
        self.stack.addWidget(widget)
        self.stack.setCurrentWidget(widget)
        fade_in(widget, duration=300)
        # clean up old widget after transition
        if old:
            QTimer.singleShot(320, lambda: (
                self.stack.removeWidget(old),
                old.deleteLater()
            ))

    # --- launch file ---------------------------------------------------------

    def _start_launch(self):
        pass  # launched externally via gazebo_hmi.launch.py

    def _stop_launch(self):
        """Kill all ROS2/Gazebo processes launched by gazebo_hmi.launch.py."""
        print("[MainHMI] Shutting down all processes...")
        patterns = [
            "gzserver", "gzclient", "gazebo",
            "slam_toolbox", "online_async",
            "robot_state_publisher",
            "nav2", "bt_navigator", "amcl",
            "map_server", "lifecycle_manager",
            "costmap", "controller_server",
            "planner_server", "behavior_server",
            "waypoint_follower", "velocity_smoother",
            "spawn_entity",
            "rviz2",
            "hand_controller",
        ]
        for p in patterns:
            subprocess.run(["pkill", "-f", p], capture_output=True)
        print("[MainHMI] All processes stopped.")

    # --- auth flow -----------------------------------------------------------

    def _show_welcome(self):
        WelcomePanel, _, _, _, load_users = _import_login()
        self.setWindowTitle("Service Robot — រ៉ូបូតសេវាកម្ម")
        self.menuBar().setVisible(False)
        users = load_users()
        self._push(WelcomePanel(
            on_signup=self._show_signup,
            on_login=self._show_picker if users else self._show_signup
        ))

    def _show_signup(self):
        _, SignupPanel, _, _, _ = _import_login()
        self.setWindowTitle("ចុះឈ្មោះ / Sign Up")
        self.menuBar().setVisible(False)
        self._push(SignupPanel(
            on_success=self._after_signup,
            on_back=self._show_welcome
        ))

    def _after_signup(self, username):
        self._show_login_for(username)

    def _show_picker(self):
        _, _, AccountPickerPanel, _, load_users = _import_login()
        self.setWindowTitle("ជ្រើសគណនី / Choose Account")
        self.menuBar().setVisible(False)
        users = load_users()
        if len(users) == 1:
            self._show_login_for(list(users.keys())[0])
        else:
            self._push(AccountPickerPanel(
                on_pick=self._show_login_for,
                on_back=self._show_welcome,
                on_signup=self._show_signup
            ))

    def _show_login_for(self, username):
        _, _, _, LoginPanel, load_users = _import_login()
        self.setWindowTitle(f"ចូល / Log In — {username}")
        self.menuBar().setVisible(False)
        users = load_users()
        self._push(LoginPanel(
            username=username,
            on_success=self._on_login_success,
            on_back=self._show_picker if len(users) > 1 else self._show_welcome
        ))

    def _on_login_success(self, username):
        # launch ROS2 on successful login
        self._start_launch()
        self._show_home(username)

    # --- home ----------------------------------------------------------------

    def _show_home(self, username):
        self._current_user = username
        self.setWindowTitle("ទំព័រដើម / Home")
        HomePanel = _import_home()
        home = HomePanel(
            username=username,
            on_logout=self._on_logout,
            on_navigate=self._navigate,
            on_start=self._show_start
        )
        home.setup_menu(self.menuBar())
        self.menuBar().setVisible(True)
        self._push(home)

    def _on_logout(self):
        self._stop_launch()
        self._show_welcome()

    # --- navigation ----------------------------------------------------------

    def _navigate(self, key):
        if key == "setup":
            self.setWindowTitle("កំណត់គោលដៅ / Setup Desire Positions")
            self.menuBar().setVisible(False)
            SetupDesirePositionsPanel = _import_setup()
            self._push(SetupDesirePositionsPanel(
                on_back=lambda: self._show_home(self._current_user),
                rviz_config=SETUP_RVIZ,
                parent_window=self
            ))
        elif key == "mapping":
            self.setWindowTitle("បង្កើតផែនទី / Mapping")
            self.menuBar().setVisible(False)
            MappingPanel = _import_mapping()
            self._push(MappingPanel(
                on_back=lambda: self._show_home(self._current_user)
            ))
        elif key == "editmap":
            pass  # TODO
        elif key == "datas":
            self.setWindowTitle("ទិន្នន័យ / Datas")
            self.menuBar().setVisible(False)
            DatasPanel = _import_datas()
            self._push(DatasPanel(
                on_back=lambda: self._show_home(self._current_user)
            ))

    def _show_start(self):
        self.setWindowTitle("ចាប់ផ្តើមមុខងារ / Start Mission")
        self.menuBar().setVisible(False)
        StartPanel = _import_start()
        self._push(StartPanel(
            on_back=lambda: self._show_home(self._current_user),
            rviz_config=ACTION_RVIZ,
            parent_window=self
        ))

    def closeEvent(self, e):
        print("[MainHMI] Window closing — cleaning up...")
        # stop current panel if it has cleanup
        current = self.stack.currentWidget()
        if current and hasattr(current, "_go_back"):
            try:
                if hasattr(current, "_rviz"):
                    current._rviz.stop()
                if hasattr(current, "_nav"):
                    current._nav.cancel()
                    current._nav.stop()
                if hasattr(current, "_slam"):
                    current._slam.stop()
                if hasattr(current, "_ros"):
                    current._ros.stop()
            except Exception as ex:
                print(f"[MainHMI] Panel cleanup error: {ex}")
        self._stop_launch()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainHMI()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()