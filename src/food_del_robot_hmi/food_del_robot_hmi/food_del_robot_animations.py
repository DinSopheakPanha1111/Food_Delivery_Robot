"""
food_del_robot_animations.py
Safe animations that do NOT use QGraphicsOpacityEffect.
QGraphicsOpacityEffect conflicts with custom paintEvent widgets (like CircleStartButton)
causing QPainter::begin errors. We use setWindowOpacity or simple show() instead.
"""
from PySide6.QtCore import QPropertyAnimation, QEasingCurve, QTimer, QPoint
from PySide6.QtWidgets import QWidget


# --- fade in (safe - no QGraphicsOpacityEffect) ------------------------------

def fade_in(widget, duration=280, delay=0):
    """
    Simply shows the widget. We avoid QGraphicsOpacityEffect entirely
    because it conflicts with any widget using a custom paintEvent.
    The panel switching in main_hmi already handles visual transitions.
    """
    def _show():
        widget.setVisible(True)
        widget.update()
    if delay > 0:
        QTimer.singleShot(delay, _show)
    else:
        _show()


def fade_out(widget, duration=200, on_done=None):
    """Hide the widget immediately."""
    widget.setVisible(False)
    if on_done:
        on_done()


# --- staggered visibility for lists ------------------------------------------

def stagger_fade_in(widgets, base_delay=0, step=60, duration=250):
    """
    Show widgets one after another with a small delay between each.
    Safe version — no QGraphicsOpacityEffect.
    """
    for i, w in enumerate(widgets):
        delay = base_delay + i * step
        if delay == 0:
            w.setVisible(True)
        else:
            QTimer.singleShot(delay, lambda ww=w: (ww.setVisible(True), ww.update()))


# --- slide in ----------------------------------------------------------------

def slide_in_from_right(widget, duration=350):
    """Slide widget in from right — only works on top-level widgets."""
    if not widget.parent():
        return
    parent_w = widget.parent().width()
    end_pos  = widget.pos()
    anim = QPropertyAnimation(widget, b"pos", widget)
    anim.setDuration(duration)
    anim.setStartValue(QPoint(parent_w, end_pos.y()))
    anim.setEndValue(end_pos)
    anim.setEasingCurve(QEasingCurve.OutCubic)
    anim.start()
    widget._slide_anim = anim


def slide_in_from_left(widget, duration=350):
    """Slide widget in from left — only works on top-level widgets."""
    if not widget.parent():
        return
    parent_w = widget.parent().width()
    end_pos  = widget.pos()
    anim = QPropertyAnimation(widget, b"pos", widget)
    anim.setDuration(duration)
    anim.setStartValue(QPoint(-parent_w, end_pos.y()))
    anim.setEndValue(end_pos)
    anim.setEasingCurve(QEasingCurve.OutCubic)
    anim.start()
    widget._slide_anim = anim


# --- pulse (for status dots) -------------------------------------------------

class PulseAnimation:
    """
    Pulses a QFrame's stylesheet color to draw attention.
    Does NOT use QGraphicsOpacityEffect.
    """
    def __init__(self, widget, color_on="#22c55e", color_off="#888888", interval=800):
        self._widget    = widget
        self._color_on  = color_on
        self._color_off = color_off
        self._state     = True
        self._timer     = QTimer()
        self._timer.setInterval(interval)
        self._timer.timeout.connect(self._tick)

    def _tick(self):
        col = self._color_on if self._state else self._color_off
        self._widget.setStyleSheet(
            f"background:{col};border-radius:7px;border:none;"
        )
        self._state = not self._state

    def start(self):
        self._timer.start()

    def stop(self):
        self._timer.stop()
        self._widget.setStyleSheet(
            f"background:{self._color_on};border-radius:7px;border:none;"
        )