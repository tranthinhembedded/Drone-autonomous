# core/style.py
from PySide6.QtWidgets import QApplication
from utils.colors import BG, TEXT, FONT, BORDER, ACCENT, PANEL_BG, ACCENT_HOVER, SUBTEXT

def apply_modern_style(app: QApplication):
    app.setStyleSheet(f"""
        QWidget {{
            background-color: {BG};
            color: {TEXT};
            font-family: {FONT};
            font-size: 15px;
        }}

        QGroupBox {{
            border: 1px solid {BORDER};
            border-radius: 8px;
            padding: 8px;
            font-size: 17px;
            font-weight: 600;
            color: {ACCENT};
            background-color: {PANEL_BG};
        }}

        QLabel {{
            color: {TEXT};
        }}

        QDoubleSpinBox {{
            background: #0F1116;
            border: 1px solid {BORDER};
            border-radius: 6px;
            padding: 3px;
            color: {ACCENT};
            selection-background-color: {ACCENT};
            font-size: 15px;
        }}

        QPushButton {{
            border-radius: 6px;
            padding: 8px;
            background-color: {ACCENT};
            color: #000;
            font-weight: bold;
        }}
        QPushButton:hover {{
            background-color: {ACCENT_HOVER};
        }}

        QTabBar::tab {{
            background: #1B1F26;
            padding: 8px 16px;
            border-radius: 6px;
            margin-right: 6px;
            color: {SUBTEXT};
        }}
        QTabBar::tab:selected {{
            background: {ACCENT};
            color: #000;
            font-weight: bold;
        }}
    """)