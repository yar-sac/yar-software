import sys
from PySide6.QtWidgets import QApplication

from main_window import MainWindow


def main():
    app = QApplication()
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

