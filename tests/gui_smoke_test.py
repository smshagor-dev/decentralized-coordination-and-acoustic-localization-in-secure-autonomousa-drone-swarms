import os
import sys


os.environ.setdefault("MPLBACKEND", "Agg")
if sys.platform.startswith("win"):
    os.environ.setdefault("QT_QPA_PLATFORM", "minimal")
else:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from gui import MainWindow
from swarm_manager import SwarmManager


def main() -> int:
    app = QApplication.instance() or QApplication([])
    swarm = SwarmManager()
    window = None
    try:
        window = MainWindow(swarm, runtime_mode="real_test")
        window.show()
        app.processEvents()
        window.hide()
        return 0
    finally:
        if window is not None:
            window.close()
        swarm.stop()
        app.processEvents()


if __name__ == "__main__":
    raise SystemExit(main())
