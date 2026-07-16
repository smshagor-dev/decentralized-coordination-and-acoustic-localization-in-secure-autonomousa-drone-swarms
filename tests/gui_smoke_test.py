import os
import sys
from pathlib import Path


os.environ.setdefault("MPLBACKEND", "Agg")
if sys.platform.startswith("win"):
    os.environ.setdefault("QT_QPA_PLATFORM", "minimal")
else:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

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
