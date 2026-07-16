import os
import sys
from pathlib import Path


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("QT_QPA_PLATFORM", "minimal")
os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/codex-qt-runtime")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from PyQt5.QtWidgets import QApplication

import gui


def main() -> int:
    app = QApplication.instance() or QApplication([])
    try:
        # Keep the smoke test lightweight for CI:
        # prove that the GUI module imports and a Qt app can bootstrap
        # under a headless platform without forcing full window setup.
        _ = gui.MainWindow
        app.processEvents()
        return 0
    except Exception:
        return 1
    finally:
        app.processEvents()


if __name__ == "__main__":
    raise SystemExit(main())
