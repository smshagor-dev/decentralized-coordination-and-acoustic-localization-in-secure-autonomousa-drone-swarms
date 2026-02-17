# Getting Started

Quick start guide for:
`Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms`

## 1. Prerequisites

- Python 3.10+ (recommended: 3.11)
- `pip`
- Git
- OS: Windows, Linux, or macOS

Optional:
- C++ toolchain (if you plan to work with `dronecontroller.cpp/.h`)

## 2. Clone and Setup

```bash
git clone https://github.com/smshagor-dev/decentralized-coordination-and-acoustic-localization.git
cd "Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms"
python -m venv venv
```

Activate virtual environment:

Windows (PowerShell):

```powershell
venv\Scripts\Activate.ps1
```

Linux/macOS:

```bash
source venv/bin/activate
```

Install dependencies:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

Optional one-command shortcut:

```bash
chmod +x quickstart.sh
./quickstart.sh
```

If you use `quickstart.sh`, verify your virtual environment is active before running `python main.py`.

## 3. Run the Project

Start full simulation + GUI:

```bash
python main.py
```

Run test mode:

```bash
python main.py --test
```

Run feature/unit test file:

```bash
python -m unittest test_dynamic_features.py
```

## 4. What You Should See

After `python main.py`:
- swarm manager starts
- GUI opens
- demo drones are created
- dynamic obstacles are active
- runtime logs are generated in `logs/`

## 5. Optional Real-Drone Mode

Set these environment variables before run:
- `REAL_DRONE_ENABLED=1`
- `REAL_DRONE_CONNECTIONS=1=udpin://:14540,2=udpin://:14541`
- `REAL_GPS_REF_LAT=<latitude>`
- `REAL_GPS_REF_LON=<longitude>`

If not set, simulation mode is used.

## 6. Project Entry Points

- `main.py`: start system
- `gui.py`: operator interface
- `swarm_manager.py`: swarm orchestration
- `drone.py`: per-drone behavior
- `dynamic_obstacles.py`: obstacle avoidance logic
- `latency_monitor.py`: RTT/jitter/fallback logic
- `acoustic_tracking.py`: TDOA localization
- `flying_ledger.py`: signed event ledger

## 7. Common First-Run Issues

GUI does not open:
- ensure dependencies from `requirements.txt` are installed
- verify virtual environment is activated

Import/module errors:
- run `pip install -r requirements.txt` again in the active venv

Unexpected runtime behavior:
- check `logs/` for system and drone logs
- run `python main.py --test` for a controlled validation run

## 8. Recommended Next Reads

- `readme.md`: full architecture + math + diagrams
- `deployment_guide.md`: deployment and operations
- `CONTRIBUTING.md`: development workflow and PR requirements
- `project_overview.md`: concise technical summary
