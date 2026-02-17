# Contributing

## Workflow
1. Fork and clone repository.
2. Create a feature branch.
3. Make focused changes.
4. Run tests locally.
5. Open a pull request with clear description.

## Branch naming
- `feature/<name>`
- `fix/<name>`
- `docs/<name>`
- `test/<name>`
- `refactor/<name>`

## Local setup
```bash
python -m venv .venv
```

Windows:
```powershell
.\.venv\Scripts\Activate.ps1
```

Linux/macOS:
```bash
source .venv/bin/activate
```

Install:
```bash
pip install -r requirements.txt
```

## Test before PR
```bash
pytest tests -q
python -m py_compile main.py 
```

## Coding guidelines
- Keep changes small and reviewable.
- Preserve existing runtime behavior unless intentionally changing it.
- Update docs when changing config, run commands, or architecture.
- Avoid adding hardcoded credentials or secrets.

## Commit message style
Use clear prefixes:
- `Add:` new feature
- `Fix:` bug fix
- `Update:` behavior/doc update
- `Refactor:` internal cleanup
- `Test:` test changes
- `Docs:` documentation only

Examples:
- `Fix: handle missing MQTT broker without crash`
- `Update: GUI route animation and SVG assets`

## Pull request checklist
- [ ] Tests pass locally
- [ ] Docs updated
- [ ] No secrets committed
- [ ] Config changes documented
- [ ] Screenshots/log snippets included for UI/runtime changes

## Update: February 17, 2026
- Added Differential Drone Immune System (Self-Healing Flight System) in dronecontroller.cpp.
- Added real-time motor health logic (RPM drop detection at >=10%), thrust redistribution, adaptive PID, and emergency return handling for 2+ degraded motors.
- Added structured immune logs, including [IMMUNE] Motor X degraded ... | Compensation Active and SWARM_ALERT behavior.
- Swarm Status drone table (ID, Role, Mode, Battery, Altitude, Status) is now fully dynamic in the GUI (defensive row updates, dynamic resizing, always-visible vertical scrollbar, smooth scrolling, and sortable columns).
- Latency indicators (C++->Py, Py Proc, Py->C++, RTT, RTT Jitter) are dynamically refreshed from runtime latency stats.
