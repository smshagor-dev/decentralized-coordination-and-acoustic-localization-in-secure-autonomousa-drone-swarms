<!--
#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################
-->

# Contributing Guide

Thanks for contributing to `Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms`.

This guide defines the expected workflow for code, documentation, testing, and pull requests.

## 1. Contribution Scope

Typical contribution areas:
- Python swarm logic (`swarm_manager.py`, `drone.py`, `dynamic_obstacles.py`)
- latency/fallback and acoustic modules (`latency_monitor.py`, `acoustic_tracking.py`)
- secure communication and ledger (`communication.py`, `flying_ledger.py`)
- GUI updates (`gui.py`)
- optional low-level C++ updates (`dronecontroller.cpp`, `dronecontroller.h`)
- docs (`readme.md`, `deployment_guide.md`, this file)

## 2. Branch Naming

Use one of these formats:
- `feature/<short-name>`
- `fix/<short-name>`
- `docs/<short-name>`
- `test/<short-name>`
- `refactor/<short-name>`

Examples:
- `feature/acoustic-confidence-threshold`
- `fix/latency-fallback-gating`
- `docs/math-rendering-cleanup`

## 3. Local Setup

```bash
python -m venv venv
```

Windows:

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

## 4. Run and Test Before PR

Run basic system checks:

```bash
python main.py --test
python -m unittest test_dynamic_features.py
```

If your change touches performance plotting, also validate:
- `performance_graphs/latency_vs_drones.py`
- `performance_graphs/auto_plot_from_csv.py`

If your change touches C++ logic, ensure your local C++ build/test flow is clean.

## 5. Coding Standards

- Keep changes focused and small.
- Do not mix unrelated refactors with feature fixes.
- Preserve existing behavior unless the PR explicitly changes behavior.
- Avoid hardcoded credentials, keys, or machine-specific paths.
- Prefer clear naming over short/ambiguous names.
- Add or update logs when changing safety-critical flow (fallback, emergency, RTH, collision risk).

## 6. Documentation Standards

When behavior changes, update docs in the same PR:
- `readme.md` for architecture/usage/math sections
- `deployment_guide.md` for developer deployment flow

For diagrams and equations:
- Mermaid labels with parentheses should be quoted.
- Use GitHub-friendly LaTeX formatting.
- Avoid unsupported LaTeX macros (for example `\operatorname`).

## 7. Commit Message Convention

Use clear prefixes:
- `Add:`
- `Fix:`
- `Update:`
- `Refactor:`
- `Test:`
- `Docs:`

Examples:
- `Fix: stabilize fallback gate under RTT jitter spikes`
- `Docs: update TDOA equations for GitHub math rendering`
- `Update: improve dynamic obstacle risk blending`

## 8. Pull Request Requirements

Each PR should include:
- concise summary of what changed
- reason for the change
- impacted modules/files
- test evidence (command output summary)
- screenshots/GIFs for GUI changes
- sample logs for runtime/safety path changes

Checklist:
- [ ] Branch name follows convention
- [ ] Code runs locally
- [ ] Tests executed
- [ ] Docs updated (if needed)
- [ ] No secrets or sensitive configs committed
- [ ] PR description includes validation details

## 9. Security and Safety Notes

- Never commit real encryption keys or private credentials.
- Treat emergency, RTH, and fallback logic as safety-critical.
- For real drone workflows, validate in simulation before hardware deployment.

## 10. Need Help

If requirements are unclear, open a draft PR and document assumptions explicitly. Early review is preferred over late large rewrites.
