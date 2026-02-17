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

## Summary

- What changed:
- Why this change is needed:

## Scope

- Affected modules/files:
- Change type: `feature` / `fix` / `docs` / `test` / `refactor`
- Behavior impact: `none` / `minor` / `major`

## Validation

Commands run:

```bash
python main.py --test
python -m unittest test_dynamic_features.py
```

If applicable, add extra validations:
- Performance graph scripts validated (`performance_graphs/latency_vs_drones.py`, `performance_graphs/auto_plot_from_csv.py`)
- C++ build/test flow validated (if native code changed)

Test results summary:
- 

## Logs / Screenshots

- GUI changes: attach screenshots or short GIF
- Runtime/safety-path changes: attach relevant log snippets

## Documentation

- [ ] Updated `readme.md` (if architecture/usage/math behavior changed)
- [ ] Updated `deployment_guide.md` (if deployment/ops flow changed)
- [ ] Updated other docs as needed

## Security and Safety Review

- [ ] No secrets/keys/credentials committed
- [ ] Safety-critical paths reviewed (fallback, emergency, RTH, collision risk)
- [ ] Real-drone behavior (if touched) validated in simulation-first flow

## PR Checklist

- [ ] Branch name follows convention (`feature/*`, `fix/*`, `docs/*`, `test/*`, `refactor/*`)
- [ ] Change is focused and does not mix unrelated refactors
- [ ] Code runs locally
- [ ] Tests executed and results included
- [ ] Docs updated where needed
- [ ] PR description includes impacted files/modules
