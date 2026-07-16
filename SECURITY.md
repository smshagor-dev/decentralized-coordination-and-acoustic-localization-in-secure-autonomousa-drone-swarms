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

# Security Policy

## Supported Versions

Security fixes are prioritized for the most recent state of the default branch.

| Version / Branch | Supported |
| --- | --- |
| Latest `main` / default branch | Yes |
| Older snapshots, forks, archived copies | No |

## Reporting a Vulnerability

Please do not open public GitHub issues for suspected security vulnerabilities.

Report privately using one of the following contacts:

- Email: `smshagor.ru@gmail.com`
- Alternate Email: `contact@smshagor.com`

Include, when possible:

- affected file, module, or subsystem
- clear reproduction steps
- impact assessment
- logs, screenshots, or proof-of-concept material
- whether the issue affects simulation only, real-mode control, or both

## Response Expectations

- Initial acknowledgment target: within 7 days
- Triage/update target: within 14 days
- Fix timeline: depends on severity, exploitability, and safety impact

If the report affects active flight safety, emergency logic, encrypted communication, ledger integrity, or mission routing, it will be treated as high priority.

## Scope

Security-sensitive areas include:

- `communication.py` and encrypted telemetry/message handling
- `.env` and connection/configuration secrets
- `flying_ledger.py` block validation, signatures, and replication flow
- `swarm_manager.py`, `drone.py`, and safety-critical control paths
- `latency_monitor.py` watchdog/fallback logic
- `acoustic_tracking.py` trust, input validation, and event propagation
- `gui.py` paths that can trigger privileged or unsafe control actions
- C++ controller code in `dronecontroller.cpp` and `dronecontroller.h`

## Safety-Critical Reporting Notes

For vulnerabilities that may affect real drones, people, or property:

- stop testing immediately if continued testing could create unsafe behavior
- prefer simulation-first reproduction details
- clearly mark the report as `Safety Critical`
- include whether the issue can trigger unintended takeoff, collision risk, emergency landing failure, return-to-home failure, or command spoofing

## Disclosure Policy

Please allow time for investigation and remediation before any public disclosure.

After a fix is available, coordinated disclosure is welcome with accurate technical details and remediation guidance.

## Hardening Guidance for Contributors

- Never commit real encryption keys, API keys, private credentials, or production drone endpoints
- Use `.env.example` for documented placeholders only
- Validate security-sensitive changes in simulation before any hardware use
- Review safety-critical flows when changing fallback, emergency, RTH, collision avoidance, or encrypted communications
- Keep dependencies updated and avoid introducing unmaintained crypto or networking packages
