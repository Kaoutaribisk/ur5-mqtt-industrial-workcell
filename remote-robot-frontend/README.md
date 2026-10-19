# UR5 MQTT Control Dashboard

React interface for the UR5 production-cell simulation. The dashboard exchanges commands and telemetry through MQTT and provides production monitoring alongside six-joint manual control.

See the [project README](../README.md) for scene setup and the complete application startup procedure.

## Views

### Production

- Live line state and MQTT, bridge, and simulator connection indicators.
- Received, processed, delivered, and completed-cycle counters.
- Incoming, processing-station, and outgoing sensor states.
- Six measured joint angles, nominal targets, and settled position errors.
- Maintenance warnings, cycle trends, and the five most recent feedback events.
- Start, pause, stop, reset-stop, maintenance reset, and demo-speed controls.

### Manual joints

Six joint cards display angle feedback and offer incremental movement controls. Available increments are 1°, 5°, and 10°. Home requests move the nominal joint targets toward zero using the existing incremental command format.

The manual emergency-stop-style control locks browser joint commands. Use **Stop cell** in the production view to hold the simulated line. Manual movements are subject to the production controller's state and interlocks.

## Install

From this directory:

```powershell
npm ci
```

## Development

Open `production_cell.ttt` in CoppeliaSim with the ZeroMQ API on port 23001. From the repository root, run the bridge:

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\workcell_backend.py
```

In a separate terminal, from this directory:

```powershell
npm start
```

The dashboard opens at [localhost:3000](http://localhost:3000/). Do not run `run_workcell.py` at the same time as the development server: both use port 3000.

## Production build

```powershell
npm run build
```

The output is written to `build/`. From the repository root, `industrial-workcell/run_workcell.py` serves that build and launches the bridge. Rebuild after frontend changes before using this launcher.

## MQTT connection

Browser connection settings are in `src/MQTTClient.js`. The default endpoint is `ws://broker.hivemq.com:8000/mqtt`.

| Topic | Purpose |
| --- | --- |
| `robot/joint_command` | Incremental manual commands |
| `robot/joint_state` | Legacy nominal joint-target feedback |
| `kaoutar/workcell/control` | Production command requests |
| `kaoutar/workcell/telemetry` | Production counters, sensors, measured angles, errors, and events |
| `kaoutar/workcell/status` | Bridge and simulator availability |
| `kaoutar/workcell/ack` | Command acceptance or rejection |

Manual commands retain the format `jointIndex:direction:degrees`, for example `0:right:10`. Joint indices are zero-based, from 0 to 5; the sixth joint rotates the tool.

Production controls depend on fresh telemetry and controller interlocks. Stale feedback disables relevant actions; command acknowledgements report whether a request was accepted. Legacy angle feedback represents nominal targets, while production telemetry supplies actual simulated joint positions.

If the broker or topic namespace changes, update both frontend and bridge settings. An HTTPS-hosted dashboard requires a secure WebSocket endpoint supported by its broker.

## Source files

| File | Responsibility |
| --- | --- |
| `src/App.js` | Application layout, tabs, and manual-control state |
| `src/WorkcellDashboard.js` | Production controls, telemetry, events, and maintenance display |
| `src/JointControl.js` | Individual manual joint controls |
| `src/MQTTClient.js` | Shared MQTT connection |
| `src/App.test.js` | Manual commands, feedback history, stop lock, and stale-state checks |
| `src/WorkcellDashboard.test.js` | Production commands, acknowledgements, interlocks, and telemetry checks |

## Checks

```powershell
npm test -- --watchAll=false --runInBand
npm run build
```

The interface uses React, MQTT.js, and react-scripts. The [workcell reference](../industrial-workcell/README.md) describes controller behavior and the maintenance model.
