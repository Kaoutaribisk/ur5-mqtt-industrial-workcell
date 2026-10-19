# UR5 Production Workcell

CoppeliaSim scene controller and Python MQTT bridge for the UR5 automated production dashboard.

The ready-to-open scene is [`production_cell.ttt`](../production_cell.ttt). See the [main README](../README.md) for installation and startup, and the [frontend README](../remote-robot-frontend/README.md) for dashboard development.

## Production flow

The incoming conveyor feeds a part to a detection position. The UR5 picks it up, loads a generic processing station, waits for processing, retrieves it, and places it on the outgoing conveyor. A cycle completes after discharge and the robot's return home.

Three proximity sensors report the incoming, station, and outgoing positions. A single product object is recycled for successive cycles. Vacuum pickup is represented by attaching the product to the tool.

## Components

| File | Purpose |
| --- | --- |
| `workcell.lua` | Source for the production controller embedded in the scene |
| `workcell_backend.py` | MQTT commands, acknowledgements, simulator connection, and telemetry |
| `run_workcell.py` | Starts the bridge and serves the built frontend at port 3000 |
| `clearance_route.json` | Calibrated poses and joint-space routes |
| `build_scene.py` | Builds the generated workcell from the original scene |
| `update_scene.py` | Installs the current controller source into a stopped scene and saves it |
| `test_backend.py` | Bridge unit tests |
| `validate_clearance.py` | Maximum-wear cycle and clearance integration check |
| `verify_clearance_guard.py` | Deliberate-obstruction check |
| `verify_watchdog.py` | Missing-heartbeat check |

The Lua source is embedded in the saved scene. Editing `workcell.lua` alone does not update an already loaded scene. Scene-building and update tools modify the simulator; use them deliberately with production stopped.

## Motion and layout

- Conveyor centers are 1.30 m apart, providing approximately 0.915 m between their inner rails.
- Calibrated routes pass around equipment and enter the processing station through its front opening.
- Continuous joint trajectories use rounded route corners and quintic endpoint easing.
- The configured robot joint-speed limit is 0.65 rad/s, approximately 37°/s.
- The saved scene uses a 0.02 s simulation step and real-time playback.
- Demo-speed controls accelerate belts and processing delays without raising the robot's speed limit.

Movement is kinematic. Selecting Bullet in CoppeliaSim does not make these joint trajectories or attached-part transfers a fully dynamic simulation.

Before each movement update, the clearance guard samples the candidate route against both conveyors and the machine. A collision or clearance below 15 mm rejects the update and latches a fault. The reported clearance lower bound is capped at 30 mm: a displayed 30 mm means **at least** 30 mm, not an exact distance.

This guard covers robot and tool geometry against the equipment. It does not provide a complete self-collision or carried-product collision assessment.

## Joint monitoring

Telemetry includes nominal target angles, actual simulated joint angles, and settled errors. Maintenance evaluation occurs after movement settles, avoiding alarms caused only by normal movement lag.

The demonstration wear model starts after 15 completed cycles. On cycle 16, the first joint receives a 0.24° offset; that offset increases by 0.24° on each subsequent cycle. Other joints use scaled offsets with alternating signs.

| Maximum settled joint error | Result |
| --- | --- |
| Below 1° | Normal |
| At least 1° | Warning |
| At least 3° | Maintenance required |

With default settings, cycle 28 reaches 3.12°. The controller completes the active cycle, returns home, and prevents another cycle until maintenance reset.

Maintenance reset requires a stopped idle or maintenance state with no carried part. It returns the robot home and clears wear and cycles since maintenance while preserving total production counters. Restarting the simulation resets the scene's counters and wear model.

The offsets are intentionally injected for demonstration; they are not measured physical wear or a validated lifetime model.

## MQTT protocol

Default broker: `broker.hivemq.com`, TCP port 1883. The dashboard uses the same broker over WebSockets.

| Topic | Payload / role |
| --- | --- |
| `robot/joint_command` | Legacy text command, such as `0:right:10` |
| `robot/joint_state` | Comma-separated nominal targets, such as `0:0,1:0,2:0,3:0,4:0,5:0` |
| `kaoutar/workcell/control` | JSON production request |
| `kaoutar/workcell/telemetry` | JSON state, counters, sensors, measured angles, targets, settled errors, and events |
| `kaoutar/workcell/status` | Retained backend, simulator, scene-ready, and simulation-running flags |
| `kaoutar/workcell/ack` | Request ID, acceptance flag, and reason |

Example requests:

```json
{"action":"start","id":"operator-start-001"}
```

```json
{"action":"speed","value":4,"id":"operator-speed-002"}
```

Supported actions are `start`, `pause`, `stop`, `reset_stop`, `maintenance`, and `speed`. Speed values range from 0.25 to 8; the dashboard exposes 1×, 4×, and 8× presets.

Commands should be non-retained. The bridge ignores retained requests and does not replay queued commands after a disconnect. Acceptance is reported through the acknowledgement topic; accepting a request does not imply that a complete production cycle has finished.

Manual commands use joint indices 0–5 and the existing direction tokens. The production bridge validates requests and applies controller interlocks. Do not run the original `robot_backend.py` concurrently.

## Connection configuration

The production bridge defaults to ZeroMQ port **23001**:

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\workcell_backend.py
```

To use a simulator configured on port 23000 instead:

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\workcell_backend.py --port 23000
```

To select another MQTT broker, pass `--broker hostname` and change the browser endpoint in `remote-robot-frontend/src/MQTTClient.js`. The production bridge does not read the original backend's `mqtt_config.py`.

`run_workcell.py` uses the bridge defaults and does not forward command-line options. For custom settings, run the bridge directly and start the frontend development server separately. Its bridge output is appended to `industrial-workcell/bridge.log` when using the launcher.

The bridge sends a heartbeat to the scene. If it is absent for approximately 2.5 seconds, the controller pauses production. Dashboard connection indicators and stale-feedback checks provide a separate operator view of availability.

## Validation

From the repository root, run unit tests without a simulator:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s industrial-workcell -p test_backend.py -v
```

Simulator integration checks require the production scene loaded on port 23001, simulation stopped, and the normal bridge disconnected. These checks control the simulator and may temporarily change scene state.

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\validate_clearance.py
```

This runs a maximum-wear cycle, checks delivery and maintenance activation, and writes `clearance_validation.json`. A recorded pass reports cycle 28, 3.12° maximum joint error, and an equipment-clearance lower bound of 30 mm.

Additional checks:

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\verify_clearance_guard.py --conveyor
.\.venv\Scripts\python.exe .\industrial-workcell\verify_watchdog.py
```

The obstruction check temporarily moves equipment to verify that the guard stops movement and restores the equipment afterward. `validate_scene.py` provides a longer production-sequence check covering the wear threshold and reset behavior. Reload the saved production scene before returning to normal operation if an integration check is interrupted.
