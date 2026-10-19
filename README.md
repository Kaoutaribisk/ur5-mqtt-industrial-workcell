# UR5 MQTT Industrial Workcell

A UR5 production-cell simulation with conveyor handling, a processing station, MQTT control, and a React monitoring dashboard.

The robot transfers a part from the incoming conveyor into a generic processing station, retrieves it after processing, and places it on the outgoing conveyor. The dashboard shows production progress, joint measurements, and simulated maintenance alerts.

## Simulation video

[View or download the simulation recording](simulation%20demo.mp4)

The recording is approximately 213 MiB. See [Sharing the recording](#sharing-the-recording) for GitHub upload options.

## Features

- Automated loading, processing, unloading, and conveyor discharge.
- Blue industrial dashboard with production and manual-control views.
- MQTT, backend, and simulator connection indicators.
- Received, processed, delivered, and completed-cycle counters.
- Six joint measurements with target angles, settled errors, and maintenance thresholds.
- Cycle history and the five most recent feedback events.
- Production start, pause, stop, stop reset, and maintenance reset controls.
- Motion clearance checks against the conveyors and processing station.

## System overview

```text
React dashboard  <-- MQTT / HiveMQ -->  Python bridge
                                           |
                                       ZeroMQ API
                                           |
                                   CoppeliaSim scene
                              UR5 + conveyors + station
```

The scene controller runs the production sequence. The Python bridge forwards commands and publishes telemetry. The browser displays the latest state and sends operator requests.

## Requirements

- CoppeliaSim, tested with version 4.10 Edu.
- Python, tested with version 3.12.
- Node.js and npm.
- Internet access for the default public HiveMQ broker.

The following commands use PowerShell from the repository root.

## Setup

Create a Python environment and install the bridge dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Install and build the dashboard:

```powershell
Set-Location remote-robot-frontend
npm ci
npm run build
Set-Location ..
```

## Run the workcell

1. Open `production_cell.ttt` in CoppeliaSim.
2. Ensure the ZeroMQ remote API uses port **23001**, matching the production bridge.
3. Run the dashboard server and bridge:

```powershell
.\.venv\Scripts\python.exe .\industrial-workcell\run_workcell.py
```

4. Open [the local dashboard](http://127.0.0.1:3000/) and select **Start** in the production view.

If CoppeliaSim is installed in its standard Windows location, it can be launched with the required port using:

```powershell
& "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe" -GzmqRemoteApi.rpcPort=23001 ".\production_cell.ttt"
```

Keep the launcher running while using the dashboard. Press `Ctrl+C` in its terminal to stop the local server and bridge. Do not run `robot_backend.py` alongside the production bridge: both consume the legacy joint-command topic.

For a different simulator port or frontend development, see the [workcell documentation](industrial-workcell/README.md) and [dashboard documentation](remote-robot-frontend/README.md).

## Production sequence

1. Feed a part to the incoming pickup position.
2. Detect the part and pick it up with the simulated vacuum tool.
3. Load the processing station through its front opening.
4. Wait for the processing interval.
5. Retrieve the part and place it on the outgoing conveyor.
6. Discharge the part and return the robot home.
7. Record the completed cycle and check maintenance conditions.

The scene reuses a single product object between cycles. At normal speed, processing takes approximately two simulated seconds. Faster demo settings accelerate conveyor travel and processing; the robot retains its configured joint-speed limit.

## Joint monitoring and maintenance

Joint feedback comes from the simulated robot. The controller compares each settled joint angle with its nominal target, excluding movement transients from the maintenance decision.

| Condition | Behavior |
| --- | --- |
| First 15 completed cycles | No injected wear offset |
| From cycle 16 | Artificial joint offsets increase each cycle |
| Settled error reaches 1° | Warning |
| Settled error reaches 3° | Maintenance required; finish the current cycle and block the next |
| Maintenance reset | Return home and clear wear and the maintenance cycle count |

The default model reaches the maintenance threshold at cycle 28, with a maximum joint error of 3.12°. Maintenance reset preserves lifetime production counters; restarting the simulation resets scene counters and wear.

These offsets are a demonstration model, not a prediction of physical robot wear.

## Controls and simulation limits

**Stop cell** holds the simulated production line and latches a stop condition. Reset the stop before starting again. The manual view's emergency-stop-style button locks browser joint commands; it is not a hardware safety circuit.

Robot motion uses kinematic joint trajectories with smooth endpoint easing. Parts attach to the vacuum tool during transfer. This is not a fully dynamic grasping or conveyor-contact simulation.

The clearance guard checks robot and tool geometry against both conveyors and the machine, stopping before a candidate movement falls below 15 mm clearance. It does not certify self-collision avoidance, carried-part clearance, or real-world machine safety.

## Repository structure

```text
production_cell.ttt                 Main production scene
production_cell.config.json         Saved scene configuration
UR5.ttm                            UR5 model
first_sim.ttt                      Original scene
robot_backend.py                   Original manual-control backend
mqtt_config.py                     Original backend MQTT settings
requirements.txt                   Python dependencies
simulation demo.mp4                Simulation recording
industrial-workcell/
  workcell.lua                     Scene production controller
  workcell_backend.py              MQTT / ZeroMQ bridge
  run_workcell.py                  Built-dashboard server and bridge launcher
  clearance_route.json             Calibrated robot routes
remote-robot-frontend/
  src/                             React dashboard and tests
```

## Validation

Run the frontend checks from `remote-robot-frontend`:

```powershell
npm test -- --watchAll=false --runInBand
npm run build
```

Run the bridge unit tests from the repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s industrial-workcell -p test_backend.py -v
```

Simulator integration checks and their requirements are described in the [workcell documentation](industrial-workcell/README.md#validation).

## MQTT configuration

The default broker is `broker.hivemq.com`: TCP port 1883 for Python and WebSocket port 8000 for the browser. The original `robot/joint_command` message format is preserved. Production topics use the `kaoutar/workcell/` prefix.

This public broker has shared topics and no project-specific access control. Use a private broker and a dedicated topic namespace for private deployments. See the [topic reference](industrial-workcell/README.md#mqtt-protocol) before changing configuration.

## Sharing the recording

The recording exceeds GitHub's [100 MiB regular-file limit](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github). Store it with [Git LFS](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage), or upload it as a [release asset](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases) and add its download link to the Simulation video section above.

Before adding the video to a commit, Git LFS users can run:

```powershell
git lfs install
git lfs track "simulation demo.mp4"
git add .gitattributes
git add -f "simulation demo.mp4"
```

Keep local environments, `.env`, `node_modules`, generated builds, logs, and temporary backup scenes out of published commits.

## Original project

The original MQTT manual-control project is credited to **BEKHTI Djalal**. This version extends that foundation with a production dashboard, automated workcell, and simulated condition monitoring.
