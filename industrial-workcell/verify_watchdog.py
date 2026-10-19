"""Local-only watchdog check; does not use MQTT or publish telemetry externally."""
import json
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client=RemoteAPIClient(port=23001)
sim=client.require('sim')
client.setStepping(True)
sim.startSimulation()
try:
    client.step()
    sim.setFloatSignal('workcell.heartbeat',sim.getSystemTime())
    sim.setStringSignal('workcell.command',json.dumps(dict(action='start',id='watchdog-test')))
    for _ in range(3): client.step()
    before=json.loads(sim.getStringSignal('workcell.telemetry'))
    assert before['running']
    sim.setFloatSignal('workcell.heartbeat',sim.getSystemTime()-10)
    for _ in range(3): client.step()
    after=json.loads(sim.getStringSignal('workcell.telemetry'))
    assert not after['running']
    assert any('heartbeat lost' in entry['message'] for entry in after['events'])
    print('PASS: stale backend heartbeat pauses the active workcell')
finally:
    sim.stopSimulation()
    client.setStepping(False)
