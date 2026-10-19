"""Exercise the saved cell using simulator stepping; leaves simulation stopped."""
import json
import uuid
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client=RemoteAPIClient(port=23001)
sim=client.require('sim')
realTime=sim.getBoolParam(sim.boolparam_realtime_simulation)
sim.setBoolParam(sim.boolparam_realtime_simulation,False)
client.setStepping(True)
sim.startSimulation()

def command(action, **fields):
    identity=uuid.uuid4().hex
    sim.setStringSignal('workcell.command',json.dumps(dict(action=action,id=identity,**fields)))
    for _ in range(10):
        client.step()
        response=sim.getStringSignal('workcell.response')
        if response and json.loads(response).get('id')==identity:
            client.step()
            return json.loads(response)
    raise AssertionError('No acknowledgement for '+action)

def telemetry():
    raw=sim.getStringSignal('workcell.telemetry')
    return json.loads(raw) if raw else {}

try:
    client.step()
    command('speed',value=8)
    command('start')
    seen=set()
    last=-1
    for n in range(120000):
        client.step()
        data=telemetry()
        if not data: raise AssertionError('No workcell telemetry: controller did not initialize')
        seen.add(data['state'])
        if data['fault']: raise AssertionError(data['fault'])
        if data['totalCycles']!=last:
            print('cycles',data['totalCycles'],'health',data['health'],'error',max(data['settledErrors']),flush=True)
            last=data['totalCycles']
        if data['state']=='maintenance': break
    else: raise AssertionError('Maintenance threshold was not reached')
    assert data['totalCycles']>=16
    assert data['received']==data['processed']==data['delivered']==data['totalCycles']
    assert data['health']=='alarm' and not data['running']
    assert max(data['settledErrors'])>=3
    assert data.get('minimumMachineClearance',1)>=.015
    command('start')
    assert not telemetry()['running'], 'Alarm did not block restart'
    command('maintenance')
    reset=telemetry()
    assert reset['cyclesSinceMaintenance']==0 and not reset['maintenanceRequired']
    assert reset['totalCycles']==data['totalCycles']
    command('start')
    command('stop')
    stopped=telemetry()
    angles=stopped['actualAngles']
    for _ in range(10): client.step()
    assert telemetry()['actualAngles']==angles
    command('start')
    assert telemetry()['stopped'] and not telemetry()['running']
    report=dict(result='PASS',cycles=data['totalCycles'],maxError=max(data['settledErrors']),minimumMachineClearance=data.get('minimumMachineClearance'),
                stages=sorted(seen),checks=['full production loop','wear onset','maintenance alarm','restart interlock','maintenance reset preserves totals','stop holds motion'])
    Path(__file__).with_name('validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
finally:
    sim.stopSimulation()
    client.setStepping(False)
    import time
    for _ in range(40):
        if sim.getSimulationState()==sim.simulation_stopped:break
        time.sleep(.05)
    sim.setBoolParam(sim.boolparam_realtime_simulation,realTime)

