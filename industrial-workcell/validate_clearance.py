"""Check one full cycle at maximum pre-maintenance wear, with the live guard."""
import json,time,uuid
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client=RemoteAPIClient(port=23001);sim=client.require('sim')
assert sim.getSimulationState()==sim.simulation_stopped
script=sim.getObject('/Workcell_controller')
originalSource=sim.getObjectStringParam(script,sim.scriptstringparam_text)
if isinstance(originalSource,bytes):originalSource=originalSource.decode('utf-8')
testSource=originalSource.replace('total=0; sinceMaintenance=0;', 'total=27; sinceMaintenance=27;')
assert testSource!=originalSource
sim.setObjectStringParam(script,sim.scriptstringparam_text,testSource)
real=sim.getBoolParam(sim.boolparam_realtime_simulation)
sim.setBoolParam(sim.boolparam_realtime_simulation,False)
display=sim.getBoolParam(sim.boolparam_display_enabled)
sim.setBoolParam(sim.boolparam_display_enabled,False)
client.setStepping(True)
sim.startSimulation()
def read():return json.loads(sim.getStringSignal('workcell.telemetry'))
def command(action,**kwargs):
    identity=uuid.uuid4().hex
    sim.setStringSignal('workcell.command',json.dumps(dict(id=identity,action=action,**kwargs)))
    for _ in range(10):
        client.step()
        response=sim.getStringSignal('workcell.response')
        if response and json.loads(response).get('id')==identity:
            client.step();return json.loads(response)
    raise AssertionError('Command not acknowledged: '+action)
try:
    client.step()
    # Exercise the largest expected wear offset directly, without rerunning 27 healthy cycles.
    command('speed',value=8)
    command('start')
    last=''
    for step in range(10000):
        client.step();data=read()
        if data['state']!=last:
            print(data['state'],'min clearance',round(data['minimumMachineClearance']*1000,1),'mm',flush=True)
            last=data['state']
        if data['fault']:raise AssertionError(data['fault'])
        if data['state']=='maintenance':break
        if not data['running']:raise AssertionError('Unexpected pause: '+str(data['events']))
    else:raise AssertionError('Cycle did not complete')
    assert data['totalCycles']==28 and data['delivered']==1
    assert data['minimumMachineClearance']>=.015
    assert data['health']=='alarm'
    report=dict(result='PASS',wearCycle=28,minimumClearanceMillimeters=data['minimumMachineClearance']*1000,maxJointSpeedRadiansPerSecond=data['maxJointSpeed'],jointErrorDegrees=max(data['settledErrors']))
    Path(__file__).with_name('clearance_validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
finally:
    sim.stopSimulation();client.setStepping(False)
    for _ in range(40):
        if sim.getSimulationState()==sim.simulation_stopped:break
        time.sleep(.05)
    sim.setBoolParam(sim.boolparam_realtime_simulation,real)
    sim.setBoolParam(sim.boolparam_display_enabled,display)
    sim.setObjectStringParam(script,sim.scriptstringparam_text,originalSource)
