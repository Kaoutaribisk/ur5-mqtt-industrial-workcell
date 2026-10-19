"""Block a future robot waypoint and check rollback before machine contact."""
import json,time,sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client=RemoteAPIClient(port=23001);s=client.require('sim')
assert s.getSimulationState()==s.simulation_stopped
real=s.getBoolParam(s.boolparam_realtime_simulation)
display=s.getBoolParam(s.boolparam_display_enabled)
s.setBoolParam(s.boolparam_realtime_simulation,False)
s.setBoolParam(s.boolparam_display_enabled,False)
conveyor='--conveyor' in sys.argv
header=s.getObject('/Infeed_belt' if conveyor else '/Machine_header');original=s.getObjectPose(header,-1)
client.setStepping(True);s.startSimulation()
try:
    client.step()
    config=s.unpackTable(s.readCustomDataBlock(s.getObject('/Workcell'),'workcell.config'))
    s.setObjectPosition(header,-1,[-config.get('beltX',.65),config.get('pickupY',-.15),.97])
    s.setStringSignal('workcell.command',json.dumps(dict(action='start',id='guard-test')))
    for step in range(2000):
        client.step()
        data=json.loads(s.getStringSignal('workcell.telemetry'))
        if data['fault']:
            assert data['fault'].startswith('Clearance stop:')
            assert data['stopped'] and not data['running']
            robot=s.createCollection(0);s.addItemToCollection(robot,s.handle_tree,s.getObject('/UR5'),0)
            collision,_=s.checkCollision(robot,header)
            s.destroyCollection(robot)
            assert collision==0,'Guard left the robot intersecting the obstacle'
            print('PASS: obstructed path stops before contact; robot remains collision-free',flush=True)
            break
    else:raise AssertionError('Guard did not trigger')
finally:
    s.stopSimulation();client.setStepping(False)
    for _ in range(40):
        if s.getSimulationState()==s.simulation_stopped:break
        time.sleep(.05)
    s.setObjectPose(header,-1,original)
    s.setBoolParam(s.boolparam_realtime_simulation,real)
    s.setBoolParam(s.boolparam_display_enabled,display)
