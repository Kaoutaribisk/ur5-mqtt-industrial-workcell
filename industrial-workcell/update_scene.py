from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from camera_view import frame_cell

client=RemoteAPIClient(port=23001)
sim=client.require('sim')
assert sim.getSimulationState()==sim.simulation_stopped
script=sim.getObject('/Workcell_controller')
for h in sim.getObjectsInTree(sim.getObject('/UR5'),sim.object_script_type,0):
    sim.setObjectInt32Param(h,sim.scriptintparam_enabled,0)
sim.setObjectStringParam(script,sim.scriptstringparam_text,Path(__file__).with_name('workcell.lua').read_text(encoding='utf-8-sig'))
config=sim.unpackTable(sim.readCustomDataBlock(sim.getObject('/Workcell'),'workcell.config'))
for joint,q in zip(sim.getObjectsInTree(sim.getObject('/UR5'),sim.object_joint_type,0),config['poses']['home']):
    sim.setJointPosition(joint,q)
camera=sim.getObject('/DefaultCamera')
frame_cell(sim)
sim.setBoolParam(sim.boolparam_realtime_simulation,True)
sim.setFloatParam(sim.floatparam_simulation_time_step,.02)
sim.saveScene(str(Path(__file__).resolve().parent.parent/'production_cell.ttt'))
print('Scene controller and camera updated')
