import json
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

c=RemoteAPIClient(port=23001); s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
r=s.getObject('/UR5'); root=s.getObject('/Workcell')
js=s.getObjectsInTree(r,s.object_joint_type,0)
original=[s.getJointPosition(j) for j in js]
poses=json.loads((Path(__file__).resolve().parent.parent/'production_cell.config.json').read_text())['poses']
robot=s.createCollection(0); s.addItemToCollection(robot,s.handle_tree,r,0)
machine=s.createCollection(0)
for h in s.getObjectsInTree(root,s.object_shape_type,0):
    if s.getObjectAlias(h).startswith('Machine_'): s.addItemToCollection(machine,s.handle_single,h,0)
def check(q):
    for j,x in zip(js,q): s.setJointPosition(j,x)
    hit,pair=s.checkCollision(robot,machine)
    result,distance,pair2=s.checkDistance(robot,machine,1)
    return hit,distance[-1] if result else 1,pair if hit else pair2
try:
    for name,q in poses.items():
        hit,dist,pair=check(q)
        print(name,hit,round(dist,4),[s.getObjectAlias(h) for h in pair],flush=True)
    sequence=['home','pick_above','pick','pick_above','machine_above','machine','machine_above','home','machine_above','machine','machine_above','place_above','place','place_above','home']
    for a,b in zip(sequence,sequence[1:]):
        start=poses[a]
        end=[x+((y-x+3.141592653589793)%(2*3.141592653589793)-3.141592653589793) for x,y in zip(start,poses[b])]
        minimum=1; collisions=0; pair=[]
        for i in range(51):
            q=[x+(y-x)*i/50 for x,y in zip(start,end)]
            hit,dist,p=check(q)
            if dist<minimum: minimum=dist; pair=p
            collisions+=hit
        print('EDGE',a,b,'hits',collisions,'min',round(minimum,4),[s.getObjectAlias(h) for h in pair],flush=True)
finally:
    for j,q in zip(js,original): s.setJointPosition(j,q)
    s.destroyCollection(robot); s.destroyCollection(machine)
