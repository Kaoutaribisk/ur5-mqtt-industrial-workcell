import json, math, random
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

c=RemoteAPIClient(port=23001); s=c.require('sim'); k=c.require('simIK')
assert s.getSimulationState()==s.simulation_stopped
r=s.getObject('/UR5'); js=s.getObjectsInTree(r,s.object_joint_type,0)
original=[s.getJointPosition(j) for j in js]
backup=Path(__file__).resolve().parent.parent/'production_cell.before-clearance.ttt'
if not backup.exists():s.saveScene(str(backup))
# Keep the machine detector out of the vertical tool insertion path.
sensor=s.getObject('/Machine_sensor')
housing=s.getObject('/Machine_sensor_housing')
s.setObjectPosition(housing,-1,[.27,.54,.93])
direction=[-.25,0,-.135]
length=math.sqrt(sum(x*x for x in direction)); z=[x/length for x in direction]
x=[0,-1,0]; y=[z[2],0,-z[0]]
s.setObjectMatrix(sensor,-1,[v for i in range(3) for v in (x[i],y[i],z[i],[.25,.54,.92][i])])
rc=s.createCollection(0); s.addItemToCollection(rc,s.handle_tree,r,0)
mc=s.createCollection(0)
for h in s.getObjectsInTree(s.getObject('/Workcell'),s.object_shape_type,0):
    if s.getObjectAlias(h).startswith('Machine_'): s.addItemToCollection(mc,s.handle_single,h,0)
e=k.createEnvironment(); g=k.createGroup(e)
k.setGroupCalculation(e,g,k.method_damped_least_squares,.01,200)
k.addElementFromScene(e,g,r,s.getObject('/Vacuum_tip'),s.getObject('/IK_target'),k.constraint_pose)
poses={}
random.seed(29)
try:
    for name,pos in {'machine':[0,.54,.825], 'machine_above':[0,.50,1.0], 'machine_entry':[0,.20,.825]}.items():
        s.setObjectPose(s.getObject('/IK_target'),-1,pos+[1,0,0,0])
        found=[]; best=0; pair=[]; collisions={}
        for attempt in range(200):
            for j in js:s.setJointPosition(j,random.uniform(-math.pi,math.pi))
            result=k.handleGroup(e,g,{'syncWorlds':True})
            if result[0]!=k.result_success:continue
            q=[(s.getJointPosition(j)+math.pi)%(2*math.pi)-math.pi for j in js]
            for j,x in zip(js,q):s.setJointPosition(j,x)
            hit,p=s.checkCollision(rc,mc)
            if hit:
                label=' / '.join(s.getObjectAlias(h) for h in p)
                collisions[label]=collisions.get(label,0)+1
            res,d,p=s.checkDistance(rc,mc,.5)
            distance=d[-1] if res else .5
            if distance>best:best=distance;pair=p
            if not hit and distance>=.025:
                if not any(sum(abs(x-y) for x,y in zip(q,old))<.05 for old in found): found.append(q)
        print(name,'free',len(found),'best',best,'closest',[s.getObjectAlias(h) for h in pair],'collisions',collisions,flush=True)
        poses[name]=found
    Path(__file__).with_name('machine_candidates.json').write_text(json.dumps(poses,indent=2))
finally:
    for j,q in zip(js,original):s.setJointPosition(j,q)
    s.destroyCollection(rc);s.destroyCollection(mc);k.eraseEnvironment(e)
