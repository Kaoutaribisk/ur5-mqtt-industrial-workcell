import json,math
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

folder=Path(__file__).resolve().parent
c=RemoteAPIClient(port=23001);s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
js=s.getObjectsInTree(s.getObject('/UR5'),s.object_joint_type,0)
original=[s.getJointPosition(j) for j in js]
poses=json.loads((folder.parent/'production_cell.config.json').read_text())['poses']
candidates=json.loads((folder/'machine_candidates.json').read_text())
rc=s.createCollection(0);s.addItemToCollection(rc,s.handle_tree,s.getObject('/UR5'),0)
mc=s.createCollection(0)
for h in s.getObjectsInTree(s.getObject('/Workcell'),s.object_shape_type,0):
    if s.getObjectAlias(h).startswith('Machine_'):s.addItemToCollection(mc,s.handle_single,h,0)
cache={}
def edge(a,b):
    key=(tuple(a),tuple(b))
    if key in cache:return cache[key]
    end=[x+(y-x+math.pi)%(2*math.pi)-math.pi for x,y in zip(a,b)]
    count=max(30,math.ceil(max(abs(y-x) for x,y in zip(a,end))/.035))
    minimum=1
    for step in range(count+1):
        for j,x,y in zip(js,a,end):s.setJointPosition(j,x+(y-x)*step/count)
        result,d,pair=s.checkDistance(rc,mc,.25)
        distance=d[-1] if result else .25
        minimum=min(minimum,distance)
        if distance<.025:break
    cache[key]=minimum
    return minimum
try:
    options=[]
    for i,entry in enumerate(candidates['machine_entry']):
        distances=[edge(poses['pick_above'],entry),edge(poses['home'],entry),edge(entry,poses['place_above'])]
        print('entry',i,'transfer margins',distances,flush=True)
        if min(distances)<.025:continue
        for j,inside in enumerate(candidates['machine']):
            margin=edge(entry,inside)
            print('machine',i,j,'insertion margin',margin,flush=True)
            if margin>=.025:options.append((min(distances+[margin]),i,j))
    if not options:raise RuntimeError('No direct safe route; intermediate routing needed')
    margin,i,j=max(options)
    poses['machine_above']=candidates['machine_entry'][i]
    poses['machine']=candidates['machine'][j]
    # Store updated calibrated targets without stale handle references.
    root=s.getObject('/Workcell')
    config=s.unpackTable(s.readCustomDataBlock(root,'workcell.config'))
    config['poses']=poses
    s.writeCustomDataBlock(root,'workcell.config',s.packTable(config))
    (folder/'clearance_route.json').write_text(json.dumps(dict(poses=poses,entryCandidate=i,machineCandidate=j,nominalMinimumClearance=margin),indent=2))
    (folder.parent/'production_cell.config.json').write_text(json.dumps(config,indent=2))
    print('SELECTED',i,j,'minimum',margin,flush=True)
finally:
    for j,q in zip(js,original):s.setJointPosition(j,q)
    s.destroyCollection(rc);s.destroyCollection(mc)
