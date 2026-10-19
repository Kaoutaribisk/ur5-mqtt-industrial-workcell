"""Widen the conveyors and select consistent IK branches clear of all equipment."""
import itertools,json,math,random,time
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

folder=Path(__file__).resolve().parent
c=RemoteAPIClient(port=23001);s=c.require('sim');ik=c.require('simIK')
s.stopSimulation()
for _ in range(60):
    if s.getSimulationState()==s.simulation_stopped:break
    time.sleep(.05)
assert s.getSimulationState()==s.simulation_stopped
backup=folder.parent/'production_cell.before-wide-layout.ttt'
if not backup.exists():s.saveScene(str(backup))
root=s.getObject('/Workcell');robot=s.getObject('/UR5')
js=s.getObjectsInTree(robot,s.object_joint_type,0)
original=[s.getJointPosition(j) for j in js]
display=s.getBoolParam(s.boolparam_display_enabled)
s.setBoolParam(s.boolparam_display_enabled,False)
shapes=s.getObjectsInTree(root,s.object_shape_type,0)
for prefix,desired in [('Infeed',-.65),('Outfeed',.65)]:
    current=s.getObjectPosition(s.getObject('/'+prefix+'_belt'),-1)[0]
    for h in s.getObjectsInTree(root,s.handle_all,0):
        if s.getObjectAlias(h).startswith(prefix+'_'):
            pos=s.getObjectPosition(h,-1);pos[0]+=desired-current;s.setObjectPosition(h,-1,pos)
    side=-1 if desired<0 else 1
    pos=[desired+side*.17,-.15,.90]
    goal=[desired,-.15,.785]
    z=[b-a for a,b in zip(pos,goal)];norm=math.sqrt(sum(v*v for v in z));z=[v/norm for v in z]
    x=[0,side,0];y=[-side*z[2],0,side*z[0]]
    s.setObjectMatrix(s.getObject('/'+prefix+'_sensor'),-1,[v for i in range(3) for v in (x[i],y[i],z[i],pos[i])])
    s.setObjectPosition(s.getObject('/'+prefix+'_sensor_housing'),-1,[desired+side*.19,-.15,.92])
s.setObjectPosition(s.getObject('/Product'),-1,[-.65,-1.15,.785])
rc=s.createCollection(0)
for h in s.getObjectsInTree(robot,s.object_shape_type,0):s.addItemToCollection(rc,s.handle_single,h,0)
obstacles=s.createCollection(0)
for h in shapes:
    if s.getObjectAlias(h).startswith(('Machine_','Infeed_','Outfeed_')):s.addItemToCollection(obstacles,s.handle_single,h,0)
helper=s.createScript(s.scripttype_customization,"""
sim=require 'sim'
function configure(j,r,o) joints=j;robot=r;obstacles=o end
function clear(q,margin)
    for i,j in ipairs(joints) do sim.setJointPosition(j,q[i]) end
    if sim.checkCollision(robot,obstacles)>0 then return false end
    return sim.checkDistance(robot,obstacles,margin)==0
end
function edge(a,b,margin)
    local n=1;local goal={}
    for i=1,6 do goal[i]=a[i]+(b[i]-a[i]+math.pi)%(2*math.pi)-math.pi;n=math.max(n,math.ceil(math.abs(goal[i]-a[i])/.025)) end
    for _,wear in ipairs({0,1.56,3.12}) do
        for step=0,n do
            local q={}
            for i=1,6 do q[i]=a[i]+(goal[i]-a[i])*step/n+math.rad(wear)*({1,.86,.72,.58,.46,.34})[i]*(i%2==0 and -1 or 1) end
            if not clear(q,margin) then return false end
        end
    end
    return true
end
function plan(a,b)
    local ompl=require 'simOMPL'
    local task=ompl.createTask('Wide_cell_transfer')
    local spaces={}
    for i,j in ipairs(joints) do
        spaces[i]=ompl.createStateSpace('j'..i,ompl.StateSpaceType.joint_position,j,{-math.pi},{math.pi},i<=3 and 1 or 0)
    end
    ompl.setStateSpace(task,spaces)
    ompl.setAlgorithm(task,ompl.Algorithm.RRTConnect)
    ompl.setStateValidationCallback(task,function(q)
        for _,wear in ipairs({0,1.56,3.12}) do
            local v={}
            for i=1,6 do v[i]=q[i]+math.rad(wear)*({1,.86,.72,.58,.46,.34})[i]*(i%2==0 and -1 or 1) end
            if not clear(v,.025) then return false end
        end
        return true
    end)
    ompl.setStateValidityCheckingResolution(task,.001)
    ompl.setStartState(task,a);ompl.setGoalState(task,b)
    ompl.setVerboseLevel(task,0)
    ompl.setup(task)
    local solved,path=ompl.compute(task,4,.3,0)
    ompl.destroyTask(task)
    for _,space in ipairs(spaces) do ompl.destroyStateSpace(space) end
    if not solved then return {} end
    local points={}
    for i=1,#path,6 do local q={} for j=1,6 do q[j]=path[i+j-1] end;table.insert(points,q) end
    return points
end
""",0,'lua')
s.setObjectAlias(helper,'Temporary_path_validation')
s.setObjectParent(helper,root,False)
s.initScript(helper)
s.callScriptFunction('configure',helper,js,rc,obstacles)
environment=ik.createEnvironment();group=ik.createGroup(environment)
ik.setGroupCalculation(environment,group,ik.method_damped_least_squares,.01,220)
ik.addElementFromScene(environment,group,robot,s.getObject('/Vacuum_tip'),s.getObject('/IK_target'),ik.constraint_pose)
old=json.loads((folder.parent/'production_cell.config.json').read_text())['poses']
points={'home':[0,-.40,1.0],'pick_above':[-.65,-.15,.93],'pick':[-.65,-.15,.825],
        'machine_above':[0,.20,.825],'machine':[0,.54,.825],
        'place_above':[.65,-.15,.93],'place':[.65,-.15,.825]}
candidates={};cache={}
def norm(q):return [(x+math.pi)%(2*math.pi)-math.pi for x in q]
def cost(a,b):
    deltas=[abs((y-x+math.pi)%(2*math.pi)-math.pi) for x,y in zip(a,b)]
    return max(deltas)+.20*sum(deltas)+.15*sum(deltas[3:])
def edge(a,b):
    key=tuple(sorted((tuple(a),tuple(b))))
    if key not in cache:cache[key]=s.callScriptFunction('edge',helper,a,b,.025)
    return cache[key]
def plan_raw(start,goal):
    """Bidirectional joint-space RRT; validity stays in short, interruptible API calls."""
    trees=[dict(nodes=[start],parents=[-1],start=True),dict(nodes=[goal],parents=[-1],start=False)]
    def length(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    def extend(tree,target):
        index=min(range(len(tree['nodes'])),key=lambda i:length(tree['nodes'][i],target))
        q=tree['nodes'][index];distance=length(q,target)
        if distance<1e-8:return index,True
        fraction=min(1,.45/distance)
        node=[x+(y-x)*fraction for x,y in zip(q,target)]
        if not edge(q,node):return None,False
        tree['nodes'].append(node);tree['parents'].append(index)
        return len(tree['nodes'])-1,fraction==1
    def ancestry(tree,index):
        path=[]
        while index>=0:path.append(tree['nodes'][index]);index=tree['parents'][index]
        return list(reversed(path))
    deadline=time.monotonic()+35
    for iteration in range(2400):
        a,b=trees
        target=b['nodes'][0] if random.random()<.2 else [random.uniform(-math.pi,math.pi) for _ in range(6)]
        ia,_=extend(a,target)
        if ia is not None:
            for _ in range(40):
                ib,reached=extend(b,a['nodes'][ia])
                if ib is None:break
                if reached:
                    pa=ancestry(a,ia);pb=ancestry(b,ib)
                    path=pa+list(reversed(pb))[1:]
                    if not a['start']:path=list(reversed(path))
                    # Greedy collision-checked shortcuts keep transfers short.
                    sparse=[path[0]];i=0
                    while i<len(path)-1:
                        for j in range(len(path)-1,i,-1):
                            if max(abs(x-y) for x,y in zip(path[i],path[j]))<math.pi and edge(path[i],path[j]):break
                        sparse.append(path[j]);i=j
                    print('Detour found:',len(path),'raw nodes,',len(sparse),'corners',flush=True)
                    return sparse
        trees=[b,a]
        if time.monotonic()>deadline:break
    return []
try:
    random.seed(37)
    saved=folder/'layout_candidates.json'
    if saved.exists():candidates=json.loads(saved.read_text())
    for name,pos in points.items():
        if name in candidates:continue
        s.setObjectPose(s.getObject('/IK_target'),-1,pos+[1,0,0,0])
        found=[]
        for attempt in range(180):
            seed=old[name] if attempt==0 else [random.uniform(-math.pi,math.pi) for _ in js]
            for j,q in zip(js,seed):s.setJointPosition(j,q)
            result=ik.handleGroup(environment,group,{'syncWorlds':True})
            if result[0]!=ik.result_success:continue
            q=norm([s.getJointPosition(j) for j in js])
            if any(sum(abs(x-y) for x,y in zip(q,other))<.06 for other in found):continue
            if s.callScriptFunction('edge',helper,q,q,.025):found.append(q)
        if not found:raise RuntimeError('No clear IK branch for '+name)
        candidates[name]=found
        print(name,len(found),'branches clear of conveyors and machine',flush=True)
        saved.write_text(json.dumps(candidates,indent=2))
    best=None
    for home,entry in []:
        if not edge(home,entry):continue
        machines=[q for q in candidates['machine'] if edge(entry,q)]
        if not machines:continue
        machine=min(machines,key=lambda q:cost(entry,q))
        picks=[];places=[]
        for above in candidates['pick_above']:
            if not edge(home,above) or not edge(above,entry):continue
            for q in candidates['pick']:
                if edge(above,q):picks.append((cost(home,above)+cost(above,entry)+2*cost(above,q),above,q))
        for above in candidates['place_above']:
            if not edge(entry,above) or not edge(above,home):continue
            for q in candidates['place']:
                if edge(above,q):places.append((cost(entry,above)+cost(above,home)+2*cost(above,q),above,q))
        if not picks or not places:continue
        pick=min(picks,key=lambda x:x[0]);place=min(places,key=lambda x:x[0])
        score=pick[0]+place[0]+2*cost(home,entry)+4*cost(entry,machine)
        if best is None or score<best[0]:
            best=(score,dict(home=home,pick_above=pick[1],pick=pick[2],machine_above=entry,machine=machine,place_above=place[1],place=place[2]))
            print('Found consistent clear cycle; angular travel score',round(score,2),flush=True)
    routes={}
    if best is None:
        print('Direct transfers obstructed; planning rounded detours',flush=True)
        # Select short, consistent docking pairs first; plan free-space transfers between them.
        docking=[]
        for entry,machine in itertools.product(candidates['machine_above'],candidates['machine']):
            if not edge(entry,machine):continue
            for above,pick in itertools.product(candidates['pick_above'],candidates['pick']):
                if not edge(above,pick):continue
                for out,place in itertools.product(candidates['place_above'],candidates['place']):
                    if not edge(out,place):continue
                    for home in candidates['home']:
                        selected=dict(home=home,pick_above=above,pick=pick,machine_above=entry,machine=machine,place_above=out,place=place)
                        score=sum(cost(selected[a],selected[b]) for a,b in [('home','pick_above'),('pick_above','machine_above'),('machine_above','home'),('machine_above','place_above'),('place_above','home')])
                        docking.append((score,selected))
        def interpolate(a,b,t):return [x+(y-x)*t for x,y in zip(a,b)]
        def rounded(raw,fraction):
            out=[raw[0]]
            for i in range(1,len(raw)-1):
                before=interpolate(raw[i],raw[i-1],fraction)
                after=interpolate(raw[i],raw[i+1],fraction)
                out.append(before)
                for n in range(1,13):
                    t=n/12
                    out.append([(1-t)**2*x+2*(1-t)*t*y+t*t*z for x,y,z in zip(before,raw[i],after)])
            out.append(raw[-1])
            return out
        failures=set()
        for score,selected in sorted(docking,key=lambda item:item[0])[:24]:
            planned={};valid=True
            required=[('home','pick_above'),('pick_above','pick'),('pick_above','machine_above'),('machine_above','machine'),('machine_above','home'),('machine_above','place_above'),('place_above','place'),('place_above','home')]
            for a,b in required:
                qa,qb=selected[a],selected[b]
                key=tuple(sorted((tuple(qa),tuple(qb))))
                if key in failures:valid=False;break
                if edge(qa,qb):path=[qa,[x+(y-x+math.pi)%(2*math.pi)-math.pi for x,y in zip(qa,qb)]]
                else:
                    print('Plan',a,'to',b,flush=True)
                    raw=plan_raw(qa,qb)
                    if not raw:failures.add(key);valid=False;break
                    path=None
                    for fraction in [.20,.10,.04]:
                        smooth=rounded(raw,fraction)
                        if all(edge(x,y) for x,y in zip(smooth,smooth[1:])):path=smooth;break
                    if path is None:failures.add(key);valid=False;break
                planned[a+'>'+b]=path
                planned[b+'>'+a]=list(reversed(path))
            if valid:best=(score,selected);routes=planned;break
        if best is None:raise RuntimeError('Could not plan a complete clear cycle')
    config=s.unpackTable(s.readCustomDataBlock(root,'workcell.config'))
    config['poses']=best[1];config['beltX']=.65;config['pickupY']=-.15
    config['routes']=routes
    s.writeCustomDataBlock(root,'workcell.config',s.packTable(config))
    (folder.parent/'production_cell.config.json').write_text(json.dumps(config,indent=2))
    (folder/'clearance_route.json').write_text(json.dumps(dict(poses=best[1],routes=routes,targets=points,beltX=.65,pickupY=-.15,margin=.025,wearTested=[0,1.56,3.12]),indent=2))
    (folder/'layout_candidates.json').write_text(json.dumps(candidates,indent=2))
    print('Selected clear wide-layout cycle',flush=True)
finally:
    for j,q in zip(js,original):s.setJointPosition(j,q)
    ik.eraseEnvironment(environment)
    s.removeObjects([helper])
    s.destroyCollection(rc);s.destroyCollection(obstacles)
    s.setBoolParam(s.boolparam_display_enabled,display)
