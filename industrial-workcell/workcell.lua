sim = require 'sim'
local json = require 'dkjson'

local function copy(t) local r={} for i,v in ipairs(t) do r[i]=v end return r end
local function deg(x) return x*180/math.pi end
local function event(message)
    eventId=eventId+1
    table.insert(events,1,{id=eventId,time=sim.getSimulationTime(),message=message})
    while #events>5 do table.remove(events) end
end
local function setPart(position)
    sim.setObjectParent(c.part,c.root,true)
    sim.setObjectPosition(c.part,sim.handle_world,position)
    sim.setObjectQuaternion(c.part,sim.handle_world,{0,0,0,1})
end
local function sensorState(handle)
    local result=sim.checkProximitySensor(handle,c.part)
    return result>0
end
local function offsets()
    local worn=math.max(0,sinceMaintenance-14) -- first worn cycle is cycle 16
    local result={}
    for i=1,6 do
        local factor=({1,.86,.72,.58,.46,.34})[i]
        result[i]=math.rad(worn*.24*factor)*(i%2==0 and -1 or 1)
    end
    return result
end
local function moveTo(name)
    reference=copy(c.poses[name])
    local wear=offsets()
    local start={}
    local goal={}
    local largest=0
    for i,j in ipairs(c.joints) do
        start[i]=sim.getJointPosition(j)
        -- Choose an equivalent nearest joint angle to avoid full turns.
        local nominal=(reference[i]+math.pi)%(2*math.pi)-math.pi
        local cyclic,limits=sim.getJointInterval(j)
        local best=nil
        for turns=-1,1 do
            local candidate=nominal+turns*2*math.pi
            if cyclic or (candidate+wear[i]>=limits[1] and candidate+wear[i]<=limits[1]+limits[2]) then
                if not best or math.abs(candidate-start[i])<math.abs(best-start[i]) then best=candidate end
            end
        end
        reference[i]=best or nominal
        goal[i]=reference[i]+wear[i]
        largest=math.max(largest,math.abs(goal[i]-start[i]))
    end
    -- Quintic easing has zero velocity AND acceleration at each endpoint.
    -- Its peak derivative is 1.875; preserve the 0.65 rad/s joint speed cap.
    motion={start=start,goal=goal,time=0,duration=math.max(.9,1.875*largest/.65),name=name}
    local path=c.routes and c.routes[lastPose..'>'..name]
    if path and #path>=2 then
        local cumulative={0};local total=0
        for n=2,#path do
            local square=0 for i=1,6 do square=square+(path[n][i]-path[n-1][i])^2 end
            total=total+math.sqrt(square);cumulative[n]=total
        end
        if total>1e-8 then
            local phase={};local correction={};local pts={}
            for i=1,6 do
                phase[i]=math.floor((start[i]-path[1][i])/(2*math.pi)+.5)*2*math.pi
                correction[i]=start[i]-path[1][i]-phase[i]-wear[i]
                reference[i]=path[#path][i]+phase[i]
            end
            for n,q in ipairs(path) do
                pts[n]={}
                for i=1,6 do pts[n][i]=q[i]+phase[i]+wear[i]+correction[i]*(1-cumulative[n]/total) end
            end
            -- Parameterize the actual worn path, including its small initial correction.
            cumulative={0};total=0
            for n=2,#pts do
                local square=0 for i=1,6 do square=square+(pts[n][i]-pts[n-1][i])^2 end
                total=total+math.sqrt(square);cumulative[n]=total
            end
            motion.points=pts;motion.cumulative=cumulative;motion.length=total
            motion.duration=math.max(.9,1.875*total/.65)
        end
    end
end
local function motionStep(dt)
    if not motion then return true end
    local nextTime=motion.time+sim.getSimulationTimeStep()*math.min(speed,1)
    local a=math.min(1,nextTime/motion.duration)
    local f=a*a*a*(10+a*(-15+6*a))
    local before={}; local candidate={}; local largest=0
    for i,j in ipairs(c.joints) do
        before[i]=sim.getJointPosition(j)
        if motion.points then
            local distance=f*motion.length
            local n=1
            while n<#motion.points-1 and motion.cumulative[n+1]<distance do n=n+1 end
            local span=motion.cumulative[n+1]-motion.cumulative[n]
            local t=span>1e-9 and (distance-motion.cumulative[n])/span or 1
            candidate[i]=motion.points[n][i]+(motion.points[n+1][i]-motion.points[n][i])*t
        else candidate[i]=motion.start[i]+(motion.goal[i]-motion.start[i])*f end
        largest=math.max(largest,math.abs(candidate[i]-before[i]))
    end
    -- Check intermediate configurations, then keep only a clear move.
    local steps=math.max(1,math.ceil(largest/.01))
    for step=1,steps do
        for i,j in ipairs(c.joints) do sim.setJointPosition(j,before[i]+(candidate[i]-before[i])*step/steps) end
        local collision,pair=sim.checkCollision(robotGeometry,machineGeometry)
        -- Threshold broad-phase avoids expensive far-field mesh distance queries.
        local close,distance,nearPair=sim.checkDistance(robotGeometry,machineGeometry,.03)
        local clearance=close>0 and distance[7] or .03
        if collision>0 or clearance<.015 then
            for i,j in ipairs(c.joints) do sim.setJointPosition(j,before[i]) end
            local objects=collision>0 and pair or nearPair
            fault='Clearance stop: '..sim.getObjectAlias(objects[1])..' near '..sim.getObjectAlias(objects[2])
            running=false; stopped=true; event(fault)
            return false
        end
        minimumClearance=math.min(minimumClearance,clearance)
    end
    motion.time=nextTime
    if a<1 then return false end
    lastPose=motion.name
    motion=nil
    for i,j in ipairs(c.joints) do
        settledErrors[i]=math.abs(deg((sim.getJointPosition(j)-reference[i]+math.pi)%(2*math.pi)-math.pi))
        if settledErrors[i]>=3 then maintenance=true end
    end
    return true
end
local function attach()
    -- Kinematic vacuum grasp; sensor/pose check prevents attaching a distant part.
    local p=sim.getObjectPosition(c.part,sim.handle_world)
    local t=sim.getObjectPosition(c.tip,sim.handle_world)
    local distance=math.sqrt((p[1]-t[1])^2+(p[2]-t[2])^2+(p[3]-t[3])^2)
    if distance>.16 then
        fault='Vacuum pickup failed: product outside capture range'
        running=false; event(fault); return false
    end
    sim.setObjectParent(c.part,c.tip,true)
    sim.setObjectPosition(c.part,c.tip,{0,0,.0375})
    sim.setObjectQuaternion(c.part,sim.handle_world,{0,0,0,1})
    carrying=true
    return true
end
local function stage(name,pose)
    state=name; elapsed=0
    if pose then moveTo(pose) end
end
local function gate(closed)
    sim.setObjectPosition(c.door,sim.handle_world,{0,.285,closed and .97 or 1.40})
end

function sysCall_init()
    c=sim.unpackTable(sim.readCustomDataBlock(sim.getObject('..'),'workcell.config'))
    -- Saved custom data does not remap numeric object handles when a scene reloads.
    -- Resolve stable aliases every time the simulation starts.
    c.root=sim.getObject('..')
    c.robot=sim.getObject('/UR5')
    c.joints=sim.getObjectsInTree(c.robot,sim.object_joint_type,0)
    c.tip=sim.getObject('/Vacuum_tip')
    c.part=sim.getObject('/Product')
    c.door=sim.getObject('/Machine_gate')
    c.lamp=sim.getObject('/Machine_lamp')
    c.sensors={sim.getObject('/Infeed_sensor'),sim.getObject('/Machine_sensor'),sim.getObject('/Outfeed_sensor')}
    c.markers={{},{}}
    for belt,prefix in ipairs({'Infeed','Outfeed'}) do
        for i=0,11 do table.insert(c.markers[belt],sim.getObject('/'..prefix..string.format('_slat_%02d',i))) end
    end
    robotGeometry=sim.createCollection(0)
    -- Explicit shapes exclude a carried product from the robot-to-machine margin.
    for _,h in ipairs(sim.getObjectsInTree(c.robot,sim.object_shape_type,0)) do
        sim.addItemToCollection(robotGeometry,sim.handle_single,h,0)
    end
    machineGeometry=sim.createCollection(0)
    for _,h in ipairs(sim.getObjectsInTree(c.root,sim.object_shape_type,0)) do
        local name=sim.getObjectAlias(h)
        if name:sub(1,8)=='Machine_' or name:sub(1,7)=='Infeed_' or name:sub(1,8)=='Outfeed_' then
            sim.addItemToCollection(machineGeometry,sim.handle_single,h,0)
        end
    end
    running=false; stopped=false; maintenance=false; fault=''; carrying=false
    minimumClearance=1
    total=0; sinceMaintenance=0; received=0; processed=0; delivered=0
    state='idle'; elapsed=0; lastPose='home'; reference=copy(c.poses.home); settledErrors={0,0,0,0,0,0}
    c.beltX=c.beltX or .65; c.pickupY=c.pickupY or -.15
    events={}; eventId=0; cycleTime=0; cycleStart=0; motion=nil; outY=c.pickupY
    speed=1; processSeconds=2; infeed=false; outfeed=false
    watchdogEnabled=false
    sim.clearStringSignal('workcell.command')
    sim.clearStringSignal('workcell.response')
    setPart({-c.beltX,-1.15,.785}); gate(false)
    for i,j in ipairs(c.joints) do sim.setJointPosition(j,reference[i]) end
    event('Cell ready; automatic mode stopped')
end

local function dispatch(cmd)
    local accepted=false
    local reason=''
    if cmd.action=='start' then
        if stopped then reason='Reset the stop latch first'
        elseif maintenance then reason='Maintenance required before next cycle'
        elseif fault~='' then reason=fault
        else running=true; accepted=true; event('Automatic mode started') end
    elseif cmd.action=='pause' then
        running=false; accepted=true; event('Automatic mode paused')
    elseif cmd.action=='stop' then
        stopped=true; running=false; accepted=true; event('STOP latched; robot and conveyors held')
    elseif cmd.action=='reset_stop' then
        stopped=false; accepted=true; event('Stop latch reset; press Start to resume')
    elseif cmd.action=='maintenance' then
        if running or carrying or (state~='idle' and state~='maintenance') then
            reason='Maintenance reset requires a completed cycle and a stopped cell'
        else
            sinceMaintenance=0; maintenance=false; fault=''; settledErrors={0,0,0,0,0,0}
            state='idle'; motion=nil; lastPose='home'
            reference=copy(c.poses.home)
            for i,j in ipairs(c.joints) do sim.setJointPosition(j,reference[i]) end
            accepted=true; event('Simulated maintenance complete; wear reset')
        end
    elseif cmd.action=='speed' then
        local v=tonumber(cmd.value)
        if v and v>=.25 and v<=8 then speed=v; accepted=true else reason='Speed must be between 0.25 and 8' end
    elseif cmd.action=='manual' then
        local id=tonumber(cmd.joint)
        local value=tonumber(cmd.value)
        local direction=cmd.direction
        if state~='idle' or running or stopped or maintenance then reason='Manual control requires idle, unlocked cell'
        elseif not id or id%1~=0 or id<0 or id>5 or not value or value<0 or value>360 then reason='Invalid joint command'
        elseif direction~='up' and direction~='down' and direction~='left' and direction~='right' then reason='Invalid direction'
        else
            local sign=(direction=='up' or direction=='right') and 1 or -1
            reference[id+1]=reference[id+1]+sign*math.rad(value)
            sim.setJointPosition(c.joints[id+1],reference[id+1])
            accepted=true; event('Manual J'..(id+1)..' '..direction..' '..value..' deg')
        end
    else reason='Unknown command' end
    if not accepted then event('Rejected: '..reason) end
    sim.setStringSignal('workcell.response',json.encode({id=cmd.id or '',accepted=accepted,reason=reason}))
end

function sysCall_actuation()
    local beat=sim.getFloatSignal('workcell.heartbeat')
    if beat then watchdogEnabled=true end
    if watchdogEnabled and (not beat or sim.getSystemTime()-beat>2.5) and running then
        running=false; event('Backend heartbeat lost; automatic mode paused')
    end
    local raw=sim.getStringSignal('workcell.command')
    if raw then
        sim.clearStringSignal('workcell.command')
        local ok,cmd=pcall(json.decode,raw)
        if ok and type(cmd)=='table' then dispatch(cmd) end
    end
    infeed=false; outfeed=false
    if not running or stopped then return end
    local dt=sim.getSimulationTimeStep()*speed
    elapsed=elapsed+dt
    if state=='idle' then
        cycleStart=sim.getSimulationTime()
        sim.setShapeColor(c.part,nil,sim.colorcomponent_ambient_diffuse,{1,.60,.12})
        setPart({-c.beltX,-1.15,.785}); stage('feeding')
    elseif state=='feeding' then
        infeed=true
        local p=sim.getObjectPosition(c.part,sim.handle_world)
        p[2]=math.min(c.pickupY,p[2]+dt*.45); sim.setObjectPosition(c.part,sim.handle_world,p)
        if p[2]>=c.pickupY and sensorState(c.sensors[1]) then
            received=received+1; event('Product detected at infeed'); stage('approach_pick','pick_above')
        end
    elseif state=='approach_pick' and motionStep(dt) then stage('pick','pick')
    elseif state=='pick' and motionStep(dt) then if attach() then stage('lift_in','pick_above') end
    elseif state=='lift_in' and motionStep(dt) then stage('approach_machine','machine_above')
    elseif state=='approach_machine' and motionStep(dt) then stage('load_machine','machine')
    elseif state=='load_machine' and motionStep(dt) then
        carrying=false; setPart({0,.54,.785}); stage('retract_machine','machine_above')
    elseif state=='retract_machine' and motionStep(dt) then stage('clear_machine','home')
    elseif state=='clear_machine' and motionStep(dt) then
        if not sensorState(c.sensors[2]) then fault='Machine sensor did not detect product'; running=false; event(fault)
        else gate(true); stage('processing'); event('Processing started') end
    elseif state=='processing' and elapsed>=processSeconds then
        sim.setShapeColor(c.part,nil,sim.colorcomponent_ambient_diffuse,{.15,.75,.65})
        processed=processed+1; gate(false); event('Processing complete'); stage('approach_unload','machine_above')
    elseif state=='approach_unload' and motionStep(dt) then stage('unload_machine','machine')
    elseif state=='unload_machine' and motionStep(dt) then if attach() then stage('lift_out','machine_above') end
    elseif state=='lift_out' and motionStep(dt) then stage('approach_place','place_above')
    elseif state=='approach_place' and motionStep(dt) then stage('place','place')
    elseif state=='place' and motionStep(dt) then
        carrying=false; setPart({c.beltX,c.pickupY,.785}); stage('retract_out','place_above')
    elseif state=='retract_out' and motionStep(dt) then
        if not sensorState(c.sensors[3]) then fault='Outfeed sensor did not detect product'; running=false; event(fault)
        else outY=c.pickupY; stage('discharging','home') end
    elseif state=='discharging' then
        outfeed=true; outY=math.max(-1.15,outY-dt*.45)
        sim.setObjectPosition(c.part,sim.handle_world,{c.beltX,outY,.785})
        local homeDone=motionStep(dt)
        if outY<=-1.15 and homeDone then
            delivered=delivered+1; total=total+1; sinceMaintenance=sinceMaintenance+1
            cycleTime=sim.getSimulationTime()-cycleStart
            event('Cycle '..total..' complete; product delivered')
            if maintenance then running=false; stage('maintenance'); event('Maintenance required: settled joint error reached 3 deg')
            else stage('idle') end
        end
    end
    for belt,list in ipairs(c.markers) do
        if (belt==1 and infeed) or (belt==2 and outfeed) then
            for _,h in ipairs(list) do
                local p=sim.getObjectPosition(h,sim.handle_world)
                p[2]=p[2]+dt*.45*(belt==1 and 1 or -1)
                if p[2]>-.04 then p[2]=p[2]-1.2 elseif p[2]<-1.24 then p[2]=p[2]+1.2 end
                sim.setObjectPosition(h,sim.handle_world,p)
            end
        end
    end
end

function sysCall_sensing()
    local actual={}; local targets={}; local errors={}; local maxError=0
    for i,j in ipairs(c.joints) do
        actual[i]=deg(sim.getJointPosition(j)); targets[i]=deg(reference[i])
        errors[i]=settledErrors[i]; maxError=math.max(maxError,errors[i])
    end
    local health=maintenance and 'alarm' or (maxError>=1 and 'warning' or 'healthy')
    sim.setShapeColor(c.lamp,nil,sim.colorcomponent_ambient_diffuse,
        stopped and {1,.1,.1} or (health=='alarm' and {1,.1,.1} or (health=='warning' and {1,.65,.05} or {.1,.9,.4})))
    local data={schema=1,simulationTime=sim.getSimulationTime(),state=state,running=running,stopped=stopped,
        fault=fault,health=health,maintenanceRequired=maintenance,totalCycles=total,cyclesSinceMaintenance=sinceMaintenance,
        maxJointSpeed=.65,clearanceGuard=.015,minimumMachineClearance=minimumClearance,
        minimumEquipmentClearanceLowerBound=minimumClearance,
        wearStartsAfter=15,received=received,processed=processed,delivered=delivered,cycleTime=cycleTime,
        targetAngles=targets,actualAngles=actual,settledErrors=errors,warningLimit=1,alarmLimit=3,
        infeedRunning=infeed,outfeedRunning=outfeed,machineBusy=state=='processing',vacuum=carrying,
        sensors={infeed=sensorState(c.sensors[1]),machine=sensorState(c.sensors[2]),outfeed=sensorState(c.sensors[3])},
        events=events,speed=speed,motionActive=motion~=nil,wearModel='simulated joint offset; kinematic cell'}
    sim.setStringSignal('workcell.telemetry',json.encode(data))
end

function sysCall_cleanup()
    if robotGeometry then sim.destroyCollection(robotGeometry) end
    if machineGeometry then sim.destroyCollection(machineGeometry) end
    sim.clearFloatSignal('workcell.heartbeat')
    sim.clearStringSignal('workcell.telemetry')
    sim.clearStringSignal('workcell.command')
    sim.clearStringSignal('workcell.response')
end

