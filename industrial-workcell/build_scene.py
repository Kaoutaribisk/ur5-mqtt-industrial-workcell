"""Build the kinematic teaching workcell in a separate CoppeliaSim scene."""
import argparse
import json
import math
import random
from pathlib import Path
import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from camera_view import frame_cell


def build(output, port, replace=False):
    client = RemoteAPIClient(port=port)
    client.socket.setsockopt(zmq.RCVTIMEO, 10000)
    sim = client.require('sim')
    route_path=Path(__file__).with_name('clearance_route.json')
    route_data=json.loads(route_path.read_text(encoding='utf-8'))
    belt_x=route_data.get('beltX',.65)
    pickup_y=route_data.get('pickupY',-.15)
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop the simulation before building the workcell.')
    if sim.getObject('/Workcell', {'noError': True}) != -1:
        raise RuntimeError('A Workcell already exists. Open the original scene before rebuilding.')
    output = str(Path(output).resolve())
    if Path(output).exists() and not replace:
        raise FileExistsError(f'Refusing to overwrite existing scene: {output}')
    # Save the original under the new name before making any scene changes.
    sim.saveScene(output)
    robot = sim.getObject('/UR5')
    joints = sim.getObjectsInTree(robot, sim.object_joint_type, 0)
    if len(joints) != 6:
        raise RuntimeError('Expected exactly six UR5 joints.')
    sim.setModelProperty(robot, sim.getModelProperty(robot) | sim.modelproperty_not_dynamic)
    for h in sim.getObjectsInTree(robot,sim.object_script_type,0):
        sim.setObjectInt32Param(h,sim.scriptintparam_enabled,0)
    for joint in joints:
        sim.setJointMode(joint, sim.jointmode_kinematic)
    root = sim.createDummy(0.02)
    sim.setObjectAlias(root, 'Workcell')
    sim.setObjectPosition(robot, -1, [0, 0, 0.49])

    def shape(name, size, pos, color, parent=root, cylinder=False):
        handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder if cylinder else sim.primitiveshape_cuboid, size, 0)
        sim.setObjectAlias(handle, name)
        sim.setObjectPosition(handle, -1, pos)
        sim.setObjectParent(handle, parent, True)
        sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
        return handle

    navy, blue, metal = [0.08, .15, .24], [.12, .36, .65], [.5, .57, .64]
    shape('Cell_platform', [2.3, 2.5, .08], [0, -.1, .04], [.21, .25, .30])
    shape('Robot_pedestal', [.30, .30, .42], [0, 0, .28], metal)
    shape('Robot_mount', [.39, .39, .04], [0, 0, .47], navy)
    belt_markers = [[], []]
    for n, x in enumerate([-belt_x, belt_x]):
        prefix = 'Infeed' if n == 0 else 'Outfeed'
        shape(prefix + '_frame', [.36, 1.25, .12], [x, -.63, .64], metal)
        shape(prefix + '_belt', [.29, 1.20, .045], [x, -.63, .7225], navy)
        for dx in [-.18, .18]:
            shape(prefix + '_rail', [.025, 1.25, .055], [x + dx, -.63, .76], blue)
            for y in [-1.1, -.17]:
                shape(prefix + '_leg', [.04, .04, .58], [x + dx, y, .35], metal)
        for i in range(12):
            belt_markers[n].append(shape(prefix + f'_slat_{i:02}', [.28, .014, .003], [x, -1.20 + i*.10, .747], [.28, .39, .5]))
    shape('Machine_base', [.70, .56, .62], [0, .58, .40], blue)
    shape('Machine_bed', [.48, .39, .035], [0, .54, .7275], metal)
    for x in [-.32, .32]:
        shape('Machine_side', [.045, .58, .48], [x, .58, .95], navy)
    shape('Machine_back', [.67, .035, .48], [0, .86, .95], navy)
    shape('Machine_header', [.67, .06, .08], [0, .30, 1.2], blue)
    door = shape('Machine_gate', [.56, .025, .34], [0, .285, 1.40], [.35, .55, .72])
    lamp = shape('Machine_lamp', [.055, .055, .10], [.28, .84, 1.25], [.2, .9, .45], cylinder=True)
    part = shape('Product', [.075, .075, .075], [-belt_x, -1.15, .785], [1, .60, .12])
    connection = next(h for h in sim.getObjectsInTree(robot,sim.handle_all,0) if sim.getObjectAlias(h)=='connection')
    cup = shape('Vacuum_tool', [.065, .065, .10], [0,0,0], [.1,.14,.18], connection, True)
    sim.setObjectPosition(cup, connection, [0, 0, .05])
    sim.setObjectQuaternion(cup, connection, [0,0,0,1])
    tip = sim.createDummy(.012)
    sim.setObjectAlias(tip, 'Vacuum_tip')
    sim.setObjectParent(tip, connection, False)
    sim.setObjectPosition(tip, connection, [0,0,.10])
    target = sim.createDummy(.015)
    sim.setObjectAlias(target, 'IK_target')
    sim.setObjectParent(target, root, True)

    sensors = []
    for name, pos in [('Infeed_sensor', [-belt_x,pickup_y,1.0]), ('Machine_sensor',[0,.54,1.0]), ('Outfeed_sensor',[belt_x,pickup_y,1.0])]:
        floats = [0, .4, .12, .12, .12, .12, 0, .04, .04, .5, .5, 0, .01, 0, 0]
        sensor = sim.createProximitySensor(sim.proximitysensor_ray, 16, 5, [16,16,1,1,1,1,0,0], floats)
        sim.setObjectAlias(sensor, name)
        sim.setObjectParent(sensor, root, True)
        sim.setObjectPosition(sensor, -1, pos)
        sim.setObjectQuaternion(sensor, -1, [1,0,0,0])
        sensors.append(sensor)
        shape(name + '_housing', [.035,.035,.04], [pos[0],pos[1],pos[2]+.03], [.15,.65,.9])

    poses=route_data['poses']
    for name,pos in route_data.get('targets',{}).items():
        for joint,q in zip(joints,poses[name]):sim.setJointPosition(joint,q)
        if math.dist(sim.getObjectPosition(tip,-1),pos)>.003:raise RuntimeError('Calibrated robot geometry changed: '+name)
    sensor=next(h for h in sensors if sim.getObjectAlias(h)=='Machine_sensor')
    housing=sim.getObject('/Machine_sensor_housing')
    sim.setObjectPosition(housing,-1,[.27,.54,.93])
    z=[-.25,0,-.135]; length=math.sqrt(sum(x*x for x in z)); z=[x/length for x in z]
    x=[0,-1,0]; y=[z[2],0,-z[0]]
    sim.setObjectMatrix(sensor,-1,[v for i in range(3) for v in (x[i],y[i],z[i],[.25,.54,.92][i])])
    for prefix,center in [('Infeed',-belt_x),('Outfeed',belt_x)]:
        side=-1 if center<0 else 1
        pos=[center+side*.17,pickup_y,.90]
        z=[-side*.17,0,-.115]; length=math.sqrt(sum(v*v for v in z));z=[v/length for v in z]
        x=[0,side,0]; y=[-side*z[2],0,side*z[0]]
        sim.setObjectMatrix(sim.getObject('/'+prefix+'_sensor'),-1,[v for i in range(3) for v in (x[i],y[i],z[i],pos[i])])
        sim.setObjectPosition(sim.getObject('/'+prefix+'_sensor_housing'),-1,[center+side*.19,pickup_y,.92])
    for j,q in zip(joints,poses['home']): sim.setJointPosition(j,q)
    config = dict(robot=robot,joints=joints,root=root,tip=tip,part=part,door=door,lamp=lamp,
                  sensors=sensors,markers=belt_markers,poses=poses,routes=route_data.get('routes',{}),beltX=belt_x,pickupY=pickup_y)
    sim.writeCustomDataBlock(root, 'workcell.config', sim.packTable(config))
    script = sim.createScript(sim.scripttype_simulation, Path(__file__).with_name('workcell.lua').read_text(encoding='utf-8'), 0, 'lua')
    sim.setObjectAlias(script, 'Workcell_controller')
    sim.setObjectParent(script,root,False)
    # A wide, slightly elevated view of both conveyors and station.
    camera = sim.getObject('/DefaultCamera')
    frame_cell(sim)
    sim.setBoolParam(sim.boolparam_realtime_simulation,True)
    sim.setFloatParam(sim.floatparam_simulation_time_step,.02)
    sim.saveScene(output)
    Path(output).with_suffix('.config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    print('Saved:',output)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--port',type=int,default=23001)
    parser.add_argument('--replace-generated',action='store_true')
    args=parser.parse_args()
    build(args.output,args.port,args.replace_generated)
