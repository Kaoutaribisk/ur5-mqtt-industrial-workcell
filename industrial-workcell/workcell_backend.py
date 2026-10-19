"""MQTT bridge for production_cell.ttt. Run instead of robot_backend.py."""
import argparse
import json
import math
import queue
import time
import uuid

import paho.mqtt.client as mqtt
import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

PREFIX='kaoutar/workcell'
CONTROL=PREFIX+'/control'
TELEMETRY=PREFIX+'/telemetry'
STATUS=PREFIX+'/status'
ACK=PREFIX+'/ack'
LEGACY_COMMAND='robot/joint_command'
LEGACY_STATE='robot/joint_state'
ACTIONS={'start','pause','stop','reset_stop','maintenance','speed'}


def decode_command(topic, payload):
    if topic==LEGACY_COMMAND:
        joint,direction,value=payload.decode('utf-8').split(':')
        joint=int(joint); value=float(value)
        if joint not in range(6) or direction not in {'left','right','up','down'} or not math.isfinite(value) or not 0<=value<=360:
            raise ValueError('Invalid manual joint command')
        return dict(action='manual',joint=joint,direction=direction,value=value,id=str(uuid.uuid4()))
    cmd=json.loads(payload)
    if not isinstance(cmd,dict) or cmd.get('action') not in ACTIONS:
        raise ValueError('Unknown workcell action')
    cmd={key:cmd[key] for key in ('action','id','value') if key in cmd}
    if not isinstance(cmd.get('id',''),str) or len(cmd.get('id',''))>128:
        raise ValueError('Invalid command id')
    cmd.setdefault('id',str(uuid.uuid4()))
    if cmd['action']=='speed':
        value=cmd.get('value')
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .25<=value<=8:
            raise ValueError('Speed must be between 0.25 and 8')
    return cmd


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=23001)
    parser.add_argument('--broker',default='broker.hivemq.com')
    args=parser.parse_args()
    commands=queue.Queue(maxsize=32)
    connected=False
    api=None; sim=None; pending=None; pending_at=0; disconnect_handled=False
    bridge=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,client_id='kaoutar-cell-'+uuid.uuid4().hex[:8])
    bridge.will_set(STATUS,json.dumps(dict(backendConnected=False,simulatorConnected=False)),qos=1,retain=True)

    def publish(topic,data,retain=False):
        bridge.publish(topic,json.dumps(data,allow_nan=False),qos=0,retain=retain)

    def on_connect(client,userdata,flags,reason,properties):
        nonlocal connected,disconnect_handled
        connected=not reason.is_failure
        disconnect_handled=False
        if connected:
            client.subscribe([(CONTROL,0),(LEGACY_COMMAND,0)])
            print('MQTT connected; production bridge active',flush=True)

    def on_disconnect(client,userdata,flags,reason,properties):
        nonlocal connected
        connected=False
        # Never execute queued movement after a reconnect.
        while not commands.empty():
            try: commands.get_nowait()
            except queue.Empty: break

    def on_message(client,userdata,message):
        if message.retain: return
        try:
            command=decode_command(message.topic,message.payload)
            commands.put_nowait(command)
        except (ValueError,UnicodeError,TypeError,queue.Full) as error:
            publish(ACK,dict(accepted=False,reason=str(error),id=''))

    bridge.on_connect=on_connect; bridge.on_disconnect=on_disconnect; bridge.on_message=on_message
    bridge.reconnect_delay_set(1,10)
    bridge.connect_async(args.broker,1883,keepalive=10)
    bridge.loop_start()
    print('Connecting to CoppeliaSim on port',args.port,flush=True)
    try:
        while True:
            try:
                if sim is None:
                    api=RemoteAPIClient(port=args.port)
                    api.socket.setsockopt(zmq.RCVTIMEO,2000)
                    api.socket.setsockopt(zmq.LINGER,0)
                    sim=api.require('sim')
                    print('CoppeliaSim connected',flush=True)
                exists=sim.getObject('/Workcell',{'noError':True})!=-1
                state=sim.getSimulationState()
                active=state not in (sim.simulation_stopped,sim.simulation_paused)
                if not connected:
                    if active and exists and not disconnect_handled:
                        sim.setStringSignal('workcell.command',json.dumps(dict(action='pause',id='broker-loss')))
                        disconnect_handled=True
                    time.sleep(.1)
                    continue
                sim.setFloatSignal('workcell.heartbeat',sim.getSystemTime())
                status=dict(backendConnected=True,simulatorConnected=True,sceneReady=exists,simulationRunning=active)
                publish(STATUS,status,retain=True)
                raw=sim.getStringSignal('workcell.telemetry') if active and exists else None
                if raw:
                    data=json.loads(raw)
                    data.update(status)
                    data['timestamp']=time.time()
                    publish(TELEMETRY,data)
                    bridge.publish(LEGACY_STATE,','.join(f'{i}:{angle:.1f}' for i,angle in enumerate(data['targetAngles'])),retain=False)
                if pending:
                    response=sim.getStringSignal('workcell.response')
                    response=json.loads(response) if response else {}
                    if response.get('id')==pending['id']:
                        publish(ACK,response); pending=None
                    elif time.monotonic()-pending_at>4:
                        publish(ACK,dict(id=pending['id'],accepted=False,reason='Simulator command acknowledgement timed out'))
                        pending=None
                if not pending:
                    try: cmd=commands.get_nowait()
                    except queue.Empty: cmd=None
                    if cmd:
                        if not exists:
                            publish(ACK,dict(id=cmd['id'],accepted=False,reason='Open production_cell.ttt first'))
                        elif cmd['action']=='start' and state==sim.simulation_stopped:
                            # Initialize the scene first; send Start only after init has cleared old signals.
                            sim.startSimulation()
                            deadline=time.monotonic()+3
                            while not sim.getStringSignal('workcell.telemetry') and time.monotonic()<deadline:
                                time.sleep(.05)
                            sim.setFloatSignal('workcell.heartbeat',sim.getSystemTime())
                            sim.setStringSignal('workcell.command',json.dumps(cmd)); pending=cmd; pending_at=time.monotonic()
                        elif active:
                            sim.setStringSignal('workcell.command',json.dumps(cmd)); pending=cmd; pending_at=time.monotonic()
                        else:
                            publish(ACK,dict(id=cmd['id'],accepted=False,reason='Simulation is stopped or suspended; press Start in CoppeliaSim'))
                time.sleep(.1)
            except (Exception,) as error:
                if connected: publish(STATUS,dict(backendConnected=True,simulatorConnected=False,error=str(error)),retain=True)
                print('Simulator unavailable:',error,flush=True)
                if pending:
                    publish(ACK,dict(id=pending['id'],accepted=False,reason='Simulator connection lost')); pending=None
                sim=None
                if api:
                    api.socket.close(linger=0); api.context.term(); api=None
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if sim:
            try: sim.setStringSignal('workcell.command',json.dumps(dict(action='pause',id='bridge-exit')))
            except Exception: pass
        publish(STATUS,dict(backendConnected=False,simulatorConnected=False),retain=True)
        bridge.disconnect(); bridge.loop_stop()


if __name__=='__main__': main()
