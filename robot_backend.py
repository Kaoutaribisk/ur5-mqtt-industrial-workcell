import time
import math
import threading
import paho.mqtt.client as mqtt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from mqtt_config import MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_COMMAND, MQTT_TOPIC_STATE


client = RemoteAPIClient()
sim = client.require('sim')
print("Connected to CoppeliaSim!\n")


ur5 = sim.getObject('/UR5')
joints = sim.getObjectsInTree(ur5, sim.object_joint_type)
joint_count = len(joints)
joint_angles = [0.0 for _ in range(joint_count)]
step = math.radians(10) 
print(f" Found {joint_count} joints\n")


def on_connect(client, userdata, flags, rc):
    print(" Connected to MQTT broker")
    client.subscribe(MQTT_TOPIC_COMMAND)

def on_message(client, userdata, msg):
    command = msg.payload.decode()
 
    try:
        joint_id, direction, value = command.split(":")
        joint_id = int(joint_id)
        value = float(value)
        if 0 <= joint_id < joint_count:
            if direction == "up" or direction == "right":
                joint_angles[joint_id] += math.radians(value)
            elif direction == "down" or direction == "left":
                joint_angles[joint_id] -= math.radians(value)
            sim.setJointTargetPosition(joints[joint_id], joint_angles[joint_id])
            print(f"Joint {joint_id} moved to {math.degrees(joint_angles[joint_id]):.1f}°")
    except Exception as e:
        print(" Invalid command:", command, e)

client_mqtt = mqtt.Client()
client_mqtt.on_connect = on_connect
client_mqtt.on_message = on_message
client_mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)


def publish_joint_states():
    while True:
        state_str = ",".join([f"{i}:{math.degrees(joint_angles[i]):.1f}" for i in range(joint_count)])
        client_mqtt.publish(MQTT_TOPIC_STATE, state_str)
        time.sleep(0.1) 


threading.Thread(target=publish_joint_states, daemon=True).start()
client_mqtt.loop_forever()

