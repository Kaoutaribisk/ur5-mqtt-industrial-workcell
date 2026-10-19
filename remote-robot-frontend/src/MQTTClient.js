import mqtt from 'mqtt';

export const COMMAND_TOPIC = 'robot/joint_command';
export const STATE_TOPIC = 'robot/joint_state';

export default function MQTTClient() {
  const client = mqtt.connect('ws://broker.hivemq.com:8000/mqtt', {
    reconnectPeriod: 2000,
    clean: true,
    // Never replay movement commands after a connection loss.
    queueQoSZero: false,
  });
  client.on('connect', () => client.subscribe(STATE_TOPIC));
  return client;
}
