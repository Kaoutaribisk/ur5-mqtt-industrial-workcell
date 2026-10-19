import { act, fireEvent, render, screen, within } from '@testing-library/react';
import WorkcellDashboard, { CELL_PREFIX, validTelemetry } from './WorkcellDashboard';

const sample = () => ({
  schema:1,state:'idle',targetAngles:[0,0,0,0,0,0],actualAngles:[0,0,0,0,0,0],settledErrors:[0,0,0,0,0,0],
  totalCycles:0,cyclesSinceMaintenance:0,received:0,processed:0,delivered:0,cycleTime:0,
  running:false,stopped:false,maintenanceRequired:false,motionActive:false,health:'healthy',events:[],
  sensors:{infeed:false,machine:false,outfeed:false},speed:1,
});
let client, listeners;
beforeEach(() => {
  jest.useFakeTimers(); listeners={};
  client={connected:true,on:jest.fn((name,callback) => { listeners[name]=callback; }),removeListener:jest.fn(),subscribe:jest.fn(),publish:jest.fn()};
});
afterEach(() => jest.useRealTimers());
function receive(name,data) { act(() => listeners.message(`${CELL_PREFIX}/${name}`,{toString:() => JSON.stringify(data)})); }
function live(data=sample()) {
  receive('status',{backendConnected:true,simulatorConnected:true,sceneReady:true,simulationRunning:true});
  receive('telemetry',data);
}
test('rejects malformed monitoring packets', () => {
  expect(validTelemetry(sample())).toBe(true);
  expect(validTelemetry({...sample(),actualAngles:[1]})).toBe(false);
  expect(validTelemetry({...sample(),settledErrors:[NaN,0,0,0,0,0]})).toBe(false);
});
test('only sends non-retained control with live backend status and waits for acknowledgement', () => {
  render(<WorkcellDashboard client={client} brokerOnline />);
  const start=screen.getByRole('button',{name:/Start auto/});
  expect(start).toBeDisabled(); live(); expect(start).toBeEnabled();
  fireEvent.click(start);
  const [topic,payload,options]=client.publish.mock.calls[0];
  expect(topic).toBe(`${CELL_PREFIX}/control`);
  expect(JSON.parse(payload).action).toBe('start');
  expect(options).toEqual({qos:0,retain:false});
  expect(start).toBeDisabled();
  receive('ack',{id:JSON.parse(payload).id,accepted:true});
  expect(screen.getByText('Command accepted')).toBeInTheDocument();
});
test('maintenance alarm blocks start and enables reset only at a completed cycle', () => {
  render(<WorkcellDashboard client={client} brokerOnline />);
  live({...sample(),state:'maintenance',health:'alarm',maintenanceRequired:true,settledErrors:[3.12,2,1,0,0,0],totalCycles:28});
  expect(screen.getByRole('alert')).toHaveTextContent('Maintenance required');
  expect(screen.getByRole('button',{name:/Start auto/})).toBeDisabled();
  expect(screen.getByRole('button',{name:/Complete maintenance/})).toBeEnabled();
  live({...sample(),state:'pick',running:false,vacuum:true});
  expect(screen.getByRole('button',{name:/Complete maintenance/})).toBeDisabled();
});
test('stale telemetry hides old counts and disables control when the bridge heartbeat expires', () => {
  render(<WorkcellDashboard client={client} brokerOnline />);
  live({...sample(),totalCycles:123,delivered:123});
  expect(within(screen.getByRole('group',{name:'Production counters'})).getAllByText('123')).toHaveLength(2);
  act(() => jest.advanceTimersByTime(3500));
  expect(within(screen.getByRole('group',{name:'Production counters'})).queryByText('123')).not.toBeInTheDocument();
  expect(screen.getByText('NO LIVE DATA')).toBeInTheDocument();
  expect(screen.getByRole('button',{name:/Start auto/})).toBeDisabled();
});
