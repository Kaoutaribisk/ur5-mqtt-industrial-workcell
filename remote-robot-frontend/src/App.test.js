import { act, fireEvent, render, screen } from '@testing-library/react';
import App from './App';
import MQTTClient from './MQTTClient';

jest.mock('./MQTTClient', () => ({
  __esModule: true,
  default: jest.fn(),
  COMMAND_TOPIC: 'robot/joint_command',
  STATE_TOPIC: 'robot/joint_state',
}));

let client;
let handlers;
beforeEach(() => {
  jest.useFakeTimers();
  handlers = {};
  client = {
    connected: true,
    on: jest.fn((event, callback) => { (handlers[event] ||= []).push(callback); }),
    subscribe: jest.fn(), publish: jest.fn(), removeListener: jest.fn(), end: jest.fn(),
  };
  MQTTClient.mockReturnValue(client);
});
afterEach(() => jest.useRealTimers());
function feedback(value = '0:20,1:-10,2:0,3:0,4:0,5:0') {
  act(() => handlers.message.forEach(callback => callback('robot/joint_state', { toString: () => value })));
}
test('feedback keeps the five most recent actual target changes and ignores repeated states', () => {
  renderManual();
  feedback('0:0,1:0,2:0,3:0,4:0,5:0');
  for (let angle = 10; angle <= 60; angle += 10) feedback(`0:${angle},1:0,2:0,3:0,4:0,5:0`);
  feedback('0:60,1:0,2:0,3:0,4:0,5:0');
  expect(screen.getAllByRole('listitem')).toHaveLength(5);
  expect(screen.getAllByRole('listitem')[0]).toHaveTextContent('50.0° → 60.0°');
  expect(screen.queryByText('0.0° → 10.0°')).not.toBeInTheDocument();
});
test('shows personalized branding and requires complete feedback before enabling movement', () => {
  renderManual();
  expect(screen.getByRole('heading', { name: 'UR5 Control' })).toBeInTheDocument();
  expect(screen.getByText('Health unverified')).toBeInTheDocument();
  const right = screen.getByRole('button', { name: 'Joint 1 right' });
  expect(right).toBeDisabled();
  feedback('0:20,1:bad');
  expect(right).toBeDisabled();
  feedback();
  expect(right).toBeEnabled();
  expect(screen.getByText('20.0')).toBeInTheDocument();
  fireEvent.click(right);
  expect(client.publish).toHaveBeenCalledWith('robot/joint_command', '0:right:10', { qos: 0, retain: false }, expect.any(Function));
});
test('home preserves the existing incremental format for positive and negative angles', () => {
  renderManual(); feedback();
  fireEvent.click(screen.getByRole('button', { name: /Return home/ }));
  expect(client.publish.mock.calls.map(call => call[1])).toEqual(['0:left:20', '1:up:10']);
});
test('stop locks controls without publishing unsupported commands and survives reconnect', () => {
  renderManual(); feedback();
  fireEvent.click(screen.getByRole('button', { name: /Emergency stop/ }));
  expect(client.publish).not.toHaveBeenCalled();
  act(() => handlers.connect.forEach(callback => callback())); feedback();
  expect(screen.getByRole('button', { name: 'Joint 1 right' })).toBeDisabled();
  expect(screen.getByRole('button', { name: /Return home/ })).toBeDisabled();
  fireEvent.click(screen.getByRole('button', { name: /Unlock controls/ }));
  expect(screen.getByRole('button', { name: 'Joint 1 right' })).toBeEnabled();
});
test('stale feedback and connection loss disable movement and cleanup closes the client', () => {
  const { unmount } = renderManual(); feedback();
  act(() => jest.advanceTimersByTime(3500));
  expect(screen.getByRole('button', { name: 'Joint 1 right' })).toBeDisabled();
  feedback();
  act(() => { client.connected = false; handlers.close.forEach(callback => callback()); });
  expect(screen.getByRole('button', { name: 'Joint 1 right' })).toBeDisabled();
  unmount();
  expect(client.end).toHaveBeenCalledWith(true);
});


function renderManual() {
  const result = render(<App />);
  fireEvent.click(screen.getByRole('button', { name: 'Manual joints' }));
  return result;
}

