import React, { useEffect, useRef, useState } from 'react';
import './WorkcellDashboard.css';

export const CELL_PREFIX = 'kaoutar/workcell';
export function validTelemetry(data) {
  return data && data.schema === 1 && typeof data.state === 'string' &&
    ['targetAngles', 'actualAngles', 'settledErrors'].every(key => Array.isArray(data[key]) && data[key].length === 6 && data[key].every(Number.isFinite)) &&
    ['totalCycles', 'cyclesSinceMaintenance', 'received', 'processed', 'delivered', 'cycleTime'].every(key => Number.isFinite(data[key]) && data[key] >= 0) &&
    ['running', 'stopped', 'maintenanceRequired', 'motionActive'].every(key => typeof data[key] === 'boolean') &&
    ['healthy','warning','alarm'].includes(data.health) &&
    Array.isArray(data.events) && data.events.every(event => Number.isFinite(event.time) && typeof event.message === 'string');
}

const phase = value => value ? value.replace(/_/g, ' ') : 'Awaiting cell';
const angle = value => Number.isFinite(value) ? `${value.toFixed(2)}°` : '—';
const names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];

export default function WorkcellDashboard({ client, brokerOnline, onManualAvailability }) {
  const [data, setData] = useState(null);
  const [status, setStatus] = useState(null);
  const [lastData, setLastData] = useState(0);
  const [lastStatus, setLastStatus] = useState(0);
  const [clock, setClock] = useState(Date.now());
  const [ack, setAck] = useState('');
  const [pending, setPending] = useState(null);
  const [history, setHistory] = useState([]);
  const sequence = useRef(0);
  const pendingRef = useRef(null);
  const lastCycleRef = useRef(-1);
  const fresh = brokerOnline && lastData > 0 && clock - lastData < 3000 && status?.simulationRunning;
  const bridgeFresh = brokerOnline && lastStatus > 0 && clock - lastStatus < 3000 && status?.backendConnected;
  const canSend = bridgeFresh && status?.simulatorConnected && status?.sceneReady;
  const current = fresh ? data : null;

  useEffect(() => {
    if (!client) return;
    const subscribe = () => client.subscribe([`${CELL_PREFIX}/telemetry`, `${CELL_PREFIX}/status`, `${CELL_PREFIX}/ack`]);
    const receive = (topic, payload) => {
      let message;
      try { message = JSON.parse(payload.toString()); } catch { return; }
      const now = Date.now();
      if (topic === `${CELL_PREFIX}/telemetry` && validTelemetry(message)) {
        setData(message); setLastData(now); setClock(now);
        if (message.totalCycles !== lastCycleRef.current) {
          if (message.totalCycles < lastCycleRef.current) setHistory([]);
          lastCycleRef.current = message.totalCycles;
          const errors = message.settledErrors;
          setHistory(previous => [...previous, { cycle: message.totalCycles, error: Math.max(...errors) }].slice(-35));
        }
      } else if (topic === `${CELL_PREFIX}/status` && message && typeof message.backendConnected === 'boolean') {
        setStatus(message); setLastStatus(now); setClock(now);
      } else if (topic === `${CELL_PREFIX}/ack` && message?.id && message.id === pendingRef.current?.id) {
        setAck(message.accepted ? 'Command accepted' : `Rejected: ${message.reason || 'Unknown reason'}`);
        pendingRef.current = null; setPending(null);
      }
    };
    client.on('connect', subscribe); client.on('message', receive);
    if (client.connected) subscribe();
    return () => { client.removeListener('connect', subscribe); client.removeListener('message', receive); };
  }, [client]);

  useEffect(() => {
    const timer = setInterval(() => {
      const now = Date.now(); setClock(now);
      if (pendingRef.current && now - pendingRef.current.time > 7000) {
        pendingRef.current = null; setPending(null); setAck('No acknowledgement received. Check the backend.');
      }
    }, 500);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    if (onManualAvailability && data) onManualAvailability(Boolean(fresh && !data.running && !data.stopped && !data.maintenanceRequired && data.state === 'idle'));
  }, [fresh, data, onManualAvailability]);

  function send(action, value) {
    if (!client?.connected || !canSend) return;
    const id = `web-${Date.now()}-${++sequence.current}`;
    const command = { action, id, ...(value === undefined ? {} : { value }) };
    const waiting = { id, time: Date.now() };
    pendingRef.current = waiting; setPending(waiting); setAck('Waiting for controller acknowledgement…');
    client.publish(`${CELL_PREFIX}/control`, JSON.stringify(command), { qos: 0, retain: false }, error => {
      if (error && pendingRef.current?.id === id) {
        pendingRef.current = null; setPending(null); setAck('Command could not be published.');
      }
    });
  }

  const canStart = canSend && !pending && !current?.running && !current?.stopped && !current?.maintenanceRequired && !current?.fault;
  const resetAllowed = current && !current.running && !current.vacuum && ['idle', 'maintenance'].includes(current.state);
  const counts = [['Received', 'received'], ['Processed', 'processed'], ['Delivered', 'delivered'], ['Completed cycles', 'totalCycles']];
  const maxError = current ? Math.max(...current.settledErrors) : null;
  return <section className="cell-dashboard" aria-label="Production dashboard">
    <div className="cell-title"><div><h1>Production cell</h1><p>UR5 / Conveyor / Processing station</p></div><span className={`cell-badge ${current?.stopped ? 'alarm' : current?.health || 'offline'}`}>{!current ? 'NO LIVE DATA' : current.stopped ? 'STOPPED' : current.maintenanceRequired ? 'MAINTENANCE' : current.running ? 'RUNNING' : 'PAUSED'}</span></div>
    <div className="cell-connections">
      <span className={brokerOnline ? 'online' : ''}>MQTT <b>{brokerOnline ? 'Connected' : 'Offline'}</b></span>
      <span className={bridgeFresh ? 'online' : ''}>Backend <b>{bridgeFresh ? 'Connected' : 'No heartbeat'}</b></span>
      <span className={canSend ? 'online' : ''}>CoppeliaSim <b>{!bridgeFresh ? 'Unknown' : !status?.simulatorConnected ? 'Disconnected' : !status?.sceneReady ? 'Open production_cell.ttt' : status.simulationRunning ? 'Simulation running' : 'Simulation stopped'}</b></span>
    </div>
    <div className="cell-counters" role="group" aria-label="Production counters">{counts.map(([label,key]) => <article key={key}><span>{label}</span><strong>{current ? current[key] : '—'}</strong><small>{key === 'totalCycles' ? 'Since simulation start' : 'pieces'}</small></article>)}</div>
    <div className="cell-toolbar">
      <button className="cell-start" disabled={!canStart} onClick={() => send('start')}>▶ Start auto</button>
      <button disabled={!canSend || !current?.running || Boolean(pending)} onClick={() => send('pause')}>Ⅱ Pause</button>
      <button className="cell-stop" disabled={!canSend || current?.stopped} onClick={() => send('stop')}>■ Stop cell</button>
      {data?.stopped && <button disabled={!canSend || Boolean(pending)} onClick={() => send('reset_stop')}>Reset stop latch</button>}
      <label>Demo speed <select aria-label="Demo speed" value={data?.speed || 1} disabled={!canSend || Boolean(pending)} onChange={event => send('speed',Number(event.target.value))}><option value={1}>1×</option><option value={4}>4×</option><option value={8}>8×</option></select></label>
      <span className="cell-command-status" role="status">{ack}</span>
    </div>
    {!current && <p className="cell-empty">{bridgeFresh && status?.sceneReady && !status.simulationRunning ? 'Ready. Start auto initializes the simulation and begins production.' : 'Waiting for workcell telemetry. Run the production backend with production_cell.ttt open.'}</p>}
    {current?.fault && <div className="cell-alarm" role="alert">Process fault: {current.fault}</div>}
    {current?.maintenanceRequired && <div className="cell-alarm" role="alert">Maintenance required — settled joint error exceeded 3°. {current.running ? 'Current cycle will finish; the next cycle is blocked.' : 'Production is blocked until maintenance reset.'}</div>}
    {current?.health === 'warning' && !current.maintenanceRequired && <div className="cell-warning" role="status">Joint wear warning — settled position error exceeds 1°.</div>}
    <div className="cell-layout"><div>
      <section className="cell-panel"><div className="cell-panel-heading"><h2>Line status</h2><span className="cell-phase">{phase(current?.state)}</span></div>
        <div className="cell-flow">{[
          ['Infeed', current?.infeedRunning ? 'Moving' : 'Idle', current?.sensors?.infeed],
          ['UR5', current?.vacuum ? 'Holding piece' : current?.motionActive ? 'Moving' : 'Idle', current?.vacuum],
          ['Machine', current?.machineBusy ? 'Processing' : 'Idle', current?.sensors?.machine],
          ['Outfeed', current?.outfeedRunning ? 'Moving' : 'Idle', current?.sensors?.outfeed],
        ].map(([name,state,detected]) => <div key={name} className={current && detected ? 'detected' : ''}><strong>{name}</strong><span>{current ? state : '—'}</span><small>{current ? detected ? '● Product detected' : '○ Clear' : '○ No data'}</small></div>)}</div>
        <div className="cell-stats"><span>Last cycle <strong>{current?.cycleTime > 0 ? `${current.cycleTime.toFixed(1)} s` : '—'}</strong></span><span>Since maintenance <strong>{current ? current.cyclesSinceMaintenance : '—'} cycles</strong></span><span>Max settled error <strong>{angle(maxError)}</strong></span></div>
      </section>
      <section className="cell-panel"><div className="cell-panel-heading"><h2>Joint position monitoring</h2><span className="cell-phase">WARNING 1° / ALARM 3°</span></div>
        <div className="cell-table-scroll"><table className="cell-joints"><thead><tr><th>Joint</th><th>Target</th><th>Measured</th><th>Settled error</th><th>Condition</th></tr></thead><tbody>{names.map((name,i) => {
          const error = current?.settledErrors[i]; const health = error >= 3 ? 'alarm' : error >= 1 ? 'warning' : 'healthy';
          return <tr key={name}><th><b>J{i+1}</b> {name}</th><td>{angle(current?.targetAngles[i])}</td><td>{angle(current?.actualAngles[i])}</td><td>{angle(error)}</td><td><span className={`joint-health ${current ? health : 'offline'}`}>{!current ? 'No data' : error >= 3 ? 'Maintenance' : error >= 1 ? 'Warning' : 'Normal'}</span></td></tr>;
        })}</tbody></table></div><p className="cell-note">Measured positions come from simulated joints. Error is checked at completed moves; motion transients are excluded.</p>
      </section>
    </div><aside>
      <section className="cell-panel"><div className="cell-panel-heading"><h2>Wear trend</h2><span className="cell-phase">SIMULATED</span></div>
        <svg className="wear-chart" viewBox="0 0 300 155" role="img" aria-label="Maximum settled joint error by completed cycle">
          {[0,1,2,3].map(value => <g key={value}><line x1="30" y1={130-value*33} x2="284" y2={130-value*33} stroke={value === 3 ? '#e28686' : '#d8e3f1'} strokeDasharray={value === 3 ? '4 3' : '0'}/><text x="3" y={134-value*33}>{value}°</text></g>)}
          {fresh && history.length > 0 && <polyline fill="none" stroke="#2379da" strokeWidth="2.5" points={history.map((point,i) => `${30 + i/Math.max(1,history.length-1)*254},${130-Math.min(3.6,point.error)*33}`).join(' ')} />}
          <text x="30" y="151">{history.length ? `Cycle ${history[0].cycle}` : 'No completed cycles'}</text><text x="220" y="151">{history.length ? history[history.length-1].cycle : ''}</text>
        </svg>
        <p className="cell-note">Wear begins on cycle 16. Worst joint increases by 0.24° per cycle. Thresholds are demonstration values.</p>
        <button className="maintenance-reset" disabled={!canSend || !resetAllowed || Boolean(pending)} onClick={() => send('maintenance')}>Complete maintenance / reset wear</button>
        <p className="cell-note">Available at a completed cycle with the cell stopped. Production totals are preserved.</p>
      </section>
      <section className="cell-panel"><div className="cell-panel-heading"><h2>Event log</h2><span className="cell-phase">LAST 5</span></div><ol className="cell-events">{current?.events?.length ? current.events.slice(0,5).map(event => <li key={event.id}><time>{event.time.toFixed(1)} s</time><span>{event.message}</span></li>) : <li>No live events</li>}</ol></section>
    </aside></div>
    <p className="cell-note">Robot motion is capped at 37°/s. Demo speed accelerates conveyors and processing only. Counters reset when the simulation restarts.</p>
  </section>;
}
