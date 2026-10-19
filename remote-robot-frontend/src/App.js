import React, { useEffect, useRef, useState } from 'react';
import MQTTClient, { COMMAND_TOPIC, STATE_TOPIC } from './MQTTClient';
import JointControl, { directions } from './JointControl';
import './App.css';
import WorkcellDashboard from './WorkcellDashboard';

function Status({ name, value, active }) {
  return <div className="status-item"><span className={`status-dot ${active ? 'active' : ''}`} /><span>{name}</span><strong>{value}</strong></div>;
}

export default function App() {
  const [view, setView] = useState('production');
  const [cellClient, setCellClient] = useState(null);
  const [cellManualAllowed, setCellManualAllowed] = useState(true);
  const clientRef = useRef(null);
  const stoppedRef = useRef(false);
  const feedbackRef = useRef(0);
  const previousAngles = useRef(null);
  const changeId = useRef(0);
  const [changes, setChanges] = useState([]);
  const [connection, setConnection] = useState('Connecting');
  const [angles, setAngles] = useState(Array(6).fill(null));
  const [lastFeedback, setLastFeedback] = useState(0);
  const [now, setNow] = useState(Date.now());
  const [stopped, setStopped] = useState(false);
  const [step, setStep] = useState(10);
  const [received, setReceived] = useState(0);
  const [sent, setSent] = useState(0);
  const [notice, setNotice] = useState('Waiting for the robot’s first feedback.');
  const online = connection === 'Connected';
  const fresh = online && lastFeedback > 0 && now - lastFeedback < 3000;
  const ready = fresh && !stopped && cellManualAllowed;

  useEffect(() => {
    const client = MQTTClient();
    clientRef.current = client;
    setCellClient(client);
    const connected = () => setConnection('Connected');
    const reconnect = () => setConnection('Reconnecting');
    const offline = () => { feedbackRef.current = 0; setConnection('Disconnected'); };
    const error = () => { feedbackRef.current = 0; setConnection('Connection error'); };
    const message = (topic, payload) => {
      if (topic !== STATE_TOPIC) return;
      const parts = payload.toString().split(',');
      const next = Array(6).fill(null);
      for (const part of parts) {
        const match = part.match(/^([0-5]):(-?\d+(?:\.\d+)?)$/);
        if (!match || next[Number(match[1])] !== null) return;
        const angle = Number(match[2]);
        if (!Number.isFinite(angle)) return;
        next[Number(match[1])] = angle;
      }
      if (next.some(angle => angle === null)) return;
      const time = Date.now();
      // Only record changed targets; the backend repeats unchanged states at 10 Hz.
      const previous = previousAngles.current;
      if (previous) {
        const updates = next.flatMap((angle, id) => angle === previous[id] ? [] : [{
          id: ++changeId.current, joint: id + 1, before: previous[id], after: angle,
          time: new Date(time).toLocaleTimeString('en-GB', { hour12: false }),
        }]);
        if (updates.length) setChanges(history => [...updates.reverse(), ...history].slice(0, 5));
      }
      previousAngles.current = next;
      feedbackRef.current = time;
      setAngles(next); setLastFeedback(time); setNow(time);
      setReceived(count => count + 1);
    };
    client.on('connect', connected); client.on('reconnect', reconnect);
    client.on('offline', offline); client.on('close', offline);
    client.on('error', error); client.on('message', message);
    if (client.connected) connected();
    const timer = setInterval(() => setNow(Date.now()), 500);
    return () => {
      clearInterval(timer);
      client.removeListener('connect', connected); client.removeListener('reconnect', reconnect);
      client.removeListener('offline', offline); client.removeListener('close', offline);
      client.removeListener('error', error); client.removeListener('message', message);
      client.end(true); clientRef.current = null;
    };
  }, []);

  function command(id, direction, value) {
    const client = clientRef.current;
    if (stoppedRef.current || !client?.connected || !feedbackRef.current || Date.now() - feedbackRef.current >= 3000) return;
    client.publish(COMMAND_TOPIC, `${id}:${direction}:${value}`, { qos: 0, retain: false }, err => {
      if (err) setNotice('Command could not be sent. Check the broker connection.');
    });
    setSent(count => count + 1);
    setNotice(`Sent J${id + 1} ${direction} · ${value}°. Awaiting robot feedback.`);
  }

  function home() {
    if (!ready) return;
    angles.forEach((angle, id) => {
      if (angle !== 0 && angle !== null) command(id, directions(id)[angle > 0 ? 0 : 1], Math.abs(angle));
    });
    setNotice('Zero target requested for all six joints.');
  }

  function stop() {
    stoppedRef.current = true; setStopped(true);
    setNotice('Dashboard commands locked. Existing robot motion may continue; stop the simulation in CoppeliaSim.');
  }

  return <div className="dashboard">
    <aside className="sidebar">
      <a className="brand" href="#workspace"><span className="brand-mark">K<span>↗</span></span><span>KAOUTAR<span className="brand-sub">ROBOTICS LAB</span></span></a>
      <div className="sidebar-label">WORKSPACE</div>
      <a className="nav-item" href="#workspace"><span>⌘</span> Production cell <span className="nav-dot" /></a>
      <div className="sidebar-bottom"><span className="avatar">K</span><div>Kaoutar</div></div>
    </aside>
    <main id="workspace">
      <header className="topbar"><span>WORKSPACE <span className="slash">/</span> UR5 CONTROL</span><span className="session"><span className={`status-dot ${fresh ? 'active' : ''}`} />{fresh ? 'Feedback live' : 'Awaiting feedback'}</span></header>
      <nav className="workspace-tabs" aria-label="Dashboard view"><button className={view === 'production' ? 'selected' : ''} onClick={() => setView('production')}>Production</button><button className={view === 'manual' ? 'selected' : ''} onClick={() => setView('manual')}>Manual joints</button></nav>
      <div hidden={view !== 'production'}><WorkcellDashboard client={cellClient} brokerOnline={online} onManualAvailability={setCellManualAllowed} /></div>
      <div hidden={view !== 'manual'}>
      <section className="intro"><h1>UR5 Control</h1><span className="robot-tag">UR5 <span>6 DOF</span></span></section>
      <section className="connection-strip" aria-label="Connection status">
        <Status name="MQTT broker" value={connection} active={online} />
        <Status name="Python backend" value={fresh ? 'Feedback active' : 'No recent feedback'} active={fresh} />
        <Status name="CoppeliaSim" value="Health unverified" active={false} />
      </section>
      <div className="workspace-grid"><section className="controls">
        <div className="section-heading"><div><p className="eyebrow">MANUAL CONTROL</p><h2>Joint workspace</h2></div><label className="step-select">Step size <select value={step} onChange={e => setStep(Number(e.target.value))}><option value={1}>1° · Fine</option><option value={5}>5° · Precise</option><option value={10}>10° · Standard</option></select></label></div>
        <div className="joint-grid">{angles.map((angle, id) => <JointControl key={id} jointId={id} angle={angle} step={step} disabled={!ready} onCommand={command} />)}</div>
        <section className="feedback-panel" aria-label="Recent joint changes">
          <div className="section-heading"><h2>Feedback</h2><span className="feedback-count">LAST 5 CHANGES</span></div>
          {changes.length ? <ol className="feedback-list">{changes.map(change => <li key={change.id}>
            <time>{change.time}</time><strong>J{change.joint}</strong><span>{change.before.toFixed(1)}° → {change.after.toFixed(1)}°</span>
          </li>)}</ol> : <p className="feedback-empty">No joint changes yet.</p>}
          <p className="activity" role="status">{notice}</p>
        </section>
      </section><aside className="right-panel">
        <section className="robot-panel"><div className="section-heading"><h2>Robot</h2><span className="model-pill">UR5</span></div>
          <svg className="robot-art" viewBox="0 0 300 220" role="img" aria-label="Stylized UR5 robotic arm"><defs><pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse"><path d="M24 0H0V24" fill="none" stroke="#d2e1f3" strokeWidth="1" /></pattern></defs><rect width="300" height="220" fill="url(#grid)"/><ellipse cx="149" cy="193" rx="83" ry="12" fill="#c5d7ee"/><path d="M115 188h56l-7-23h-42z" fill="#476b99"/><path d="M143 168v-48l-48-44 64-39 54 49" fill="none" stroke="#7fa2cd" strokeWidth="21" strokeLinejoin="round"/><path d="M143 165v-45L96 76l64-39 53 49" fill="none" stroke="#c7dbf4" strokeWidth="10" strokeLinejoin="round"/>{[[143,120],[96,76],[160,37],[213,86]].map(([x,y],i)=><g key={i}><circle cx={x} cy={y} r="14" fill="#1d4e89"/><circle cx={x} cy={y} r="7" fill="#9dc4f4"/></g>)}<path d="M212 98v22m-9-1v13m18-13v13" fill="none" stroke="#1d4e89" strokeWidth="7"/><circle cx="143" cy="166" r="13" fill="#1d4e89"/></svg>
          <div className="robot-caption"><span>Universal Robots<small>Illustration · view in CoppeliaSim</small></span><strong>6 axes</strong></div>
        </section>
        <section className="telemetry"><h2>System status</h2><dl><div><dt>Control mode</dt><dd>{stopped ? 'Locked' : ready ? 'Manual' : 'Waiting'}</dd></div><div><dt>Feedback age</dt><dd>{lastFeedback ? `${Math.max(0, (now-lastFeedback)/1000).toFixed(1)} s` : '—'}</dd></div><div><dt>Messages received</dt><dd>{received}</dd></div><div><dt>Commands sent</dt><dd>{sent}</dd></div></dl><p className="telemetry-note">Target angles only; actual positions are not reported.</p></section>
        <section className="safety-panel"><button className="home-button" disabled={!ready} onClick={home}>⌂ <span>Return home<small>Set all joint targets to 0°</small></span>↗</button><button className="stop-button" disabled={stopped} onClick={stop}><span className="stop-symbol">■</span>{stopped ? 'Controls locked' : 'Emergency stop'}</button><p>Locks this dashboard only. To halt robot motion, stop CoppeliaSim.</p>{stopped && <button className="resume-button" onClick={() => { stoppedRef.current = false; setStopped(false); setNotice('Dashboard unlocked. Controls require fresh feedback.'); }}>Unlock controls</button>}</section>
      </aside></div>
      </div>
    </main>
  </div>;
}

