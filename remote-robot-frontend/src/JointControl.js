import React from 'react';

const names = ['Base rotation', 'Shoulder', 'Elbow', 'Wrist pitch', 'Wrist yaw', 'Tool rotation'];
export const directions = (id) => [0, 4, 5].includes(id) ? ['left', 'right'] : ['down', 'up'];

export default function JointControl({ jointId, angle, step, disabled, onCommand }) {
  const [negative, positive] = directions(jointId);
  return <article className="joint-card">
    <div className="card-heading"><span className="joint-number">0{jointId + 1}</span><span className="axis-label">AXIS J{jointId + 1}</span></div>
    <h3>{names[jointId]}</h3>
    <div className="angle">{angle === null ? '—' : angle.toFixed(1)}<span>°</span></div>
    <div className="angle-track"><span style={{ width: angle === null ? '0%' : `${Math.min(100, Math.abs(angle) / 360 * 100)}%` }} /></div>
    <div className="joint-actions">
      <button disabled={disabled} aria-label={`Joint ${jointId + 1} ${negative}`} onClick={() => onCommand(jointId, negative, step)}><span>−</span> {step}°</button>
      <button disabled={disabled} aria-label={`Joint ${jointId + 1} ${positive}`} onClick={() => onCommand(jointId, positive, step)}><span>+</span> {step}°</button>
    </div>
    <p className="card-caption">{angle === null ? 'Awaiting feedback' : 'Reported target angle'}</p>
  </article>;
}
