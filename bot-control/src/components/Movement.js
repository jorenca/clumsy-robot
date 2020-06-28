import React, { useState } from 'react';

import GamepadControl from './GamepadControl.js';
import Planner from './Planner';
import './Movement.scss';
import gamepadIcon from './gamepad_icon.png';

export default function Movement({ sharedTelemetry }) {
  const [speed, setSpeed] = useState('150');

  const move = (left, right, overrideSpeed) => {
    const speedToUse = overrideSpeed || speed;
    return fetch(`/move/${left}/${right}/${speedToUse}`);
  };
  const cstop = () => fetch('/cstop');
  const cTurn = (angle) => fetch(`/motor_cmd/C ${angle} ;`);

  return (
    <div className="movement-container">
      Movement

      <Keypad move={move} cstop={cstop} />
      <SpeedSlider speed={speed} setSpeed={(e) => setSpeed(e.target.value)} />
    </div>
  );
}
// FIXME GEORGI implement move Planner
//       <Planner move={move} cTurn={cTurn} sharedTelemetry={sharedTelemetry} />


const SpeedSlider = ({ speed, setSpeed }) =>
<div className="speed-slider">
  <input type="range" min="120" max="600" value={speed} onChange={setSpeed} />
</div>;

const Keypad = ({ move, cstop }) => {
  const [upActive, setUpActive] = useState(false);
  const markUp = () => { setUpActive(true); setTimeout(setUpActive, 100, false); };
  const moveUp = (dist, overrideSpeed) => { markUp(); move(dist, dist, overrideSpeed); };

  const [downActive, setDownActive] = useState(false);
  const markDown = () => { setDownActive(true); setTimeout(setDownActive, 100, false); };
  const moveDown = (dist, overrideSpeed) => { markDown(); move(-dist, -dist, overrideSpeed); };

  const [leftActive, setLeftActive] = useState(false);
  const markLeft = () => { setLeftActive(true); setTimeout(setLeftActive, 100, false); };
  const moveLeft = (dist, overrideSpeed) => { markLeft(); move(-dist, dist, overrideSpeed); };

  const [rightActive, setRightActive] = useState(false);
  const markRight = () => { setRightActive(true); setTimeout(setRightActive, 100, false); };
  const moveRight = (dist, overrideSpeed) => { markRight(); move(dist, -dist, overrideSpeed); };

  const [hasGamepad, setHasGamepad] = useState(false);

  return (
    <div className="keypad">
      <GamepadControl
        moveUp={moveUp}
        moveDown={moveDown}
        moveLeft={moveLeft}
        moveRight={moveRight}
        setHasGamepad={setHasGamepad}
        stop={cstop}
        />

      <table>
        <tbody>
          <tr>
            <td />
            <td>
              <Control onClick={() => moveUp(3)} isActive={upActive}>🡅</Control>
            </td>
            <td>
              <img
                src={gamepadIcon}
                alt="Gamepad support"
                style={{
                  width: '45px',
                  height: '45px',
                  backgroundColor: hasGamepad ? '#2cd027' : 'transparent'
                }} />
              </td>
          </tr>
          <tr>
            <td><Control onClick={() => moveLeft(1)} isActive={leftActive}>&lt;</Control></td>
            <td>
              <Control onClick={() => cstop()} style={{ backgroundColor: 'red' }}>
                STOP
              </Control>
            </td>
            <td><Control onClick={() => moveRight(1)} isActive={rightActive}>&gt;</Control></td>
          </tr>
          <tr>
            <td />
            <td><Control onClick={() => moveDown(3)} isActive={downActive}>🡇</Control></td>
            <td/>
          </tr>
        </tbody>
      </table>
    </div>
  );
}

const Control = ({ onClick, children, style, isActive }) =>
<button
  onClick={onClick}
  style={{
    width: '50px',
    height: '50px',
    backgroundColor: isActive ? '#2cd027' : '#e5e5e5',
    ...style
  }}>
  {children || 'X'}
</button>
