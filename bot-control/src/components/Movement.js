import React, { useState } from 'react';
import GamepadControl from './GamepadControl.js';
import './Movement.scss';
import gamepadIcon from './gamepad_icon.png';

export default function Movement() {
  const [speed, setSpeed] = useState('60');

  const move = (left, right, overrideSpeed) => {
    const speedToUse = overrideSpeed || speed;
    fetch(`/move/${left}/${speedToUse}/${right}/${speedToUse}`);
  };
  const cstop = () => fetch('/cstop');

  return (
    <div className="movement-container">
      Movement
      <SpeedSlider speed={speed} setSpeed={(e) => setSpeed(e.target.value)} />
      <Keypad move={move} cstop={cstop} />
    </div>
  );
}

const SpeedSlider = ({ speed, setSpeed }) =>
<div className="speed-slider">
  <input type="range" min="10" max="99" value={speed} onChange={setSpeed} />
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
            <td />
            <td>
              <Control onClick={() => moveUp(1)} isActive={upActive}>🡅🡅</Control>
            </td>
            <td />
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
            <td />
            <td />
            <td><Control onClick={() => moveUp(0.2)}>🡅</Control></td>
            <td />
            <td />
          </tr>
          <tr>
            <td><Control onClick={() => moveLeft(1)} isActive={leftActive}>&lt;&lt;</Control></td>
            <td><Control onClick={() => moveLeft(0.2)}>&lt;</Control></td>
            <td style={{ backgroundColor: 'gray' }} />
            <td><Control onClick={() => moveRight(0.2)}>&gt;</Control></td>
            <td><Control onClick={() => moveRight(1)} isActive={rightActive}>&gt;&gt;</Control></td>
          </tr>
          <tr>
            <td />
            <td />
            <td><Control onClick={() => moveDown(0.2)}>🡇</Control></td>
            <td />
            <td />
          </tr>
          <tr>
            <td />
            <td />
            <td><Control onClick={() => moveDown(1)} isActive={downActive}>🡇🡇</Control></td>
            <td />
            <td>
              <Control onClick={() => cstop()} style={{ backgroundColor: 'red' }}>
                STOP
              </Control>
            </td>
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
