import React from 'react';
import './Movement.scss';

export default class Telemetry extends React.Component {
  state = {
    speed: '60'
  };

  move = (left, right) => {
    const { speed } = this.state;
    fetch(`/move/${left}/${speed}/${right}/${speed}`);
  };

  setSpeed = e => this.setState({ speed: e.target.value });

  cstop = () => fetch('/cstop');

  render() {
    const { speed } = this.state;
    const { move, setSpeed, cstop } = this;
    return (
      <div className="movement-container">
        Movement

        <SpeedSlider speed={speed} setSpeed={setSpeed} />
        <Keypad move={move} cstop={cstop} />
      </div>
    );
  }
}

const SpeedSlider = ({ speed, setSpeed }) =>
<div className="speed-slider">
  <input type="range" min="10" max="99" value={speed} onChange={setSpeed} />
</div>;

const Keypad = ({ move, cstop }) =>
<div className="keypad">
  <table>
    <tbody>
      <tr>
        <td />
        <td />
        <td><Control onClick={() => move(1, 1)}/></td>
        <td />
        <td />
      </tr>
      <tr>
        <td />
        <td />
        <td><Control onClick={() => move(0.2, 0.2)}/></td>
        <td />
        <td />
      </tr>
      <tr>
        <td><Control onClick={() => move(-1, 1)} /></td>
        <td><Control onClick={() => move(-0.2, 0.2)} /></td>
        <td style={{ backgroundColor: 'gray' }} />
        <td><Control onClick={() => move(0.2, -0.2)}/></td>
        <td><Control onClick={() => move(1, -1)}/></td>
      </tr>
      <tr>
        <td />
        <td />
        <td><Control onClick={() => move(-0.2, -0.2)}/></td>
        <td />
        <td />
      </tr>
      <tr>
        <td />
        <td />
        <td><Control onClick={() => move(-1, -1)}/></td>
        <td />
        <td>
          <Control onClick={() => cstop()} style={{ backgroundColor: 'red' }}>
            STOP
          </Control>
        </td>
      </tr>
    </tbody>
  </table>
</div>;

const Control = ({ onClick, children, style }) =>
<button onClick={onClick} style={{ width: '50px', height: '50px', ...style }}>
  {children || 'X'}
</button>
