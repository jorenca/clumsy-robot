import React from 'react';

export default class Telemetry extends React.Component {
  state = {
    speed: 60
  };

  doMove = (left, right) => {
    const { speed } = this.state;
    fetch(`/move/${left}/${speed}/${right}/${speed}`);
  };

  render() {
    const move = this.doMove;
    return (
      <div>
        Movement

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
              <td><Control onClick={() => move(1, -1)} /></td>
              <td><Control onClick={() => move(0.2, -0.2)} /></td>
              <td style={{ backgroundColor: 'gray' }} />
              <td><Control onClick={() => move(-0.2, 0.2)}/></td>
              <td><Control onClick={() => move(-1, 1)}/></td>
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
              <td />
            </tr>
          </tbody>
        </table>
      </div>
    );
  }
}

const Control = ({ onClick }) =>
<button onClick={onClick} style={{ width: '50px', height: '50px' }}>
  X
</button>
