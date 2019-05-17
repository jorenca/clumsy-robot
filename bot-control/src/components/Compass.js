import React from 'react';
import './Compass.scss';

class Compass extends React.Component {
  render() {
    const isFixedPointer = this.props.isFixedPointer;

    const heading = Number(this.props.heading);
    const pointerRotation = heading + 90 - 15;
    const bodyRotation = isFixedPointer ? -heading : 0;

    return (
      <div className="compass-wrapper">
        <div className="compass-wrapper2">
          <div className="compass" style={{
            transform: `rotate(${bodyRotation}deg)`
          }}>
            <span>N</span>
            <span>E</span>
            <span>S</span>
            <span>W</span>

            <div className="compass-pointer" style={{
              transform: `rotate(${pointerRotation}deg)`
            }}></div>
          </div>
        </div>
      </div>
    );
  }
}

export default Compass;
