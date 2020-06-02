import React from 'react';
import Compass from './Compass';
import XYPlot from './XYPlot';
import './Telemetry.scss';

const TELEMETRY_ADDRESS = "/telemetry";
var telemetrySource = new EventSource(TELEMETRY_ADDRESS);

class Telemetry extends React.Component {
  state = {
    heading: null,
    acceleration: null
  };

  componentDidMount() {
    telemetrySource.addEventListener('message', e => this.updateTelemetry(e.data), false);
  }

  updateTelemetry(rawData) {
    const data = JSON.parse(rawData);
    // console.log('telemetery data received', data);
    this.setState(data);
    this.props.shareData(data);
  }

  render() {
    const { heading, acceleration, temperature, battery } = this.state;
    return (
      <div className="telemetry-container">
        Telemetry
        <Battery battery={battery} />
        <Heading heading={heading} />
        <Acceleration acceleration={acceleration} />
        <Temperature temperature={temperature} />
      </div>
    );
  }
}

const Battery = ({ busV, power, current }) =>
<div >
  <table width="100%">
    <tr><td>Bus voltage</td><td>{busV || '???'}V</td></tr>
    <tr><td>Power draw</td><td>{power || '???'}W</td></tr>
    <tr><td>Current</td><td>{current || '???'}mA</td></tr>
  </table>
</div>

const Heading = ({ heading }) =>
<div className="telemetry-row">
  <div style={{ width: '100px' }}>HDG: {heading}</div>
  <Compass heading={heading} />
  <Compass heading={heading} isFixedPointer={true} />
</div>

const Acceleration = ({ acceleration }) =>
<div className="telemetry-row">
  <div style={{ width: '200px' }}>Acceleration</div>
  <XYPlot {...acceleration} bound={1024} />
</div>;

const Temperature = ({ temperature }) =>
<div className="telemetry-row">
  Temperature: {temperature}
</div>


export default Telemetry;
