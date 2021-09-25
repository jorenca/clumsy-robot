import React from 'react';
//import Compass from './telemetry/Compass';
//import XYPlot from './telemetry/XYPlot';
import Battery from './telemetry/Battery';
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
    const { battery } = this.state;
    return (
      <div className="telemetry-container">
        Telemetry
        <hr/>
        <Battery battery={battery} />
      </div>
    );
  }
}

//        <hr/>
//        <div className="telemetry-row">
//          <Compass heading={heading} />
//          <XYPlot {...acceleration} bound={1024} />
//        </div>
//        <div style={{ textAlign: 'left' }}>
//          <div style={{ width: '100px' }}>Heading: {heading}</div>
//          <div style={{ backgroundColor: temperature < 35 ? '' : 'orange'}}>
//            Temperature: {temperature}C
//          </div>
//        </div>

export default Telemetry;
