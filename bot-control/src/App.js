import React from 'react';
import './App.css';
import Telemetry from './components/Telemetry';
import Movement from './components/Movement';
import CameraFeed from './components/CameraFeed';
import DirectCommand from './components/DirectCommand';

class App extends React.Component {
  state = {
    sharedTelemetry: {}
  };

  shareTelemetry = data => this.setState({
    sharedTelemetry: { ...this.state.sharedTelemetry, ...data }
  });

  render() {
    const { sharedTelemetry } = this.state;

    return (
      <div className="App">
        <CameraFeed proximity={sharedTelemetry.proximity} />
        <div className="side-panel">
          <Movement sharedTelemetry={sharedTelemetry}/>
          <Telemetry shareData={this.shareTelemetry} />
          <DirectCommand />
        </div>
      </div>
    );
  }
}

export default App;
