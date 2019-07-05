import React from 'react';
import './App.css';
import Telemetry from './components/Telemetry';
import Movement from './components/Movement';
import CameraFeed from './components/CameraFeed';

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
          <Movement />
          <Telemetry shareData={this.shareTelemetry} />
        </div>
      </div>
    );
  }
}

export default App;
