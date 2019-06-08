import React from 'react';
import './App.css';
import Telemetry from './components/Telemetry';
import Movement from './components/Movement';

class App extends React.Component {

  render() {
    return (
      <div className="App">
        <Movement />
        <Telemetry />
      </div>
    );
  }
}

export default App;
