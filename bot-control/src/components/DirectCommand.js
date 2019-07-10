import React from 'react';
import './DirectCommand.scss';

class DirectCommand extends React.Component {
  state = {
    visible: false
  };

  sendCommand = cmd => {
    this.setState({ visible: false });
    fetch(`/motor_cmd/${cmd}`);
  };

  toggleInputVisibility = () => this.setState({ visible: !this.state.visible });

  render() {
    const { visible } = this.state;
    return (
      <div className="direct-command-container">
        <button onClick={this.toggleInputVisibility}>Send direct command</button>
        {visible && <CommandInput onSend={this.sendCommand} />}
      </div>
    );
  }
}

const CommandInput = ({ onSend }) =>
<div className="command-input">
  <form onSubmit={e => {
    e.preventDefault();
    onSend(e.target.command.value);
  }}>
    <textarea rows="10" name="command" />
    <pre>
      M leftRotations leftRPM rightRotations rightRPM<br />
      D[L|R] timeMs stepFrequency direction<br/>
      CSTOP<br />
      <br />
      <br />
      CMD1;<br />
      M_CMD1:M_CMD2:M_CMD3:...;
    </pre>
    <button type="submit">Send</button>
  </form>
</div>;

export default DirectCommand;
