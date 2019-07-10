import React from 'react';
import './DirectCommand.scss';

class DirectCommand extends React.Component {
  state = {
    visible: false,
    lastCmd: null
  };

  sendCommand = cmd => {
    this.setState({ lastCmd: cmd });
    fetch(`/motor_cmd/${cmd}`);
  };

  toggleInputVisibility = () => this.setState({ visible: !this.state.visible });

  render() {
    const { visible, lastCmd } = this.state;
    return (
      <div className="direct-command-container">
        <button onClick={this.toggleInputVisibility}>Send direct command</button>
        {visible && <CommandInputModal onSend={this.sendCommand} last={lastCmd} />}
      </div>
    );
  }
}

class CommandInputModal extends React.Component {
  state = {
    cmd: ''
  };

  render() {
    const { cmd } = this.state;
    const { onSend, last } = this.props;
    return (
      <div className="command-input">
        <textarea
          rows="10"
          value={cmd}
          onChange={e => this.setState({ cmd: e.target.value })}
        />
        <button onClick={() => onSend(cmd)}>Send</button>
        
        <pre>
          M leftRotations leftRPM rightRotations rightRPM<br />
          D[L|R] timeMs stepFrequency direction<br/>
          CSTOP<br />
          <br />
          <br />
          CMD1;<br />
          M_CMD1:M_CMD2:M_CMD3:...;
        </pre>

        {last && <div className="last-sent-container">
          <pre>
            Last sent:<br />
            {last}
          </pre>
          <button onClick={() => onSend(last) }>
            Resend
          </button>
        </div>}
      </div>
    );
  }
}

export default DirectCommand;
