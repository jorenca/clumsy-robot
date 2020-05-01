
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')

module.exports = {
  connectToMotorBoard: async () => {
    const portsList = await SerialPort.list();
    console.log('Available COM ports:', portsList);
    const comName = (portsList[1] || portsList[0]).comName;
    console.log('Using telemetry provider on COM port', comName);
    const comPort = new SerialPort(comName, { baudRate: 115200 });
    const comParser = new Readline();
    comPort.pipe(comParser);

    return {
      addListener: callb => comParser.on('data', callb),
      send: data => {
        console.log(`< ${data} (message length ${data.length})`);
        comPort.write(data);
      }
    };
  },
  connectToDetectorBoard: () => null
};
