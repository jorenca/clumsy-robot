const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const _ = require('lodash');

module.exports = {
  connectToMotorBoard: async () => {
    const portsList = await SerialPort.list();
    console.log('Available COM ports:', portsList);

    const comName = _(portsList)
      .filter({
        vendorId: '0D28',
        productId: '0204'
      }).first().comName;
    console.log('[INFO] Motor board connected as', comName);
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
