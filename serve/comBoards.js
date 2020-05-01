const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const _ = require('lodash');

SerialPort.list()
  .then(portsList => console.log('Available COM devices:', portsList));

module.exports = {
  connectToMotorBoard: async () => {
    const portsList = await SerialPort.list();
    const comDevice = _(portsList)
      .filter({
        serialNumber: '9900000037944e45002c4011000000450000000097969901',
        productId: '0204'
      }).first();
    if (!comDevice) {
      console.error('[ERROR] Motor board not found. Is it connected?')
      return {
        addListener: _.noop,
        send: _.noop
      }
    };

    console.log('[INFO] Motor board connected as', comDevice);
    const comPort = new SerialPort(comDevice.comName, { baudRate: 115200 });

    const comParser = new Readline();
    comPort.pipe(comParser);

    return {
      addListener: callb => comParser.on('data', callb),
      send: data => {
        console.log(`[MB]< ${data} (length ${data.length})`);
        comPort.write(data);
      }
    };
  },
  ///////////////////////////////////////////////////////////////////////

  connectToDetectorBoard: async () => {
    const portsList = await SerialPort.list();
    const comDevice = _(portsList)
      .filter({
        serialNumber: '6&156d72de&0&2',
        productId: '7523'
      }).first();
    if (!comDevice) {
      console.error('[ERROR] IR detection module not found. Is it connected?')
      return {
        addListener: _.noop,
        send: _.noop
      }
    };

    console.log('[INFO] IR detection module connected as', comDevice);
    const comPort = new SerialPort(comDevice.comName, { baudRate: 9600 });

    const comParser = new Readline();
    comPort.pipe(comParser);

    return {
      addListener: callb => comParser.on('data', callb),
      send: data => {
        console.log(`[IR]< ${data} (length ${data.length})`);
        comPort.write(data);
      }
    };
  }
};
