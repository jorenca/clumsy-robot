const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const _ = require('lodash');

SerialPort.list()
  .then(portsList => console.log('Available COM devices:', portsList));

module.exports = {
  connect: async () => {
    const portsList = await SerialPort.list();
    const comDevice = _(portsList)
      .filter({
        serialNumber: '9900000037944e45002c4011000000450000000097969901',
        productId: '0204'
      }).first();
    if (!comDevice) {
      console.error('[ERROR] Motor board not found. Is it connected?');
      const notConnPrint = (...args) => console.log('[NOT CONNECTED MB]<', ...args);
      return {
        addListener: _.noop,
        sendRaw: notConnPrint,
        doBasicMove: notConnPrint,
        doMove: notConnPrint,
        doDirectMove: notConnPrint
      };
    };

    console.log('[INFO] Motor board connected as', comDevice);
    const comPort = new SerialPort(comDevice.comName, { baudRate: 115200 });

    const comParser = new Readline();
    comPort.pipe(comParser);

    const sendRaw = (data) => {
      console.log(`[MB]< ${data} (length ${data.length})`);
      comPort.write(data);
    };

    const doMove = ({ xr, xrpm, yr, yrpm }) =>
      sendRaw(`M ${xr} ${xrpm} ${yr} ${yrpm} ;`);

    const doBasicMove = ({ direction, revs, rpm }) => {
      doMove({
        xr: revs * (direction === 'left' ? -1 : (direction === 'forward' ? 1 : 0)),
        yr: revs * (direction === 'right' ? -1 : (direction === 'forward' ? 1 : 0)),
        xrpm: rpm,
        yrpm: rpm
      });
    };

    return {
      addListener: callb => comParser.on('data', callb),
      sendRaw,
      cstop: sendRaw('CSTOP;'),
      doBasicMove,
      doMove,
      doDirectMove: ({ timeMs, frequency, dir }) => sendRaw(`DR ${timeMs} ${frequency} ${dir} ;`)
    };
  }
};
