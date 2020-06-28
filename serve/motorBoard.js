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
        vendorId: '1a86',
        productId: '7523'
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
      comPort.write(data + '\n');
    };

    const rawMoveXY = ({ x, y, feed }) => {
      // G91 - relative mode
      sendRaw(`G91 G1 F${Number(feed).toString()} X${(-Number(x)).toPrecision(3)} Y${Number(y).toPrecision(3)}`);
    };

    const doBasicMove = ({ direction, revs, rpm }) => {
      rawMoveXY({
        x: revs * 10 * (direction === 'left' ? -1 : (direction === 'forward' ? 1 : 0)),
        y: revs * 10 * (direction === 'right' ? -1 : (direction === 'forward' ? 1 : 0)),
      });
    };

    return {
      addListener: callb => comParser.on('data', callb),
      sendRaw,
      cstop: () => { sendRaw('\x18'); sendRaw('$X'); }, // soft reset + clear alarm
      doBasicMove,
      rawMoveXY,
      //doDirectMove: ({ timeMs, frequency, dir }) => sendRaw(`DR ${timeMs} ${frequency} ${dir} ;`)
    };
  }
};
