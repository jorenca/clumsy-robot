const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const Ready = require('@serialport/parser-ready')
const _ = require('lodash');

const GRBL_INITIALIZED = "Grbl 0.9j ['$' for help]";

SerialPort.list()
  .then(portsList => console.log('Available COM devices:', portsList));

let outputToAwait = '';
let resolveCb = null;
const onDataConsiderResolveWait = data => {
  if (!!outputToAwait && !!data && data.includes(outputToAwait)) {
    resolveCb(data);
  }
};
const waitForOutput = async out => {
    outputToAwait = out;
    return new Promise(resolve => { resolveCb = resolve; });
};

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
    comParser.on('data', onDataConsiderResolveWait);
    comParser.on('data', line => console.log(`[MB]> ${line}`));

    const sendRaw = (data) => {
      console.log(`[MB]< ${data} (length ${data.length})`);
      comPort.write(data + '\n');
    };

    const rawMoveXY = ({ x, y, feed }) => {
      // G91 is Relative movement mode
      sendRaw(`G91 G1 F${Number(feed).toString()} X${Number(x).toPrecision(3)} Y${Number(y).toPrecision(3)}`);
    };

    const doBasicMove = ({ direction, revs, rpm }) => {
      rawMoveXY({
        x: revs * 10 * (direction === 'left' ? -1 : (direction === 'forward' ? 1 : 0)),
        y: revs * 10 * (direction === 'right' ? -1 : (direction === 'forward' ? 1 : 0)),
      });
    };

    await waitForOutput(GRBL_INITIALIZED);

    sendRaw('$3=1'); // Invert X axis
    await waitForOutput('ok');

    sendRaw('$100=70'); // X steps per cm
    await waitForOutput('ok');
    sendRaw('$101=70'); // Y steps per cm
    await waitForOutput('ok');

    sendRaw('$110=5000'); // X max feed rate
    await waitForOutput('ok');
    sendRaw('$111=5000'); // Y max feed rate
    await waitForOutput('ok');

    sendRaw('$120=90.0'); // Max X acceleration
    await waitForOutput('ok');
    sendRaw('$121=90.0'); // Max Y acceleration
    await waitForOutput('ok');

    return {
      addListener: callb => comParser.on('data', callb),
      sendRaw,
      cstop: async () => {
        sendRaw('\x18');  // soft reset
        await waitForOutput(GRBL_INITIALIZED);
        sendRaw('$X'); // clear alarm
        await waitForOutput('ok');
      },
      doBasicMove,
      rawMoveXY,
      //doDirectMove: ({ timeMs, frequency, dir }) => sendRaw(`DR ${timeMs} ${frequency} ${dir} ;`)
    };
  }
};
