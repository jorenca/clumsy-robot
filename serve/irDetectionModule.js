const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const _ = require('lodash');

const RollingDataCache = () => {
  let data = [];

  const getSince = timestamp => _.filter(data, e => e.time > timestamp);

  return {
    addSample: sample => data.push({ sample, time: _.now() }),
    getSince,
    flushOld: timestamp => data = getSince(timestamp)
  };
};

const pulsesToCoordinates = pulses => {
  const [front, right, backRight, backLeft, left ] = pulses;
  // TODO send as part of telemetry
};

const dataToMove = data => {
  const { pulses, counts, totalSamples } = data;
  const [front, right, backRight, backLeft, left] = pulses;

  if (front >= 29) return { direction: 'none' };

  if (left > front) return { direction: 'left', revs: 0.3 };
  if (backLeft > front) return { direction: 'left', revs: 1.5 };

  if (right > front) return { direction: 'right', revs: 0.3 };
  if (backRight > front) return { direction: 'right', revs: 1.5 };

  if (front > 5) return { direction: 'forward', revs: 1.5 };
  return { direction: 'none' };
};

module.exports = {
  connect: async () => {
    const portsList = await SerialPort.list();
    const comDevice = _(portsList)
      .filter({
        //serialNumber: '6&156d72de&0&2',
        productId: '7523'
      }).first();
    if (!comDevice) {
      console.error('[ERROR] IR detection module not found. Is it connected?')
      return null;
    }

    console.log('[INFO] IR detection module connected as', comDevice);
    const comPort = new SerialPort(comDevice.comName, { baudRate: 9600 });

    const comParser = new Readline();
    comPort.pipe(comParser);
    comParser.on('data', _.throttle(line => console.log(`[IR]> ${line}`), 1000));

    const cache = RollingDataCache();
    comParser.on('data', line => cache.addSample(JSON.parse(line)));

    return {
      getMoveToTarget: () => {
        const useEventsSince = _.now() - 1000;
        const recentEvents = cache.getSince(useEventsSince);
        if (recentEvents.length <= 1) return;

        cache.flushOld(useEventsSince);

        return dataToMove(_.last(recentEvents).sample); // TODO use all recent events
      }
    };
  }
};
