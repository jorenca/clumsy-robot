const headingCalc = (function() {
  let data = [];
  return {
    addDataPoint: dp => data.push(Number(dp)),
    isReady: () => data.length >= 2,
    avg: () => {
      const resSin = data.reduce((acc, e) => Math.sin(e * Math.PI / 180.0) + acc, 0) / data.length;
      const resCos = data.reduce((acc, e) => Math.cos(e * Math.PI / 180.0) + acc, 0) / data.length;
      data = [];

      return Math.floor(Math.atan2(resSin, resCos) * 180 / Math.PI);
    }
  };
})();

module.exports.recalcWithNewDataLine = (line) => {
  const heading = line.match(/H(\d+)/)[1];
  headingCalc.addDataPoint(heading);

  const acceleration = {
    x: line.match(/AX(-?\d+)/)[1],
    y: line.match(/AY(-?\d+)/)[1],
    z: line.match(/AZ(-?\d+)/)[1]
  };
  const temperature = line.match(/T(\d+)/)[1];

  const partialRes = { acceleration, temperature };
  return headingCalc.isReady()
    ? { ...partialRes, heading: headingCalc.avg() }
    : partialRes;
};
