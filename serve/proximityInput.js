const i2c = require('i2c-bus');

const PROXIMITY_SENSOR_I2C_ADDR = 0x40;
const PROXIMITY_SENSOR_I2C_DATA_REG = 0x5e;
const i2c1 = i2c && i2c.openSync(1);

module.exports.create = ({ readInterval, callback }) => {
    const sendProximityData = () => {
      const proximity = i2c1.readByteSync(PROXIMITY_SENSOR_I2C_ADDR, PROXIMITY_SENSOR_I2C_DATA_REG);
      callback(proximity);
    };
    if (i2c1) {
      setInterval(sendProximityData, readInterval);
    }
  };
