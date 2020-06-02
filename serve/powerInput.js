const i2c = require('i2c-bus');
const i2c1 = i2c && i2c.openSync(1);

const CURRENT_SENSOR_I2C_ADDR = 0x41;
const INA219_REG_BUSVOLTAGE = 0x02;
const INA219_REG_POWER = 0x03;
const INA219_REG_CURRENT = 0x04;

module.exports.create = ({ readInterval, callback }) => {
    const sendData = () => {
      const busRaw = i2c1.readByteSync(CURRENT_SENSOR_I2C_ADDR, INA219_REG_BUSVOLTAGE);
      const busV = (busRaw >> 3) * 4;

      const power = i2c1.readByteSync(CURRENT_SENSOR_I2C_ADDR, INA219_REG_POWER);
      const current = i2c1.readByteSync(CURRENT_SENSOR_I2C_ADDR, INA219_REG_CURRENT);

      callback({ busV, power, current });
    };
    if (i2c1) {
      setInterval(sendData, readInterval);
    }
  };
