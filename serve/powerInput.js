const ina219 = require('./ina219.js');

const CURRENT_SENSOR_I2C_ADDR = 0x41;
const INA219_REG_BUSVOLTAGE = 0x02;
const INA219_REG_POWER = 0x03;
const INA219_REG_CURRENT = 0x04;

ina219.init(CURRENT_SENSOR_I2C_ADDR);

module.exports.create = ({ readInterval, callback }) => {
    const sendData = async () => {
      const busV = await ina219.getBusVoltage_V();
      const currentmA = ina219.getCurrent_mA();
      const power = busV * currentmA * 1000;

      callback({ busV, power, current });
    };

    if (ina219) {
      ina219.calibrate32V2A()
        .then(() => setInterval(sendData, readInterval));
    }
  };
