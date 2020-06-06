
const CURRENT_SENSOR_I2C_ADDR = 0x41;
const ina219 = require('ina219-async')(CURRENT_SENSOR_I2C_ADDR);

module.exports.create = ({ readInterval, callback }) => {
    const sendData = async () => {
      const busV = await ina219.getBusVoltage_V();
      const currentmA = await ina219.getCurrent_mA();
      const current = currentmA / 1000;
      const power = busV * current;

      callback({ busV, power, current });
    };

    if (ina219) {
      ina219.calibrate32V2A()
        .then(() => setInterval(sendData, readInterval));
    }
  };
