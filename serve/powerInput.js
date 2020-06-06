
const CURRENT_SENSOR_I2C_ADDR = 0x41;
const ina219 = require('ina219-async')(CURRENT_SENSOR_I2C_ADDR);

module.exports.create = ({ readInterval, callback }) => {
    const sendData = async () => {
      const busV = await ina219.getBusVoltage_V();
      const shuntV = await ina219.getShuntVoltage_mV() / 1000;
      const currentmA = await ina219.getCurrent_mA();
      const current = currentmA / 1000;

      callback({ busV, batteryV: busV + shuntV, current });
    };

    if (ina219) {
      ina219.calibrate32V2A()
        .then(() => setInterval(sendData, readInterval));
    }
  };
