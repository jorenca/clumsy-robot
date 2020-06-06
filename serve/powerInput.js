
const CURRENT_SENSOR_I2C_ADDR = 0x41;
const ina219 = require('./ina219.js')(CURRENT_SENSOR_I2C_ADDR);

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
