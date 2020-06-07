import React, { useState } from 'react';
import _ from 'lodash';
import './Battery.scss';

const classNames = (map) => _(map)
  .toPairs()
  .filter(([k, v]) => !!v)
  .map(([k]) => k)
  .join(' ')

const Battery = ({ battery = {} }) => {
  const percent = _.floor(((battery.batteryV / 3 - 3.3) / 0.9 ) * 100);
  const [batteryIndicatorLevel, setIndicatorLevel] = useState(100);
  if (batteryIndicatorLevel > percent) setIndicatorLevel(percent);

  return (
    <div className="battery-indicator">
      <table>
        <tbody>
          <tr>
            <td>Battery voltage</td><td>{_.floor(battery.batteryV, 1).toFixed(1)}V</td>
            <td rowSpan="4" width="110px">
              <div>
                <div className="perc-value">{_.round(batteryIndicatorLevel)}%</div>

                <div className="battery">
                  <div id="battery-level" className={classNames({
                    high: batteryIndicatorLevel > 50,
                    medium: batteryIndicatorLevel <=50 && batteryIndicatorLevel > 20,
                    low: batteryIndicatorLevel <= 20
                  })} style={{ width: batteryIndicatorLevel }}></div>
                </div>
              </div>
            </td>
          </tr>
          <tr><td>Bus voltage</td><td>{_.floor(battery.busV, 1).toFixed(1)}V</td></tr>
          <tr><td>Current</td><td>{_.ceil(battery.current, 2).toFixed(2)}A</td></tr>
          <tr><td>Power draw</td><td>{_.ceil(battery.current * battery.batteryV, 1).toFixed(1)}W</td></tr>
          <tr>
          </tr>
        </tbody>
      </table>
    </div>
  );
};

export default Battery;
