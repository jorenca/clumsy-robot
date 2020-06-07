import React from 'react';
import _ from 'lodash';
import './Battery.scss';

const classNames = (map) => _(map)
  .toPairs()
  .filter(([k, v]) => !!v)
  .map(([k]) => k)
  .join(' ')

const Battery = ({ battery = {} }) => {
  const percent = _.floor(((battery.batteryV / 3 - 3.3) / 0.9 ) * 100);

  return (
    <div className="battery-indicator">
      <table>
        <tbody>
          <tr>
            <td>Battery voltage</td><td>{_.floor(battery.batteryV, 1).toFixed(1)}V</td>
            <td rowspan="4" width="110px">
              <div>
                <div className="perc-value">{_.round(percent)}%</div>

                <div className="battery">
                  <div id="battery-level" className={classNames({
                    high: percent > 50,
                    medium: percent <=50 && percent > 20,
                    low: percent <= 20
                  })} style={{ width: percent }}></div>
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
