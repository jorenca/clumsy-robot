import React from 'react';
import _ from 'lodash';
import './Battery.scss';

const classNames = (map) => _(map)
  .toPairs()
  .filter(([k, v]) => !!v)
  .map(([k]) => k)
  .join(' ')

const Battery = ({ battery = {} }) => {
  const percent = ((_.round(battery.busV, 2) / 3 - 3.7)/0.5)*100;

  return (
    <div className="battery-indicator">
      <table width="100%">
        <tbody>
          <tr><td>Bus voltage</td><td>{_.round(battery.busV, 1) || '???'}V</td></tr>
          <tr><td>Current</td><td>{_.round(battery.current, 2) || '???'}mA</td></tr>
          <tr><td>Power draw</td><td>{_.round(battery.power, 1) || '???'}W</td></tr>
        </tbody>
      </table>

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
    </div>
  );
};

export default Battery;
