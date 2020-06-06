import React from 'react';
import _ from 'lodash';
import './Battery.scss';

const classNames = (map) => _(map)
  .toPairs()
  .filter(([k, v]) => !!v)
  .map(([k]) => k)
  .join(' ')

const Battery = ({ battery = {} }) => {
  const percent = ((battery.busV / 3 - 3.3)/0.9)*100;

  return (
    <div className="battery-indicator">
      <table width="100%">
        <tbody>
          <tr><td>Bus voltage</td><td>{_.floor(battery.busV, 1) || '???'}V</td></tr>
          <tr><td>Current</td><td>{_.ceil(battery.current, 2) || '???'}A</td></tr>
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
          })} style={{ width: _.round(percent, 1) }}></div>
        </div>
      </div>
    </div>
  );
};

export default Battery;
