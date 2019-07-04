import React from 'react';
import './CameraFeed.scss';

const CameraFeed = ({ proximity }) =>
<div className="feed-container">
  <ProximityGauge proximity={Number.parseInt(proximity || '255')} />
</div>;

const MAX_SENSOR_READING = 203;
const MIN_SENSOR_READING = 18;
const SENSOR_READING_RANGE = MAX_SENSOR_READING - MIN_SENSOR_READING;
const distReadingMapper = x => 0.237 * x - 0.39;
const ProximityGauge = ({ proximity }) => {
  const valueInBounds = Math.max(Math.min(proximity, MAX_SENSOR_READING), MIN_SENSOR_READING);
  const distCm = distReadingMapper(valueInBounds);
  const percentage = (valueInBounds - MIN_SENSOR_READING) / SENSOR_READING_RANGE;

  const text = proximity > MAX_SENSOR_READING ? '>50cm or <4cm' : distCm + 'cm';

  return (
    <div className="proximity-gauge-container">
      <div className="gauge-rail">
        <div className="gauge-block" style={{ top:  (1 - percentage) * 90 /* slider height is 10% */ + '%' }} />
      </div>
      <div>{text}</div>
    </div>
  );
};

export default CameraFeed;
