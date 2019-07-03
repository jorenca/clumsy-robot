import React from 'react';
import './CameraFeed.scss';

const CameraFeed = ({ proximity }) =>
<div className="feed-container">
  <ProximityGauge proximity={Number.parseInt(proximity || '255')} />
</div>;

const MAX_SENSOR_READING = 200;
const ProximityGauge = ({ proximity }) => {
  const valueInBounds = Math.min(proximity, MAX_SENSOR_READING);
  const text = proximity > MAX_SENSOR_READING ? '>50cm or <4cm' : proximity + 'cm';

  return (
    <div className="proximity-gauge-container">
      <input type="range" min="10" max="200" value={proximity} orient="vertical" name="proximity-gauge-slider" />
      <div>{text}</div>
    </div>
  );
};

export default CameraFeed;
