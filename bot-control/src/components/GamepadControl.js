import React from 'react';
import _ from 'lodash';
import Gamepad from 'react-gamepad';

// left and right revs (float), speed up to 99 rpm
const moveCallUnthrottled = (left, right, speed) => fetch(`/move/${left}/${speed}/${right}/${speed}`);
const moveCall = _.throttle(moveCallUnthrottled, 500);
const revsInHalfSec = (speed) => speed /* rpm */ / (60 * 2);

const Move = {
  up: (speed) => moveCall(revsInHalfSec(speed), revsInHalfSec(speed), speed),
  down: (speed) => moveCall(-revsInHalfSec(speed), -revsInHalfSec(speed), speed),
  left: (speed) => moveCall(revsInHalfSec(speed), 0, speed),
  right: (speed) => moveCall(0, revsInHalfSec(speed), speed)
};

const AxisData = {
  x: 0,
  y: 0
};

const AxisUpdate = {
  LeftStickY: (value) => AxisData.y = value,
  LeftStickX: (value) => AxisData.x = value
};

const applyMovementLoop = () => {
  if (AxisData.x === 0 && AxisData.y === 0) {
    return;
  }

  if (Math.abs(AxisData.y) > Math.abs(AxisData.x)) {
    AxisData.y > 0 ? Move.up(AxisData.y) : Move.down(Math.abs(AxisData.y));
  } else {
    AxisData.x < 0 ? Move.left(Math.abs(AxisData.x)) : Move.right(AxisData.x);
  }
};

setInterval(applyMovementLoop, 450); // some tolerance

const MIN_SPEED = 50;
const MAX_SPEED = 99;
const onAxisChange = (axis, value) => {
  console.log(axis, ' / ', value);
  let requestedSpeed = value * MAX_SPEED;
  if (Math.abs(requestedSpeed) < MIN_SPEED) {
    requestedSpeed = 0;
  }

  const applyAxisUpdate = AxisUpdate[axis] || _.noop;
  applyAxisUpdate(requestedSpeed);
};

export default () => <Gamepad onAxisChange={onAxisChange} ><p /></Gamepad>;
