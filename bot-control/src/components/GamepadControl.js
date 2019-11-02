import React, { useRef, useEffect, useState } from 'react';
import _ from 'lodash';
import Gamepad from 'react-gamepad';

const revsInHalfSec = (speed) => speed /* rpm */ / (60 * 2);

function useInterval(callback, delay) {
  const savedCallback = useRef();

  // Remember the latest callback.
  useEffect(() => {
    savedCallback.current = callback;
  }, [callback]);

  // Set up the interval.
  useEffect(() => {
    function tick() {
      savedCallback.current();
    }
    if (delay !== null) {
      let id = setInterval(tick, delay);
      return () => clearInterval(id);
    }
  }, [delay]);
}

const MIN_SPEED = 20;
const MAX_SPEED = 99;
export default ({ moveUp, moveDown, moveLeft, moveRight, setHasGamepad }) => {
  const [xAxisValue, setXAxis] = useState(0);
  const [yAxisValue, setYAxis] = useState(0);

  const onAxisChange = (axis, value) => {
    let requestedSpeed = value * MAX_SPEED;
    if (Math.abs(requestedSpeed) < MIN_SPEED) {
      requestedSpeed = 0;
    }

    const applyAxisUpdate = {
      LeftStickY: setYAxis,
      LeftStickX: setXAxis,
      RightStickY: setYAxis,
      RightStickX: setXAxis,
    }[axis] || _.noop;
    applyAxisUpdate(requestedSpeed);
  };

  useInterval(() => {
    if (xAxisValue === 0 && yAxisValue === 0) {
      return;
    }

    if (Math.abs(yAxisValue) > Math.abs(xAxisValue)) { // move ahead / back
      const speed = yAxisValue;
      if (speed > 0) {
        moveUp(revsInHalfSec(speed), speed);
      } else {
        moveDown(revsInHalfSec(Math.abs(speed)), Math.abs(speed));
      }
    } else { // turn left or right
      const speed = xAxisValue / 2;
      if (speed < 0) {
        moveLeft(revsInHalfSec(Math.abs(speed)), Math.abs(speed));
      } else {
        moveRight(revsInHalfSec(speed), speed);
      }
    }
  }, 450);

  return (
    <Gamepad
      onAxisChange={onAxisChange}
      onConnect={() => setHasGamepad(true)}
      onDisconnect={() => setHasGamepad(false)}>
      <p />
    </Gamepad>
  );
}
