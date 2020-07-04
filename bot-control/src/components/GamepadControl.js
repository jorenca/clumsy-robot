import React, { useRef, useEffect, useState } from 'react';
import _ from 'lodash';
import Gamepad from 'react-gamepad';
import { Joystick } from 'react-joystick-component';

const CALLS_PER_SEC = 2;
const mmsInPartSec = (feedRate) => (feedRate / (CALLS_PER_SEC  * 60)) * 0.9 /* magic to get the timing right */;

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

const MIN_SPEED = 500;
const MAX_SPEED = 3000;
export default ({ baseMove, moveUp, moveDown, moveLeft, moveRight, setHasGamepad, stop }) => {
  const [xAxisValue, setXAxis] = useState(0);
  const [yAxisValue, setYAxis] = useState(0);
  const [turboCoeff, setTurboCoeff] = useState(1);

  const axisValueToSpeed = value /* 0-1 */ => {
      let requestedSpeed = value * MAX_SPEED;
      if (Math.abs(requestedSpeed) < MIN_SPEED) {
        return 0;
      }
      return requestedSpeed;
  };

  const onAxisChange = (axis, value) => {
    const applyAxisUpdate = {
      LeftStickY: setYAxis,
      LeftStickX: setXAxis,
      RightStickY: setYAxis,
      RightStickX: setXAxis,
    }[axis] || _.noop;
    applyAxisUpdate(value);
  };

  const onButtonPressed = (button) => {
    if (button === 'Y') {
      moveLeft(0.1, 40);
    } else if (button === 'B') {
      moveRight(0.1, 40);
    } else if (button === 'RT') {
      setTurboCoeff(2.5);
    } else if (button === 'LT') {
      stop();
    }
  };

  const onButtonUp = (button) => {
    if (button === 'RT') {
      setTurboCoeff(1);
    }
  };

  useInterval(() => {
    if (xAxisValue === 0 && yAxisValue === 0) {
      return;
    }

    const speed = axisValueToSpeed(yAxisValue);
    let left = mmsInPartSec(speed);
    let right = mmsInPartSec(speed);

    if (xAxisValue > 0 && Math.abs(xAxisValue) > 0.25) {
      right -= Math.abs(xAxisValue / 2) * right;
    } else {
      left -= Math.abs(xAxisValue / 2) * left;
    }

    if (left === 0 && right === 0) return;

    baseMove(left, right, Math.abs(speed));
  }, (1000 / CALLS_PER_SEC));

  return (
    <div>
      <div className="virt-joystick-container">
        <Joystick
          size={150}
          baseColor="gray"
          stickColor="lightgray"
          move={({ x, y }) => {
            setXAxis(x / 75);
            setYAxis(y / 75);
          }}
          stop={() => {
            setXAxis(0);
            setYAxis(0);
          }} />
      </div>

      <Gamepad
        onAxisChange={onAxisChange}
        onButtonDown={onButtonPressed}
        onButtonUp={onButtonUp}
        onConnect={() => setHasGamepad(true)}
        onDisconnect={() => setHasGamepad(false)}>
        <p />
      </Gamepad>
    </div>
  );
}
