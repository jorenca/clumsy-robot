import React, { useRef, useEffect, useState } from 'react';
import _ from 'lodash';
import Gamepad from 'react-gamepad';
import { Joystick } from 'react-joystick-component';

const CALLS_PER_SEC = 3;
const revsInPartSec = (speed) => speed /* rpm */ / (60 * CALLS_PER_SEC);

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
export default ({ moveUp, moveDown, moveLeft, moveRight, setHasGamepad, stop }) => {
  const [xAxisValue, setXAxis] = useState(0);
  const [yAxisValue, setYAxis] = useState(0);
  const [turboCoeff, setTurboCoeff] = useState(1);

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

    if (Math.abs(yAxisValue) > Math.abs(xAxisValue)) { // move ahead / back
      const speed = yAxisValue * turboCoeff;
      if (speed > 0) {
        moveUp(revsInPartSec(speed), speed);
      } else {
        moveDown(revsInPartSec(Math.abs(speed)), Math.abs(speed));
      }
    } else { // turn left or right
      const speed = xAxisValue / 2;
      if (speed < 0) {
        moveLeft(revsInPartSec(Math.abs(speed)), Math.abs(speed));
      } else {
        moveRight(revsInPartSec(speed), speed);
      }
    }
  }, (1000 / CALLS_PER_SEC) - 20 /* some toleranse */);

  return (
    <div>
      <div className="virt-joystick-container">
        <Joystick
          size={100}
          baseColor="gray"
          stickColor="lightgray"
          move={({ x, y }) => {
            setXAxis(x / 50);
            setYAxis(y / 50);
          }}
          stop={() => {
            setXAxis(0);
            setYAxis(0);
            stop();
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
