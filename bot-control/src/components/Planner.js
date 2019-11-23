import React, { useState, useCallback } from 'react';
import './Planner.scss';

export default function Planner({ move, cTurn, sharedTelemetry }) {
  const [isVisible, setVisible] = useState(false);

  const moveDist = 2; // todo make configurable

  const commands = {
    FW: () => move(moveDist, moveDist, 100),
    BW: () => move(-moveDist, -moveDist, 100),
    'CW 90': () => move(-2, 2, 50),
    'Coord CW 90': () => cTurn((sharedTelemetry.heading + 90) % 360)
  };

  return (
    <div className="planning-container">
      <button
        className="planning-toggle"
        onClick={() => setVisible(!isVisible)}
        >
        Move planner
      </button>
      {isVisible && <PlannerModal commands={commands} />}
    </div>
  );
}

function timeout(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

let terminatePlan = false;
async function executePlan(plan, commands, stepCallback) {
  for (const stepIndex in plan) {
    if (terminatePlan) {
      terminatePlan = false;
      break;
    }
    const step = plan[stepIndex];
    stepCallback(stepIndex);
    await commands[step]();
    await timeout(2000);
  }
  stepCallback(null);
}

function PlannerModal({ commands }) {
  const [plan, setPlan] = useState([]);
  const addToPlan = (...moves) => setPlan([...plan, ...moves]);

  const [runningIndex, setRunningIndex] = useState(null);
  const runPlan = useCallback(
    () => executePlan(plan, commands, setRunningIndex),
    [plan, commands]
  );

  return (
    <div className="planning-modal">
      <div className="planning-steps-container">
        <button onClick={() => addToPlan('FW')}>Add forward</button>
        <button onClick={() => addToPlan('BW')}>Add backward</button>
        <button onClick={() => addToPlan('CW 90')}>CW 90</button>
        <button onClick={() => addToPlan('Coord CW 90')}>Coordinated CW 90</button>
      </div>
      <hr />

      {runningIndex === null
        ? <button onClick={runPlan}>Run plan</button>
        : <button onClick={() => terminatePlan = true}>STOP plan</button>
       }
      <button onClick={() => setPlan([])}>Clear plan</button>
      <hr />

      <div className="plan-step-container">
        Planned steps:<br />
        {plan.map((step, i) =>
          <span
            className={"plan-step" + (i == runningIndex ? ' active' : '')}
            key={i}>
            {i + '. ' + step}
          </span>
        )}
      </div>
    </div>
  );
}
