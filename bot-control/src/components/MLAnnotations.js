import React, { useState } from 'react';
import './MLAnnotations.scss';

import CocoSsdAnnotations from './ml-detection/CocoSsdAnnotations';
import BodyPixAnnotations from './ml-detection/BodyPixAnnotations';


export default function MLAnnotations() {
  const [cocoModelEnabled, setCocoModelEnabled] = useState(false);
  const [bodyPixModelEnabled, setBodyPixModelEnabled] = useState(false);

  return (
    <div className="ml-annotation-container">
      <div className="ml-activate-buttons">
        <button onClick={() => setCocoModelEnabled(!cocoModelEnabled)}>
          {cocoModelEnabled ? 'Disable' : 'Enable'} CocoSsd detection
        </button>
        <button onClick={() => setBodyPixModelEnabled(!bodyPixModelEnabled)}>
          {bodyPixModelEnabled ? 'Disable' : 'Enable'} BodyPix detection
        </button>
      </div>
      <CocoSsdAnnotations enabled={cocoModelEnabled} />
      <BodyPixAnnotations enabled={bodyPixModelEnabled} />
    </div>
  );
}