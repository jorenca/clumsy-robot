import React, { useState, useRef, useEffect } from 'react';
import './MLAnnotations.scss';

const ANNOTATION_INTERVAL_MS = 120;
let model = null;

const getCameraFeedElement = () => document.getElementById('camera-feed');



function CocoSsdAnnotations({ enabled }) {
  const [predictions, setPredictions] = useState([]);
  const detectionTimeoutRef = useRef();

  useEffect(() => {
    const predict = async () => {
      if (enabled) {
        if (!model) {
          model = await window.cocoSsd.load();
        }
        setPredictions(await model.detect(getCameraFeedElement()));
      }

      detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    };

    detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    return () => clearTimeout(detectionTimeoutRef.current);
  }, [enabled]);

  const cameraFeedElement = getCameraFeedElement();
  if (!enabled || !cameraFeedElement || !cameraFeedElement.complete) {
    return null;
  }

  const createAnnotationElements = (prediction, index) => {
    const { bbox, score } = prediction;

    const annotationElementStyle = {
      left: bbox[0],
      top: Math.max(bbox[1], 0),
      width: bbox[2],
      height: Math.min(bbox[3], cameraFeedElement.height - bbox[1] - 5) // make sure annotation doesn't go outside image
    };

    return (
      <div className="object-annotation" style={annotationElementStyle} key={index}>
        <p className="annotation-title">
          {prediction.class} ({Math.round(parseFloat(score) * 100)}% confidence)
        </p>
      </div>
    );
  };

  return predictions
            .filter(p => p.score > 0.60)
            .map(createAnnotationElements);
}

export default function MLAnnotations() {
  const [cocoModelEnabled, setCocoModelEnabled] = useState(false);


  return (
    <div className="ml-annotation-container">
      <div className="ml-activate-buttons">
        <button onClick={() => setCocoModelEnabled(!cocoModelEnabled)}>
          {cocoModelEnabled ? 'Disable' : 'Enable'} CocoSsd detection
        </button>
        <button>Enable person detection</button>
      </div>
      <CocoSsdAnnotations enabled={cocoModelEnabled} />
    </div>
  );
}