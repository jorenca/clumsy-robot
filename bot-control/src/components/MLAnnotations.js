import React, { useState, useRef, useEffect } from 'react';
import './MLAnnotations.scss';

const ANNOTATION_INTERVAL_MS = 100;
let model = null;

const getCameraFeedElement = () => document.getElementById('camera-feed');

export default function MLAnnotations({ sharedTelemetry }) {
  const [predictions, setPredictions] = useState([]);

  const detectionTimeoutRef = useRef();
  const predict = async () => {
    if (!model) {
      model = await window.cocoSsd.load();
    }
    const newPredictions = await model.detect(getCameraFeedElement());
    setPredictions(newPredictions);

    detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
  };

  React.useEffect(() => {
    detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    return () => clearTimeout(detectionTimeoutRef.current);
  }, []); // Make sure the effect runs only once

  const createAnnotationElements = prediction => {
    const { bbox, score } = prediction;

    const imageHeight = getCameraFeedElement().height;
    const annotationElementStyle = {
      left: bbox[0],
      top: bbox[1],
      width: bbox[2],
      height: Math.min(bbox[3], imageHeight - bbox[1] - 5) // make sure annotation doesn't go outside image
    };

    return (
      <div className="object-annotation" style={annotationElementStyle}>
        <p className="annotation-title">
          {prediction.class} ({Math.round(parseFloat(score) * 100)}% confidence)
        </p>
      </div>
    );
  };

  return (
    <div className="ml-annotation-container">
      {
        predictions
            .filter(p => p.score > 0.60)
            .map(createAnnotationElements)
      }
    </div>
  );
}