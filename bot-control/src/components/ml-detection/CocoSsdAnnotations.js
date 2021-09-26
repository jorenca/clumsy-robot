import React, { useState, useRef, useEffect } from 'react';

const ANNOTATION_INTERVAL_MS = 120;
let cocoModel = null;

const getCameraFeedElement = () => document.getElementById('camera-feed');

const predictionBoxToDivStyle = (bbox, maxY) => ({
  left: bbox[0],
  top: Math.max(bbox[1], 0),
  width: bbox[2],
  height: Math.min(bbox[3], maxY - bbox[1] - 5) // make sure annotation doesn't go outside image
});

export default function CocoSsdAnnotations({ enabled }) {
  const [predictions, setPredictions] = useState([]);
  const [inputSize, setInputSize] = useState({});
  const detectionTimeoutRef = useRef();

  useEffect(() => {
    const predict = async () => {
      if (!enabled) {
        setPredictions([]);
        return;
      }
      if (!cocoModel) {
        cocoModel = await window.cocoSsd.load();
      }

      const cameraFeedElement = getCameraFeedElement();
      if (cameraFeedElement && cameraFeedElement.complete) {
        const cameraImageRect = cameraFeedElement.getBoundingClientRect();
        setInputSize({
          width: cameraImageRect.width,
          height: cameraImageRect.height
        });

        setPredictions(await cocoModel.detect(cameraFeedElement));
      }

      detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    };

    detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    return () => clearTimeout(detectionTimeoutRef.current);
  }, [enabled]);

  return predictions.filter(p => p.score > 0.60)
    .map((prediction, index) => (
      <div
          className="object-annotation"
          style={predictionBoxToDivStyle(prediction.bbox, inputSize.height)}
          key={index}>
        <p className="annotation-title">
          {prediction.class} ({Math.round(parseFloat(prediction.score) * 100)}% confidence)
        </p>
      </div>
    ));
}