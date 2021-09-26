import React, { useState, useRef, useEffect } from 'react';

const ANNOTATION_INTERVAL_MS = 300;

const getCameraFeedElement = () => document.getElementById('camera-feed');


let bodyPixModel = null;
async function performSegmentation(imgElementInput, model = 'ResNet50') {
  if (!bodyPixModel) {
    bodyPixModel = await window.bodyPix.load({ architecture: model });
  }

  //const newPredictions = [await bodyPixModel.segmentPerson(cameraFeedElement)];
  return await bodyPixModel.segmentMultiPerson(imgElementInput);
};

function drawSegment({y: ay, x: ax}, {y: by, x: bx}, ctx, scale = 1) {
  ctx.beginPath();
  ctx.moveTo(ax * scale, ay * scale);
  ctx.lineTo(bx * scale, by * scale);

  ctx.lineWidth = 3;
  ctx.strokeStyle = 'rgba(0, 255, 100, 0.5)';
  ctx.stroke();
}

function drawSkeleton(keypoints, ctx, minConfidence = 0.1) {
  const adjacentKeyPoints = window.posenet.getAdjacentKeyPoints(keypoints, minConfidence);
  adjacentKeyPoints.forEach(keypoints =>
    drawSegment(keypoints[0].position, keypoints[1].position, ctx)
  );
}

export default function BodyPixAnnotations({ enabled }) {
  const [predictions, setPredictions] = useState([]);
  const [canvasStyle, setCanvasStyle] = useState({});
  const detectionTimeoutRef = useRef();
  const canvasRef = useRef();

  useEffect(() => {
    const predict = async () => {
      if (!enabled) { return; }

      const cameraFeedElement = getCameraFeedElement();
      if (cameraFeedElement && cameraFeedElement.complete) {
        const cameraImageRect = cameraFeedElement.getBoundingClientRect();
        setCanvasStyle({
          width: cameraImageRect.width,
          height: cameraImageRect.height,
          top: cameraImageRect.top,
          left: cameraImageRect.left
        });

        setPredictions(await performSegmentation(cameraFeedElement));
      }

      detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    };

    // use timeout instead of interval to make sure we don't fall behind on annotating
    detectionTimeoutRef.current = setTimeout(predict, ANNOTATION_INTERVAL_MS);
    return () => clearTimeout(detectionTimeoutRef.current);
  }, [enabled]);


  useEffect(() => {
    if (!canvasStyle.width || !enabled) {
      return;
    }
    const canvas = canvasRef.current;
    canvas.width = canvasStyle.width;
    canvas.height = canvasStyle.height;
    const ctx = canvas.getContext('2d');

    predictions.forEach(prediction => drawSkeleton(prediction.pose.keypoints, ctx));

    return () => ctx.clearRect(0, 0, canvas.width, canvas.height);
  }, [predictions, canvasStyle, enabled]);

  return <canvas ref={canvasRef} id="bodypix-mask-canvas" style={canvasStyle}></canvas>;
}