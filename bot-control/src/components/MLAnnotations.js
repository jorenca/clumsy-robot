import React, { useState } from 'react';

let model = null;
const loadModel = () => window.cocoSsd.load().then(function (loadedModel) {
  model = loadedModel;
  annotate();
});

const annotate = async () => {
  // Now let's start classifying a frame in the stream.
  const predictions = await model.detect(document.getElementById('camera-feed'));
  console.log(predictions);


  // Now lets loop through predictions and draw them to the live view if
  // they have a high confidence score.
//  for (let n = 0; n < predictions.length; n++) {
//    // If we are over 66% sure we are sure we classified it right, draw it!
//    if (predictions[n].score < 0.66) continue;
//
//    const p = document.createElement('p');
//    p.innerText = predictions[n].class  + ' - with '
//        + Math.round(parseFloat(predictions[n].score) * 100)
//        + '% confidence.';
//    p.style = 'margin-left: ' + predictions[n].bbox[0] + 'px; margin-top: '
//        + (predictions[n].bbox[1] - 10) + 'px; width: '
//        + (predictions[n].bbox[2] - 10) + 'px; top: 0; left: 0;';
//
//    const highlighter = document.createElement('div');
//    highlighter.setAttribute('class', 'highlighter');
//    highlighter.style = 'left: ' + predictions[n].bbox[0] + 'px; top: '
//        + predictions[n].bbox[1] + 'px; width: '
//        + predictions[n].bbox[2] + 'px; height: '
//        + predictions[n].bbox[3] + 'px;';
//
//    liveView.appendChild(highlighter);
//    liveView.appendChild(p);
//    children.push(highlighter);
//    children.push(p);
//  }

  // Call this function again to keep predicting when the browser is ready.
  window.requestAnimationFrame(annotate);
};

export default function MLAnnotations({ sharedTelemetry }) {
  return (
    <div className="ml-annotation-container">
      ML Annotations
      <button onClick={loadModel}>Activate</button>

    </div>
  );
}