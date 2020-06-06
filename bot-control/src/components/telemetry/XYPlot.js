import React from 'react';
import './XYPlot.scss';

class XYPlot extends React.Component {
  static defaultProps = {
    width: 120, // px
    height: 120, // px
    x: 0,
    y: 0,
    bound: 100,
    markerSize: 20 // px
  };

  calcPxOffset({ width, height, bound, x, y }) {
    return {
      top: Math.floor((height / 2) * (1 - y / bound)),
      left: Math.floor((width / 2) * (1 + x / bound))
    }
  }

  // TODO is an optimization here needed?
  // shouldComponentUpdate(nextProps) {
  //   const epsilon = 2;
  //   const currOff = this.calcPxOffset(this.props);
  //   const nextOff = this.calcPxOffset(nextProps);
  // }

  render() {
    const { width, height, markerSize } = this.props;
    const pxOff = this.calcPxOffset(this.props);
    const top = pxOff.top - markerSize / 2;
    const left = pxOff.left - markerSize / 2;
    return (
      <div className="xyplot-container" style={{ width: width + 'px', height: height + 'px' }}>
        <div className="xyplot-marker" style={{
          top: top + 'px',
          left: left + 'px',
          width: markerSize + 'px',
          height: markerSize + 'px',
          borderRadius: markerSize / 2 + 'px'
        }}/>
        <div className="xyplot-midpoint" style={{
          top: height / 2 - 5,
          left: height / 2 - 5,
        }} />
      </div>
    );
  }
}

export default XYPlot;
