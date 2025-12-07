import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './InteractiveDiagram.module.css';

const InteractiveDiagram = ({ diagramData, className }) => {
  const [activeElement, setActiveElement] = useState(null);
  const [hoveredElement, setHoveredElement] = useState(null);
  const [animationState, setAnimationState] = useState({});
  const canvasRef = useRef(null);
  const containerRef = useRef(null);

  const handleElementClick = (elementId) => {
    setActiveElement(activeElement === elementId ? null : elementId);
    // Trigger animation for clicked element
    setAnimationState(prev => ({
      ...prev,
      [elementId]: !prev[elementId]
    }));
  };

  const handleElementHover = (elementId) => {
    setHoveredElement(elementId);
  };

  const handleElementLeave = () => {
    setHoveredElement(null);
  };

  // Render SVG diagram based on data
  const renderDiagram = () => {
    if (!diagramData || !diagramData.elements) return null;

    return (
      <svg
        className={styles.diagramSvg}
        viewBox={diagramData.viewBox || "0 0 800 600"}
        preserveAspectRatio="xMidYMid meet"
      >
        {/* Background elements */}
        {diagramData.background && (
          <rect
            x="0"
            y="0"
            width="100%"
            height="100%"
            fill={diagramData.background.fill || "white"}
            opacity={diagramData.background.opacity || 0.1}
          />
        )}

        {/* Connection lines */}
        {diagramData.connections && diagramData.connections.map((conn, index) => (
          <line
            key={`conn-${index}`}
            x1={conn.from.x}
            y1={conn.from.y}
            x2={conn.to.x}
            y2={conn.to.y}
            stroke={conn.color || "#667eea"}
            strokeWidth={conn.width || 2}
            strokeDasharray={conn.dashed ? "5,5" : "none"}
            className={styles.connectionLine}
          />
        ))}

        {/* Diagram elements */}
        {diagramData.elements.map((element) => {
          const isActive = activeElement === element.id;
          const isHovered = hoveredElement === element.id;
          const elementClasses = clsx(
            styles.diagramElement,
            isActive && styles.activeElement,
            isHovered && styles.hoveredElement,
            element.type && styles[element.type]
          );

          switch (element.type) {
            case 'circle':
              return (
                <circle
                  key={element.id}
                  cx={element.x}
                  cy={element.y}
                  r={element.radius || 30}
                  fill={element.fill || "#667eea"}
                  stroke={element.stroke || "#764ba2"}
                  strokeWidth={element.strokeWidth || 2}
                  className={elementClasses}
                  onClick={() => handleElementClick(element.id)}
                  onMouseEnter={() => handleElementHover(element.id)}
                  onMouseLeave={handleElementLeave}
                >
                  {element.label && (
                    <title>{element.label}</title>
                  )}
                </circle>
              );
            case 'rectangle':
              return (
                <rect
                  key={element.id}
                  x={element.x}
                  y={element.y}
                  width={element.width || 80}
                  height={element.height || 60}
                  rx={element.rx || 8}
                  fill={element.fill || "#764ba2"}
                  stroke={element.stroke || "#667eea"}
                  strokeWidth={element.strokeWidth || 2}
                  className={elementClasses}
                  onClick={() => handleElementClick(element.id)}
                  onMouseEnter={() => handleElementHover(element.id)}
                  onMouseLeave={handleElementLeave}
                >
                  {element.label && (
                    <title>{element.label}</title>
                  )}
                </rect>
              );
            case 'path':
              return (
                <path
                  key={element.id}
                  d={element.d}
                  fill={element.fill || "none"}
                  stroke={element.stroke || "#667eea"}
                  strokeWidth={element.strokeWidth || 2}
                  className={elementClasses}
                  onClick={() => handleElementClick(element.id)}
                  onMouseEnter={() => handleElementHover(element.id)}
                  onMouseLeave={handleElementLeave}
                >
                  {element.label && (
                    <title>{element.label}</title>
                  )}
                </path>
              );
            default:
              return (
                <circle
                  key={element.id}
                  cx={element.x}
                  cy={element.y}
                  r={element.radius || 25}
                  fill={element.fill || "#667eea"}
                  stroke={element.stroke || "#764ba2"}
                  strokeWidth={element.strokeWidth || 2}
                  className={elementClasses}
                  onClick={() => handleElementClick(element.id)}
                  onMouseEnter={() => handleElementHover(element.id)}
                  onMouseLeave={handleElementLeave}
                >
                  {element.label && (
                    <title>{element.label}</title>
                  )}
                </circle>
              );
          }
        })}

        {/* Labels */}
        {diagramData.labels && diagramData.labels.map((label, index) => (
          <text
            key={`label-${index}`}
            x={label.x}
            y={label.y}
            fontSize={label.fontSize || "14px"}
            fill={label.color || "#333"}
            textAnchor={label.anchor || "middle"}
            className={styles.diagramLabel}
          >
            {label.text}
          </text>
        ))}
      </svg>
    );
  };

  return (
    <div className={clsx(styles.interactiveDiagram, className)} ref={containerRef}>
      <div className={styles.diagramContainer}>
        {renderDiagram()}
      </div>

      {/* Element information panel */}
      {activeElement && diagramData.elements && (
        <div className={styles.infoPanel}>
          {(() => {
            const element = diagramData.elements.find(el => el.id === activeElement);
            return (
              <div>
                <h4>{element?.label || activeElement}</h4>
                <p>{element?.description || 'No description available.'}</p>
                {element?.properties && (
                  <div className={styles.properties}>
                    {Object.entries(element.properties).map(([key, value]) => (
                      <div key={key} className={styles.property}>
                        <strong>{key}:</strong> {value}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            );
          })()}
        </div>
      )}

      {/* Controls */}
      <div className={styles.controls}>
        <button
          className={clsx(styles.controlButton, styles.resetButton)}
          onClick={() => {
            setActiveElement(null);
            setAnimationState({});
          }}
        >
          Reset Diagram
        </button>
        {diagramData.animationControls && (
          <button
            className={clsx(styles.controlButton, styles.animateButton)}
            onClick={() => {
              // Trigger animation sequence
              const newAnimationState = {};
              diagramData.elements.forEach(element => {
                newAnimationState[element.id] = Math.random() > 0.5;
              });
              setAnimationState(newAnimationState);
            }}
          >
            Animate
          </button>
        )}
      </div>
    </div>
  );
};

// Example diagram data for a humanoid robot system
InteractiveDiagram.defaultProps = {
  diagramData: {
    viewBox: "0 0 800 600",
    background: {
      fill: "linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%)",
      opacity: 0.3
    },
    elements: [
      {
        id: 'head',
        type: 'circle',
        x: 400,
        y: 150,
        radius: 40,
        fill: '#667eea',
        stroke: '#764ba2',
        label: 'Head Unit',
        description: 'Contains cameras, microphones, and processing units for perception.',
        properties: {
          'Sensors': '2x RGB cameras, Microphone array',
          'CPU': 'NVIDIA Jetson Orin',
          'Weight': '2.5 kg'
        }
      },
      {
        id: 'torso',
        type: 'rectangle',
        x: 360,
        y: 200,
        width: 80,
        height: 120,
        fill: '#764ba2',
        stroke: '#667eea',
        label: 'Torso',
        description: 'Main body containing power systems and main controller.',
        properties: {
          'Battery': '48V 20Ah Li-ion',
          'Controller': 'ROS 2 based',
          'Weight': '8.0 kg'
        }
      },
      {
        id: 'left-arm',
        type: 'path',
        d: 'M 360 220 Q 300 250 320 300 Q 340 350 360 380',
        stroke: '#667eea',
        strokeWidth: 8,
        label: 'Left Arm',
        description: '7-DOF arm with force/torque sensors in each joint.',
        properties: {
          'DOF': '7 joints',
          'Gripper': 'Adaptive 2-finger',
          'Payload': '5 kg'
        }
      },
      {
        id: 'right-arm',
        type: 'path',
        d: 'M 440 220 Q 500 250 480 300 Q 460 350 440 380',
        stroke: '#667eea',
        strokeWidth: 8,
        label: 'Right Arm',
        description: '7-DOF arm with force/torque sensors in each joint.',
        properties: {
          'DOF': '7 joints',
          'Gripper': 'Adaptive 2-finger',
          'Payload': '5 kg'
        }
      },
      {
        id: 'left-leg',
        type: 'path',
        d: 'M 370 320 Q 360 400 370 480',
        stroke: '#764ba2',
        strokeWidth: 10,
        label: 'Left Leg',
        description: '6-DOF leg with 6-axis force/torque sensors.',
        properties: {
          'DOF': '6 joints',
          'Foot': 'Force sensing',
          'Range': '±15° pitch/roll'
        }
      },
      {
        id: 'right-leg',
        type: 'path',
        d: 'M 430 320 Q 440 400 430 480',
        stroke: '#764ba2',
        strokeWidth: 10,
        label: 'Right Leg',
        description: '6-DOF leg with 6-axis force/torque sensors.',
        properties: {
          'DOF': '6 joints',
          'Foot': 'Force sensing',
          'Range': '±15° pitch/roll'
        }
      }
    ],
    connections: [
      { from: { x: 400, y: 190 }, to: { x: 400, y: 200 }, color: '#333', width: 2 },
      { from: { x: 360, y: 260 }, to: { x: 320, y: 300 }, color: '#333', width: 1, dashed: true },
      { from: { x: 440, y: 260 }, to: { x: 480, y: 300 }, color: '#333', width: 1, dashed: true }
    ],
    labels: [
      { x: 400, y: 100, text: 'Humanoid Robot Architecture', fontSize: '18px', anchor: 'middle' },
      { x: 300, y: 350, text: 'Left Arm', fontSize: '12px', anchor: 'middle' },
      { x: 500, y: 350, text: 'Right Arm', fontSize: '12px', anchor: 'middle' },
      { x: 340, y: 500, text: 'Left Leg', fontSize: '12px', anchor: 'middle' },
      { x: 460, y: 500, text: 'Right Leg', fontSize: '12px', anchor: 'middle' }
    ],
    animationControls: true
  }
};

export default InteractiveDiagram;