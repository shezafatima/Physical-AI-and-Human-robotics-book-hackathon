import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './Interactive3DViewer.module.css';

const Interactive3DViewer = ({ modelData, className }) => {
  const [rotation, setRotation] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [lastMousePosition, setLastMousePosition] = useState({ x: 0, y: 0 });
  const [selectedPart, setSelectedPart] = useState(null);
  const [animationStep, setAnimationStep] = useState(0);
  const containerRef = useRef(null);

  const handleMouseDown = (e) => {
    setIsDragging(true);
    setLastMousePosition({ x: e.clientX, y: e.clientY });
  };

  const handleMouseMove = (e) => {
    if (!isDragging) return;

    const deltaX = e.clientX - lastMousePosition.x;
    const deltaY = e.clientY - lastMousePosition.y;

    setRotation(prev => ({
      x: Math.max(-90, Math.min(90, prev.x - deltaY * 0.5)),
      y: prev.y + deltaX * 0.5
    }));

    setLastMousePosition({ x: e.clientX, y: e.clientY });
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handlePartClick = (partId) => {
    setSelectedPart(selectedPart === partId ? null : partId);
  };

  // Animation for robotic movements
  useEffect(() => {
    if (selectedPart) {
      const interval = setInterval(() => {
        setAnimationStep(prev => (prev + 1) % 100);
      }, 50);
      return () => clearInterval(interval);
    }
  }, [selectedPart]);

  const getPartStyle = (part) => {
    const isSelected = selectedPart === part.id;
    // Calculate position based on 3D rotation
    const position = calculatePartPosition(part, { x: rotation.x, y: rotation.y });

    return {
      transform: `translate3d(${position.x || 0}px, ${position.y || 0}px, ${position.z || 0}px)`,
      backgroundColor: isSelected ? part.selectedColor || '#667eea' : part.color || '#764ba2',
      opacity: isSelected ? 0.9 : 0.8,
      transition: 'all 0.3s ease',
      animation: isSelected ? `pulse 2s infinite` : 'none',
      position: 'absolute',
      left: part.style?.left || '50%',
      top: part.style?.top || '50%',
      width: part.style?.width || '40px',
      height: part.style?.height || '40px',
      borderRadius: part.style?.borderRadius || '50%',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      color: 'white',
      fontWeight: 'bold',
      fontSize: '12px',
      cursor: 'pointer',
      zIndex: isSelected ? 100 : 10
    };
  };

  const calculatePartPosition = (part, rotation) => {
    // Simple 3D projection based on rotation
    const centerX = 150; // Half of modelContainer width
    const centerY = 200; // Half of modelContainer height

    // Base positions
    const baseX = part.style?.left ? parseInt(part.style.left) - centerX : 0;
    const baseY = part.style?.top ? parseInt(part.style.top) - centerY : 0;
    const baseZ = part.style?.zIndex || 0;

    // Apply rotation transformations
    const radX = rotation.x * Math.PI / 180;
    const radY = rotation.y * Math.PI / 180;

    // Simple 3D rotation projection
    let projX = baseX * Math.cos(radY) - baseZ * Math.sin(radY);
    let projY = baseY * Math.cos(radX) - baseZ * Math.sin(radX);
    let projZ = baseX * Math.sin(radY) + baseZ * Math.cos(radY);

    // Scale based on Z for depth perception
    const scale = 1 + (projZ / 1000);

    return {
      x: projX * scale,
      y: projY * scale,
      z: projZ
    };
  };

  return (
    <div
      className={clsx(styles.viewerContainer, className)}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
      ref={containerRef}
    >
      <div className={styles.viewerHeader}>
        <h3>{modelData.title || 'Interactive 3D Model'}</h3>
        <p>{modelData.description || 'Click and drag to rotate the model'}</p>
      </div>

      <div className={styles.viewerScene}>
        <div
          className={styles.modelContainer}
          onMouseDown={handleMouseDown}
          style={{
            cursor: isDragging ? 'grabbing' : 'grab',
            position: 'relative',
            width: '300px',
            height: '400px'
          }}
        >
          {modelData.parts && modelData.parts.map((part) => (
            <div
              key={part.id}
              className={clsx(
                styles.modelPart,
                selectedPart === part.id && styles.selectedPart,
                part.type && styles[part.type]
              )}
              style={getPartStyle(part)}
              onClick={() => handlePartClick(part.id)}
              title={part.label || part.id}
            >
              <div className={styles.partLabel}>{part.label || part.id}</div>
              {part.info && (
                <div className={clsx(styles.partInfo, selectedPart === part.id && styles.infoVisible)}>
                  <h4>{part.label}</h4>
                  <p>{part.info}</p>
                  {part.specs && (
                    <div className={styles.specs}>
                      {Object.entries(part.specs).map(([key, value]) => (
                        <div key={key} className={styles.spec}>
                          <strong>{key}:</strong> {value}
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )}
            </div>
          ))}
        </div>
      </div>

      <div className={styles.viewerControls}>
        <button
          className={styles.controlButton}
          onClick={() => setRotation({ x: 0, y: 0 })}
        >
          Reset View
        </button>

        <button
          className={clsx(styles.controlButton, styles.animateButton)}
          onClick={() => {
            // Trigger animation sequence
            const newRotation = {
              x: Math.random() * 60 - 30,
              y: Math.random() * 60 - 30
            };
            setRotation(newRotation);
          }}
        >
          Random View
        </button>
      </div>

      {selectedPart && modelData.parts && (
        <div className={styles.partDetails}>
          {(() => {
            const part = modelData.parts.find(p => p.id === selectedPart);
            return (
              <div>
                <h4>{part?.label || selectedPart}</h4>
                <p>{part?.info || 'No information available.'}</p>
                {part?.specs && (
                  <div className={styles.specs}>
                    {Object.entries(part.specs).map(([key, value]) => (
                      <div key={key} className={styles.spec}>
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
    </div>
  );
};

// Example model data for a humanoid robot
Interactive3DViewer.defaultProps = {
  modelData: {
    title: "Humanoid Robot 3D Model",
    description: "Click and drag to rotate. Click on parts for more information.",
    parts: [
      {
        id: 'head',
        type: 'head',
        label: 'Head Unit',
        info: 'Contains stereo cameras, microphones, and edge AI processor for real-time perception and decision making.',
        color: '#667eea',
        selectedColor: '#5a6fd8',
        specs: {
          'Weight': '2.5 kg',
          'Sensors': 'Stereo cameras, IMU, Microphones',
          'Processor': 'NVIDIA Jetson Orin',
          'DOF': '2 (pan/tilt)'
        },
        style: {
          width: '60px',
          height: '60px',
          top: '60px',
          left: '120px',
          borderRadius: '50%'
        }
      },
      {
        id: 'torso',
        type: 'torso',
        label: 'Torso',
        info: 'Main body housing power systems, main controller, and communication modules.',
        color: '#764ba2',
        selectedColor: '#684191',
        specs: {
          'Weight': '8.0 kg',
          'Battery': '48V 20Ah Li-ion',
          'Controller': 'ROS 2 based',
          'Connectors': 'Power, Data, Ethernet'
        },
        style: {
          width: '80px',
          height: '120px',
          top: '130px',
          left: '110px',
          borderRadius: '15px'
        }
      },
      {
        id: 'left-arm',
        type: 'arm',
        label: 'Left Arm',
        info: '7-DOF arm with force/torque sensors in each joint for precise manipulation.',
        color: '#667eea',
        selectedColor: '#5a6fd8',
        specs: {
          'DOF': '7 joints',
          'Gripper': 'Adaptive 2-finger',
          'Payload': '5 kg',
          'Reach': '1.2m'
        },
        style: {
          width: '20px',
          height: '140px',
          top: '140px',
          left: '80px',
          borderRadius: '10px'
        }
      },
      {
        id: 'right-arm',
        type: 'arm',
        label: 'Right Arm',
        info: '7-DOF arm with force/torque sensors in each joint for precise manipulation.',
        color: '#667eea',
        selectedColor: '#5a6fd8',
        specs: {
          'DOF': '7 joints',
          'Gripper': 'Adaptive 2-finger',
          'Payload': '5 kg',
          'Reach': '1.2m'
        },
        style: {
          width: '20px',
          height: '140px',
          top: '140px',
          left: '200px',
          borderRadius: '10px'
        }
      },
      {
        id: 'left-leg',
        type: 'leg',
        label: 'Left Leg',
        info: '6-DOF leg with 6-axis force/torque sensors for stable locomotion.',
        color: '#764ba2',
        selectedColor: '#684191',
        specs: {
          'DOF': '6 joints',
          'Foot': 'Force sensing',
          'Range': '±15° pitch/roll',
          'Max Load': '100 kg'
        },
        style: {
          width: '25px',
          height: '160px',
          top: '260px',
          left: '115px',
          borderRadius: '10px 10px 5px 5px'
        }
      },
      {
        id: 'right-leg',
        type: 'leg',
        label: 'Right Leg',
        info: '6-DOF leg with 6-axis force/torque sensors for stable locomotion.',
        color: '#764ba2',
        selectedColor: '#684191',
        specs: {
          'DOF': '6 joints',
          'Foot': 'Force sensing',
          'Range': '±15° pitch/roll',
          'Max Load': '100 kg'
        },
        style: {
          width: '25px',
          height: '160px',
          top: '260px',
          left: '160px',
          borderRadius: '10px 10px 5px 5px'
        }
      }
    ]
  }
};

export default Interactive3DViewer;