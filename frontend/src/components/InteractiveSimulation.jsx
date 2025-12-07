import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './InteractiveSimulation.module.css';

const InteractiveSimulation = ({ simulationData, className }) => {
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [simulationSpeed, setSimulationSpeed] = useState(1);
  const [particles, setParticles] = useState([]);
  const intervalRef = useRef(null);
  const canvasRef = useRef(null);

  // Initialize simulation
  useEffect(() => {
    if (isPlaying) {
      intervalRef.current = setInterval(() => {
        setCurrentTime(prev => {
          const newTime = prev + 0.1 * simulationSpeed;
          return newTime > (simulationData.duration || 10) ? 0 : newTime;
        });

        // Generate particles for visual effects
        if (Math.random() > 0.7) {
          setParticles(prev => [
            ...prev.slice(-20), // Keep only last 20 particles
            {
              id: Date.now(),
              x: Math.random() * 100,
              y: Math.random() * 100,
              life: 1,
              type: Math.random() > 0.5 ? 'sensor' : 'data'
            }
          ]);
        }
      }, 100);
    } else {
      clearInterval(intervalRef.current);
    }

    return () => clearInterval(intervalRef.current);
  }, [isPlaying, simulationSpeed, simulationData.duration]);

  // Update particles
  useEffect(() => {
    const particleInterval = setInterval(() => {
      setParticles(prev =>
        prev.map(p => ({ ...p, life: p.life - 0.02 }))
           .filter(p => p.life > 0)
      );
    }, 50);

    return () => clearInterval(particleInterval);
  }, []);

  const togglePlay = () => {
    setIsPlaying(!isPlaying);
  };

  const resetSimulation = () => {
    setIsPlaying(false);
    setCurrentTime(0);
    setParticles([]);
  };

  const calculateRobotPosition = (robot, time) => {
    if (!robot.path) return { x: robot.startX || 50, y: robot.startY || 50 };

    // Simple animation along a path
    const progress = (time % (simulationData.duration || 10)) / (simulationData.duration || 10);
    const path = robot.path;

    if (path.length === 0) return { x: robot.startX || 50, y: robot.startY || 50 };

    const segmentIndex = Math.floor(progress * (path.length - 1));
    const segmentProgress = (progress * (path.length - 1)) % 1;

    if (segmentIndex >= path.length - 1) {
      return path[path.length - 1];
    }

    const start = path[segmentIndex];
    const end = path[segmentIndex + 1];

    return {
      x: start.x + (end.x - start.x) * segmentProgress,
      y: start.y + (end.y - start.y) * segmentProgress
    };
  };

  const renderEnvironment = () => {
    if (!simulationData.environment) return null;

    return (
      <div className={styles.environment}>
        {simulationData.environment.obstacles && simulationData.environment.obstacles.map((obstacle, index) => (
          <div
            key={`obstacle-${index}`}
            className={styles.obstacle}
            style={{
              left: `${obstacle.x}%`,
              top: `${obstacle.y}%`,
              width: `${obstacle.width || 5}%`,
              height: `${obstacle.height || 5}%`,
              backgroundColor: obstacle.color || '#ff6b6b'
            }}
          />
        ))}

        {simulationData.environment.targets && simulationData.environment.targets.map((target, index) => (
          <div
            key={`target-${index}`}
            className={styles.target}
            style={{
              left: `${target.x}%`,
              top: `${target.y}%`,
              width: `${target.radius || 3}%`,
              height: `${target.radius || 3}%`
            }}
          >
            <div className={styles.targetInner} />
          </div>
        ))}
      </div>
    );
  };

  const renderRobots = () => {
    if (!simulationData.robots) return null;

    return simulationData.robots.map(robot => {
      const position = calculateRobotPosition(robot, currentTime);
      const isSelected = selectedRobot === robot.id;

      return (
        <div
          key={robot.id}
          className={clsx(
            styles.robot,
            isSelected && styles.selectedRobot,
            robot.type && styles[robot.type]
          )}
          style={{
            left: `${position.x}%`,
            top: `${position.y}%`,
            backgroundColor: robot.color || '#667eea',
            transform: `translate(-50%, -50%) ${isSelected ? 'scale(1.2)' : 'scale(1)'}`,
            transition: 'all 0.3s ease'
          }}
          onClick={() => setSelectedRobot(isSelected ? null : robot.id)}
        >
          <div className={styles.robotIcon}>{robot.icon || '🤖'}</div>
          {isSelected && (
            <div className={styles.robotInfo}>
              <h4>{robot.name}</h4>
              <p>{robot.description}</p>
              <div className={styles.robotStats}>
                <div>Speed: {robot.speed || '1.0x'}</div>
                <div>Battery: {Math.max(0, 100 - currentTime * 10)}%</div>
              </div>
            </div>
          )}
        </div>
      );
    });
  };

  return (
    <div className={clsx(styles.simulationContainer, className)}>
      <div className={styles.simulationHeader}>
        <h3>{simulationData.title || 'Interactive Simulation'}</h3>
        <p>{simulationData.description || 'Click play to start the simulation'}</p>
      </div>

      <div className={styles.simulationScene}>
        <div className={styles.simulationArea}>
          {renderEnvironment()}
          {renderRobots()}

          {/* Particles for visual effects */}
          {particles.map(particle => (
            <div
              key={particle.id}
              className={clsx(styles.particle, styles[particle.type])}
              style={{
                left: `${particle.x}%`,
                top: `${particle.y}%`,
                opacity: particle.life,
                transform: `scale(${particle.life})`
              }}
            />
          ))}
        </div>

        <div className={styles.simulationTimeline}>
          <div className={styles.timelineBar}>
            <div
              className={styles.timelineProgress}
              style={{ width: `${(currentTime / (simulationData.duration || 10)) * 100}%` }}
            />
          </div>
          <div className={styles.timelineInfo}>
            Time: {currentTime.toFixed(1)}s / {(simulationData.duration || 10)}s
          </div>
        </div>
      </div>

      <div className={styles.simulationControls}>
        <button
          className={clsx(styles.controlButton, isPlaying ? styles.pauseButton : styles.playButton)}
          onClick={togglePlay}
        >
          {isPlaying ? '⏸️ Pause' : '▶️ Play'}
        </button>

        <button
          className={clsx(styles.controlButton, styles.resetButton)}
          onClick={resetSimulation}
        >
          🔄 Reset
        </button>

        <div className={styles.speedControl}>
          <label>Speed:</label>
          <select
            value={simulationSpeed}
            onChange={(e) => setSimulationSpeed(Number(e.target.value))}
            className={styles.speedSelect}
          >
            <option value={0.5}>0.5x</option>
            <option value={1}>1x</option>
            <option value={2}>2x</option>
            <option value={5}>5x</option>
          </select>
        </div>
      </div>

      {selectedRobot && simulationData.robots && (
        <div className={styles.robotDetails}>
          {(() => {
            const robot = simulationData.robots.find(r => r.id === selectedRobot);
            return (
              <div>
                <h4>{robot?.name || selectedRobot}</h4>
                <p>{robot?.description || 'No description available.'}</p>
                <div className={styles.robotSpecs}>
                  {robot?.specs && Object.entries(robot.specs).map(([key, value]) => (
                    <div key={key} className={styles.spec}>
                      <strong>{key}:</strong> {value}
                    </div>
                  ))}
                </div>
              </div>
            );
          })()}
        </div>
      )}
    </div>
  );
};

// Example simulation data for a humanoid robot navigation scenario
InteractiveSimulation.defaultProps = {
  simulationData: {
    title: "Humanoid Robot Navigation Simulation",
    description: "Simulate a humanoid robot navigating through an environment with obstacles.",
    duration: 30, // seconds
    environment: {
      obstacles: [
        { x: 20, y: 30, width: 8, height: 8, color: '#ff6b6b' },
        { x: 60, y: 40, width: 6, height: 12, color: '#ff9f43' },
        { x: 40, y: 70, width: 10, height: 6, color: '#fd79a8' },
        { x: 75, y: 20, width: 5, height: 15, color: '#fdcb6e' }
      ],
      targets: [
        { x: 85, y: 85, radius: 4 },
        { x: 15, y: 80, radius: 3 }
      ]
    },
    robots: [
      {
        id: 'humanoid1',
        name: 'Atlas-like Robot',
        description: 'Advanced humanoid robot with 28 DOF and force control.',
        icon: '🤖',
        color: '#667eea',
        startX: 10,
        startY: 10,
        speed: '1.2 m/s',
        type: 'humanoid',
        specs: {
          'Height': '1.8m',
          'Weight': '80kg',
          'DOF': '28',
          'Sensors': 'LIDAR, Cameras, Force/Torque'
        },
        path: [
          { x: 10, y: 10 },
          { x: 25, y: 25 },
          { x: 40, y: 40 },
          { x: 55, y: 50 },
          { x: 70, y: 65 },
          { x: 85, y: 85 }
        ]
      },
      {
        id: 'wheeled1',
        name: 'Wheeled Assistant',
        description: 'Mobile manipulator for object transportation.',
        icon: '🛼',
        color: '#764ba2',
        startX: 5,
        startY: 50,
        speed: '0.8 m/s',
        type: 'wheeled',
        specs: {
          'Payload': '10kg',
          'Battery': '4h',
          'Arms': '2x 7DOF',
          'Wheels': '4 omni-directional'
        },
        path: [
          { x: 5, y: 50 },
          { x: 20, y: 55 },
          { x: 35, y: 60 },
          { x: 50, y: 55 },
          { x: 65, y: 50 },
          { x: 80, y: 45 }
        ]
      }
    ]
  }
};

export default InteractiveSimulation;