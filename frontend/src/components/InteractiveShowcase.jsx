import React from 'react';
import clsx from 'clsx';
import InteractiveDiagram from './InteractiveDiagram';
import Interactive3DViewer from './Interactive3DViewer';
import InteractiveSimulation from './InteractiveSimulation';
import styles from './InteractiveShowcase.module.css';

const InteractiveShowcase = () => {
  return (
    <section className={styles.interactiveShowcase}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>Interactive Learning Experience</h2>
            <p>Explore Physical AI concepts through interactive diagrams, 3D models, and simulations.</p>
          </div>
        </div>

        <div className="row">
          <div className="col col--12">
            <div className={styles.interactiveDiagram}>
              <InteractiveDiagram />
            </div>
          </div>
        </div>

        <div className="row">
          <div className="col col--6">
            <div className={styles.interactive3d}>
              <Interactive3DViewer />
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.interactiveSimulation}>
              <InteractiveSimulation />
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default InteractiveShowcase;