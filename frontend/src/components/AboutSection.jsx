import React from 'react';
import clsx from 'clsx';
import styles from './AboutSection.module.css';

const AboutSection = () => {
  return (
    <section className={styles.aboutSection}>
      <div className="container padding-horiz--md">
        <div className="row">
          <div className="col col--8 col--offset--2">
            <h2>About This Coursebook</h2>
            <p>
              This interactive coursebook combines comprehensive theory with practical examples,
              diagrams, code samples, and exercises to reinforce your learning in Physical AI & Humanoid Robotics.
            </p>
            <p>
              Each chapter includes hands-on exercises and real-world applications to help you
              master the concepts and apply them to actual robotic systems.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
};

export default AboutSection;