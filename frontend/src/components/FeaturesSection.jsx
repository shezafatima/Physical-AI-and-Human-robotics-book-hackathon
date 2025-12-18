import React from 'react';
import clsx from 'clsx';
import styles from './FeaturesSection.module.css';

const FeaturesSection = () => {
  const features = [
    {
      title: 'Physical AI Fundamentals',
      description: 'Learn the core concepts of Physical AI and how it applies to humanoid robotics systems.'
    },
    {
      title: 'ROS 2 & Simulation',
      description: 'Master ROS 2 fundamentals and robot simulation with Gazebo and Unity environments.'
    },
    {
      title: 'AI Integration',
      description: 'Implement Vision-Language-Action systems and conversational AI for robotics.'
    }
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, index) => (
            <div key={index} className={clsx('col col--4', styles.featureCard)}>
              <div className="card">
                <div className="card__body">
                  <h3>{feature.title}</h3>
                  <p>{feature.description}</p>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default FeaturesSection;