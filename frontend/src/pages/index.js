import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import InteractiveDiagram from '../components/InteractiveDiagram';
import Interactive3DViewer from '../components/Interactive3DViewer';
import InteractiveSimulation from '../components/InteractiveSimulation';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg', styles.animatedButton)}
            to="/docs/intro">
            Start Learning - 5 min ⏱️
          </Link>
          <Link
            className={clsx('button button--outline button--lg', styles.animatedButton)}
            to="/docs/chapters/chapter1">
            Explore Chapters
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--4', styles.featureCard)}>
            <div className="card">
              <div className="card__body">
                <h3>Physical AI Fundamentals</h3>
                <p>Learn the core concepts of Physical AI and how it applies to humanoid robotics systems.</p>
              </div>
            </div>
          </div>
          <div className={clsx('col col--4', styles.featureCard)}>
            <div className="card">
              <div className="card__body">
                <h3>ROS 2 & Simulation</h3>
                <p>Master ROS 2 fundamentals and robot simulation with Gazebo and Unity environments.</p>
              </div>
            </div>
          </div>
          <div className={clsx('col col--4', styles.featureCard)}>
            <div className="card">
              <div className="card__body">
                <h3>AI Integration</h3>
                <p>Implement Vision-Language-Action systems and conversational AI for robotics.</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function InteractiveShowcase() {
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
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-Native Interactive Coursebook with RAG Chatbot for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <InteractiveShowcase />
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
      </main>
    </Layout>
  );
}