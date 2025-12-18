import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './HeroSection.module.css';

const HeroSection = () => {
  const {siteConfig} = useDocusaurusContext();

  return (
    <section className={styles.heroSection}>
      <div className={styles.heroBanner}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master Physical AI & Humanoid Robotics with our interactive coursebook featuring AI-powered assistance,
              3D simulations, and hands-on projects.
            </p>
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
          <div className={styles.heroImage}>
            <img
              src="/img/hero.png"
              alt="Physical AI & Humanoid Robotics"
              className={styles.heroImg}
            />
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;