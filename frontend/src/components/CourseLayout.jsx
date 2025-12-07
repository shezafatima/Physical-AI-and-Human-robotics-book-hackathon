import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CourseLayout.module.css';
import Chatbot from './Chatbot';

const CourseLayout = ({ children, title, description, showChatbot = true, className }) => {
  const [isChatbotOpen, setIsChatbotOpen] = useState(true);
  const [sidebarOpen, setSidebarOpen] = useState(false);

  const toggleChatbot = () => {
    setIsChatbotOpen(!isChatbotOpen);
  };

  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };

  return (
    <div className={clsx(styles.courseLayout, className)}>
      {/* Header */}
      <header className={styles.header}>
        <div className={styles.headerContent}>
          <button className={styles.menuButton} onClick={toggleSidebar}>
            <span>☰</span>
          </button>
          <div className={styles.headerText}>
            <h1>{title}</h1>
            {description && <p>{description}</p>}
          </div>
        </div>
      </header>

      {/* Main content area */}
      <div className={styles.mainContainer}>
        {/* Sidebar */}
        <aside className={clsx(styles.sidebar, sidebarOpen && styles.sidebarOpen)}>
          <div className={styles.sidebarContent}>
            <button className={styles.closeSidebar} onClick={toggleSidebar}>
              ×
            </button>
            <h3>Course Navigation</h3>
            <nav className={styles.navMenu}>
              <a href="/docs/intro" className={styles.navLink}>Introduction</a>
              <a href="/docs/chapters/chapter1" className={styles.navLink}>Chapter 1</a>
              <a href="/docs/chapters/chapter2" className={styles.navLink}>Chapter 2</a>
              <a href="/docs/chapters/chapter3" className={styles.navLink}>Chapter 3</a>
              <a href="/docs/chapters/chapter4" className={styles.navLink}>Chapter 4</a>
              <a href="/docs/chapters/chapter5" className={styles.navLink}>Chapter 5</a>
              <a href="/docs/chapters/chapter6" className={styles.navLink}>Chapter 6</a>
              <a href="/docs/chapters/chapter7" className={styles.navLink}>Chapter 7</a>
              <a href="/docs/chapters/chapter8" className={styles.navLink}>Chapter 8</a>
              <a href="/docs/advanced/ros2" className={styles.navLink}>ROS 2</a>
              <a href="/docs/advanced/simulation" className={styles.navLink}>Simulation</a>
              <a href="/docs/advanced/humanoids" className={styles.navLink}>Humanoids</a>
            </nav>
          </div>
        </aside>

        {/* Overlay for mobile sidebar */}
        {sidebarOpen && (
          <div className={styles.sidebarOverlay} onClick={toggleSidebar}></div>
        )}

        {/* Main content */}
        <main className={styles.mainContent}>
          {children}
        </main>

        {/* Floating Chatbot */}
        {showChatbot && (
          <div className={clsx(styles.chatbotContainer, isChatbotOpen && styles.chatbotOpen)}>
            <button className={styles.chatbotToggle} onClick={toggleChatbot}>
              {isChatbotOpen ? '×' : '🤖'}
            </button>
            {isChatbotOpen && <Chatbot />}
          </div>
        )}
      </div>

      {/* Progress bar at the bottom */}
      <div className={styles.progressBarContainer}>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: '45%' }}></div>
        </div>
        <span className={styles.progressText}>45% Complete</span>
      </div>
    </div>
  );
};

export default CourseLayout;