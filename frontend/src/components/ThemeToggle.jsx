import React from 'react';
import { useTheme } from '../contexts/ThemeContext';
import clsx from 'clsx';
import styles from './ThemeToggle.module.css';

const ThemeToggle = ({ className }) => {
  const { theme, toggleTheme } = useTheme();

  return (
    <div className={clsx(styles.themeToggle, className)}>
      <button
        onClick={toggleTheme}
        className={styles.themeButton}
        aria-label={`Switch to ${theme === 'light' ? 'dark' : 'light'} theme`}
      >
        {theme === 'light' ? '🌙' : '☀️'}
      </button>
      <span className={styles.themeLabel}>
        {theme === 'light' ? 'Dark Mode' : 'Light Mode'}
      </span>
    </div>
  );
};

export default ThemeToggle;