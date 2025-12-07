import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ThemeProvider, useTheme } from '../contexts/ThemeContext';
import ThemeToggle from '../components/ThemeToggle';
import clsx from 'clsx';
import styles from './Layout.module.css';

const LayoutWithTheme = (props) => {
  const { theme } = useTheme();

  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
      </OriginalLayout>
      <div className={styles.themeToggleContainer}>
        <ThemeToggle />
      </div>
    </>
  );
};

const Layout = (props) => {
  return (
    <ThemeProvider>
      <LayoutWithTheme {...props} />
    </ThemeProvider>
  );
};

export default Layout;