import React from 'react';
import { ThemeProvider } from '../contexts/ThemeContext';

const Root = ({ children }) => {
  return (
    <ThemeProvider>
      {children}
    </ThemeProvider>
  );
};

export default Root;