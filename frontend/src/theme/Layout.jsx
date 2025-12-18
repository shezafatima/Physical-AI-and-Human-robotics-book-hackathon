import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ThemeProvider } from '../contexts/ThemeContext';
import Footer from '../components/Footer';
import ThemeToggle from '../components/ThemeToggle';

const Layout = (props) => {
  return (
    <ThemeProvider>
      <OriginalLayout {...props}>
        {/* Add theme toggle in the main layout, positioned to not conflict with navbar */}
        <div style={{ position: 'fixed', top: '80px', right: '1rem', zIndex: 10000 }}>
          <ThemeToggle />
        </div>
        {props.children}
        <Footer />
      </OriginalLayout>
    </ThemeProvider>
  );
};

export default Layout;