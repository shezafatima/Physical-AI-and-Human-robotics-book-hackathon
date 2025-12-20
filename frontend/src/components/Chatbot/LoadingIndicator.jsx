import React from 'react';

/**
 * LoadingIndicator component - Shows loading state when waiting for response
 */
const LoadingIndicator = () => {
  return (
    <div style={{
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'flex-start',
      padding: '1rem 0',
      marginLeft: '1rem'
    }}>
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: '0.5rem',
        padding: '0.5rem 1rem',
        backgroundColor: '#f8f9fa',
        borderRadius: '18px',
        boxShadow: '0 1px 2px rgba(0,0,0,0.1)'
      }}>
        <div style={{
          display: 'flex',
          gap: '0.25rem'
        }}>
          <div style={{
            width: '8px',
            height: '8px',
            backgroundColor: '#007bff',
            borderRadius: '50%',
            animation: 'bounce 1.5s infinite'
          }} />
          <div style={{
            width: '8px',
            height: '8px',
            backgroundColor: '#007bff',
            borderRadius: '50%',
            animation: 'bounce 1.5s infinite',
            animationDelay: '0.2s'
          }} />
          <div style={{
            width: '8px',
            height: '8px',
            backgroundColor: '#007bff',
            borderRadius: '50%',
            animation: 'bounce 1.5s infinite',
            animationDelay: '0.4s'
          }} />
        </div>
        <span style={{ marginLeft: '0.5rem', fontSize: '0.9rem', color: '#6c757d' }}>
          Thinking...
        </span>
      </div>
    </div>
  );
};

export default LoadingIndicator;