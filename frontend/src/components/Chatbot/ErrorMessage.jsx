import React from 'react';

/**
 * ErrorMessage component - Displays error messages to the user
 */
const ErrorMessage = ({ message }) => {
  return (
    <div style={{
      padding: '1rem',
      margin: '1rem 0',
      backgroundColor: '#f8d7da',
      color: '#721c24',
      border: '1px solid #f5c6cb',
      borderRadius: '0.375rem',
      fontSize: '0.875rem'
    }}>
      <div style={{
        display: 'flex',
        alignItems: 'center'
      }}>
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          style={{ marginRight: '0.5rem', flexShrink: 0 }}
        >
          <circle cx="12" cy="12" r="10" />
          <line x1="12" y1="8" x2="12" y2="12" />
          <line x1="12" y1="16" x2="12.01" y2="16" />
        </svg>
        <span>{message}</span>
      </div>
    </div>
  );
};

export default ErrorMessage;