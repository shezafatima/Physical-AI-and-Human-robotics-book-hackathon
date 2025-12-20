import React from 'react';
import { marked } from 'marked';

/**
 * MessageDisplay component - Displays chat messages in the conversation
 */
const MessageDisplay = ({ messages }) => {
  return (
    <div className="message-display">
      {messages.map((message) => (
        <div
          key={message.id}
          className={`message ${message.role}`}
          style={{
            display: 'flex',
            flexDirection: 'column',
            marginBottom: '1rem',
            alignItems: message.role === 'user' ? 'flex-end' : 'flex-start'
          }}
        >
          <div
            style={{
              maxWidth: '80%',
              padding: '0.75rem 1rem',
              borderRadius: '18px',
              backgroundColor: message.role === 'user' ? '#007bff' : '#f8f9fa',
              color: message.role === 'user' ? 'white' : 'black',
              wordWrap: 'break-word',
              boxShadow: '0 1px 2px rgba(0,0,0,0.1)'
            }}
          >
            <div style={{ marginBottom: '0.5rem' }}>
              {message.role === 'assistant' ? (
                <div dangerouslySetInnerHTML={{ __html: marked(message.content || '') }} />
              ) : (
                message.content
              )}
            </div>

            {message.sources && message.sources.length > 0 && (
              <div
                style={{
                  fontSize: '0.8rem',
                  color: message.role === 'user' ? 'rgba(255,255,255,0.8)' : '#6c757d',
                  marginTop: '0.5rem',
                  paddingTop: '0.5rem',
                  borderTop: message.role === 'user' ? '1px solid rgba(255,255,255,0.3)' : '1px solid #dee2e6'
                }}
              >
                <strong>Sources:</strong>
                <ul style={{ margin: '0.25rem 0 0 1rem', padding: 0 }}>
                  {message.sources.slice(0, 3).map((source, index) => (
                    <li key={index} style={{ fontSize: '0.75rem' }}>
                      {source}
                    </li>
                  ))}
                  {message.sources.length > 3 && (
                    <li style={{ fontSize: '0.75rem' }}>
                      ... and {message.sources.length - 3} more
                    </li>
                  )}
                </ul>
              </div>
            )}
          </div>

          <div
            style={{
              fontSize: '0.7rem',
              color: '#6c757d',
              marginTop: '0.25rem',
              marginRight: message.role === 'user' ? '0.5rem' : '0',
              marginLeft: message.role === 'assistant' ? '0.5rem' : '0'
            }}
          >
            {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
          </div>
        </div>
      ))}
    </div>
  );
};

export default MessageDisplay;