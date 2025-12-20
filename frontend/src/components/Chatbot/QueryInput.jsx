import React, { useState } from 'react';
import { getSelectedText, sanitizeSelectedText } from '../../utils/textSelection';

/**
 * QueryInput component - Input field for user queries
 */
const QueryInput = ({ onSendMessage }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim()) {
      // Get selected text and sanitize it
      const selectedText = getSelectedText();
      const sanitizedSelectedText = selectedText ? sanitizeSelectedText(selectedText) : null;

      onSendMessage(inputValue, sanitizedSelectedText);
      setInputValue('');
    }
  };

  return (
    <form onSubmit={handleSubmit} className="query-input-form">
      <div style={{
        display: 'flex',
        alignItems: 'center',
        border: '1px solid #ced4da',
        borderRadius: '25px',
        padding: '0.25rem 1rem',
        backgroundColor: 'white'
      }}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the course content..."
          style={{
            flex: 1,
            border: 'none',
            outline: 'none',
            padding: '0.75rem 0',
            fontSize: '1rem',
            backgroundColor: 'transparent'
          }}
          onKeyPress={(e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
              e.preventDefault();
              handleSubmit(e);
            }
          }}
        />
        <button
          type="submit"
          disabled={!inputValue.trim()}
          style={{
            background: 'none',
            border: 'none',
            cursor: inputValue.trim() ? 'pointer' : 'default',
            opacity: inputValue.trim() ? 1 : 0.5,
            padding: '0.5rem',
            borderRadius: '50%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            style={{ transform: 'rotate(-45deg)' }}
          >
            <line x1="22" y1="2" x2="11" y2="13" />
            <polygon points="22 2 15 22 11 13 2 9 22 2" />
          </svg>
        </button>
      </div>
    </form>
  );
};

export default QueryInput;