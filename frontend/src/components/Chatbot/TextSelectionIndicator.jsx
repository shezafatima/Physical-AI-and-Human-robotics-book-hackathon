import React, { useState, useEffect } from 'react';
import { getSelectedText, addSelectionListener } from '../../utils/textSelection';

/**
 * TextSelectionIndicator component - Shows visual feedback when user selects text
 * that can be used as context for the chatbot
 */
const TextSelectionIndicator = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState(null);
  const [showIndicator, setShowIndicator] = useState(false);
  const [indicatorPosition, setIndicatorPosition] = useState({ top: 0, left: 0 });

  useEffect(() => {
    // Add selection listener to track text selection changes
    const removeListener = addSelectionListener((selectionDetails) => {
      if (selectionDetails && selectionDetails.text) {
        setSelectedText(selectionDetails.text);
        setShowIndicator(true);

        // Position the indicator near the selection
        if (selectionDetails.rect) {
          setIndicatorPosition({
            top: selectionDetails.rect.top + window.scrollY - 40,
            left: selectionDetails.rect.left + window.scrollX + (selectionDetails.rect.width / 2)
          });
        }
      } else {
        setSelectedText(null);
        setShowIndicator(false);
      }
    });

    // Clean up the listener on unmount
    return () => {
      removeListener();
    };
  }, []);

  // Callback when text is selected
  useEffect(() => {
    if (selectedText && onTextSelected) {
      onTextSelected(selectedText);
    }
  }, [selectedText, onTextSelected]);

  if (!showIndicator || !selectedText) {
    return null;
  }

  // Calculate how much text is selected to provide feedback
  const wordCount = selectedText.split(/\s+/).filter(word => word.length > 0).length;
  const charCount = selectedText.length;

  return (
    <div
      style={{
        position: 'fixed',
        top: indicatorPosition.top,
        left: indicatorPosition.left,
        transform: 'translateX(-50%)',
        backgroundColor: '#4ade80',
        color: 'white',
        padding: '4px 8px',
        borderRadius: '4px',
        fontSize: '12px',
        zIndex: 9999,
        boxShadow: '0 2px 6px rgba(0, 0, 0, 0.2)',
        pointerEvents: 'none',
        minWidth: '120px',
        textAlign: 'center'
      }}
    >
      <div style={{ fontWeight: 'bold', fontSize: '11px' }}>✓ Text Selected</div>
      <div style={{ fontSize: '10px' }}>
        {wordCount} {wordCount === 1 ? 'word' : 'words'}, {charCount} {charCount === 1 ? 'char' : 'chars'}
      </div>
    </div>
  );
};

export default TextSelectionIndicator;