/**
 * Utility functions for handling text selection in the browser
 */

/**
 * Get the currently selected text in the document
 * @returns {string|null} The selected text or null if no text is selected
 */
export const getSelectedText = () => {
  const selection = window.getSelection ? window.getSelection() : document.selection;
  if (selection && selection.toString().trim()) {
    return selection.toString().trim();
  }
  return null;
};

/**
 * Get detailed information about the current text selection
 * @returns {Object|null} Selection details or null if no text is selected
 */
export const getSelectionDetails = () => {
  const selection = window.getSelection ? window.getSelection() : document.selection;

  if (!selection || selection.toString().trim() === '') {
    return null;
  }

  const selectedText = selection.toString().trim();
  if (!selectedText) {
    return null;
  }

  // Get the range of the selection
  let range = null;
  if (selection.rangeCount > 0) {
    range = selection.getRangeAt(0);
  }

  return {
    text: selectedText,
    range: range,
    startContainer: range ? range.startContainer : null,
    startOffset: range ? range.startOffset : null,
    endContainer: range ? range.endContainer : null,
    endOffset: range ? range.endOffset : null,
    rect: range ? range.getBoundingClientRect() : null
  };
};

/**
 * Check if there is currently selected text
 * @returns {boolean} True if there is selected text, false otherwise
 */
export const hasSelectedText = () => {
  const selectedText = getSelectedText();
  return selectedText !== null && selectedText.length > 0;
};

/**
 * Clear the current text selection
 */
export const clearSelection = () => {
  if (window.getSelection) {
    window.getSelection().removeAllRanges();
  } else if (document.selection) {
    document.selection.empty();
  }
};

/**
 * Add an event listener for text selection changes
 * @param {Function} callback - Function to call when text selection changes
 * @returns {Function} Function to remove the event listener
 */
export const addSelectionListener = (callback) => {
  const handleSelectionChange = () => {
    callback(getSelectionDetails());
  };

  document.addEventListener('selectionchange', handleSelectionChange);

  // Return a function to remove the event listener
  return () => {
    document.removeEventListener('selectionchange', handleSelectionChange);
  };
};

/**
 * Get the word or phrase around a specific point in the document
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} radius - Radius around the point to search (default 50px)
 * @returns {string|null} The text around the point or null if not found
 */
export const getTextAroundPoint = (x, y, radius = 50) => {
  // Create a range from the coordinates
  const range = document.caretRangeFromPoint ?
    document.caretRangeFromPoint(x, y) :
    document.elementFromPoint(x, y);

  if (!range) return null;

  // If caretRangeFromPoint was used, we have a range
  if (range instanceof Range) {
    // Try to expand the selection to get more context
    const rect = range.getBoundingClientRect();
    if (rect && rect.width > 0 && rect.height > 0) {
      return range.toString().trim();
    }
  }

  // If we got an element instead of a range
  if (range instanceof Element) {
    return range.textContent ? range.textContent.trim() : null;
  }

  return null;
};

/**
 * Sanitize selected text to prevent security issues
 * @param {string} text - The text to sanitize
 * @returns {string} Sanitized text
 */
export const sanitizeSelectedText = (text) => {
  if (typeof text !== 'string') {
    return '';
  }

  // Remove potentially dangerous content
  return text
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
    .replace(/javascript:/gi, '') // Remove javascript: protocol
    .replace(/data:/gi, '') // Remove data: protocol
    .replace(/vbscript:/gi, '') // Remove vbscript: protocol
    .replace(/on\w+\s*=/gi, '') // Remove event handlers
    .trim();
};

export default {
  getSelectedText,
  getSelectionDetails,
  hasSelectedText,
  clearSelection,
  addSelectionListener,
  getTextAroundPoint,
  sanitizeSelectedText
};