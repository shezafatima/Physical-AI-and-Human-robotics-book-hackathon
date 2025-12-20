import React, { useState, useEffect } from 'react';
import MessageDisplay from './MessageDisplay';
import QueryInput from './QueryInput';
import LoadingIndicator from './LoadingIndicator';
import ErrorMessage from './ErrorMessage';
import { chatWithBot } from '../../services/api';
import { getSelectedText, sanitizeSelectedText } from '../../utils/textSelection';

/**
 * ChatInterface component - Main chat interface for the RAG chatbot
 */
const ChatInterface = ({ sessionId = null }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [currentSessionId, setCurrentSessionId] = useState(sessionId || null);

  // Initialize session if not provided
  useEffect(() => {
    if (!currentSessionId) {
      const newSessionId = `sess-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      setCurrentSessionId(newSessionId);
    }
  }, [currentSessionId]);

  /**
   * Handle sending a message to the backend
   * @param {string} message - The message to send
   * @param {string} [selectedText] - Selected text for context (optional)
   */
  const handleSendMessage = async (message, selectedText = null) => {
    if (!message.trim()) return;

    // If no selected text is provided, try to get it from the current selection
    const actualSelectedText = selectedText || getSelectedText();
    const sanitizedSelectedText = actualSelectedText ? sanitizeSelectedText(actualSelectedText) : null;

    try {
      setIsLoading(true);
      setError(null);

      // Add user message to the chat
      const userMessage = {
        id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 5)}`,
        role: 'user',
        content: message,
        sources: [],
        timestamp: new Date().toISOString(),
        selectedText: sanitizedSelectedText || undefined // Only include if there's selected text
      };

      setMessages(prev => [...prev, userMessage]);

      // Send the message to the backend
      const response = await chatWithBot(
        message,
        sanitizedSelectedText,
        sanitizedSelectedText ? 'selected_text' : 'full_content',
        currentSessionId
      );

      // Add assistant response to the chat
      const assistantMessage = {
        id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 5)}`,
        role: 'assistant',
        content: response.response,
        sources: response.sources || [],
        timestamp: response.timestamp || new Date().toISOString()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error sending message:', err);
      setError(`Failed to send message: ${err.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>RAG Chatbot</h3>
        <p>Ask questions about the course content</p>
      </div>

      <div className="chat-messages-container">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your AI assistant for the course materials.</p>
            <p>Ask me anything about the content, or select text on the page and ask questions about it.</p>
          </div>
        ) : (
          <MessageDisplay messages={messages} />
        )}

        {isLoading && <LoadingIndicator />}
        {error && <ErrorMessage message={error} />}
      </div>

      <div className="chat-input-container">
        <QueryInput onSendMessage={handleSendMessage} />
      </div>
    </div>
  );
};

export default ChatInterface;