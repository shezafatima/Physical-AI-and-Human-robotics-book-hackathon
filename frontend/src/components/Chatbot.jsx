import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';
import { chatWithBot } from '../services/api';
import { marked } from 'marked';

const Chatbot = ({ className, onClose, initialOpen = true }) => {
  const [isVisible, setIsVisible] = useState(initialOpen);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics course. How can I help you today?",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (e, selectedText = null) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
      selectedText: selectedText || null
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the actual API service with selected text support
      const contextMode = selectedText ? 'selected_text' : 'full_content';
      const response = await chatWithBot(inputValue, selectedText, contextMode);

      // Handle error responses from the API
      if (response.status === 'error' || response.error) {
        const errorMessage = {
          id: Date.now() + 1,
          text: response.response || "Sorry, I encountered an error processing your request. Please try again.",
          sender: 'bot',
          timestamp: new Date(),
          error: true
        };
        setMessages(prev => [...prev, errorMessage]);
        setIsLoading(false);
        return;
      }

      const botResponse = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        timestamp: new Date(),
        sources: response.sources || []
      };
      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'bot',
        timestamp: new Date(),
        error: true
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  const toggleVisibility = () => {
    setIsVisible(!isVisible);
    // If closing and parent provided onClose, call it
    if (isVisible && onClose) {
      onClose();
    }
  };

  if (!isVisible) {
    // Render just the toggle button when closed
    return (
      <button
        className={clsx(styles.chatbotToggle, className)}
        onClick={toggleVisibility}
        aria-label="Open chatbot"
        style={{
          background: '#2563eb',
          color: 'white',
          border: 'none',
          borderRadius: '50%',
          width: '60px',
          height: '60px',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
        }}
      >
        💬
      </button>
    );
  }

  return (
    <div className={clsx(styles.chatbotContainer, className)}>
      <div className={styles.chatHeader}>
        <div className={styles.chatTitle}>
          <span className={styles.botIcon}>🤖</span>
          <h3>Course Assistant</h3>
        </div>
        <div className={styles.chatControls}>
          <button
            className={styles.closeButton}
            onClick={toggleVisibility}
            aria-label="Close chatbot"
          >
            ×
          </button>
          <div className={styles.chatStatus}>
            <span className={styles.statusIndicator}></span>
            <span className={styles.statusText}>Online</span>
          </div>
        </div>
      </div>

      <div className={styles.chatMessages}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={clsx(
              styles.message,
              styles[message.sender],
              message.sender === 'user' ? styles.userMessage : styles.botMessage
            )}
          >
            <div className={styles.messageContent}>
              <div className={clsx(
                styles.messageText,
                message.error ? styles.errorMessage : ''
              )}>
                {message.sender === 'bot' ? (
                  <div dangerouslySetInnerHTML={{ __html: marked(message.text || '') }} />
                ) : (
                  message.text
                )}
              </div>
              {message.sender === 'bot' && !message.error && message.sources && message.sources.length > 0 && (
                <div className={styles.messageSources}>
                  <strong>Sources:</strong>
                  <ul>
                    {message.sources.slice(0, 3).map((source, idx) => (
                      <li key={typeof source === 'string' ? `${idx}-${source}` : source.chunk_id || idx}>
                        {typeof source === 'string' ? source : source.section_title || source.source_document || source.content?.substring(0, 50) + '...'}
                      </li>
                    ))}
                    {message.sources.length > 3 && (
                      <li>... and {message.sources.length - 3} more</li>
                    )}
                  </ul>
                </div>
              )}
              <div className={styles.messageTime}>{formatTime(message.timestamp)}</div>
            </div>
          </div>
        ))}
        {isLoading && (
          <div className={clsx(styles.message, styles.botMessage)}>
            <div className={styles.messageContent}>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about Physical AI or Humanoid Robotics..."
          className={styles.chatInput}
          disabled={isLoading}
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={isLoading || !inputValue.trim()}
        >
          <span>➤</span>
        </button>
      </form>
    </div>
  );
};

export default Chatbot;