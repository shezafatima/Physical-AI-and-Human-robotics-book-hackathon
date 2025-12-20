import { useState, useCallback, useEffect } from 'react';

/**
 * Custom hook for managing chat state with session persistence
 * @returns {Object} Chat state and management functions
 */
const useChatState = (initialSessionId = null) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(() => {
    // Generate a unique session ID or retrieve from localStorage
    return initialSessionId ||
           localStorage.getItem('chatSessionId') ||
           `sess-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  });

  // Load chat history from localStorage for the current session
  useEffect(() => {
    if (sessionId) {
      try {
        const savedMessages = localStorage.getItem(`chatHistory_${sessionId}`);
        if (savedMessages) {
          setMessages(JSON.parse(savedMessages));
        }
      } catch (e) {
        console.error('Failed to load chat history:', e);
      }
    }
  }, [sessionId]);

  // Save session ID to localStorage
  useEffect(() => {
    if (typeof window !== 'undefined' && sessionId) {
      localStorage.setItem('chatSessionId', sessionId);
    }
  }, [sessionId]);

  // Save chat history to localStorage whenever messages change
  useEffect(() => {
    if (typeof window !== 'undefined' && sessionId && messages) {
      try {
        localStorage.setItem(`chatHistory_${sessionId}`, JSON.stringify(messages));
      } catch (e) {
        console.error('Failed to save chat history:', e);
      }
    }
  }, [messages, sessionId]);

  /**
   * Add a new message to the conversation
   * @param {Object} message - The message object
   * @param {string} message.id - Unique message ID
   * @param {string} message.role - 'user' or 'assistant'
   * @param {string} message.content - Message content
   * @param {string} message.timestamp - ISO timestamp
   * @param {Array} message.sources - Source citations (for assistant messages)
   */
  const addMessage = useCallback((message) => {
    setMessages(prevMessages => [...prevMessages, message]);
  }, []);

  /**
   * Add a user message to the conversation
   * @param {string} content - The user's message content
   */
  const addUserMessage = useCallback((content) => {
    const message = {
      id: `msg-${Date.now()}`,
      role: 'user',
      content,
      timestamp: new Date().toISOString(),
    };
    addMessage(message);
  }, [addMessage]);

  /**
   * Add an assistant message to the conversation
   * @param {string} content - The assistant's response content
   * @param {Array} sources - Source citations
   */
  const addAssistantMessage = useCallback((content, sources = []) => {
    const message = {
      id: `msg-${Date.now()}`,
      role: 'assistant',
      content,
      timestamp: new Date().toISOString(),
      sources,
    };
    addMessage(message);
  }, [addMessage]);

  /**
   * Clear the current conversation and reset session
   */
  const clearConversation = useCallback(() => {
    setMessages([]);
    setError(null);
    // Generate new session ID when clearing conversation
    const newSessionId = `sess-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    setSessionId(newSessionId);
  }, []);

  /**
   * Load a specific session's chat history
   * @param {string} sessionID - The session ID to load
   */
  const loadSession = useCallback((sessionID) => {
    try {
      const savedMessages = localStorage.getItem(`chatHistory_${sessionID}`);
      if (savedMessages) {
        setMessages(JSON.parse(savedMessages));
        setSessionId(sessionID);
      } else {
        setMessages([]);
        setSessionId(sessionID);
      }
    } catch (e) {
      console.error('Failed to load session:', e);
      setMessages([]);
    }
  }, []);

  /**
   * Get a list of all saved sessions
   * @returns {Array} Array of session IDs
   */
  const getSavedSessions = useCallback(() => {
    if (typeof window === 'undefined') return [];

    const allKeys = Object.keys(localStorage);
    const chatHistoryKeys = allKeys.filter(key => key.startsWith('chatHistory_'));
    return chatHistoryKeys.map(key => key.replace('chatHistory_', ''));
  }, []);

  /**
   * Delete a specific session's chat history
   * @param {string} sessionID - The session ID to delete
   */
  const deleteSession = useCallback((sessionID) => {
    try {
      localStorage.removeItem(`chatHistory_${sessionID}`);
      if (sessionId === sessionID) {
        // If deleting current session, start a new one
        const newSessionId = `sess-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
        setSessionId(newSessionId);
        setMessages([]);
      }
    } catch (e) {
      console.error('Failed to delete session:', e);
    }
  }, [sessionId]);

  /**
   * Set loading state
   * @param {boolean} loading - Whether the chat is loading
   */
  const setLoading = useCallback((loading) => {
    setIsLoading(loading);
  }, []);

  /**
   * Set error state
   * @param {string|null} errorMessage - Error message or null to clear error
   */
  const setErrorState = useCallback((errorMessage) => {
    setError(errorMessage);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sessionId,
    addMessage,
    addUserMessage,
    addAssistantMessage,
    clearConversation,
    loadSession,
    getSavedSessions,
    deleteSession,
    setLoading,
    setErrorState,
  };
};

export default useChatState;