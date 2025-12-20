/**
 * Frontend data models for chat entities
 */

/**
 * Represents a user query to the chatbot
 */
export class UserQuery {
  /**
   * Create a user query
   * @param {string} message - The user's question or message
   * @param {string} [selectedText] - Text selected by user for context (optional)
   * @param {'full_content'|'selected_text'} [contextMode='full_content'] - How to process the query
   * @param {string} [sessionId] - Session identifier for maintaining conversation context
   */
  constructor(message, selectedText = null, contextMode = 'full_content', sessionId = null) {
    this.message = message;
    this.selectedText = selectedText;
    this.contextMode = contextMode;
    this.sessionId = sessionId;
    this.timestamp = new Date().toISOString();
  }

  /**
   * Convert to API request format
   * @returns {Object} Formatted for API request
   */
  toApiFormat() {
    return {
      message: this.message,
      selected_text: this.selectedText,
      context_mode: this.contextMode,
      session_id: this.sessionId,
      timestamp: this.timestamp
    };
  }
}

/**
 * Represents a response from the backend chatbot
 */
export class BackendResponse {
  /**
   * Create a backend response
   * @param {string} response - The AI-generated answer to the user's query
   * @param {Array<string>} [sources] - Citations to source documents used in the response
   * @param {'high'|'medium'|'low'|'insufficient_data'} [confidence='medium'] - Confidence level of the response
   * @param {'success'|'error'|'timeout'} [status='success'] - Processing status
   * @param {Object} [context] - Additional context information including retrieved documents
   * @param {string} [error] - Error message if processing failed
   */
  constructor(response, sources = [], confidence = 'medium', status = 'success', context = null, error = null) {
    this.response = response;
    this.sources = sources;
    this.confidence = confidence;
    this.status = status;
    this.context = context;
    this.error = error;
    this.timestamp = new Date().toISOString();
  }

  /**
   * Create from API response format
   * @param {Object} apiResponse - Response from the API
   * @returns {BackendResponse} New instance
   */
  static fromApiFormat(apiResponse) {
    return new BackendResponse(
      apiResponse.response,
      apiResponse.sources || [],
      apiResponse.confidence || 'medium',
      apiResponse.status || 'success',
      apiResponse.context || null,
      apiResponse.error || null
    );
  }
}

/**
 * Represents a chat session with conversation history
 */
export class ChatSession {
  /**
   * Create a chat session
   * @param {string} [sessionId] - Unique identifier for the chat session (auto-generated if not provided)
   * @param {Array<Object>} [messages=[]] - Array of message objects representing the conversation history
   */
  constructor(sessionId = null, messages = []) {
    this.sessionId = sessionId || this.generateSessionId();
    this.messages = messages;
    this.timestamp = new Date().toISOString();
    this.active = true;
  }

  /**
   * Generate a unique session ID
   * @returns {string} Generated session ID
   */
  generateSessionId() {
    return `sess-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Add a message to the session
   * @param {Object} message - The message to add
   * @param {'user'|'assistant'} message.role - The role of the message sender
   * @param {string} message.content - The text content of the message
   * @param {Array<string>} [message.sources] - Source citations if this is an assistant message
   */
  addMessage({ role, content, sources = [] }) {
    const message = {
      id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 5)}`,
      role,
      content,
      sources,
      timestamp: new Date().toISOString()
    };

    this.messages.push(message);
  }

  /**
   * Get user messages only
   * @returns {Array<Object>} User messages in the session
   */
  getUserMessages() {
    return this.messages.filter(msg => msg.role === 'user');
  }

  /**
   * Get assistant messages only
   * @returns {Array<Object>} Assistant messages in the session
   */
  getAssistantMessages() {
    return this.messages.filter(msg => msg.role === 'assistant');
  }

  /**
   * Clear the session history
   */
  clear() {
    this.messages = [];
  }

  /**
   * End the session
   */
  end() {
    this.active = false;
  }

  /**
   * Convert to API format
   * @returns {Object} Formatted for API
   */
  toApiFormat() {
    return {
      session_id: this.sessionId,
      messages: this.messages,
      timestamp: this.timestamp,
      active: this.active
    };
  }
}

/**
 * Represents a single message in the chat session history
 */
export class Message {
  /**
   * Create a message
   * @param {'user'|'assistant'} role - The role of the message sender
   * @param {string} content - The text content of the message
   * @param {string} [sources] - Source citations if this is an assistant message
   * @param {string} [id] - Unique identifier for the message (auto-generated if not provided)
   */
  constructor(role, content, sources = [], id = null) {
    if (!['user', 'assistant'].includes(role)) {
      throw new Error('Role must be either "user" or "assistant"');
    }

    this.id = id || `msg-${Date.now()}-${Math.random().toString(36).substr(2, 5)}`;
    this.role = role;
    this.content = content;
    this.sources = Array.isArray(sources) ? sources : [];
    this.timestamp = new Date().toISOString();
  }

  /**
   * Convert to API format
   * @returns {Object} Formatted for API
   */
  toApiFormat() {
    return {
      id: this.id,
      role: this.role,
      content: this.content,
      sources: this.sources,
      timestamp: this.timestamp
    };
  }
}