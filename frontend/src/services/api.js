// API service for connecting to the backend
const API_BASE_URL = typeof process !== 'undefined' && process.env.REACT_APP_API_URL
  ? process.env.REACT_APP_API_URL
  : 'https://shizafatima-hackathon.hf.space';

class ApiService {
  constructor() {
    this.baseURL = API_BASE_URL;
  }

  // Generic request method with timeout and retry logic
  async request(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;

    // Set default timeout to 30 seconds if not specified
    const timeout = options.timeout || 30000;
    // Set default retry attempts to 3 if not specified
    const maxRetries = options.maxRetries || 3;
    // Set default retry delay to 1000ms if not specified
    const retryDelay = options.retryDelay || 1000;

    let lastError;

    // Try the request up to maxRetries times
    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      // Create AbortController for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), timeout);

      const config = {
        headers: {
          'Content-Type': 'application/json',
          ...options.headers,
        },
        signal: controller.signal, // Add abort signal for timeout
        ...options,
      };

      try {
        const response = await fetch(url, config);
        clearTimeout(timeoutId); // Clear timeout on successful response

        // Handle different response status codes according to API contract
        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));

          // Create specific error messages based on status codes
          switch (response.status) {
            case 400:
              throw new Error(`Bad Request: ${errorData.error || 'Invalid request format'}`);
            case 408:
              throw new Error(`Request Timeout: ${errorData.error || 'Request timed out'}`);
            case 500:
              throw new Error(`Server Error: ${errorData.error || 'Internal server error occurred'}`);
            default:
              throw new Error(`HTTP error! status: ${response.status} - ${errorData.error || response.statusText}`);
          }
        }

        return await response.json();
      } catch (error) {
        clearTimeout(timeoutId); // Clear timeout on error too

        // Check if the error is due to timeout
        if (error.name === 'AbortError') {
          lastError = new Error('Request Timeout: The request took too long to complete.');
        }
        // Handle network errors and other fetch-related errors
        else if (error.name === 'TypeError' && error.message.includes('fetch')) {
          lastError = new Error('Network Error: Unable to connect to the server. Please check your connection.');
        }
        else if (error.message.includes('timeout') || error.message.includes('408')) {
          lastError = new Error('Request Timeout: The request took too long to complete.');
        }
        else {
          lastError = error;
        }

        console.error(`API request failed (attempt ${attempt + 1}/${maxRetries + 1}):`, lastError);

        // If this was the last attempt, throw the error
        if (attempt === maxRetries) {
          throw lastError;
        }

        // Wait before retrying (exponential backoff)
        const delay = retryDelay * Math.pow(2, attempt); // 1s, 2s, 4s, etc.
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }

    // This line should never be reached, but included for completeness
    throw lastError;
  }

  // Health check
  async healthCheck() {
    return this.request('/health');
  }

  // Chatbot API methods
  async sendMessage(message, selectedText = null, contextMode = 'full_content', sessionId = null) {
    return this.request('/chat', {
      method: 'POST',
      body: JSON.stringify({
        message,
        selected_text: selectedText,
        context_mode: contextMode,
        session_id: sessionId,
        timestamp: new Date().toISOString()
      })
    });
  }

  // Course content API methods
  async getCourseContent(courseId) {
    return this.request(`/courses/${courseId}`);
  }

  async getChapterContent(chapterId) {
    return this.request(`/chapters/${chapterId}`);
  }

  // Quiz API methods
  async submitQuiz(quizId, answers) {
    return this.request(`/quizzes/${quizId}/submit`, {
      method: 'POST',
      body: JSON.stringify({ answers })
    });
  }

  async getQuiz(quizId) {
    return this.request(`/quizzes/${quizId}`);
  }

  // Notes API methods
  async saveNotes(userId, contentId, notes) {
    return this.request('/notes', {
      method: 'POST',
      body: JSON.stringify({
        userId,
        contentId,
        notes,
        timestamp: new Date().toISOString()
      })
    });
  }

  async getNotes(userId, contentId) {
    return this.request(`/notes?userId=${userId}&contentId=${contentId}`);
  }

  // Progress tracking API methods
  async updateProgress(userId, contentId, progress) {
    return this.request('/progress', {
      method: 'POST',
      body: JSON.stringify({
        userId,
        contentId,
        progress,
        timestamp: new Date().toISOString()
      })
    });
  }

  async getProgress(userId, courseId) {
    return this.request(`/progress?userId=${userId}&courseId=${courseId}`);
  }
}

// Create a singleton instance
const apiService = new ApiService();
export default apiService;

// Example usage functions
export const chatWithBot = async (message, selectedText = null, contextMode = 'full_content', sessionId = null) => {
  try {
    return await apiService.sendMessage(message, selectedText, contextMode, sessionId);
  } catch (error) {
    console.error('Error chatting with bot:', error);

    // Return a proper error response according to API contract
    return {
      response: `Error: ${error.message}`,
      sources: [],
      confidence: 'insufficient_data',
      status: 'error',
      error: error.message,
      timestamp: new Date().toISOString()
    };
  }
};

export const fetchCourseContent = async (courseId) => {
  try {
    return await apiService.getCourseContent(courseId);
  } catch (error) {
    console.error('Error fetching course content:', error);
    return null;
  }
};

export const submitQuizAnswers = async (quizId, answers) => {
  try {
    return await apiService.submitQuiz(quizId, answers);
  } catch (error) {
    console.error('Error submitting quiz:', error);
    return null;
  }
};

export const getNotesFromAPI = async (userId, contentId) => {
  try {
    return await apiService.getNotes(userId, contentId);
  } catch (error) {
    console.error('Error fetching notes:', error);
    return null;
  }
};

export const saveNotesToAPI = async (userId, contentId, notes) => {
  try {
    return await apiService.saveNotes(userId, contentId, notes);
  } catch (error) {
    console.error('Error saving notes:', error);
    return null;
  }
};

export const updateProgressAPI = async (userId, contentId, progress) => {
  try {
    return await apiService.updateProgress(userId, contentId, progress);
  } catch (error) {
    console.error('Error updating progress:', error);
    return null;
  }
};
