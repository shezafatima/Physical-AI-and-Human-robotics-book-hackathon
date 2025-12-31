const API_BASE_URL =
  typeof process !== 'undefined' && process.env.REACT_APP_API_URL
    ? process.env.REACT_APP_API_URL
    : 'https://shizafatima-hackathon.hf.space';




class ApiService {
  constructor() {
    this.baseURL = API_BASE_URL;
  }

  // Generic request method with timeout and retry logic
  async request(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;

    // Log the actual request URL for debugging
    console.log(`Making API request to: ${url}`);

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
        method: options.method || 'GET', // Ensure method is specified
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
          // Add any additional headers that might help with Hugging Face Spaces
          ...options.headers,
        },
        // Add mode: 'cors' explicitly to ensure CORS is handled properly
        mode: 'cors',
        // Add credentials if needed for authentication
        credentials: 'omit', // Change to 'include' if backend requires cookies/sessions
        signal: controller.signal, // Add abort signal for timeout
        ...options,
      };

      // For POST requests specifically, ensure body is properly formatted
      if (options.method === 'POST' && options.body && typeof options.body === 'object') {
        config.body = JSON.stringify(options.body);
      }

      try {
        const response = await fetch(url, config);

        // Log the response status for debugging
        console.log(`API response status: ${response.status} for URL: ${url}`);

        clearTimeout(timeoutId); // Clear timeout on successful response

        // Handle different response status codes according to API contract
        if (!response.ok) {
          // Try to get error details
          let errorData = {};
          try {
            errorData = await response.json();
          } catch (parseError) {
            // If response is not JSON, get text
            try {
              errorData = { error: await response.text() || response.statusText };
            } catch {
              errorData = { error: response.statusText };
            }
          }

          // Create specific error messages based on status codes
          switch (response.status) {
            case 400:
              throw new Error(`Bad Request: ${errorData.error || 'Invalid request format'}`);
            case 404:
              // For 404 errors, it could be a path issue with Hugging Face Spaces
              // Log additional information for debugging
              console.error(`404 Error Details - URL: ${url}, Endpoint: ${endpoint}, BaseURL: ${this.baseURL}`);
              throw new Error(`Not Found: The endpoint ${url} does not exist. Status: ${response.status} - ${errorData.error || response.statusText}`);
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
        else if (error.message.includes('Failed to fetch') || (error.name === 'TypeError' && error.message.includes('fetch'))) {
          // Special handling for "Failed to fetch" which often indicates CORS or network issues
          lastError = new Error(`Connection Error: Unable to reach the server at ${url}. This might be a CORS issue or the backend is not accessible from this domain. Error: ${error.message}`);
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
          // For 404 errors specifically, we might want to try an alternative endpoint structure
          // that's common with Hugging Face Spaces
          if (error.message.includes('404') && !url.includes('/api/') && this.baseURL.includes('hf.space')) {
            console.log('Attempting fallback API path for Hugging Face Spaces...');
            // Try with /api prefix which is sometimes needed for Hugging Face Spaces
            try {
              const fallbackUrl = `${this.baseURL}/api${endpoint}`;
              console.log(`Trying fallback API request to: ${fallbackUrl}`);

              const fallbackResponse = await fetch(fallbackUrl, config);
              console.log(`Fallback API response status: ${fallbackResponse.status} for URL: ${fallbackUrl}`);

              if (!fallbackResponse.ok) {
                throw new Error(`Fallback request also failed with status: ${fallbackResponse.status}`);
              }

              return await fallbackResponse.json();
            } catch (fallbackError) {
              console.error('Fallback API request also failed:', fallbackError);
              // If fallback also fails, throw the original error
              throw lastError;
            }
          } else {
            throw lastError;
          }
        }

        // Wait before retrying (exponential backoff)
        const delay = retryDelay * Math.pow(2, attempt); // 1s, 2s, 4s, etc.
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }

    // This line should never be reached, but included for completeness
    throw lastError;
  }

  // Health check - using the correct backend endpoint
  async healthCheck() {
    return this.request('/v1/health');
  }

  // Chatbot API methods - mapped to existing /v1/query endpoint
  async sendMessage(message, selectedText = null, contextMode = 'full_content', sessionId = null) {
    try {
      // Send only the query field to match backend schema exactly
      return await this.request('/v1/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: message  // Only send the query field to avoid validation errors
        })
      });
    } catch (error) {
      console.error('Error in sendMessage:', error);
      throw error;
    }
  }

  // Course content API methods - placeholder implementations
  async getCourseContent(courseId) {
    // Placeholder: Return mock data or use a different approach
    // Since backend doesn't have course endpoints, we'll return a mock response
    console.warn(`Course content endpoint not implemented in backend. Course ID: ${courseId}`);
    return {
      id: courseId,
      title: `Course ${courseId}`,
      content: "Course content would be retrieved from backend",
      chapters: [],
      timestamp: new Date().toISOString()
    };
  }

  async getChapterContent(chapterId) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Chapter content endpoint not implemented in backend. Chapter ID: ${chapterId}`);
    return {
      id: chapterId,
      title: `Chapter ${chapterId}`,
      content: "Chapter content would be retrieved from backend",
      timestamp: new Date().toISOString()
    };
  }

  // Quiz API methods - placeholder implementations
  async submitQuiz(quizId, answers) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Quiz submission endpoint not implemented in backend. Quiz ID: ${quizId}`);
    return {
      quizId,
      submitted: true,
      score: null, // Would be calculated by backend
      timestamp: new Date().toISOString()
    };
  }

  async getQuiz(quizId) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Quiz endpoint not implemented in backend. Quiz ID: ${quizId}`);
    return {
      id: quizId,
      title: `Quiz ${quizId}`,
      questions: [],
      timestamp: new Date().toISOString()
    };
  }

  // Notes API methods - placeholder implementations
  async saveNotes(userId, contentId, notes) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Notes save endpoint not implemented in backend. User ID: ${userId}, Content ID: ${contentId}`);
    return {
      userId,
      contentId,
      notes,
      saved: true,
      timestamp: new Date().toISOString()
    };
  }

  async getNotes(userId, contentId) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Notes endpoint not implemented in backend. User ID: ${userId}, Content ID: ${contentId}`);
    return {
      userId,
      contentId,
      notes: "",
      exists: false,
      timestamp: new Date().toISOString()
    };
  }

  // Progress tracking API methods - placeholder implementations
  async updateProgress(userId, contentId, progress) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Progress update endpoint not implemented in backend. User ID: ${userId}, Content ID: ${contentId}`);
    return {
      userId,
      contentId,
      progress,
      updated: true,
      timestamp: new Date().toISOString()
    };
  }

  async getProgress(userId, courseId) {
    // Placeholder: Return mock data or use a different approach
    console.warn(`Progress endpoint not implemented in backend. User ID: ${userId}, Course ID: ${courseId}`);
    return {
      userId,
      courseId,
      progress: [],
      timestamp: new Date().toISOString()
    };
  }
}

// Create a singleton instance
const apiService = new ApiService();
export default apiService;

// Example usage functions
export const chatWithBot = async (message, selectedText = null, contextMode = 'full_content', sessionId = null) => {
  try {
    const response = await apiService.sendMessage(message, selectedText, contextMode, sessionId);

    // Map the backend response format to the expected frontend format with tolerance for different field names
    return {
      response: response.response || response.answer || response.message || 'No response received', // Map various possible response field names
      sources: response.sources || response.references || [], // Map various possible sources field names
      confidence: response.confidence_level || response.confidence || response.confidence_score || 'insufficient_data', // Map various possible confidence field names
      status: 'success',
      context: {
        retrieved_docs_count: response.sources ? response.sources.length : 0,
        source_docs: response.sources ? response.sources.map(source =>
          typeof source === 'string' ? source : source.source_document || source.chunk_id || source.title || 'Unknown source'
        ) : [],
        confidence_scores: response.sources ? response.sources.map(source =>
          typeof source === 'object' && source.confidence_score !== undefined ? source.confidence_score : 0
        ) : [],
        generation_metadata: response.metadata || response.generation_metadata || {}
      },
      timestamp: response.timestamp || response.created || new Date().toISOString()
    };
  } catch (error) {
    console.error('Error chatting with bot:', error);

    // Return a proper error response according to API contract
    return {
      response: `I'm sorry, but I encountered an error while processing your request. Please try again later. ${error.message ? `Details: ${error.message}` : ''}`,
      sources: [],
      confidence: 'insufficient_data',
      status: 'error',
      context: {},
      error: error.message || 'Unknown error',
      timestamp: new Date().toISOString()
    };
  }
};

export const fetchCourseContent = async (courseId) => {
  try {
    const result = await apiService.getCourseContent(courseId);
    return result;
  } catch (error) {
    console.error('Error fetching course content:', error);
    return null;
  }
};

export const submitQuizAnswers = async (quizId, answers) => {
  try {
    const result = await apiService.submitQuiz(quizId, answers);
    return result;
  } catch (error) {
    console.error('Error submitting quiz:', error);
    return null;
  }
};

export const getNotesFromAPI = async (userId, contentId) => {
  try {
    const result = await apiService.getNotes(userId, contentId);
    return result;
  } catch (error) {
    console.error('Error fetching notes:', error);
    return null;
  }
};

export const saveNotesToAPI = async (userId, contentId, notes) => {
  try {
    const result = await apiService.saveNotes(userId, contentId, notes);
    return result;
  } catch (error) {
    console.error('Error saving notes:', error);
    return null;
  }
};

export const updateProgressAPI = async (userId, contentId, progress) => {
  try {
    const result = await apiService.updateProgress(userId, contentId, progress);
    return result;
  } catch (error) {
    console.error('Error updating progress:', error);
    return null;
  }
};
