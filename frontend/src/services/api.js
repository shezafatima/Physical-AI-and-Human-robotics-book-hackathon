// API service for connecting to the backend
const API_BASE_URL = typeof process !== 'undefined' && process.env.REACT_APP_API_URL
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000';

class ApiService {
  constructor() {
    this.baseURL = API_BASE_URL;
  }

  // Generic request method
  async request(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;
    const config = {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    };

    try {
      const response = await fetch(url, config);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API request failed:', error);
      throw error;
    }
  }

  // Health check
  async healthCheck() {
    return this.request('/health');
  }

  // Chatbot API methods
  async sendMessage(message, context = null) {
    return this.request('/chat', {
      method: 'POST',
      body: JSON.stringify({
        message,
        context,
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
export const chatWithBot = async (message, context = null) => {
  try {
    return await apiService.sendMessage(message, context);
  } catch (error) {
    console.error('Error chatting with bot:', error);
    // Return a mock response for development
    return {
      response: `I understand you're asking about: "${message}". This is a simulated response from the RAG Chatbot. In a full implementation, this would connect to your backend API to provide context-aware responses based on the course materials.`,
      context: context || 'course-materials',
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