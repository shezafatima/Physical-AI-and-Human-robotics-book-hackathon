# Feature Specification: Frontend Coursebook + RAG Chatbot

## User Stories

### P1 User Story 1: Frontend Coursebook Display
As a student, I want to view a comprehensive coursebook with at least 8 chapters, so that I can learn the course material effectively.

**Acceptance Criteria:**
- The coursebook displays a minimum of 8 chapters.
- Each chapter includes an Overview, Learning Objectives, Theory, Diagrams, Code Samples, Exercises, and References.
- The UI is creative and interactive, following the specified color theme (Primary Deep Blue, Accent Neon Cyan, Neutral Background Light Gray, Text Dark Gray, Hover Neon Green).
- Interactive components such as collapsible theory sections, code tabs, image galleries, and exercise/practice cards are present and functional.
- The website has smooth scrolling and a functional sidebar navigation.
- An optional dark mode is available and works correctly.
- The entire coursebook is fully responsive across different devices.

### P2 User Story 2: RAG Chatbot Integration
As a student, I want to use an embedded chatbot that strictly answers questions based on the coursebook content, so that I can get accurate and grounded information.

**Acceptance Criteria:**
- The chatbot is embedded within the coursebook interface.
- The chatbot only provides answers strictly grounded in the coursebook content.
- For questions outside the coursebook's scope, the chatbot returns "cannot answer from context".
- The chatbot connects to the MCP server for context management.
- Chatbot responses have a P95 latency of less than 5 seconds.
- (Optional) User personalization and Urdu translation are available via Better-Auth signup.
- The chatbot functionality is validated after the full frontend deployment, with 20+ questions per chapter covering various scenarios and edge cases.

## Non-Goals

- General-purpose chatbot functionality outside the coursebook content.
- Advanced AI features not directly related to course material summarization and Q&A.
