# Research Document: Frontend-Backend Integration for Embedded RAG Chatbot

**Feature**: 1-frontend-backend-integration
**Created**: 2025-12-18
**Status**: Complete

## Decision: API Endpoint Design

**Rationale**: The backend already has a `/chat` endpoint in main.py that can be extended to support the new requirements. This endpoint follows the existing pattern and can be enhanced to support selected text context.

**Alternatives considered**:
1. Create new `/v1/chat` endpoint - Would require more changes but cleaner separation
2. Extend existing `/chat` endpoint - Uses existing infrastructure, minimal changes
3. Create `/api/rag` endpoint - Different naming convention than existing

**Chosen approach**: Extend existing `/chat` endpoint to support selected text context as it leverages existing infrastructure and follows the current pattern.

## Decision: Selected Text Context Transmission

**Rationale**: Selected text will be captured using the browser's Selection API and sent as an additional field in the query request. This approach is straightforward and allows the backend to use the selected text as additional context.

**Technical approach**:
- Use `window.getSelection()` to capture selected text
- Include selected text in the request body as a separate field
- Backend will prioritize selected text context when context_mode is "selected_text"

**Alternatives considered**:
1. Send entire page content with highlighted sections - Too much data transfer
2. Send only the selected text - Simplest approach, sufficient for context
3. Send selected text with surrounding context - More complex but potentially better results

**Chosen approach**: Send only the selected text as a separate field for simplicity and efficiency.

## Decision: Error Handling Strategies

**Rationale**: Implement comprehensive error handling with user-friendly messages and graceful degradation. Different error types will have different handling strategies to maintain good user experience.

**Error handling strategies**:
1. Network errors: Display "Connection unavailable" with retry option
2. Backend processing errors: Display "Processing error" with user-friendly message
3. Timeout errors: Display "Request timed out" with option to try again
4. Validation errors: Display specific validation feedback

**Alternatives considered**:
1. Generic error message for all errors - Less informative for users
2. Technical error messages - Confusing for non-technical users
3. Different messages for different error types - More helpful for users

**Chosen approach**: Different messages for different error types to provide helpful feedback while keeping technical details hidden from users.