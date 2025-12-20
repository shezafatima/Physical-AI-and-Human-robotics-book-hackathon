# ADR-007: RAG Agent Architecture Selection

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-18
- **Feature:** rag-agent-openai-gemini
- **Context:** Need to implement a backend RAG agent that integrates with a validated retrieval pipeline and invokes a language model (Gemini) via API key configuration to generate grounded responses based strictly on retrieved book content. The architecture must ensure responses are grounded in provided context without hallucination while maintaining good performance and proper error handling.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

### Backend Technology Stack
- Framework: FastAPI (Python 3.11) for async API handling and built-in OpenAPI documentation
- Language Model Integration: Google Generative AI SDK for Gemini with custom agent patterns
- Data Validation: Pydantic for type validation and serialization
- Server: uvicorn for ASGI server implementation
- Configuration: Pydantic BaseSettings for environment management

### Architecture Pattern
- RAG (Retrieval-Augmented Generation) pattern with clear separation between retrieval and generation services
- Agent-like interface that follows OpenAI Agent patterns but works with Gemini API directly
- Service-oriented architecture with dedicated modules for models, services, API, and configuration

### Data Model Architecture
- Query entity for user input management
- RetrievedContext entity to store retrieval results with confidence scores
- ContextChunk entity for individual text chunks with source attribution
- GeneratedResponse entity with citation tracking and confidence levels
- AgentConfiguration entity for API keys and model parameters

### API Design
- RESTful API with OpenAPI specification
- Query endpoint for submitting user requests
- Health check endpoint for service monitoring
- Proper error handling and response validation

## Consequences

### Positive

- FastAPI provides excellent async support for I/O-bound operations like API calls to external services
- Built-in OpenAPI documentation generation for easy API consumption
- Pydantic models provide strong typing and validation reducing runtime errors
- Clear separation of concerns with dedicated service modules
- Proper grounding of responses in retrieved context prevents hallucination
- Comprehensive data model with proper relationships and validation rules
- Standardized error handling and response format

### Negative

- Learning curve for team members unfamiliar with FastAPI or Pydantic
- Dependency on external APIs (Gemini) which may have rate limits or availability issues
- Additional complexity of maintaining the RAG pattern with two service interactions
- Potential latency issues when both retrieval and generation services are called sequentially
- Need for proper API key management and security considerations
- Increased complexity of testing due to external service dependencies

## Alternatives Considered

Alternative Stack A: Direct OpenAI integration with OpenAI Agents SDK only
- Would not meet requirement to use Gemini model
- Would limit flexibility in language model choice

Alternative Stack B: Simple Flask + Requests approach
- Would lack async capabilities needed for efficient API calls
- Would not provide built-in validation and documentation
- Would require more manual implementation of features FastAPI provides

Alternative Architecture C: Monolithic approach without separation of retrieval/generation
- Would create tight coupling between components
- Would make testing and maintenance more difficult
- Would not allow independent scaling of retrieval vs generation components

Alternative Data Model D: Simplified model without explicit context tracking
- Would not meet requirement for proper citation and grounding
- Would make it difficult to verify response accuracy
- Would not support the "no hallucination" requirement

## References

- Feature Spec: specs/001-rag-agent-openai-gemini/spec.md
- Implementation Plan: specs/001-rag-agent-openai-gemini/plan.md
- Related ADRs: ADR-002 (RAG Chatbot Behavior) - builds upon the RAG pattern established there
- Evaluator Evidence: specs/001-rag-agent-openai-gemini/research.md, specs/001-rag-agent-openai-gemini/data-model.md