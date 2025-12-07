# Frontend Coursebook + RAG Chatbot Implementation Plan

## 1. Scope and Dependencies

### In Scope:
- Frontend Coursebook (Docusaurus + Spec-Kit Plus) with minimum 8 chapters.
- Each chapter includes Overview, Learning Objectives, Theory, Diagrams, Code Samples, Exercises, References.
- Creative UI:
  - Color Theme: Primary Deep Blue (#0A3D62), Accent Neon Cyan (#00FFF7), Neutral Background Light Gray (#F5F5F5), Text Dark Gray (#1E1E1E), Hover Neon Green (#39FF14)
  - Interactive components: collapsible theory, code tabs, image galleries, exercise/practice cards
  - Smooth scroll, sidebar navigation, optional dark mode, fully responsive
- RAG Chatbot Integration: Embedded chatbot strictly grounded in coursebook content.
- Connection to MCP server for context management.
- Optional Better-Auth signup for user personalization and Urdu translation.

### Out of Scope:
- General-purpose chatbot functionality outside the coursebook content.
- Advanced AI features not directly related to course material summarization and Q&A.

### External Dependencies:
- Docusaurus: For frontend coursebook generation and structure.
- Spec-Kit Plus: For generative development workflow.
- Context7 MCP server: For Docusaurus documentation access and context management.
- FastAPI, Neon Postgres, Qdrant Vector DB: For RAG Chatbot backend.
- Better-Auth (optional): For user personalization and Urdu translation.
- GitHub Pages or Vercel: For website deployment.
- External LLM provider (e.g., OpenAI, Anthropic) for RAG chatbot.

## 2. Key Decisions and Rationale

### Decisions:
- **Coursebook Depth:** Comprehensive
  - *Rationale:* Ensures thorough coverage for engineering students.
- **Chapter Count:** Minimum 8 chapters
  - *Rationale:* Matches weekly course modules as per constitution.
- **Citation Style:** APA
  - *Rationale:* Adherence to academic standards and project constitution.
- **Creative UI Application:** Applied as specified (color theme, interactive components, responsiveness).
  - *Rationale:* Enhances user experience and engagement.
- **RAG Chatbot Grounding:** Strictly grounded in coursebook content (Option A from clarify phase).
  - *Rationale:* Prevents hallucinations and ensures factual accuracy based on course material, aligning with constitution.

### Trade-offs:
- **Comprehensive Coursebook:** May require more initial content creation effort.
- **Strict Chatbot Grounding:** Limits chatbot's ability to answer general knowledge questions outside the coursebook, which is an intentional design choice to maintain accuracy.

## 3. Interfaces and API Contracts

### Frontend Coursebook:
- **User Interface:** Interactive Docusaurus website.
  - Inputs: User navigation, clicks on interactive components.
  - Outputs: Displayed course content, interactive element responses.
- **Docusaurus Build Process:**
  - Inputs: Markdown files (chapters), configuration files.
  - Outputs: Static HTML, CSS, JS for the website.

### RAG Chatbot:
- **Public APIs:**
  - **Chat Endpoint (`/chat`):**
    - Input: User query (JSON: `{"query": "..."}`).
    - Output: Chatbot response (JSON: `{"response": "...", "sources": ["file:line"]}`).
    - Errors: HTTP 400 for invalid input, HTTP 500 for internal errors. "cannot answer from context" if out-of-scope, returned in response field.
  - **Context Management API (MCP Server Integration):** (Details to be determined)
    - Inputs/Outputs for feeding coursebook content for vector embedding.
    - Inputs/Outputs for retrieving context during chatbot query.
- **Versioning Strategy:** Semantic versioning for chatbot API (e.g., `/api/v1/chat`).
- **Idempotency, Timeouts, Retries:** To be considered for chatbot API calls to external services (e.g., embedding generation, LLM calls). Timeouts should be configured for external LLM calls.
- **Error Taxonomy:** Specific error codes for "out of context" (within response body), and standard HTTP error codes (4xx, 5xx).

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance:
- **Frontend:** Fast loading times for Docusaurus site (target < 2s FCP); smooth scrolling and interactive component rendering (<100ms interaction latency).
- **Chatbot:** P95 latency for chatbot responses < 5 seconds.

### Reliability:
- **Frontend:** High availability (static site deployment, CDN).
- **Chatbot:** SLOs for chatbot uptime (e.g., 99.9%); graceful degradation for external API failures (e.g., retry mechanisms, fallback responses).

### Security:
- **Frontend:** Standard web security practices for Docusaurus (e.g., content security policy).
- **Chatbot:**
  - AuthN/AuthZ: Optional Better-Auth for user personalization; secure API key management for MCP server and external LLM.
  - Data Handling: User queries and chatbot responses to be handled securely; no sensitive user data stored without explicit consent; GDPR/CCPA compliance for user data if applicable.
  - Secrets: API keys and database credentials to be managed securely (e.g., environment variables, secret management services).

### Cost:
- To be evaluated based on Docusaurus hosting, MCP server usage, external LLM/embedding service costs (e.g., token usage), and database costs. Aim for cost-efficiency.

## 5. Data Management and Migration

### Frontend Coursebook:
- **Source of Truth:** Markdown files for chapters, managed in Git.
- **Schema Evolution:** Markdown structure for chapters to be consistent; Docusaurus configuration updates managed through version control.
- **Migration and Rollback:** Content changes are handled via Git; Docusaurus build changes are managed by CI/CD.
- **Data Retention:** Git history for course content.

### RAG Chatbot:
- **Source of Truth:** Coursebook content (markdown) after processing into vector embeddings, stored in Qdrant Vector DB.
- **Schema Evolution:** Vector embedding schema (if applicable); FastAPI data models for API payloads.
- **Migration and Rollback:** Strategy for re-indexing course content and updating vector DB upon coursebook updates. Automated scripts for re-embedding.
- **Data Retention:** Vector embeddings of course content retained as long as the coursebook exists. User interaction data (if collected) to follow retention policies.

## 6. Operational Readiness

### Observability:
- **Frontend: ** Standard web analytics for Docusaurus; browser console logs for interactive component errors; performance monitoring (e.g., Lighthouse scores).
- **Chatbot:**
  - Logs: Detailed logs for chatbot requests, responses, context retrieval, and errors (FastAPI, Qdrant client, LLM client). Structured logging (JSON).
  - Metrics: Latency, error rates, token usage for chatbot interactions; Qdrant performance metrics.
  - Traces: Distributed tracing for requests across chatbot components (FastAPI, Qdrant, LLM).

<h3>Alerting:</h3>
<ul>
<li><strong>Frontend:</strong> Alerting on Docusaurus build failures.</li>
<li><strong>Chatbot:</strong> Thresholds for error rates, latency, and out-of-context responses; Qdrant health alerts.</li>
</ul>

<h3>Runbooks:</h3>
<ul>
<li>For deploying Docusaurus, updating course content, re-indexing chatbot data, troubleshooting chatbot issues, and deploying/managing the chatbot backend.</li>
</ul>

<h3>Deployment and Rollback:</h3>
<ul>
<li><strong>Frontend:</strong> Automated deployment to GitHub Pages/Vercel via CI/CD; simple rollback by deploying previous successful build from Git.</li>
<li><strong>Chatbot:</strong> Containerized deployment (e.g., Docker, Kubernetes) with CI/CD; blue/green or canary deployments for minimal downtime; clear rollback strategy to previous working version.</li>
</ul>

<h3>Feature Flags:</h3>
<ul>
<li>For optional Better-Auth integration and Urdu translation in the chatbot. Configuration for enabling/disabling interactive components.</li>
</ul>

<h2>7. Risk Analysis and Mitigation</h2>

<h3>Top 3 Risks:</h3>
<ol>
<li><strong>Chatbot Hallucinations/Out-of-Context Responses:</strong>
<ul>
<li><em>Blast Radius:</em> Misinformation provided to users, erosion of trust, negative educational impact.</li>
<li><em>Mitigation:</em> Strict grounding in coursebook content; "cannot answer from context" for out-of-scope; extensive testing (20+ questions per chapter, cover edge cases); continuous monitoring of chatbot responses; human review loop.</li>
</ul>
</li>
<li><strong>Docusaurus Content Sync and Generation Issues:</strong>
<ul>
<li><em>Blast Radius:</em> Outdated or incorrect course material on the website, broken UI.</li>
<li><em>Mitigation:</em> Automated content generation from spec files; robust CI/CD pipeline for Docusaurus build and deployment; comprehensive content validation for each chapter (required sections, citations, code, diagrams); automated UI tests.</li>
</ul>
</li>
<li><strong>Performance Degradation (Chatbot and Frontend):</strong>
<ul>
<li><em>Blast Radius:</em> Slow responses, poor user experience, disengagement.</li>
<li><em>Mitigation:</em> Performance testing (load testing, stress testing); optimize vector database queries; efficient LLM integration (e.g., prompt engineering, model selection); frontend asset optimization, caching strategies (CDN for Docusaurus, API caching); code splitting for Docusaurus.</li>
</ul>
</li>
</ol>

<h2>8. Evaluation and Validation</h2>

<h3>Definition of Done:</h3>
<ul>
<li>Complete book deployed on Docusaurus and fully navigable.</li>
<li>RAG chatbot embedded and able to answer using selected text only.</li>
<li>All spec files validated by Spec-Kit Plus without errors.</li>
<li>Backend successfully connected to Docusaurus UI.</li>
<li>GitHub integration ready for final submission.</li>
<li>Each chapter validated for required sections, correct citations (APA), working code examples, and accurate diagrams.</li>
<li>Chatbot responses tested with 20+ questions per chapter; must return "cannot answer from context" if out-of-scope.</li>
<li>Creative UI implemented as specified (color theme, interactive components, responsiveness).</li>
</ul>

<h3>Output Validation:</h3>
<ul>
<li><strong>Coursebook:</strong> Automated checks for broken links, image loading, responsive design; linting for markdown content; Docusaurus build validation.</li>
<li><strong>Chatbot:</strong> Automated unit and integration tests for response accuracy and grounding; manual review of sample interactions; load testing for performance.</li>
</ul>

<h2>9. Architectural Decision Record (ADR)</h2>
<ul>
<li><strong>ADR Suggestions:</strong>
<ul>
<li>Coursebook Content Generation Strategy: Spec-Kit Plus driven.</li>
<li>RAG Chatbot Grounding Strategy: Strict content adherence vs. broader knowledge.</li>
<li>Frontend Framework Choice: Docusaurus vs. other static site generators/frameworks.</li>
<li>RAG Chatbot Backend Technologies: FastAPI, Neon Postgres, Qdrant Vector DB.</li>
<li>Interactive UI Component Implementation: Custom React components vs. Docusaurus plugins.</li>
</ul>
</li>
</ul>

## Technical Details:
- Use research-concurrent approach (research while writing, not all upfront)
- Follow APA citation style from Constitution
- Organize by phases: Research → Foundation → Analysis → Synthesis

Implementation will be done in TWO PHASES:

Phase-1: Frontend Coursebook (Docusaurus + Spec-Kit Plus)
- Use Docusaurus docs: https://docusaurus.io/docs through Context7 MCP server
- Complete coursebook with minimum 8 chapters
- Each chapter includes Overview, Learning Objectives, Theory, Diagrams, Code Samples, Exercises, References
- Creative UI:
  - Color Theme: Primary Deep Blue (#0A3D62), Accent Neon Cyan (#00FFF7), Neutral Background Light Gray (#F5F5F5), Text Dark Gray (#1E1E1E), Hover Neon Green (#39FF14)
  - Interactive components: collapsible theory, code tabs, image galleries, exercise/practice cards
  - Smooth scroll, sidebar navigation, optional dark mode, fully responsive

Phase-2: RAG Chatbot Integration
- Embedded chatbot strictly grounded in coursebook content (Option A from clarify phase)
- Connect to MCP server for context management
- Optional Better-Auth signup for user personalization and Urdu translation
- Test chatbot responses: 20+ questions per chapter, must return "cannot answer from context" if out-of-scope
- Validate chatbot functionality after full frontend deployment