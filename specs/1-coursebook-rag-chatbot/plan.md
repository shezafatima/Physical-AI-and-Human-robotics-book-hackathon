# Implementation Plan: AI-Native Interactive Coursebook + RAG Chatbot for “Physical AI & Humanoid Robotics”

**Branch**: `1-coursebook-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [specs/1-coursebook-rag-chatbot/spec.md](specs/1-coursebook-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/1-coursebook-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create an AI-native interactive coursebook on Physical AI & Humanoid Robotics, structured with 8+ comprehensive chapters and powered by a RAG chatbot. The coursebook will be deployed on GitHub Pages and the chatbot will be strictly grounded in course context, integrated seamlessly with the Docusaurus-based website. Optional Better-Auth for personalization and Urdu translation are bonus features.

## Technical Context

**Language/Version**: Python 3.11+ (for backend), JavaScript/TypeScript (for Docusaurus frontend)
**Primary Dependencies**: FastAPI, Qdrant, Neon Postgres (for backend); React, Docusaurus (for frontend); OpenAI Agents/ChatKit SDK (for RAG logic)
**Storage**: Neon Postgres (for relational data), Qdrant (for vector embeddings), local filesystem/GitHub Pages (for Docusaurus static content)
**Testing**: Pytest (for backend), Jest/React Testing Library (for frontend), end-to-end tests for chatbot interaction and book navigation.
**Target Platform**: Web (Docusaurus static site), Cloud/Serverless (FastAPI backend for chatbot).
**Project Type**: Web application (frontend + backend)
**Performance Goals**: RAG chatbot responds within 5 seconds for 90% of interactions.
**Constraints**: Minimum 8 comprehensive chapters, strictly context-based RAG chatbot, deployable to GitHub Pages or Vercel, complete within hackathon submission window, MCP server (Context7) connected.
**Scale/Scope**: Engineering students and beginners, 8+ chapters, 1 RAG chatbot instance.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

*   **End-to-end Generative Workflow**: Utilizes Spec-Kit Plus and Claude Code for generation. **Pass**.
*   **Course Outline Accuracy**: Content generation will adhere to official course outline. **Pass**.
*   **Modular Chapter Architecture**: Chapters designed for easy regeneration. **Pass**.
*   **Clear Technical Explanations**: Spec emphasizes comprehensive and clear explanations. **Pass**.
*   **RAG Chatbot Context Adherence**: Chatbot strictly grounded in coursebook context. **Pass**.
*   **Book Content Generation**: Entirely from spec files. **Pass**.
*   **Code Example Validation**: All code examples will be validated and executable. **Pass**.
*   **Vector Embeddings Consistency**: Embeddings generated consistently across chapters. **Pass**.
*   **API Communication**: Follows OpenAI Agents + ChatKit SDK best practices (explicitly for RAG, implicitly for general API via RESTful JSON/OpenAPI clarification). **Pass**.
*   **Website Structure**: Docusaurus structure clean, readable, SEO-friendly. **Pass**.
*   **Minimum Chapters**: Minimum 8 chapters. **Pass**.
*   **Chapter Content**: Each chapter includes overview, learning objectives, diagrams, code samples, exercises. **Pass**.
*   **Chatbot Technologies**: FastAPI, Neon Postgres, Qdrant Vector DB. **Pass**.
*   **Website Framework**: Docusaurus. **Pass**.
*   **Website Deployment**: Deployable to GitHub Pages or Vercel. **Pass**.
*   **MCP Server**: Context7 must be connected for local context operations. **Pass**.

## Project Structure

### Documentation (this feature)

```text
specs/1-coursebook-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/           # Pydantic models for data (e.g., Chapter, Query, Response)
│   ├── services/         # Business logic, RAG implementation, Qdrant/Neon interaction
│   └── api/              # FastAPI endpoints for chatbot, content retrieval
└── tests/

frontend/ (Docusaurus project)
├── src/
│   ├── components/       # React components (e.g., Chatbot UI, custom content rendering)
│   ├── pages/            # Docusaurus pages (e.g., individual chapters, home)
│   └── services/         # Frontend API interaction, local state management
└── tests/

# Root level
.github/workflows/   # CI/CD for Docusaurus deployment to GitHub Pages
.specify/            # Spec-Kit Plus configurations, templates, scripts
history/             # Prompt History Records, ADRs
docs/                # Docusaurus generated content and assets (may move to frontend/build)
```

**Structure Decision**: The project will adopt a split `backend/` and `frontend/` (Docusaurus) structure at the repository root to clearly separate the RAG chatbot API from the static coursebook website, facilitating independent development and deployment while allowing seamless integration.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
