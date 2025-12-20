# Tasks: AI-Native Interactive Coursebook + RAG Chatbot

**Branch**: `1-coursebook-rag-chatbot` | **Date**: 2025-12-07 | **Spec**: [specs/1-coursebook-rag-chatbot/spec.md](specs/1-coursebook-rag-chatbot/spec.md) | **Plan**: [specs/1-coursebook-rag-chatbot/plan.md](specs/1-coursebook-rag-chatbot/plan.md)

## Summary

This document outlines the concrete, executable tasks for building the AI-Native Interactive Coursebook and RAG Chatbot. Tasks are organized by user story priority, with foundational and cross-cutting concerns addressed in initial and final phases. The goal is to deliver an 8+ chapter Docusaurus coursebook with an embedded, context-grounded RAG chatbot, deployable to GitHub Pages.

## Implementation Strategy

The project will follow an iterative, user story-driven implementation strategy, focusing on delivering a Minimum Viable Product (MVP) first, consisting of User Story 1 (Read Course Content) and User Story 2 (Interact with RAG Chatbot). Subsequent user stories and bonus features will be integrated incrementally.

## Phase 1: Setup

**Purpose**: Initialize project structure and core configurations.

### Tasks

- [X] T001 Create repository root directories for `backend/` and `frontend/`
- [X] T002 Initialize Git repository and configure `.gitignore`
- [X] T003 Initialize Docusaurus project in `frontend/` directory
- [X] T004 Configure basic Docusaurus theme and navigation in `frontend/docusaurus.config.js`
- [X] T005 Initialize FastAPI project in `backend/` directory
- [X] T006 Configure `backend/requirements.txt` with initial dependencies (FastAPI, uvicorn, qdrant-client, psycopg2-binary, openai, langchain)
- [X] T007 Set up local development environment for Python and Node.js

## Phase 2: Foundational Components

**Purpose**: Establish core infrastructure for content generation, data persistence, and semantic representation.

### Tasks

- [X] T008 [P] Configure MCP server (Context7) for local context operations (if not already done)
- [X] T009 [P] Implement base database connection and configuration in `backend/src/services/database.py`
- [X] T010 [P] Implement Qdrant client initialization and connection in `backend/src/services/vector_db.py`
- [X] T011 [P] Develop a content parsing and chunking utility for text to be embedded in `backend/src/services/content_processor.py`
- [X] T012 [P] Implement content embedding generation logic using OpenAI embeddings in `backend/src/services/embedding_generator.py`

## Phase 3: User Story 1 - Read Course Content (P1)

**Goal**: Enable users to read comprehensive, generated course chapters.

**Independent Test**: Navigate to any chapter on the deployed coursebook website, confirm all sections (overview, objectives, theory, diagrams, code, exercises) are present, content renders correctly, and code samples appear executable.

### Tasks

- [X] T013 [US1] Develop content generation scripts for chapters (theory, diagrams, code, exercises) in `.specify/scripts/content_generation.py`
- [X] T014 [US1] Generate initial content for "Introduction to Physical AI" chapter in `frontend/docs/chapters/chapter1.md`
- [X] T015 [US1] Generate initial content for "ROS 2 Fundamentals" chapter in `frontend/docs/chapters/chapter2.md`
- [X] T016 [US1] Generate initial content for "Robot Simulation with Gazebo & Unity" chapter in `frontend/docs/chapters/chapter3.md`
- [X] T017 [US1] Generate initial content for "NVIDIA Isaac Platform" chapter in `frontend/docs/chapters/chapter4.md`
- [X] T018 [US1] Generate initial content for "Humanoid Robot Development" chapter in `frontend/docs/chapters/chapter5.md`
- [X] T019 [US1] Generate initial content for "Vision-Language-Action (VLA)" chapter in `frontend/docs/chapters/chapter6.md`
- [X] T020 [US1] Generate initial content for "Conversational Robotics & GPT Integration" chapter in `frontend/docs/chapters/chapter7.md`
- [X] T021 [US1] Generate initial content for "Capstone Project: Autonomous Humanoid" chapter in `frontend/docs/chapters/chapter8.md`
- [X] T022 [US1] Implement Docusaurus configuration to include all generated chapters in `frontend/docusaurus.config.js`
- [X] T023 [US1] Validate chapter completeness and rendering for all generated chapters in `frontend/`
- [X] T024 [US1] Add APA citations to generated chapter content in `frontend/docs/**/*.md` (requires manual or automated check)

## Phase 4: User Story 2 - Interact with RAG Chatbot (P1)

**Goal**: Enable students to get accurate, context-grounded answers from a RAG chatbot.

**Independent Test**: Ask 20+ questions per chapter to the chatbot. 95% of direct questions must be answered correctly from book context, and out-of-scope questions must receive a "cannot answer" response.

### Tasks

- [X] T025 [P] [US2] Create OpenAPI specification for chatbot API in `backend/contracts/chatbot_api.yaml`
- [X] T026 [US2] Implement FastAPI endpoint for chatbot queries (`/chat`) in `backend/src/api/chatbot.py`
- [X] T027 [US2] Implement RAG retrieval logic using Qdrant in `backend/src/services/rag_service.py`
- [X] T028 [US2] Implement response generation logic (strict grounding) using LLM in `backend/src/services/rag_service.py`
- [X] T029 [US2] Develop Docusaurus React component for chatbot UI in `frontend/src/components/Chatbot.jsx`
- [X] T030 [US2] Integrate chatbot UI component into Docusaurus layout/pages in `frontend/src/components/CourseLayout.jsx`
- [X] T031 [US1] Implement frontend service for interacting with backend chatbot API in `frontend/src/services/api.js`
- [X] T032 [US2] Ingest all generated chapter content into Qdrant for vector embeddings (script or one-time process)
- [X] T033 [US2] Conduct extensive testing of the RAG chatbot with 20+ context-based questions per chapter

## Phase 5: User Story 3 - Access Code Examples (P2)

**Goal**: Ensure all code examples are executable and clearly presented.

**Independent Test**: Verify all code examples compile/run correctly in their respective simulated environments (Gazebo/Isaac Sim) and are well-formatted in the coursebook.

### Tasks

- [X] T034 [US3] Review and format all code examples within `frontend/docs/**/*.md` for readability and syntax highlighting
- [X] T035 [US3] Set up CI/CD or local scripts to automatically test code examples in Gazebo/Isaac Sim (e.g., `sim_validator.py`)
- [X] T036 [US3] Document setup procedures for running code examples locally in `docs/running_examples.md`
- [X] T037 [US3] Validate all code examples compile and run successfully in simulated environments

## Phase 6: User Story 4 - Complete Exercises (P2)

**Goal**: Provide clear and actionable end-of-chapter exercises.

**Independent Test**: Confirm that each chapter concludes with a dedicated, understandable section of exercises.

### Tasks

- [X] T038 [US4] Review all generated chapter exercises in `frontend/docs/**/*.md` for clarity, relevance, and actionability

## Phase 7: ADR (Architecture Decision Records) & Bonus Features

**Goal**: Document key architectural decisions and integrate optional bonus features.

**Independent Test**: All key architectural decisions are formally documented with options and trade-offs. Optional features are functional if enabled.

### Tasks

- [X] T039 Create ADR for "Coursebook Content Depth" (Comprehensive) in `history/adr/001-coursebook-depth.md`
- [X] T040 Create ADR for "RAG Chatbot Behavior" (Strict Grounding) in `history/adr/002-rag-chatbot-behavior.md`
- [X] T041 Create ADR for "Chapter Count" (8+ minimum) in `history/adr/003-chapter-count.md`
- [X] T042 Create ADR for "Personalization" (Optional Better-Auth) in `history/adr/004-personalization.md`
- [X] T043 Create ADR for "Translation" (Optional Urdu) in `history/adr/005-translation.md`
- [X] T044 Create ADR for "Citation Style" (APA) in `history/adr/006-citation-style.md`
- [X] T045 Implement optional Better-Auth signup for user personalization (frontend & backend)
- [X] T046 Implement optional Urdu translation feature for a single chapter (frontend & RAG adaptation)

## Phase 8: Deployment & Polish

**Goal**: Deploy the coursebook and chatbot, ensure quality, and finalize documentation.

**Independent Test**: The complete book and chatbot are deployed and fully functional on GitHub Pages, meeting all success criteria.

### Tasks

- [X] T047 Configure Docusaurus for deployment to GitHub Pages in `frontend/docusaurus.config.js`
- [X] T048 Set up GitHub Actions workflow for automated Docusaurus deployment in `.github/workflows/deploy.yml`
- [X] T049 Implement automated plagiarism check for all generated content in `backend/src/services/plagiarism_checker.py`
- [X] T050 Perform final end-to-end testing of the entire system (book navigation, chatbot, code examples, exercises)
- [X] T051 Update `README.md` with project overview, setup instructions, and deployment details
- [X] T052 Ensure all documentation (spec, plan, tasks, ADRs) is up-to-date and consistent

## Dependency Graph

*   Phase 1 (Setup) -> Phase 2 (Foundational Components)
*   Phase 2 (Foundational Components) -> Phase 3 (User Story 1 - Read Course Content)
*   Phase 2 (Foundational Components) -> Phase 4 (User Story 2 - Interact with RAG Chatbot)
*   Phase 3 (User Story 1) -> Phase 5 (User Story 3 - Access Code Examples)
*   Phase 3 (User Story 1) -> Phase 6 (User Story 4 - Complete Exercises)
*   Phase 4 (User Story 2) -> Phase 7 (ADR & Bonus Features - for full context)
*   Phase 7 (ADR & Bonus Features) -> Phase 8 (Deployment & Polish)

## Parallel Execution Opportunities

*   **Phase 1 & 2:** Many setup and foundational tasks can be parallelized (e.g., Docusaurus setup, FastAPI setup, Qdrant/Neon client setup).
*   **Within Phase 3 (User Story 1):** Generating individual chapter content (T0014-T0021) can be done in parallel.
*   **Within Phase 4 (User Story 2):** Developing chatbot API spec (T0025), implementing RAG retrieval/response (T0027-T0028), and developing frontend UI (T0029) can be done in parallel.
*   **Phase 5 (User Story 3):** Code example review (T0034), CI/CD setup (T0035), and documentation (T0036) can be parallelized.
*   **Phase 7 (ADR & Bonus Features):** Creating individual ADRs (T0039-T0044) can be done in parallel. Implementing Better-Auth (T0045) and Urdu translation (T0046) can be parallelized with each other, but depend on relevant foundational components.

## Suggested MVP Scope

The MVP will focus on delivering the core coursebook content and the strictly context-grounded RAG chatbot. This includes:

*   **Phase 1: Setup**
*   **Phase 2: Foundational Components**
*   **Phase 3: User Story 1 - Read Course Content**
*   **Phase 4: User Story 2 - Interact with RAG Chatbot**
*   **Key parts of Phase 8**: Deployment to GitHub Pages (T0047, T0048) and initial `README.md` (T0051).

This MVP will provide a fully functional, navigable coursebook with an interactive RAG chatbot, allowing students to access information and ask questions within the course context. The remaining phases (User Story 3 & 4, remaining ADRs, full bonus features, comprehensive polish) can be pursued as fast follows.