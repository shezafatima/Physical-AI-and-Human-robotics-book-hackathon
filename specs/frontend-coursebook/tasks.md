# Frontend Coursebook + RAG Chatbot Tasks

## Implementation Strategy
- MVP first, incremental delivery.
- Frontend Coursebook will be developed in parallel with the RAG Chatbot backend, with integration as a final step.

## Dependencies
- Phase 1 (Setup) must be completed before Phase 2.
- Phase 2 (Foundational - Frontend Coursebook Structure) must be completed before Phase 3.
- Phase 3 (User Story 1: Frontend Coursebook Chapters) and Phase 4 (User Story 2: RAG Chatbot Backend) can be developed in parallel up to the integration steps.
- Phase 5 (Integration & Polish) depends on completion of Phase 3 and 4.

## Phase 1: Setup

- [ ] T001 Initialize Docusaurus project in the root directory.
- [ ] T002 Integrate Spec-Kit Plus into the Docusaurus build process.

## Phase 2: Foundational - Frontend Coursebook Structure

- [ ] T003 Configure Docusaurus basic layout and navigation in `docusaurus.config.js`.
- [ ] T004 Set up custom color theme (Primary Deep Blue, Accent Neon Cyan, etc.) in `src/css/custom.css`.
- [ ] T005 Implement smooth scroll functionality (if not default) and sidebar navigation.
- [ ] T006 Implement optional dark mode toggle.
- [ ] T007 Ensure responsive design across various screen sizes by applying CSS media queries.

## Phase 3: User Story 1: Frontend Coursebook Chapters (P1)

### Story Goal
Develop a comprehensive coursebook with a minimum of 8 chapters, each containing specified interactive and content elements, adhering to APA citation style.

### Independent Test Criteria
- All 8 chapters are accessible and navigable on the Docusaurus site.
- Each chapter contains Overview, Learning Objectives, Theory, Diagrams, Code Samples, Exercises, and References sections.
- Interactive components (collapsible theory, code tabs, image galleries, practice cards) function correctly.
- All content adheres to APA citation style.
- All code examples are functional and accurate.
- All diagrams are accurate and display correctly.

### Implementation Tasks

- [ ] T008 [P] [US1] Create `docs/chapter1.md` with Overview, Learning Objectives, Theory, Diagrams, Code Samples, Exercises, References sections.
- [ ] T009 [P] [US1] Implement collapsible component for Theory section in `src/components/CollapsibleTheory.js`.
- [ ] T010 [P] [US1] Implement code tabs component for Code Samples section in `src/components/CodeTabs.js`.
- [ ] T011 [P] [US1] Implement image gallery component for Diagrams section in `src/components/ImageGallery.js`.
- [ ] T012 [P] [US1] Implement exercise/practice card component for Exercises section in `src/components/PracticeCard.js`.
- [ ] T013 [P] [US1] Populate `docs/chapter1.md` with initial content and apply APA citations.
- [ ] T014 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter1.md`.
- [ ] T015 [P] [US1] Create `docs/chapter2.md` and populate with content and interactive components.
- [ ] T016 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter2.md`.
- [ ] T017 [P] [US1] Create `docs/chapter3.md` and populate with content and interactive components.
- [ ] T018 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter3.md`.
- [ ] T019 [P] [US1] Create `docs/chapter4.md` and populate with content and interactive components.
- [ ] T020 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter4.md`.
- [ ] T021 [P] [US1] Create `docs/chapter5.md` and populate with content and interactive components.
- [ ] T022 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter5.md`.
- [ ] T023 [P] [US1] Create `docs/chapter6.md` and populate with content and interactive components.
- [ ] T024 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter6.md`.
- [ ] T025 [P] [US1] Create `docs/chapter7.md` and populate with content and interactive components.
- [ ] T026 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter7.md`.
- [ ] T027 [P] [US1] Create `docs/chapter8.md` and populate with content and interactive components.
- [ ] T028 [P] [US1] Validate content, citations, code, and diagrams for `docs/chapter8.md`.

## Phase 4: User Story 2: RAG Chatbot Backend (P1)

### Story Goal
Implement an embedded RAG chatbot strictly grounded in the coursebook content, with optional user personalization and Urdu translation, and robust testing.

### Independent Test Criteria
- Chatbot is embedded in the Docusaurus site and responds to queries.
- Chatbot responses are strictly grounded in coursebook content and return "cannot answer from context" for out-of-scope questions.
- Chatbot successfully connects to the MCP server for context management.
- If enabled, Better-Auth allows user signup and personalization.
- If enabled, Urdu translation functions correctly.
- Chatbot passes 20+ test questions per chapter.

### Implementation Tasks

- [ ] T029 [P] [US2] Set up FastAPI project structure in `backend/fastapi_app/`.
- [ ] T030 [P] [US2] Configure Neon Postgres database for chatbot backend.
- [ ] T031 [P] [US2] Integrate Qdrant Vector DB for vector embeddings.
- [ ] T032 [P] [US2] Implement content processing pipeline for coursebook to embeddings in `backend/scripts/process_content.py`.
- [ ] T033 [P] [US2] Develop `/chat` API endpoint in `backend/fastapi_app/main.py`.
- [ ] T034 [P] [US2] Integrate chatbot with MCP server for context management.
- [ ] T035 [P] [US2] Implement optional Better-Auth signup and user personalization.
- [ ] T036 [P] [US2] Implement Urdu translation feature (if Better-Auth is integrated).
- [ ] T037 [P] [US2] Develop comprehensive chatbot test suite in `backend/tests/chatbot_tests.py`.
- [ ] T038 [P] [US2] Test chatbot responses with 20+ questions per chapter.
- [ ] T039 [P] [US2] Create Docusaurus plugin for embedding chatbot UI into the frontend.
- [ ] T040 [P] [US2] Integrate chatbot frontend components into Docusaurus.

## Phase 5: Integration & Polish

### Story Goal
Ensure the entire system (coursebook and chatbot) is fully integrated, validated, and ready for deployment.

### Independent Test Criteria
- Complete book deployed on Docusaurus and fully navigable.
- RAG chatbot embedded and able to answer using selected text only.
- All spec files validated by Spec-Kit Plus without errors.
- Backend successfully connected to Docusaurus UI.
- GitHub integration ready for final submission.
- All previous independent test criteria are met.

### Implementation Tasks

- [ ] T041 Integrate frontend coursebook with RAG chatbot backend.
- [ ] T042 Configure GitHub Pages or Vercel for deployment.
- [ ] T043 Perform end-to-end system validation based on Definition of Done.
- [ ] T044 Final review and update of `constitution.md` if necessary.

## Summary

- Total task count: 44
- Task count per user story:
    - Setup: 2
    - Foundational - Frontend Coursebook Structure: 5
    - User Story 1: Frontend Coursebook Chapters: 21
    - User Story 2: RAG Chatbot Backend: 12
    - Integration & Polish: 4
- Parallel opportunities identified: Tasks within User Story 1 and User Story 2 phases can be developed in parallel.
- Independent test criteria for each story are provided within their respective phases.
- Suggested MVP scope: Completion of Phase 1, Phase 2, and initial tasks of Phase 3 (e.g., Chapter 1-2) to establish the basic coursebook structure and content.
- Format validation: All tasks follow the checklist format (checkbox, ID, optional [P], optional [Story], description with file path).
