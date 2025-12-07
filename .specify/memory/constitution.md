<!--
Sync Impact Report:
Version change: 1.0.0 → 1.0.1
Modified principles: None
Added sections: Success Criteria
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: TODO(RATIFICATION_DATE): Needs to be set upon official adoption.
-->
# AI-Native Interactive Coursebook + RAG Chatbot for “Physical AI & Humanoid Robotics” Constitution

## Core Principles

### End-to-end Generative Workflow
Utilize Spec-Kit Plus and Claude Code for a fully generative development workflow, ensuring consistency and automation.

### Course Outline Accuracy
Maintain strict accuracy and consistency with the official course outline to ensure educational integrity.

### Modular Chapter Architecture
Design chapters with a modular architecture to facilitate easy regeneration, updates, and maintenance.

### Clear Technical Explanations
Provide clear, concise, and accurate technical explanations suitable for engineering students.

### RAG Chatbot Context Adherence
The RAG chatbot MUST answer strictly from the provided book context, preventing hallucinations.

## Key Standards

*   Book Content Generation: Entirely from spec files (one spec per chapter/module).
*   Code Example Validation: All code examples must be validated and executable.
*   Vector Embeddings Consistency: Vector embeddings must be generated consistently across chapters.
*   API Communication: Follow OpenAI Agents + ChatKit SDK best practices.
*   Website Structure: Docusaurus structure must remain clean, readable, and SEO-friendly.

## Project Constraints

*   Minimum Chapters: 8 full chapters matching weekly course modules.
*   Chapter Content: Each chapter must include: overview, learning objectives, diagrams, code samples, exercises.
*   Chatbot Technologies: FastAPI, Neon Postgres, Qdrant Vector DB.
*   Website Framework: Docusaurus.
*   Website Deployment: Must be deployable to GitHub Pages or Vercel.
*   MCP Server: Context7 must be connected for local context operations.

## Success Criteria

*   Complete book deployed on Docusaurus and fully navigable.
*   RAG chatbot embedded and able to answer using selected text only.
*   All spec files validated by Spec-Kit Plus without errors.
*   Backend successfully connected to Docusaurus UI.
*   GitHub integration ready for final submission.

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance.

**Version**: 1.0.1 | **Ratified**: TODO(RATIFICATION_DATE): Needs to be set upon official adoption. | **Last Amended**: 2025-12-06
