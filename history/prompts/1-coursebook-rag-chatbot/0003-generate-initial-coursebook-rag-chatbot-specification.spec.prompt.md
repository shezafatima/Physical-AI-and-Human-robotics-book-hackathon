---
id: 0003
title: Generate Initial Coursebook RAG Chatbot Specification
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20251101
feature: 1-coursebook-rag-chatbot
branch: master
user: unknown
command: /sp.specify
labels: ["feature spec", "rag chatbot", "coursebook"]
links:
  spec: specs/1-coursebook-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-coursebook-rag-chatbot/spec.md
  - specs/1-coursebook-rag-chatbot/checklists/requirements.md
tests:

---

## Prompt

```text
AI-Native Interactive Coursebook + RAG Chatbot for “Physical AI & Humanoid Robotics”

Target audience: Engineering students and beginners learning robotics, ROS 2, simulation tools, and humanoid systems
Focus: Generating a fully structured Docusaurus coursebook + integrating a context-based RAG chatbot using FastAPI, Qdrant, and OpenAI Agents

Success criteria:
- Produces 8+ fully written chapters aligned with the official weekly course structure
- Generates clean, structured Markdown suitable for Docusaurus
- Includes diagrams, code samples, learning objectives, and end-of-chapter exercises
- RAG chatbot correctly answers questions based only on book context
- Backend (FastAPI + Qdrant + Neon) fully spec’d with endpoints, flow, and integration steps

Constraints:
- Minimum 8 chapters, each with overview → theory → examples → exercises
- All outputs in Markdown, spec-driven, and regenerable via Spec-Kit Plus
- Chatbot must use: FastAPI, Qdrant Vector DB, Neon Postgres, OpenAI Agents/ChatKit SDK
- Book deployable on GitHub Pages or Vercel
- Timeline: Complete generation and integration within hackathon submission window

Not building:
- Full robotics simulations or hardware control programs
- Advanced ROS 2 real-world robot deployment
- Integration with robotics hardware (Jetson, sensors, actuators)
- Full-fledged multilingual translation system (only single-chapter optional Urdu button)
```

## Response snapshot

Created the specification for the AI-Native Interactive Coursebook + RAG Chatbot, including user scenarios, functional requirements, key entities, and success criteria. The spec has been validated and all checklist items passed after refactoring to remove implementation details.

## Outcome

- ✅ Impact: Feature specification created and validated.
- 🧪 Tests: N/A
- 📁 Files: specs/1-coursebook-rag-chatbot/spec.md, specs/1-coursebook-rag-chatbot/checklists/requirements.md created/updated.
- 🔁 Next prompts: Proceed to planning phase using `/sp.plan`.
- 🧠 Reflection: Successfully parsed user input, generated a comprehensive spec, and refined it to be technology-agnostic during validation.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec failed validation due to implementation details leakage, resolved by refactoring.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
