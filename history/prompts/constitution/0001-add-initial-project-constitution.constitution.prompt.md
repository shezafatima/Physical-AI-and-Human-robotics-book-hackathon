---
id: 0001
title: Add Initial Project Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["governance", "initial setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:

---

## Prompt

```text
Project: AI-Native Interactive Coursebook + RAG Chatbot for “Physical AI & Humanoid Robotics”

Core principles:
- End-to-end generative workflow using Spec-Kit Plus + Claude Code
- Accuracy and consistency with the official course outline
- Modular chapter architecture for easy regeneration
- Clear technical explanations suitable for engineering students
- RAG chatbot must answer strictly from book context (no hallucination)

Key standards:
- Book generated entirely from spec files (one spec per chapter/module)
- All code examples must be validated and executable
- Vector embeddings must be generated consistently across chapters
- API communication must follow OpenAI Agents + ChatKit SDK best practices
- Docusaurus structure must remain clean, readable, and SEO-friendly

Constraints:
- Minimum 8 full chapters matching weekly course modules
- Each chapter must include: overview, learning objectives, diagrams, code samples, exercises
- Chatbot must use: FastAPI, Neon Postgres, Qdrant Vector DB
- Website mus
```

## Response snapshot

Updated the project constitution to version 1.0.0 based on the provided principles, standards, and constraints. Identified follow-up actions for template alignment and setting the ratification date.

## Outcome

- ✅ Impact: Constitution updated, initial project governance established.
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Review template alignment and set ratification date.
- 🧠 Reflection: Successfully updated constitution and identified next steps.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
