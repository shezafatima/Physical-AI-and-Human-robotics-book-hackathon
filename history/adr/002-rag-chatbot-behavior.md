# ADR 002: RAG Chatbot Behavior

## Status
Accepted

## Context
The RAG (Retrieval-Augmented Generation) chatbot needs to behave in a specific way when answering questions. There was a decision to be made about whether the chatbot should attempt to answer all questions or strictly ground responses in course materials.

## Decision
We will implement a strictly context-grounded approach where the chatbot will:

- Only answer questions based on information available in the course materials
- Respond with "cannot answer from context" for out-of-scope questions
- Provide references to specific course content when answering
- Include confidence scores for retrieved information

## Rationale
This approach ensures academic integrity and prevents the chatbot from generating potentially incorrect information. Students will receive accurate answers that are directly tied to the course content, reinforcing their learning.

## Consequences
- Positive: Students receive accurate, course-grounded information
- Positive: Prevents hallucination of facts
- Negative: May be less flexible for tangential questions
- Negative: Requires comprehensive course content for effective responses