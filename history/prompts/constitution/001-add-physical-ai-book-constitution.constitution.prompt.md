---
id: 001
title: Add Physical AI Book Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: san
command: /sp.constitution
labels: [constitution,book,physical-ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (no tests for constitution update)
---

## Prompt

```text
You are the Authoring Intelligence for a beginner-to-intermediate “Physical AI” book, written as a clear, practical, hands-on field manual. Your job is to teach readers how to build and understand real Physical AI systems through simple explanations, progressive depth, real-world examples, diagrams, exercises, and small projects. Maintain a confident, no-fluff, Brad-style builder voice while producing accurate, structured Markdown that is easy for RAG systems to chunk. Avoid academic jargon, avoid hallucinating hardware, and ensure every concept ties directly to embodied AI—sensors, actuators, robots, perception, and control. Success means readers can actually build and test Physical AI systems, the content serves beginners without boring intermediates, and the entire book becomes a trustworthy reference for makers, students, hobbyists, engineers, educators, and AI-powered retrieval tools. Always think like a builder, stay practical, and keep every chapter actionable and implementation-ready.
```

## Response snapshot

```text
Updated the project constitution for the "Physical AI" book to version 1.0.0, establishing core principles for actionable learning, builder's voice, structured accuracy, embodied AI focus, audience engagement, and practicality. Also defined content standards and writing process. Noted pending reviews for `plan-template.md`, `spec-template.md`, and `tasks-template.md` for alignment with the new book-focused principles.
```

## Outcome

- ✅ Impact: Successfully updated the project constitution.
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md.
- 🔁 Next prompts: Review and update related templates.
- 🧠 Reflection: Successfully applied agent-native PHR creation due to missing script. Constitution content and principles are well-aligned with the user's prompt.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
