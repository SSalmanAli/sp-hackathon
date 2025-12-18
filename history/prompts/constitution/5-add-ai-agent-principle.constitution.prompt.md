---
id: 5
title: Add AI Agent Principle
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "ai-agent", "rag-architecture", "principle"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

update constitution The system must include an AI Answering Agent responsible solely for reasoning over provided context and responding to user queries within the Physical AI Book chatbot. This agent must be implemented using the OpenAI SDK configured with a Gemini-compatible API key and must strictly follow tool-augmented reasoning. The agent is not permitted to retrieve raw data directly; instead, it must rely on an explicit Content Extraction tool provided to it, which returns structured book content or user-selected text. The agent must always prefer tool usage when context is required and must never hallucinate content outside the extracted results. All responses must be grounded exclusively in tool-provided context, clearly structured, and suitable for direct rendering in the chatbot UI. If the extracted content does not contain the answer, the agent must explicitly state that the information is not available in the book. The agent must maintain deterministic, explainable behavior, respect UI constraints, and remain modular

## Response snapshot

Updated the Physical AI Book constitution to include a new principle "XI. AI Answering Agent as First-Class Capability" that establishes the requirements for an AI answering agent in the chatbot. The principle specifies that the agent must use tool-augmented reasoning, rely on a Content Extraction tool for context, never hallucinate content, ground all responses in tool-provided context, and maintain deterministic, explainable behavior. The version was incremented from 1.4.0 to 1.5.0 to reflect this minor addition.

## Outcome

- ✅ Impact: New AI Answering Agent principle added to constitution
- 🧪 Tests: none
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: none
- 🧠 Reflection: Clear architectural boundaries help maintain separation of concerns in AI systems

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A