---
id: 4
title: Add Data Retrieval Principle
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "data-retrieval", "rag-architecture", "principle"]
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

update constitution, The system must treat Data Retrieval as a first-class, independent capability within the Physical AI Book RAG architecture. Retrieval responsibilities are strictly limited to semantic search and must never include prompt construction, LLM generation, or UI logic. The retrieval process must embed user queries using the same Cohere embedding model used during indexing, query the existing Qdrant vector database, apply relevance scoring and similarity thresholds, and return only the most relevant content chunks. All retrieved results must be deduplicated, relevance-sorted, and returned with complete metadata, including module, lesson, section, and chunk identifiers. Retrieval outputs must be deterministic, accurate, and optimized for downstream RAG consumption, with clear structure and minimal noise. The system must prioritize correctness, traceability, and production readiness, ensuring retrieval can be independently validated and reused by higher-level agents without modification.

## Response snapshot

Updated the Physical AI Book constitution to include a new principle about Data Retrieval as a first-class capability within the RAG architecture. The new principle (X. Data Retrieval as First-Class Capability) defines strict boundaries for retrieval responsibilities, specifying that retrieval is limited to semantic search and must not include prompt construction, LLM generation, or UI logic. The principle also details technical requirements for the retrieval process including using the same Cohere embedding model, querying Qdrant vector database, applying relevance scoring, and returning properly structured results with complete metadata.

## Outcome

- ✅ Impact: New RAG architecture principle added to constitution
- 🧪 Tests: none
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: none
- 🧠 Reflection: Clear architectural boundaries help maintain separation of concerns in RAG systems

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A