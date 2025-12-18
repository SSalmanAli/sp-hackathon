<!--
Sync Impact Report:
Version change: 1.4.0 → 1.5.0
Modified principles: None
Added sections:
  - XI. AI Answering Agent as First-Class Capability
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI Book Constitution

## Core Principles

### I. Actionable Learning
Every concept and explanation must directly enable readers to build and test Physical AI systems. Content must be implementation-ready, featuring simple explanations, progressive depth, real-world examples, diagrams, exercises, and small projects.

### II. Builder's Voice
Maintain a confident, no-fluff, Brad-style builder voice. Avoid academic jargon to ensure accessibility for beginner-to-intermediate readers.

### III. Structured Accuracy
Produce accurate, structured Markdown that is easy for RAG systems to chunk. Prioritize technical truthfulness and avoid hallucinating hardware.

### IV. Embodied AI Focus
Every concept must tie directly to embodied AI, covering sensors, actuators, robots, perception, and control. This ensures a practical focus on real Physical AI systems.

### V. Audience Engagement
Content must serve beginners without boring intermediates, fostering a trustworthy reference for a broad audience of makers, students, hobbyists, engineers, educators, and AI-powered retrieval tools.

### VI. Practicality First
Always think like a builder, staying practical and ensuring every chapter is actionable and implementation-ready.

## UI/UX Principles

### VII. YouTube-Inspired Color Scheme
The user interface must feature the distinctive YouTube color scheme of red (#FF0000), white (#FFFFFF), and black (#000000) to create a recognizable and high-contrast visual identity. The primary color should be red for key interactive elements and branding, white for backgrounds and text contrast, and black for text and accent elements to ensure maximum readability and visual impact.

### VIII. Structured Information Architecture
The landing page must follow a logical information flow with clearly defined sections: Hero section for immediate impact, Overview section for quick understanding, Learning Outcomes section for value proposition, and Features section for detailed benefits.

### IX. Interactive Chat Support Modal
The user interface must include a floating chat button that opens a chat modal for AI-powered support. The chat modal must be easily accessible, visually distinct, and provide immediate assistance to users about the book content. The modal must close when clicked outside the container to ensure intuitive user interaction.

## RAG Architecture Principles

### X. Data Retrieval as First-Class Capability
The system must treat Data Retrieval as a first-class, independent capability within the Physical AI Book RAG architecture. Retrieval responsibilities are strictly limited to semantic search and must never include prompt construction, LLM generation, or UI logic. The retrieval process must embed user queries using the same Cohere embedding model used during indexing, query the existing Qdrant vector database, apply relevance scoring and similarity thresholds, and return only the most relevant content chunks. All retrieved results must be deduplicated, relevance-sorted, and returned with complete metadata, including module, lesson, section, and chunk identifiers. Retrieval outputs must be deterministic, accurate, and optimized for downstream RAG consumption, with clear structure and minimal noise. The system must prioritize correctness, traceability, and production readiness, ensuring retrieval can be independently validated and reused by higher-level agents without modification.

### XI. AI Answering Agent as First-Class Capability
The system must include an AI Answering Agent responsible solely for reasoning over provided context and responding to user queries within the Physical AI Book chatbot. This agent must be implemented using the OpenAI SDK configured with a Gemini-compatible API key and must strictly follow tool-augmented reasoning. The agent is not permitted to retrieve raw data directly; instead, it must rely on an explicit Content Extraction tool provided to it, which returns structured book content or user-selected text. The agent must always prefer tool usage when context is required and must never hallucinate content outside the extracted results. All responses must be grounded exclusively in tool-provided context, clearly structured, and suitable for direct rendering in the chatbot UI. If the extracted content does not contain the answer, the agent must explicitly state that the information is not available in the book. The agent must maintain deterministic, explainable behavior, respect UI constraints, and remain modular.

## Landing Page Structure

The Docusaurus landing page must include the following sections in order:

1. **Hero Section**: A visually striking header with the main title, tagline, and primary call-to-action button
2. **Overview Section**: A quick summary of the book's purpose and value proposition
3. **Learning Outcomes Section**: Clear information about what users will master and learn
4. **Features Section**: Detailed features and benefits of the content

## Content Standards

Content must be free of academic jargon. All hardware references must be real and verifiable; no hallucinated hardware. Every concept must link directly to embodied AI components: sensors, actuators, robots, perception, and control. Focus on simple, clear explanations that gradually increase in depth, supported by practical examples, diagrams, exercises, and small projects.

## Visual Design Standards

The visual design must follow these guidelines:
- Use the YouTube-inspired color scheme of red (#FF0000), white (#FFFFFF), and black (#000000) for high contrast and visual impact
- Implement sleek, modern styling with clean lines and ample whitespace
- Ensure responsive design works across all device sizes
- Maintain consistent typography with clear visual hierarchy
- Use red as the primary color for interactive elements, buttons, and key highlights
- Use white for backgrounds and content areas to ensure readability
- Use black for primary text and important accent elements
- Include clear and compelling call-to-action buttons using the red color

## Chat Support UI Standards

The chat support modal must adhere to these UI standards:
- Implement as a floating action button (FAB) positioned in the bottom-right corner of the screen
- Use a distinctive icon (speech bubble or chat bubble) with the primary red color
- The modal container must have a clean, modern design with the YouTube-inspired color scheme
- Include a header with "AI Assistant" or "Book Support" title
- Implement a message history area with alternating message bubbles for user and AI messages
- Include an input area with a text field and send button
- Add a subtle backdrop overlay that closes the modal when clicked
- Ensure the modal has proper z-index to appear above other content
- Implement smooth open/close animations for better user experience
- Make the modal responsive and usable on all device sizes
- Include a close button (X) in the top-right corner of the modal as an alternative closing method

## Writing Process

Chapters should be designed to be actionable and implementation-ready. Content should be structured for easy consumption by both human readers and RAG systems, ensuring clear chunking and discoverability. Continuous review to ensure content serves beginners effectively while remaining engaging for intermediate readers.

## Governance

This constitution establishes the foundational principles and guidelines for the "Physical AI" book. All content creation, editing, and review processes must adhere to these principles. Amendments to this constitution require a documented rationale and a consensus among the authoring intelligence and human collaborators. Regular reviews will be conducted to ensure ongoing compliance and the continued delivery of high-quality, actionable content.

**Version**: 1.5.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-17
