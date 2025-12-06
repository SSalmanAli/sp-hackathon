---
id: 2
title: Create Physical AI Book Spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-book
branch: 1-physical-ai-book
user: san
command: /sp.specify
labels: [spec, physical-ai, book, education]
links:
  spec: ../specs/1-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-book/spec.md
 - specs/1-physical-ai-book/checklists/requirements.md
 - history/prompts/physical-ai-book/2-create-physical-ai-book-spec.spec.prompt.md
tests:
---

## Prompt

Create a complete, detailed Specification Document for the Physical AI e-book based entirely on the Constitution. Include an overview explaining the book's purpose, target audience, hands-on philosophy, definition of Physical AI, and learner outcomes. Define a full book structure containing 4 modules with 3 lessons each, and for every module provide titles, descriptions, and learning outcomes; for every lesson provide titles, descriptions, what students will build, required tools, and a hands-on, code-first focus. Incorporate all required technical topics: ROS 2 fundamentals with rclpy and URDF; Gazebo and Unity digital twin simulation with physics and sensors; NVIDIA Isaac Sim, synthetic data, Isaac ROS VSLAM, and Nav2 path planning; and Vision-Language-Action systems including Whisper, LLM planning, and a capstone humanoid autonomy project. Add content guidelines for tone, formatting, pedagogy, diagrams, demos, and implementation-first writing. Include a reusable lesson template covering context, importan

## Response snapshot

I've created a complete specification document for the Physical AI e-book based on the constitution. The specification includes:

1. Overview with purpose, target audience, definition of Physical AI, and hands-on philosophy
2. Complete book structure with 4 modules and 3 lessons each:
   - Module 1: ROS 2 Fundamentals and Robot Modeling
   - Module 2: Digital Twin Simulation and Physics
   - Module 3: NVIDIA Isaac Ecosystem and Navigation
   - Module 4: Vision-Language-Action Systems and Capstone
3. Content guidelines for tone, formatting, pedagogy, and implementation-first writing
4. Reusable lesson template structure
5. User scenarios and testing requirements
6. Functional requirements and success criteria

The specification follows the builder's voice and practical approach defined in the constitution, with a focus on hands-on learning and implementation-ready content.

## Outcome

- ✅ Impact: Complete Physical AI e-book specification created with detailed structure
- 🧪 Tests: No tests required at specification stage
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan)
- 🧠 Reflection: Specification fully addresses all required technical topics and learning objectives

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All checklist items validated successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase