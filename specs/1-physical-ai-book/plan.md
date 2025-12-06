# Implementation Plan: Physical AI Book in Docusaurus

**Branch**: `1-physical-ai-book` | **Date**: 2025-12-06 | **Spec**: [link to spec](../spec.md)
**Input**: Feature specification from `/specs/1-physical-ai-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a comprehensive Physical AI e-book using Docusaurus v3 framework, implementing a modular structure that follows the builder's voice and practical approach defined in the constitution. The site will include 4 modules with 3 lessons each, hands-on examples, diagrams, and interactive elements, deployed on GitHub Pages with integrated search and RAG capabilities.

## Technical Context

**Language/Version**: Node.js 18+, TypeScript, JavaScript
**Primary Dependencies**: Docusaurus v3, React 18, Node.js, npm/yarn
**Storage**: Static files hosted via GitHub Pages
**Testing**: Jest for unit tests, Cypress for E2E tests (NEEDS CLARIFICATION)
**Target Platform**: Web-based documentation site, GitHub Pages
**Project Type**: Static site generator (web)
**Performance Goals**: Fast loading times, SEO optimized, mobile responsive
**Constraints**: Must be accessible to beginners, structured for RAG systems, maintainable
**Scale/Scope**: 12+ lessons, multiple modules, supporting assets and diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Actionable Learning**: All content must be implementation-ready with working examples
2. **Builder's Voice**: Maintain confident, no-fluff approach avoiding academic jargon
3. **Structured Accuracy**: Content must be structured for RAG system chunking
4. **Embodied AI Focus**: All content must tie directly to embodied AI concepts
5. **Audience Engagement**: Content must serve beginners without boring intermediates
6. **Practicality First**: Every chapter must be actionable and implementation-ready

## Project Structure

### Documentation (this feature)

```text
specs/1-physical-ai-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docusaurus/
├── docs/
│   ├── module-1-ros2/
│   │   ├── lesson-1-1-ros2-architecture.md
│   │   ├── lesson-1-2-robot-modeling-urdf.md
│   │   └── lesson-1-3-robot-control-rclpy.md
│   ├── module-2-simulation/
│   │   ├── lesson-2-1-gazebo-simulation.md
│   │   ├── lesson-2-2-unity-digital-twin.md
│   │   └── lesson-2-3-sensor-simulation.md
│   ├── module-3-isaac/
│   │   ├── lesson-3-1-isaac-sim-synthetic-data.md
│   │   ├── lesson-3-2-isaac-ros-vslam.md
│   │   └── lesson-3-3-nav2-navigation.md
│   ├── module-4-vla/
│   │   ├── lesson-4-1-vision-language-integration.md
│   │   ├── lesson-4-2-llm-robot-planning.md
│   │   └── lesson-4-3-humanoid-autonomy-capstone.md
│   ├── assets/
│   │   ├── diagrams/
│   │   ├── code-examples/
│   │   └── images/
│   └── reusable-intelligence/
│       ├── lesson-template.md
│       └── content-guidelines.md
├── src/
│   ├── components/
│   │   ├── InteractiveDemo/
│   │   ├── CodeRunner/
│   │   └── ChatBot/
│   ├── pages/
│   └── css/
│       └── custom.css
├── static/
│   └── img/
├── docusaurus.config.ts
├── sidebars.ts
├── package.json
├── tsconfig.json
└── babel.config.js
```

**Structure Decision**: Single Docusaurus project with modular folder structure following the 4 modules and 12 lessons as defined in the specification. Assets are organized in a dedicated folder with subfolders for diagrams, code examples, and images. Reusable intelligence content is stored separately for maintainability.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |