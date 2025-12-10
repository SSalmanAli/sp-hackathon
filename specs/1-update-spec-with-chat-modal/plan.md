# Implementation Plan: Update Spec with Chat Modal UI

**Branch**: `1-update-spec-with-chat-modal` | **Date**: 2025-12-10 | **Spec**: [link](../specs/1-update-spec-with-chat-modal/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a floating chat button that opens an AI assistant modal for users browsing the Physical AI Book website. The modal will follow YouTube-inspired color scheme (red, white, black) and provide immediate assistance about book content. The implementation will include a responsive UI with click-outside-to-close functionality and message history display.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: TypeScript/JavaScript (for Docusaurus compatibility)
**Primary Dependencies**: React (Docusaurus framework), clsx (CSS utility), React hooks for state management
**Storage**: N/A (UI component only, no persistent storage)
**Testing**: Jest, React Testing Library (for component testing)
**Target Platform**: Web (Docusaurus static site)
**Project Type**: Web
**Performance Goals**: Modal opens in <1 second, responsive across all device sizes
**Constraints**: Must be accessible, keyboard navigable, and follow WCAG standards
**Scale/Scope**: Single component implementation for existing Docusaurus site

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Principle VII (YouTube-Inspired Color Scheme)**: Implementation will use red (#FF0000), white (#FFFFFF), and black (#000000) colors as specified
- ✅ **Principle IX (Interactive Chat Support Modal)**: Implementation includes floating chat button that opens modal and closes when clicked outside container
- ✅ **Chat Support UI Standards**: Implementation follows all specified UI standards:
  - Floating action button in bottom-right corner
  - Red-colored distinctive icon
  - Clean modern design with YouTube color scheme
  - Header with "AI Assistant" title
  - Message history with alternating bubbles
  - Input area with text field and send button
  - Backdrop overlay that closes modal when clicked
  - Proper z-index for layering
  - Smooth animations
  - Responsive design for all device sizes
  - Close (X) button in top-right corner

## Project Structure

### Documentation (this feature)

```text
specs/1-update-spec-with-chat-modal/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── ChatModal/
│   │   ├── ChatModal.tsx          # Main chat modal component
│   │   ├── ChatModal.module.css   # Component-specific styles
│   │   ├── ChatButton.tsx         # Floating chat button component
│   │   └── ChatButton.module.css  # Button-specific styles
│   └── index.tsx                  # Export components
├── css/
│   └── custom.css                 # Global CSS variables and styles
└── pages/
    └── index.tsx                  # Landing page integration
```

**Structure Decision**: Single web component implementation with React for Docusaurus compatibility. The chat modal will be implemented as a reusable React component with associated styling and state management.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|