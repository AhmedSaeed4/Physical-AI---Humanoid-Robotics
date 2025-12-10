# Implementation Plan: Text Selection to Chat

**Branch**: `004-text-selection` | **Date**: December 10, 2025 | **Spec**: [link](/specs/004-text-selection/spec.md)
**Input**: Feature specification from `/specs/004-text-selection/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable users to select text from book content and send it directly to the AI chatbot with additional context. This involves creating a custom React hook for text selection detection, a contextual popup component, integrating with Docusaurus DocItem, updating the ChatBot component to handle selected text, and enhancing the backend RAG endpoint to properly process the selected text as additional context.

## Technical Context

**Language/Version**: TypeScript 5.x (Frontend), Python 3.12+ (Backend), Node.js 20+ (Auth)
**Primary Dependencies**: Docusaurus 3.x, React 19, FastAPI, Qdrant Client
**Storage**: N/A (No new storage needed, uses existing Qdrant and PostgreSQL)
**Testing**: Jest (Frontend), pytest (Backend), Cypress (E2E)
**Target Platform**: Web application (Docusaurus-based documentation site)
**Project Type**: Web (Frontend/Backend/Auth Server tri-fold architecture)
**Performance Goals**: <0.5s button appearance after selection, <3s from selection to chat initiation
**Constraints**: Must not interfere with normal reading experience, responsive design for all devices
**Scale/Scope**: Works with existing user base, handles text selections up to 1000 characters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance
- [x] Service Isolation: Feature maintains tri-fold separation - frontend component detects selection, communicates via API to backend, auth remains separate
- [x] Guardrail Implementation: Selected text will pass through existing openai-agents guardrail layer as part of the chat request
- [x] Profile-Driven: User profile data will continue to be injected into system prompts as part of the enhanced backend processing
- [x] Truth in Markdown: Feature enhances RAG pipeline by adding selected text context while preserving existing documentation sources
- [x] Identity Propagation: JWT validation remains unchanged, only the request payload structure changes to include selected text
- [x] CORS Configuration: No changes needed - uses existing communication patterns between frontend and backend

### Technology Standards Compliance
- [x] Stack Requirements: Uses Docusaurus 3.x, TypeScript 5.x, Python 3.12+, FastAPI as required
- [x] Directory Structure: Will follow existing backend/, frontend/ organization as specified
- [x] Security: No changes to secret handling - uses existing .env file approach

## Project Structure

### Documentation (this feature)

```text
specs/004-text-selection/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   └── backend/
│       ├── main.py          # Enhanced /api/chat endpoint to handle selected_text
│       └── chatkit_adapter.py # RAG pipeline updates for selected text context
└── tests/

frontend/
├── src/
│   ├── components/
│   │   └── TextSelection/   # New directory for text selection components
│   │       ├── useTextSelection.tsx    # Custom hook for text selection detection
│   │       └── SelectionPopup.tsx      # Contextual popup component
│   ├── theme/
│   │   └── DocItem/         # Swizzled Docusaurus component
│   │       └── index.tsx    # Wrapper for DocItem with text selection integration
│   └── pages/
│       └── chat.tsx         # Updated chat page to handle selected text
└── tests/
```

**Structure Decision**: This is a web application feature that follows the existing tri-fold architecture (Frontend/Backend/Auth). The implementation adds new components to the frontend (frontend/) and enhances the backend API in the existing backend/ directory structure. The feature integrates with the existing ChatBot component and RAG pipeline without changing the core architecture.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
