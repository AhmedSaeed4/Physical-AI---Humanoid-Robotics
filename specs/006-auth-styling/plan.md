# Implementation Plan: Add Styling to Auth Components (Preserve Functionality)

**Branch**: `006-auth-styling` | **Date**: 2025-12-15 | **Spec**: /specs/006-auth-styling/spec.md
**Input**: Feature specification from `/specs/006-auth-styling/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement professional CSS Module styling for auth components (LoginForm.tsx, SignupForm.tsx, ProfileForm.tsx, auth.tsx, profile.tsx) while preserving all existing functionality. Add a navbar profile button that conditionally displays based on authentication status. Use Infima CSS variables for consistent dark mode support. All existing TypeScript logic, hooks, and event handlers must remain unchanged.

## Technical Context

**Language/Version**: TypeScript 5.x (Frontend), Python 3.12+ (Backend), Node.js 20+ (Auth Server)
**Primary Dependencies**: Docusaurus 3.x, React 19, CSS Modules, Infima, Better Auth
**Storage**: N/A (No new storage needed, uses existing PostgreSQL and Qdrant)
**Testing**: Jest/React Testing Library (Frontend), pytest (Backend)
**Target Platform**: Web (Docusaurus site)
**Project Type**: Web (Frontend styling update with navbar component)
**Performance Goals**: Maintain existing performance, no degradation of auth functionality
**Constraints**: Must preserve all existing TypeScript logic, hooks, and event handlers
**Scale/Scope**: Single feature update affecting auth components and navbar

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance
- [x] Service Isolation: Verify architecture maintains tri-fold separation (Frontend/Backend/Auth Server) - Styling only affects frontend, maintains separation
- [x] Guardrail Implementation: Confirm all AI interactions pass through guardrail layer - No changes to AI interactions
- [x] Profile-Driven: Verify user profile data is injected into system prompts - No changes to profile data usage
- [x] Truth in Markdown: Ensure vector DB synchronization with documentation sources - No changes to documentation or vector DB
- [x] Identity Propagation: Confirm JWT validation and secure auth flow - No changes to auth flow, only styling
- [x] CORS Configuration: Verify cross-origin settings for ports 3000, 3001, 8000 - No changes to CORS configuration

### Technology Standards Compliance
- [x] Stack Requirements: Docusaurus 3.x, Python 3.12+, Node.js 20+ - Uses existing stack
- [x] Directory Structure: Confirm backend/, auth-server/, test-docs/ organization - No changes to directory structure
- [x] Security: Verify secrets in .env files, not committed to version control - No new secrets, follows existing pattern

## Project Structure

### Documentation (this feature)

```text
specs/006-auth-styling/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   └── Auth/
│   │       ├── LoginForm.tsx
│   │       ├── SignupForm.tsx
│   │       ├── ProfileForm.tsx
│   │       └── Auth.module.css          # New CSS Module
│   ├── pages/
│   │   ├── auth.tsx
│   │   └── profile.tsx
│   └── theme/
│       └── NavbarItem/
│           ├── NavbarItemCustomNavbarProfileButton.tsx  # New component
│           └── NavbarProfileButton.module.css  # New CSS Module
│   └── components/
│       └── Auth/
│           └── Profile.module.css       # New CSS Module
└── docusaurus.config.js                 # Updated configuration

# Existing structure maintained
backend/
├── src/
│   └── main.py
└── requirements.txt

auth-server/
├── src/
│   └── server.js
└── package.json
```

**Structure Decision**: Web application with frontend styling update. The feature affects only the frontend components, adding CSS Modules for auth components and creating a new NavbarProfileButton component. The existing tri-fold architecture (frontend/backend/auth-server) is maintained with no changes to backend or auth-server.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
