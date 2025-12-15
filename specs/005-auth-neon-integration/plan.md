# Implementation Plan: User Authentication and Persistent Chat History

**Branch**: `005-auth-neon-integration` | **Date**: 2025-12-13 | **Spec**: [specs/005-auth-neon-integration/spec.md](specs/005-auth-neon-integration/spec.md)
**Input**: Feature specification from `/specs/005-auth-neon-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Add user authentication with Better Auth and persistent RAG chat history using Neon PostgreSQL. Integrates with existing RAG chatbot system to provide per-user chat persistence, learning profile personalization, and secure session management while maintaining existing Qdrant/Gemini/OpenAI Agents SDK functionality.

## Technical Context

**Language/Version**: Python 3.12+ (Backend), TypeScript 5.x (Frontend), Node.js 20+ (Auth Server)
**Primary Dependencies**: FastAPI (Backend), Docusaurus 3.x/React 19 (Frontend), Better Auth/Express (Auth), PostgreSQL (Neon), Qdrant Client, OpenAI Agents SDK, Google Gemini
**Storage**: PostgreSQL (Neon) for users/sessions/chat history, Qdrant (cloud) for vector search
**Testing**: pytest (Backend), Jest/React Testing Library (Frontend), Postman/curl for API testing
**Target Platform**: Web application (Docusaurus static site + API services)
**Project Type**: Web application with tri-fold architecture (Frontend/Backend/Auth Server)
**Performance Goals**: Account creation <3 min, login <30 sec, chat persistence 100% reliable, 100 concurrent users
**Constraints**: Must preserve existing RAG functionality (Qdrant search, Gemini responses, guardrails), maintain CORS for ports 3000/3001/8000, use existing project structure
**Scale/Scope**: 100+ users, 50+ chat messages per user, 3 learning preference categories with 4 options each

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance
- [x] Service Isolation: Architecture maintains tri-fold separation (Frontend/Backend/Auth Server) - Auth Server is new Node.js service
- [x] Guardrail Implementation: All AI interactions continue to pass through existing OpenAI Agents SDK guardrail layer
- [x] Profile-Driven: User profile data (education level, programming experience, robotics background) will be injected into system prompts for personalized responses
- [x] Truth in Markdown: Vector DB (Qdrant) remains synchronized with documentation sources; auth doesn't affect this
- [x] Identity Propagation: Better Auth provides JWT tokens; Backend validates tokens before processing chat requests
- [x] CORS Configuration: CORS will be configured for ports 3000 (frontend), 3001 (auth), 8000 (backend)

### Technology Standards Compliance
- [x] Stack Requirements: Docusaurus 3.x (existing), Python 3.12+ (existing), Node.js 20+ (new auth server)
- [x] Directory Structure: Follows existing patterns: backend/, auth-server/ (new), frontend/ (frontend)
- [x] Security: Secrets remain in .env files; database credentials for Neon PostgreSQL added to .env

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/ (existing)
├── src/backend/
│   ├── main.py                    # FastAPI app with chat endpoints
│   ├── chatkit_adapter.py         # ChatKit ↔ RAG adapter
│   ├── store.py                   # ChatKit store implementation
│   └── database.py                # Qdrant database integration
├── migrations/                    # NEW: PostgreSQL schema migrations
│   ├── 001_create_auth_tables.sql
│   ├── 002_create_chat_history.sql
│   └── migrate.py                 # Migration runner
└── tests/

auth-server/ (NEW)
├── src/
│   ├── index.js                   # Express app with Better Auth
│   ├── auth.js                    # Better Auth configuration
│   ├── routes/
│   │   ├── auth.js                # /api/auth/* endpoints
│   │   └── user.js                # /api/user/* endpoints
│   └── database/
│       └── neon.js                # Neon PostgreSQL connection
├── migrations/                    # Better Auth schema migrations
└── package.json

frontend/ (existing)
├── src/
│   ├── components/
│   │   ├── ChatBot/               # Existing ChatKit components
│   │   │   ├── index.tsx          # Main component with auth gate
│   │   │   ├── ChatBotAuthenticated.tsx
│   │   │   └── ChatBotSimple.tsx
│   │   ├── Auth/                  # NEW: Auth components
│   │   │   ├── LoginForm.tsx
│   │   │   ├── SignupForm.tsx
│   │   │   └── ProfileForm.tsx
│   │   └── TextSelection/         # Existing text selection
│   ├── services/
│   │   ├── authService.ts         # NEW: Auth API calls
│   │   ├── chatService.ts         # Existing chat service
│   │   └── chatkitService.ts      # Existing ChatKit service
│   ├── contexts/
│   │   ├── AuthContext.tsx        # NEW: Auth state management
│   │   └── ChatContext.tsx        # Existing chat context
│   └── pages/
│       ├── auth.tsx               # NEW: Auth page
│       └── profile.tsx            # NEW: Profile page
└── tests/
```

**Structure Decision**: Web application (Option 2) with tri-fold architecture. Uses existing backend/ and frontend/ directories, adds new auth-server/ directory. Follows Constitution's service isolation principle.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
