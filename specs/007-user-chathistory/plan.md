# Implementation Plan: User Chat History Persistence

**Branch**: `007-user-chathistory` | **Date**: 2025-12-15 | **Spec**: [specs/007-user-chathistory/spec.md](./spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace the in-memory MemoryStore with a NeonStore that persists ChatKit threads and messages to the Neon PostgreSQL database. This enables per-user chat history that survives server restarts while maintaining data isolation between authenticated users. The implementation will include database schema changes, a new NeonStore class implementing the ChatKit store interface, and updates to the backend API to properly handle user authentication context for thread filtering.

## Technical Context

**Language/Version**: Python 3.12+, TypeScript 5.x, Node.js 20+
**Primary Dependencies**: FastAPI, ChatKit, Neon PostgreSQL, Better Auth, psycopg2
**Storage**: PostgreSQL (Neon) for chat threads and messages
**Testing**: pytest for backend, Jest for auth server
**Target Platform**: Web application (Docusaurus frontend, FastAPI backend, Node auth server)
**Project Type**: Web (Frontend/Backend/Auth Server tri-fold architecture)
**Performance Goals**: Load chat threads within 2 seconds for users with up to 100 previous threads
**Constraints**: Must maintain service isolation, support 1000+ concurrent users, ensure data isolation between users
**Scale/Scope**: Support unlimited message history per chat thread, handle multiple concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance
- [X] Service Isolation: Architecture maintains tri-fold separation (Frontend/Backend/Auth Server) - database changes will be in backend service only
- [X] Guardrail Implementation: Chat history persistence does not affect AI guardrail functionality
- [X] Profile-Driven: User profile data will be properly associated with chat threads through authentication context
- [X] Truth in Markdown: This feature does not affect vector DB synchronization with documentation sources
- [X] Identity Propagation: JWT validation from auth server will be used to filter threads by user ID
- [X] CORS Configuration: No changes to cross-origin settings needed for this feature

### Technology Standards Compliance
- [X] Stack Requirements: Uses Python 3.12+, Docusaurus 3.x, Node.js 20+ as required
- [X] Directory Structure: Changes will be in backend/ and follow established organization
- [X] Security: Database connection will use DATABASE_URL from .env file, not committed to version control

### Post-Design Compliance Check
- [X] Database schema properly implements user isolation via userId foreign key
- [X] Neon PostgreSQL integration follows project's database standards
- [X] API contracts maintain compatibility with existing frontend components
- [X] Implementation maintains backward compatibility where needed

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
backend/
├── src/
│   ├── backend/
│   │   ├── neon_store.py          # New NeonStore implementation
│   │   ├── store.py               # Existing MemoryStore (kept as backup)
│   │   └── main.py                # Updated to use NeonStore
│   └── migrations/
│       └── 003_create_chatkit_tables.sql  # New database schema
└── tests/

frontend/
├── src/
│   └── components/
│       └── ChatBot/
│           └── FloatingChatWidget.tsx  # Updated to send user context
└── tests/

auth-server/
└── [existing auth server structure]
```

**Structure Decision**: Web application with tri-fold architecture (Frontend/Backend/Auth Server). Database schema changes will be in backend/migrations/, new NeonStore implementation in backend/src/backend/, and minor updates to existing files to support user-specific chat persistence.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
