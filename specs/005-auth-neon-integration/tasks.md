---
description: "Task list for User Authentication and Persistent Chat History feature"
---

# Tasks: 005-auth-neon-integration

**Input**: Design documents from `/specs/005-auth-neon-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - not explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `auth-server/src/`, `frontend/src/` per plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create auth-server directory structure per implementation plan in auth-server/
- [X] T002 Initialize auth-server Node.js project with Better Auth dependencies in auth-server/package.json
- [X] T003 [P] Configure environment variables for all services in .env files
- [X] T004 [P] Setup Neon PostgreSQL database connection configuration in auth-server/database/neon.js
- [X] T005 [P] Create database migration files in backend/migrations/

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup Better Auth configuration with custom user fields in auth-server/src/auth.js
- [X] T007 [P] Create JWT validation middleware for backend in backend/src/middleware/auth.py
- [X] T008 [P] Configure CORS for all three services (ports 3000, 3001, 8000)
- [X] T009 Run database migrations to create auth tables and chat history in backend/migrations/migrate.py
- [X] T010 Setup shared JWT secret configuration across auth-server and backend

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---
## Phase 3: User Story 1 - Create Account and Set Learning Preferences (Priority: P1) 🎯 MVP

**Goal**: Users can create accounts with email, password, name, and learning preferences

**Independent Test**: Create a new account, set learning preferences, verify account exists in database with correct preferences

### Implementation for User Story 1

- [X] T011 [P] [US1] Create signup API endpoint in auth-server/src/routes/auth.js
- [X] T012 [P] [US1] Create signup form component in frontend/src/components/Auth/SignupForm.tsx
- [X] T013 [P] [US1] Create auth service for frontend API calls in frontend/src/services/authService.ts
- [X] T014 [P] [US1] Create Auth context for state management in frontend/src/contexts/AuthContext.tsx
- [X] T015 [US1] Implement signup form validation and error handling in frontend/src/components/Auth/SignupForm.tsx
- [X] T016 [US1] Add custom user profile fields to Better Auth configuration in auth-server/src/auth.js
- [X] T017 [US1] Create auth page routing in frontend/src/pages/auth.tsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---
## Phase 4: User Story 2 - Login and Access Chat History (Priority: P1)

**Goal**: Returning users can log in and see their previous chat conversations

**Independent Test**: Log in with existing credentials, verify previous chat history loads correctly

### Implementation for User Story 2

- [X] T018 [P] [US2] Create login API endpoint in auth-server/src/routes/auth.js
- [X] T019 [P] [US2] Create login form component in frontend/src/components/Auth/LoginForm.tsx
- [X] T020 [P] [US2] Implement session validation in backend middleware in backend/src/middleware/auth.py
- [X] T021 [US2] Add login functionality to auth service in frontend/src/services/authService.ts
- [X] T022 [US2] Implement auto-redirect for expired sessions in frontend/src/contexts/AuthContext.tsx
- [X] T023 [US2] Create chat history API endpoint in backend/src/backend/main.py
- [X] T024 [US2] Implement chat history loading in frontend chat service in frontend/src/services/chatService.ts

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---
## Phase 5: User Story 3 - Chat with Personalized Responses (Priority: P2)

**Goal**: Logged-in users receive responses personalized to their learning preferences

**Independent Test**: Ask technical questions, verify responses are appropriately tailored to user's education level and programming experience

### Implementation for User Story 3

- [X] T025 [P] [US3] Modify chat endpoint to require authentication in backend/src/backend/main.py
- [X] T026 [P] [US3] Create user profile extraction from JWT in backend/src/middleware/auth.py
- [X] T027 [P] [US3] Implement profile injection into system prompts in backend/src/backend/chatkit_adapter.py
- [X] T028 [US3] Update chat service to include auth token in frontend/src/services/chatService.ts
- [X] T029 [US3] Modify ChatBot component to require authentication in frontend/src/components/ChatBot/index.tsx
- [X] T030 [US3] Implement selected text saving with chat messages in backend/src/backend/main.py
- [X] T031 [US3] Update chat interface to show personalized indicators in frontend/src/components/ChatBot/ChatBotAuthenticated.tsx

**Checkpoint**: All user stories should now be independently functional

---
## Phase 6: User Story 4 - Update Learning Preferences (Priority: P3)

**Goal**: Users can update their learning preferences after initial signup

**Independent Test**: Update preferences, verify subsequent chat responses reflect new settings

### Implementation for User Story 4

- [X] T032 [P] [US4] Create profile update API endpoint in auth-server/src/routes/user.js
- [X] T033 [P] [US4] Create profile form component in frontend/src/components/Auth/ProfileForm.tsx
- [X] T034 [P] [US4] Create profile page routing in frontend/src/pages/profile.tsx
- [X] T035 [US4] Implement profile update in auth service in frontend/src/services/authService.ts
- [X] T036 [US4] Add profile update functionality to Auth context in frontend/src/contexts/AuthContext.tsx
- [X] T037 [US4] Implement preference validation in profile form in frontend/src/components/Auth/ProfileForm.tsx

**Checkpoint**: All user stories should now be independently functional

---
## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T038 [P] Documentation updates in quickstart.md validation
- [X] T039 Code cleanup and refactoring across all services
- [X] T040 [P] Performance optimization for chat history queries with indexes
- [X] T041 Security hardening for JWT tokens and session management
- [X] T042 Error handling improvements across all API endpoints
- [X] T043 [P] Add loading states and user feedback in all frontend components
- [X] T044 Implement session expiration handling in frontend auth context

---
## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 for user accounts
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1/US2 for authentication and profiles
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 for user accounts

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---
## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
Task: "Create signup API endpoint in auth-server/src/routes/auth.js"
Task: "Create signup form component in frontend/src/components/Auth/SignupForm.tsx"
Task: "Create auth service for frontend API calls in frontend/src/services/authService.ts"
Task: "Create Auth context for state management in frontend/src/contexts/AuthContext.tsx"
```

---
## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Account creation)
   - Developer B: User Story 2 (Login & chat history)
   - Developer C: User Story 3 (Personalized chat)
3. Stories complete and integrate independently

---
## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence