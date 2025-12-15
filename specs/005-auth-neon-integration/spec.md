# Feature Specification: User Authentication and Persistent Chat History

**Feature Branch**: `005-auth-neon-integration`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "name the new branch "005-auth-neon-integration"  # Better Auth + Neon Integration for RAG Chatbot - Implementation Prompt

> **Feature:** User authentication with Better Auth and persistent RAG chat history using Neon PostgreSQL
> **Tech Stack:** ChatKit, OpenAI Agents SDK, Qdrant DB, Gemini LLM/Embeddings, Docusaurus
> **Created:** December 13, 2025

---

## Feature Overview

This integration adds to your **existing RAG chatbot system**:

1. **User Authentication** via Better Auth (email/password, self-hosted)
2. **User Accounts in Neon** - All users stored in Neon PostgreSQL
3. **User Profiles** with learning preferences (education level, programming experience)
4. **RAG Chat History Persistence** - Save user queries & AI responses to Neon
5. **Personalized AI Responses** based on user profile data

> **All data lives in Neon PostgreSQL:** Users, sessions, and RAG chat conversations stored in one database.

### Your Existing RAG System (Already Working)

Your RAG chatbot already has:
- ✅ **ChatKit** for chat UI components
- ✅ **OpenAI Agents SDK** with guardrails (using **Gemini LLM** via OpenAI-compatible API)
- ✅ **Gemini Embeddings** for query/document vectors
- ✅ **Qdrant** vector database for book content search
- ✅ Text selection context feature
- ✅ In-memory chat history during session

### What This Integration Adds

- 🆕 **Persistent storage** of all RAG conversations in Neon
- 🆕 **Per-user chat history** linked to authenticated users
- 🆕 **Chat reload on page refresh** - conversations survive browser close
- 🆕 **Selected text saved** with each chat message for context
- 🆕 **User profiles** for personalized AI response complexity

---

## ⚠️ IMPORTANT: Adapt to Project Structure

> [!IMPORTANT]
> **You must analyze your project structure first before implementing.**
>
> 1. Explore your existing files and folder hierarchy
> 2. Identify where auth, database, and chat logic currently live
> 3. Adapt the implementation to fit YOUR structure
> 4. NOT assume any specific file paths or naming conventions"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Create Account and Set Learning Preferences (Priority: P1)

A new user wants to create an account and set their learning preferences so they can get personalized responses from the RAG chatbot.

**Why this priority**: This is the foundational user journey - without account creation, no other features can be used. It establishes the user identity and personalization settings.

**Independent Test**: Can be fully tested by creating a new account, setting learning preferences, and verifying the account exists in the database with correct preferences.

**Acceptance Scenarios**:

1. **Given** a user is on the signup page, **When** they enter valid name, email, password, confirm password, and select learning preferences, **Then** their account is created, they are logged in, and redirected to the chat interface
2. **Given** a user is on the signup page, **When** they enter an email that already exists, **Then** they see an error message "An account with this email already exists" and can try a different email
3. **Given** a user is on the signup page, **When** they enter a password less than 8 characters, **Then** they see an error message "Password must be at least 8 characters" and can correct it

---

### User Story 2 - Login and Access Chat History (Priority: P1)

A returning user wants to log in and see their previous chat conversations so they can continue learning from where they left off.

**Why this priority**: This enables the core value proposition of persistent chat history - users need to access their saved conversations across sessions.

**Independent Test**: Can be fully tested by logging in with existing credentials and verifying that previous chat history loads correctly.

**Acceptance Scenarios**:

1. **Given** a user has an existing account, **When** they enter correct email and password, **Then** they are logged in, their chat history loads, and they can continue chatting
2. **Given** a user has an expired session, **When** they try to access the chat, **Then** they are automatically redirected to login with message "Session expired, please login again"
3. **Given** a user enters incorrect credentials, **When** they try to login, **Then** they see an error message "Invalid email or password" and can try again

---

### User Story 3 - Chat with Personalized Responses (Priority: P2)

A logged-in user wants to ask questions to the RAG chatbot and receive responses personalized to their learning preferences.

**Why this priority**: This delivers the personalized learning experience - the core value beyond basic authentication.

**Independent Test**: Can be fully tested by asking technical questions and verifying that responses are appropriately tailored to the user's education level and programming experience.

**Acceptance Scenarios**:

1. **Given** a user with "Beginner" programming experience, **When** they ask a technical question about code, **Then** they receive an explanation with simpler terminology and more examples
2. **Given** a user with "Advanced" programming experience, **When** they ask the same technical question, **Then** they receive an explanation with more technical depth and fewer basic examples
3. **Given** a user selects text from documentation, **When** they ask a question about the selected text, **Then** the selected text is saved with their chat message for context

---

### User Story 4 - Update Learning Preferences (Priority: P3)

A user wants to update their learning preferences after initial signup to better match their evolving skill level.

**Why this priority**: This allows users to refine their experience over time as their skills improve.

**Independent Test**: Can be fully tested by updating preferences and verifying that subsequent chat responses reflect the new settings.

**Acceptance Scenarios**:

1. **Given** a user is on their profile page, **When** they update their programming experience from "Beginner" to "Intermediate", **Then** the preferences are saved and subsequent chat responses reflect the increased complexity
2. **Given** a user tries to save preferences without selecting an education level, **When** they click "Save Preferences", **Then** they see an error message prompting them to select an education level

---

### Edge Cases

- What happens when a user tries to access the chat interface without being authenticated? → They see a "Login to Chat" button instead of the chat interface
- How does the system handle network errors during authentication? → Shows "Unable to connect. Please try again." with retry option
- What happens when the database is unavailable? → Shows appropriate error message and prevents authentication/chat operations
- How does the system handle concurrent login attempts from the same user? → Allows only one active session per user, invalidating previous sessions
- What happens when a user's session expires mid-chat? → Auto-redirects to login with message "Session expired, please login again"

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST allow users to create accounts with email, password, name, and learning preferences
- **FR-002**: System MUST validate email format and ensure password meets minimum security requirements (at least 8 characters)
- **FR-003**: Users MUST be able to log in with email and password to access their chat history
- **FR-004**: System MUST persist user profiles including education level, programming experience, and robotics background
- **FR-005**: System MUST save all RAG chat conversations with user ID, message, response, and optional selected text
- **FR-006**: System MUST load previous chat history when a user logs in
- **FR-007**: System MUST personalize AI responses based on user's learning preferences
- **FR-008**: Users MUST be able to update their learning preferences after initial signup
- **FR-009**: System MUST protect chat endpoints to require authenticated users
- **FR-010**: System MUST handle session expiration gracefully with appropriate user feedback
- **FR-011**: System MUST show appropriate error messages for invalid credentials, network errors, and validation failures
- **FR-012**: System MUST prevent unauthorized access to other users' chat history

### Key Entities *(include if feature involves data)*

- **User**: Represents a person using the system. Has email, name, learning preferences (education level, programming experience, robotics background), and creation timestamp.
- **Session**: Represents an active user login session. Has user reference, token, and expiration timestamp.
- **Chat History**: Represents a conversation between a user and the AI. Has user reference, message text, response text, optional selected text, and creation timestamp.

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can create an account and set learning preferences in under 3 minutes
- **SC-002**: Users can log in and access their chat history in under 30 seconds
- **SC-003**: 95% of users successfully complete account creation on first attempt
- **SC-004**: Chat conversations persist across browser sessions with 100% reliability
- **SC-005**: Users report increased satisfaction with personalized responses compared to generic responses
- **SC-006**: System handles 100 concurrent authenticated users without performance degradation
