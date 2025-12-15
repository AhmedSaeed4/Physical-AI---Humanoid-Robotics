---
id: 0001
title: Create auth neon integration spec
stage: spec
date: 2025-12-13
surface: agent
model: claude-opus-4-5-20251101
feature: 005-auth-neon-integration
branch: 005-auth-neon-integration
user:
command: /sp.specify
labels: ["authentication", "database", "chat-history", "personalization"]
links:
  spec: specs/005-auth-neon-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-auth-neon-integration/spec.md
  - specs/005-auth-neon-integration/checklists/requirements.md
tests:
  -
---

## Prompt

name the new branch "005-auth-neon-integration"  # Better Auth + Neon Integration for RAG Chatbot - Implementation Prompt

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
> 4. NOT assume any specific file paths or naming conventions

---

## Implementation

### 🎯 Analyze Project & Set Up Auth Server

```
First, explore my project structure to understand the current hierarchy.

Then create a Node.js auth server using Better Auth with Neon PostgreSQL:

1. ANALYZE: Look at my existing project folders. Find where I have:
   - Backend code (Python/FastAPI or similar)
   - Frontend code (Docusaurus/React)
   - Any existing auth-related files

2. CREATE auth server (adapt location to my project):
   - Use Better Auth library
   - Connect to Neon PostgreSQL via DATABASE_URL
   - Enable email/password authentication
   - Add user profile fields: softwareBackground, hardwareBackground, educationLevel, programmingExperience, roboticsBackground
   - Set CORS for my frontend origin

3. CREATE Express server with endpoints:
   - /api/auth/* - Better Auth handler
   - /api/user/me - Get current user
   - /api/user/update - Update user profile

4. CREATE database migrations for Better Auth tables:
   - "user" table with TEXT id (UUID), email, name, profile fields
   - "session" table with userId reference
   - "account" table for OAuth support
   - Use camelCase column names

Adapt all file paths and imports to match MY project's existing structure.
```

---

### 🎯 Add Chat History Table to Neon

```
Create a migration for persisting RAG chat history in Neon PostgreSQL.

1. ANALYZE: Find where my database migrations or schema files live in the project.

2. CREATE migration for "chat_history" table:
   - id: SERIAL PRIMARY KEY
   - userId: TEXT NOT NULL (references Better Auth "user" table)
   - message: TEXT NOT NULL (user's question to RAG chatbot)
   - response: TEXT NOT NULL (AI response from Gemini/OpenAI Agents)
   - selectedText: TEXT (optional, for text selection context)
   - createdAt: TIMESTAMP NOT NULL DEFAULT NOW()

3. ADD indexes:
   - Index on userId for filtering
   - Index on createdAt DESC for ordering

4. CREATE migration runner script that:
   - Connects to Neon with SSL
   - Executes the migration
   - Logs success/failure

IMPORTANT: userId is TEXT (UUID string), NOT INTEGER. Better Auth uses UUID strings.
Use camelCase column names to match Better Auth conventions.
```

---

### 🎯 Backend Database Functions for Chat History

```
Add database functions to save and retrieve RAG chat history.

1. ANALYZE: Find my existing backend code that handles:
   - RAG queries (OpenAI Agents SDK, Qdrant search, Gemini)
   - Any existing database connections
   - The chat endpoint that processes user queries

2. CREATE or UPDATE database utility with:

   get_chat_history(user_id: str, limit: int = 50):
   - Query "chat_history" table from Neon
   - Filter by "userId" (TEXT column)
   - Order by "createdAt" DESC
   - Return list with: id, message, response, selectedText, createdAt

   save_chat_message(user_id: str, message: str, response: str, selected_text: str = None):
   - Insert into "chat_history" table
   - Use camelCase columns: "userId", "selectedText", "createdAt"
   - Return the created chat ID

3. INTEGRATE with my existing RAG chat endpoint:
   - After generating AI response, call save_chat_message()
   - Pass user_id from authenticated session
   - Include selected_text if user used text selection feature

Adapt to my existing code patterns and import conventions.
```

---

### 🎯 Session Validation in Backend

```
Add session validation to protect RAG chat endpoints.

1. ANALYZE: Find my existing backend that has:
   - The RAG chat endpoint (uses OpenAI Agents SDK, Qdrant, Gemini)
   - Any existing auth middleware or decorators

2. CREATE session validation:
   - Extract "better-auth.session_token" cookie from request
   - Validate session by calling auth server: GET /api/auth/get-session
   - Return user data including id, email, profile fields
   - Raise 401 if not authenticated

3. UPDATE RAG chat endpoint:
   - Require authenticated user
   - Extract user_id and profile from session
   - Pass to existing OpenAI Agents SDK agent for personalization
   - Save chat to database with user_id

4. CREATE /api/chat/history endpoint:
   - Requires authenticated user
   - Returns chat history for current user from Neon

Do NOT break my existing RAG functionality (Qdrant search, Gemini, guardrails).
```

---

### 🎯 Frontend Auth UI & Pages

```
Add complete authentication UI to my Docusaurus frontend.

1. ANALYZE: Find my existing frontend structure:
   - Where React components live
   - Where the ChatBot/ChatKit component is
   - Existing pages and routing
   - Existing CSS/styling patterns

2. MODIFY ChatKit/ChatBot Component - Login Gate:
   When user is NOT authenticated:
   - DO NOT show the ChatKit chat interface
   - Instead, show a "Login to Chat" button in place of the chat UI
   - Button should have a chat icon and text like "Login to start chatting"
   - Clicking the button redirects to /auth page
   - Style the button to match the chat widget's design

   When user IS authenticated:
   - Show the normal ChatKit chat interface
   - Fetch and display chat history
   - Include user name/avatar in header (optional)

3. CREATE Auth Page (/auth or /login):
   - Clean, modern auth form UI
   - Two modes: Sign In and Sign Up (toggle between)

   Sign In Mode:
   - Email input field
   - Password input field
   - "Sign In" button
   - Link: "Don't have an account? Sign Up"

   Sign Up Mode:
   - Name input field
   - Email input field
   - Password input field
   - Confirm Password input field

   Learning Preferences (inline during signup):
   - Education Level: High School | Undergraduate | Graduate | Professional
   - Programming Experience: No Experience | Beginner | Intermediate | Advanced
   - Robotics Background: No Experience | Hobbyist | Academic | Professional
   (Use selectable pill buttons for each, same style as profile page)

   - "Sign Up" button
   - Link: "Already have an account? Sign In"

   After successful signup:
   - Redirect to home page or ChatBot (user is ready to chat)

4. CREATE Profile Page (/profile):
   (For updating learning preferences after initial signup)

   Header: "Learning Preferences"
   Subtitle: "Update your preferences to get personalized content that matches your learning journey"

   - Pre-fill with user's current saved preferences

   Section 1 - Education Level:
   - Helper text: "This helps us adjust the complexity of explanations"
   - SELECTABLE BUTTONS (only one can be selected):
     * High School
     * Undergraduate
     * Graduate
     * Professional
   - Selected button should have highlighted/filled style
   - Unselected buttons should have outline style

   Section 2 - Programming Experience:
   - Helper text: "We'll adjust code examples based on your level"
   - SELECTABLE BUTTONS:
     * No Experience
     * Beginner
     * Intermediate
     * Advanced

   Section 3 - Robotics Background:
   - Helper text: "This affects how we explain hardware concepts"
   - SELECTABLE BUTTONS:
     * No Experience
     * Hobbyist
     * Academic
     * Professional

   "Save Preferences" button at bottom
   - Saves to auth server /api/user/update
   - Shows success message
   - Redirects to home or previous page

5. ERROR HANDLING - Must handle all these cases:

   Auth Errors:
   - Invalid email format → "Please enter a valid email address"
   - Password too short → "Password must be at least 8 characters"
   - Passwords don't match → "Passwords do not match"
   - Email already exists → "An account with this email already exists"
   - Invalid credentials → "Invalid email or password"
   - Network error → "Unable to connect. Please try again."

   Session Errors:
   - Session expired → Auto-redirect to login with message "Session expired, please login again"
   - Not authenticated → Redirect to auth page

   Profile Errors:
   - Update failed → "Failed to save preferences. Please try again."
   - Show loading states during API calls
   - Disable buttons while submitting

   General:
   - All error messages should appear near the relevant field or as toast/alert
   - Use red color for errors, green for success
   - Clear errors when user starts typing again

6. STYLING:
   - Match existing dark theme/glassmorphism style of the project
   - Selectable buttons should look like pill/chip buttons
   - Selected state: filled background, white text
   - Unselected state: transparent/outline, light text
   - Smooth transitions on hover and selection
   - Responsive design for mobile

Adapt all styling to match my existing design patterns.
Use credentials: 'include' for all auth API calls.
```

---

### 🎯 Test Complete Integration

```
Test the complete auth + chat history integration.

1. VERIFY database setup:
   - Run migrations
   - Check tables exist: user, session, chat_history

2. START all services (adapt commands to my project):
   - Auth server
   - Backend (with RAG)
   - Frontend (Docusaurus)

3. TEST auth flow:
   - Sign up new user
   - Verify user in Neon database
   - Sign in and check session cookie

4. TEST RAG chat with persistence:
   - Send message to chatbot
   - Verify RAG works (Qdrant search, Gemini response)
   - Check chat saved in Neon: SELECT * FROM "chat_history"
   - Refresh page, verify history loads

5. TEST personalization:
   - Update profile to "Beginner"
   - Ask technical question
   - Update profile to "Advanced"
   - Ask same question
   - Compare response complexity

6. TEST text selection:
   - Select text from docs
   - Ask about selection
   - Verify selectedText saved in chat_history
```

---

## Database Schema Reference

### Better Auth Tables (in Neon)

```sql
-- All tables use camelCase and TEXT for IDs
CREATE TABLE "user" (
    "id" TEXT PRIMARY KEY,           -- UUID string
    "email" TEXT NOT NULL UNIQUE,
    "name" TEXT NOT NULL,
    "educationLevel" TEXT,
    "programmingExperience" TEXT,
    "softwareBackground" TEXT,
    "hardwareBackground" TEXT,
    "roboticsBackground" TEXT,
    "createdAt" TIMESTAMP DEFAULT NOW()
);

CREATE TABLE "session" (
    "id" TEXT PRIMARY KEY,
    "userId" TEXT REFERENCES "user"("id") ON DELETE CASCADE,
    "token" TEXT,
    "expiresAt" TIMESTAMP NOT NULL
);
```

### Chat History Table (in Neon)

```sql
CREATE TABLE "chat_history" (
    "id" SERIAL PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "message" TEXT NOT NULL,      -- User's question
    "response" TEXT NOT NULL,     -- AI response from RAG
    "selectedText" TEXT,          -- Text selection context (optional)
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);
```

---

## Environment Variables

```env
# Neon PostgreSQL (shared by auth and backend)
DATABASE_URL=postgresql://user:password@host/database?sslmode=require

# Existing RAG config (keep these)
GEMINI_API_KEY=your_gemini_key
QDRANT_URL=https://your-cluster.qdrant.cloud
QDRANT_API_KEY=your_qdrant_key
OPENAI_API_KEY=your_openai_key  # If using ChatKit
```

---

## Key Notes

1. **User ID is TEXT (UUID)** - Better Auth uses UUID strings, not integers
2. **Use camelCase** - Better Auth convention for column names
3. **SSL for Neon** - Always use `sslmode=require` or `ssl: { rejectUnauthorized: false }`
4. **Cookie credentials** - Use `credentials: 'include'` in fetch requests
5. **Don't break RAG** - All prompts emphasize preserving existing Qdrant/Gemini/Agents functionality

---

## Response snapshot

Created feature specification for user authentication and persistent chat history integration. Analyzed existing project structure to understand backend (FastAPI), frontend (Docusaurus/React), and existing patterns. Created comprehensive spec with 4 prioritized user stories, 12 functional requirements, 6 measurable success criteria, and quality checklist. Specification focuses on user needs without implementation details, ready for planning phase.

## Outcome

- ✅ Impact: Established clear requirements for auth + chat persistence feature
- 🧪 Tests: All requirements have testable acceptance criteria
- 📁 Files: Created spec.md and quality checklist
- 🔁 Next prompts: Ready for `/sp.plan` or `/sp.clarify`
- 🧠 Reflection: Successfully extracted user needs from technical implementation prompt

## Evaluation notes (flywheel)

- Failure modes observed: None - specification meets all quality criteria
- Graders run and results (PASS/FAIL): All checklist items pass
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Proceed to planning phase
