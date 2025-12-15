# Research: User Authentication and Persistent Chat History

**Feature**: 005-auth-neon-integration
**Date**: 2025-12-13
**Purpose**: Resolve technical unknowns and establish implementation patterns

## Research Questions

### 1. Better Auth Library Selection and Configuration

**Decision**: Use Better Auth (better-auth) library with email/password authentication

**Rationale**:
- Better Auth is specifically designed for modern web applications with PostgreSQL
- Provides built-in session management, JWT tokens, and secure password hashing
- Supports custom user profile fields (needed for learning preferences)
- Self-hosted option aligns with project's architecture principles
- Active community and good documentation

**Alternatives considered**:
- **Auth.js (NextAuth)**: More Next.js focused, less suitable for Express standalone server
- **Passport.js**: More complex configuration, requires more boilerplate
- **Custom JWT implementation**: Security risks, reinventing the wheel

**Implementation details**:
- Install: `npm install better-auth`
- Configure with Neon PostgreSQL connection
- Add custom fields: `educationLevel`, `programmingExperience`, `roboticsBackground`
- Set session cookie domain for cross-origin auth

### 2. Neon PostgreSQL Connection and SSL Configuration

**Decision**: Use `DATABASE_URL` with `sslmode=require` and connection pooling

**Rationale**:
- Neon requires SSL for secure connections
- Connection pooling improves performance for multiple concurrent requests
- `sslmode=require` ensures encrypted connections to Neon's servers
- Environment variable configuration aligns with project's security standards

**Implementation details**:
```javascript
// auth-server/database/neon.js
const { Pool } = require('pg');
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false // Required for Neon SSL
  }
});
```

### 3. Session Validation Between Services

**Decision**: Backend validates JWT tokens from auth server via shared secret

**Rationale**:
- Maintains service isolation (auth server handles auth, backend validates)
- JWT tokens can be validated without database calls (stateless)
- Shared secret ensures only authorized services can issue valid tokens
- Follows Constitution's "Identity Propagation" principle

**Implementation details**:
- Auth server signs JWT with secret key
- Backend validates JWT using same secret
- Token includes: user ID, email, profile fields, expiration
- Backend extracts user profile for personalization in system prompts

### 4. CORS Configuration for Three Services

**Decision**: Configure CORS for ports 3000 (frontend), 3001 (auth), 8000 (backend)

**Rationale**:
- Frontend (Docusaurus) runs on port 3000
- Auth server runs on port 3001
- Backend (FastAPI) runs on port 8000
- All services need to communicate cross-origin during development
- Production will have different domains but same CORS principles

**Implementation details**:
```javascript
// auth-server CORS
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:8000'],
  credentials: true
}));

// backend CORS (FastAPI)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 5. Database Migration Strategy

**Decision**: Use SQL migration files with a simple Python migration runner

**Rationale**:
- SQL files are version-controlled and reviewable
- Simple runner ensures migrations can be run in any environment
- Separate migrations for auth tables and chat history table
- Follows Constitution's "Migration Check" deployment gate

**Implementation details**:
- `backend/migrations/001_create_auth_tables.sql` - Better Auth schema
- `backend/migrations/002_create_chat_history.sql` - Chat history table
- `backend/migrations/migrate.py` - Runs migrations in order
- Better Auth also manages its own migrations via library

### 6. Personalization Injection into System Prompts

**Decision**: Inject user profile data into OpenAI Agents SDK system prompt

**Rationale**:
- System prompt is where context about user's background should go
- OpenAI Agents SDK allows dynamic system prompt modification
- Profile data helps LLM tailor response complexity and examples
- Follows Constitution's "Profile-Driven Personalization" principle

**Implementation details**:
```python
# In backend chat endpoint
system_prompt = f"""
You are a technical tutor for robotics and programming.
The user has the following background:
- Education level: {user_profile.education_level}
- Programming experience: {user_profile.programming_experience}
- Robotics background: {user_profile.robotics_background}

Tailor your explanations to match their experience level.
"""
```

### 7. Chat History Pagination and Performance

**Decision**: Implement server-side pagination with 50 messages per page

**Rationale**:
- Prevents loading entire chat history at once
- 50 messages provides good balance between performance and usability
- DESC order by createdAt shows most recent messages first
- Index on (userId, createdAt DESC) for efficient queries

**Implementation details**:
```sql
-- Index for performance
CREATE INDEX idx_chat_history_user_created ON "chat_history" ("userId", "createdAt" DESC);

-- Query with pagination
SELECT * FROM "chat_history"
WHERE "userId" = $1
ORDER BY "createdAt" DESC
LIMIT 50 OFFSET $2;
```

### 8. Error Handling Patterns

**Decision**: Consistent error responses across all services

**Rationale**:
- Users need clear feedback for auth failures, validation errors, network issues
- Consistent patterns make frontend error handling simpler
- Follows REST best practices for HTTP status codes

**Implementation details**:
- **400 Bad Request**: Validation errors (password too short, invalid email)
- **401 Unauthorized**: Invalid credentials, expired session
- **409 Conflict**: Email already exists
- **500 Internal Server Error**: Database errors, unexpected failures
- All errors include user-friendly message and error code

## Integration Patterns

### Frontend ↔ Auth Server
- Auth forms POST to `/api/auth/signin` and `/api/auth/signup`
- Session cookie set by auth server, included automatically with `credentials: 'include'`
- Profile updates POST to `/api/user/update`

### Frontend ↔ Backend
- All chat requests include `Authorization: Bearer <token>` header
- Backend validates token before processing
- Chat history fetched from `/api/chat/history`

### Auth Server ↔ Backend
- Shared JWT secret for token validation
- No direct API calls between them (stateless validation)

## Security Considerations

1. **Password Security**: Better Auth uses bcrypt for password hashing
2. **JWT Security**: Tokens expire (24 hours), signed with strong secret
3. **SQL Injection**: Parameterized queries via pg library
4. **XSS Protection**: Frontend sanitizes user input, backend validates
5. **CORS**: Strict origin configuration prevents unauthorized domains

## Performance Optimizations

1. **Database Indexes**: Indexes on userId, createdAt for chat history queries
2. **Connection Pooling**: PostgreSQL connection pool in auth server
3. **JWT Caching**: Backend can cache validated tokens for short period
4. **Pagination**: Chat history loaded in pages, not all at once

## Testing Strategy

1. **Auth Flow Tests**: Signup, login, session validation
2. **Chat Persistence Tests**: Messages saved and retrieved correctly
3. **Personalization Tests**: Different profiles receive appropriately tailored responses
4. **Error Handling Tests**: All error cases handled gracefully
5. **Integration Tests**: All three services work together correctly