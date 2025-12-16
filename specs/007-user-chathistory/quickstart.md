# Quickstart: User Chat History Persistence

## Overview
This feature replaces the in-memory chat storage with persistent storage in Neon PostgreSQL, enabling users to maintain their chat history across sessions and server restarts.

## Prerequisites
- Python 3.12+
- Node.js 20+
- Docusaurus 3.x
- Neon PostgreSQL database with user table
- Running auth server on port 3001

## Setup Steps

### 1. Database Migration
Run the database migration to create the chatkit tables:
```bash
cd backend
# Apply the migration script to create chatkit_thread, chatkit_thread_item, and chatkit_attachment tables
psql $DATABASE_URL -f migrations/003_create_chatkit_tables.sql
```

### 2. Backend Configuration
- Ensure `DATABASE_URL` is set in `backend/.env`
- The NeonStore will automatically use this connection

### 3. Code Integration
- The backend will use NeonStore instead of MemoryStore
- User authentication context will be passed to filter threads by user ID

## Key Files
- `backend/migrations/003_create_chatkit_tables.sql` - Database schema
- `backend/src/backend/neon_store.py` - Persistent storage implementation
- `backend/src/backend/main.py` - Updated to use NeonStore
- `frontend/src/components/ChatBot/FloatingChatWidget.tsx` - Updated to pass user context

## Testing
1. Start all services: `npm run serve` (frontend), `npm run serve` (auth), `uv run main.py` (backend)
2. Authenticate as a user
3. Create a chat thread and send messages
4. Refresh the page - verify chat history persists
5. Restart the backend server - verify chat history still persists