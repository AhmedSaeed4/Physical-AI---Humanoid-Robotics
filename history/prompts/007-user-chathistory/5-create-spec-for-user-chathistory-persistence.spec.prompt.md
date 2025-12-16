---
id: 5
title: "Create spec for user chat history persistence"
stage: spec
date: 2025-12-15
surface: agent
model: claude-opus-4-5-20251101
feature: "007-user-chathistory"
branch: "007-user-chathistory"
user: "user"
command: "/sp.specify"
labels: ["spec", "chat-history", "persistence"]
links:
  spec: "specs/007-user-chathistory/spec.md"
  ticket: null
  adr: null
  pr: null
files: []
tests: []
---

# Create spec for user chat history persistence

## Prompt Text
```
name the new branch "007-user-chathistory" heres my specs:
## Goal
Replace the in-memory \`MemoryStore\` with a \`NeonStore\` that persists ChatKit threads and messages to the Neon PostgreSQL database. This enables per-user chat history that survives server restarts.

---

## Critical Context

### Current Architecture
- **Frontend**: Uses \`@openai/chatkit-react\` in \`FloatingChatWidget.tsx\`
- **Backend**: FastAPI with ChatKit server (\`main.py\`)
- **Current Store**: \`MemoryStore\` in \`backend/src/backend/store.py\` (stores in RAM, lost on restart)
- **Auth**: Users are authenticated via \`better-auth\` (auth-server on port 3001), user IDs stored in \`user\` table

### Database (Neon PostgreSQL)
Connection: \`DATABASE_URL\` env var in \`backend/.env\`

**Existing Tables:**
- \`user\` (id TEXT PRIMARY KEY, name, email, etc.)
- \`chat_history\` (old table - can be repurposed or replaced)

---

## Implementation Steps

### Step 1: Create New Database Tables (Migration)

Create file: \`backend/migrations/003_create_chatkit_tables.sql\`

\`\`\`sql
-- ChatKit threads table
CREATE TABLE IF NOT EXISTS "chatkit_thread" (
    "id" TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "metadata" JSONB DEFAULT '{}',
    "createdAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- ChatKit thread items (messages) table
CREATE TABLE IF NOT EXISTS "chatkit_thread_item" (
    "id" TEXT PRIMARY KEY,
    "threadId" TEXT NOT NULL REFERENCES "chatkit_thread"("id") ON DELETE CASCADE,
    "type" TEXT NOT NULL,  -- 'user', 'assistant', etc.
    "content" JSONB NOT NULL,  -- Store full item as JSON
    "createdAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- ChatKit attachments table
CREATE TABLE IF NOT EXISTS "chatkit_attachment" (
    "id" TEXT PRIMARY KEY,
    "threadId" TEXT REFERENCES "chatkit_thread"("id") ON DELETE CASCADE,
    "content" JSONB NOT NULL,
    "createdAt" TIMESTAMP WITH ZONE DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS "idx_chatkit_thread_user" ON "chatkit_thread" ("userId");
CREATE INDEX IF NOT EXISTS "idx_chatkit_thread_created" ON "chatkit_thread" ("createdAt" DESC);
CREATE INDEX IF NOT EXISTS "idx_chatkit_item_thread" ON "chatkit_thread_item" ("threadId");
CREATE INDEX IF NOT EXISTS "idx_chatkit_item_created" ON "chatkit_thread_item" ("createdAt");
\`\`\`

### Step 2: Create NeonStore Class

Create file: \`backend/src/backend/neon_store.py\`

**IMPORTANT**: The NeonStore must implement the **same interface** as MemoryStore. Copy the structure from \`store.py\` exactly.

**Required Methods (14 total):**
1. \`generate_thread_id(context)\` - Generate unique thread ID
2. \`generate_item_id(item_type, thread, context)\` - Generate unique item ID
3. \`load_thread(thread_id, context)\` - Load or create thread
4. \`save_thread(thread, context)\` - Save thread metadata
5. \`load_threads(limit, after, order, context)\` - List threads (paginated)
6. \`delete_thread(thread_id, context)\` - Delete thread and items
7. \`load_thread_items(thread_id, after, limit, order, context)\` - Load messages (paginated)
8. \`add_thread_item(thread_id, item, context)\` - Add/update message
9. \`save_item(thread_id, item, context)\` - Alias for add_thread_item
10. \`load_item(thread_id, item_id, context)\` - Load single message
11. \`delete_thread_item(thread_id, item_id, context)\` - Delete message
12. \`save_attachment(attachment, context)\` - Save attachment
13. \`load_attachment(attachment_id, context)\` - Load attachment
14. \`delete_attachment(attachment_id, context)\` - Delete attachment

**Key Implementation Details:**

\`\`\`python
import uuid
import json
from datetime import datetime, timezone
from typing import Any
import psycopg2
from psycopg2.extras import RealDictCursor
import os

from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page

class NeonStore(Store[dict]):
    \"\"\"PostgreSQL-backed store for ChatKit (Neon)\"\"\"

    def __init__(self):
        self.database_url = os.getenv('DATABASE_URL')

    def _get_connection(self):
        \"\"\"Get database connection\"\"\"
        return psycopg2.connect(dsn=self.database_url, sslmode='require')

    def _get_user_id_from_context(self, context: dict) -> str | None:
        \"\"\"Extract user ID from context\"\"\"
        if not context:
            return None
        user = context.get('user', {})
        return user.get('id') or user.get('sub')

    # ... implement all 14 methods ...
\`\`\`

**Critical: User ID Extraction**
The \`context\` dict passed to each method contains user info. Extract it like this:
\`\`\`python
user_id = self._get_user_id_from_context(context)
\`\`\`

**Critical: ThreadItem Serialization**
ThreadItem objects must be serialized to JSON for storage:
\`\`\`python
# To save:
item_json = item.model_dump_json()

# To load (reconstruct):
from chatkit.types import UserMessageItem, AssistantMessageItem
# Parse JSON and use appropriate class
\`\`\`

**Critical: Filter by User**
When loading threads, ALWAYS filter by user_id:
\`\`\`sql
SELECT * FROM "chatkit_thread" WHERE "userId" = %s ORDER BY "createdAt" DESC LIMIT %s
\`\`\`

### Step 3: Update main.py to Use NeonStore

In \`backend/src/backend/main.py\`:

**Change 1**: Import NeonStore instead of MemoryStore
\`\`\`python
# OLD:
from .store import MemoryStore

# NEW:
from .neon_store import NeonStore
\`\`\`

**Change 2**: Replace store initialization
\`\`\`python
# OLD:
store = MemoryStore()

# NEW:
store = NeonStore()
\`\`\`

**Change 3**: Fix the \`/api/chatkit\` endpoint to pass REAL user ID (lines 333-354)

Currently it uses hardcoded \`"test_user"\`. Change to extract from auth:

\`\`\`python
@app.post("/api/chatkit")
async def chatkit_endpoint_no_auth(request: Request):
    request_body = await request.body()

    # Try to get user from localStorage auth (passed in headers or request)
    import json
    try:
        request_json = json.loads(request_body)
        # Check if user info was passed from frontend
        user_info = request_json.get('user', None)
        selected_text = request_json.get('selected_text', None)
    except json.JSONDecodeError:
        user_info = None
        selected_text = None

    # Build context with user info
    if user_info:
        context = {"user": user_info}
    else:
        # Fallback for unauthenticated (should not happen if auth is enforced)
        context = {"user": {"id": "anonymous", "name": "Anonymous"}}

    if selected_text:
        context['selected_text'] = selected_text

    result = await server.process(request_body, context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")
\`\`\`

### Step 4: Update Frontend to Send User ID

In \`frontend/src/components/ChatBot/FloatingChatWidget.tsx\`:

The ChatKit API config needs to send user information. Currently it's:
\`\`\`typescript
api: {
    url: \`\${backendUrl}/api/chatkit\`,
    domainKey: 'localhost',
},
\`\`\`

We need to include user data. Since the user is in localStorage (\`auth_user\`), fetch it and include in requests.

**Option A (Simplest)**: Use a custom fetch wrapper that adds user info to the request body.

**Option B**: Pass user ID as a query param or header.

---

## DO NOT Do These Things

1. **DO NOT** delete or modify the existing \`MemoryStore\` class - keep it as backup
2. **DO NOT** change the ChatKit Store interface - NeonStore must implement the exact same methods
3. **DO NOT** use synchronous database calls - all methods are \`async\`
4. **DO NOT** forget to handle the case where \`user_id\` is None/missing
5. **DO NOT** forget to close database connections after use (use try/finally or context managers)
6. **DO NOT** forget to serialize/deserialize ThreadItem objects properly

---

## Testing Checklist

After implementation:
1. [ ] Can create a new thread (first message in chat)
2. [ ] Thread persists after page refresh
3. [ ] Thread persists after server restart
4. [ ] Different users see their own threads only
5. [ ] Messages appear in correct order
6. [ ] Deleting a thread removes all its messages
7. [ ] No errors in server logs during chat

---

## Files Summary

| File | Action |
|------|--------|
| \`backend/migrations/003_create_chatkit_tables.sql\` | CREATE new |
| \`backend/src/backend/neon_store.py\` | CREATE new |
| \`backend/src/backend/main.py\` | MODIFY (import, init, endpoint) |
| \`frontend/src/components/ChatBot/FloatingChatWidget.tsx\` | MODIFY (send user info) |

---

## Environment Variables Required

Ensure these are set in \`backend/.env\`:
- \`DATABASE_URL\` - Neon PostgreSQL connection string (already exists)
```

## Response Text
Created feature branch 007-user-chathistory and wrote comprehensive specification for user chat history persistence feature. The spec includes user scenarios, functional requirements, key entities, and success criteria focused on enabling persistent chat history that survives server restarts and maintains user privacy.

## Outcome
Specification created successfully with all required sections completed. The specification is ready for the planning phase.

## Evaluation
The specification meets all requirements with clear user scenarios, testable functional requirements, and measurable success criteria. No clarification markers were needed as reasonable assumptions were made based on the provided context.