# Data Model: User Chat History Persistence

## Database Schema

### chatkit_thread table
- **id** (TEXT, PRIMARY KEY): Unique identifier for the thread (UUID)
- **userId** (TEXT, NOT NULL): Foreign key referencing user table (for data isolation)
- **metadata** (JSONB, DEFAULT '{}'): Stores thread metadata (title, creation date, last message date)
- **createdAt** (TIMESTAMP WITH TIME ZONE, DEFAULT NOW()): Thread creation timestamp
- **updatedAt** (TIMESTAMP WITH TIME ZONE, DEFAULT NOW()): Last update timestamp

**Relationships**:
- References `user` table via userId field with CASCADE delete

**Indexes**:
- idx_chatkit_thread_user on (userId) - for efficient user-specific queries
- idx_chatkit_thread_created on (createdAt DESC) - for chronological thread listing

### chatkit_thread_item table
- **id** (TEXT, PRIMARY KEY): Unique identifier for the message (UUID)
- **threadId** (TEXT, NOT NULL): Foreign key referencing chatkit_thread table
- **type** (TEXT, NOT NULL): Message type ('user', 'assistant', etc.)
- **content** (JSONB, NOT NULL): Full message content stored as JSON
- **createdAt** (TIMESTAMP WITH TIME ZONE, DEFAULT NOW()): Message creation timestamp

**Relationships**:
- References `chatkit_thread` table via threadId field with CASCADE delete

**Indexes**:
- idx_chatkit_item_thread on (threadId) - for retrieving messages in a thread
- idx_chatkit_item_created on (createdAt) - for chronological message ordering

### chatkit_attachment table (defined but not used)
- **id** (TEXT, PRIMARY KEY): Unique identifier for the attachment (UUID)
- **threadId** (TEXT): Foreign key referencing chatkit_thread table
- **content** (JSONB, NOT NULL): Attachment content stored as JSON
- **createdAt** (TIMESTAMP WITH TIME ZONE, DEFAULT NOW()): Creation timestamp

**Note**: This table is defined for interface compatibility but will not be used as file attachments are prohibited per feature requirements.

## Validation Rules
- All thread IDs must be unique
- All message IDs must be unique
- userId must reference an existing user in the user table
- threadId must reference an existing thread in chatkit_thread table
- type must be one of the valid ChatKit message types
- content must be valid JSONB format
- createdAt values cannot be in the future

## State Transitions
- Threads are created when first message is added to a new conversation
- Threads are updated when new messages are added (updatedAt field)
- Threads and all associated messages are deleted together via CASCADE
- Messages are immutable once created (no updates, only create/delete operations)