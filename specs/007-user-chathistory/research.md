# Research: User Chat History Persistence

## Decision: Neon PostgreSQL Store Implementation
**Rationale**: Replace in-memory MemoryStore with NeonStore to persist ChatKit threads and messages in Neon PostgreSQL database, enabling user chat history that survives server restarts while maintaining data isolation between users.

## Alternatives Considered:
1. **Keep in-memory storage**: Simple but loses data on server restarts, violating core requirement
2. **Use Redis**: Could work but adds additional infrastructure dependency when PostgreSQL is already available
3. **File-based storage**: Possible but harder to manage and scale than database storage
4. **Neon PostgreSQL Store (Chosen)**: Uses existing infrastructure, provides persistence, supports user isolation, and scales well

## Database Schema Design
- **chatkit_thread**: Stores chat thread information with userId foreign key for user association
- **chatkit_thread_item**: Stores individual messages with thread association and chronological ordering
- **chatkit_attachment**: Table exists but will not be used since attachments are prohibited (per clarifications)

## Key Technical Decisions:
1. **User Context Handling**: Extract user ID from authentication context to filter threads by user
2. **Thread ID Generation**: Use UUIDs to ensure unique thread identifiers across all users
3. **Message Ordering**: Store timestamps to maintain chronological order of messages
4. **Data Isolation**: Use userId in all queries to ensure users only see their own threads
5. **Error Handling**: Fail gracefully when database is unavailable with user notifications

## Implementation Approach:
1. Create database migration for new tables
2. Implement NeonStore class with all required ChatKit store interface methods
3. Update backend to use NeonStore instead of MemoryStore
4. Modify frontend to pass user context to backend