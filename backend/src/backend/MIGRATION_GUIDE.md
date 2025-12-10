# Migration Guide: From Custom ChatBot to ChatKit Implementation

This guide describes how to migrate from the original custom React chatbot to the new ChatKit implementation.

## Overview

This migration replaces the custom React chat interface with OpenAI's ChatKit components while preserving all backend functionality including the OpenAI Agents SDK, RAG pipeline, and Qdrant integration. The implementation uses LitellmModel for multi-provider support, which is specifically recommended for ChatKit to handle ID collision issues with non-OpenAI providers.

## Frontend Changes

### 1. Component Replacement
**Before:**
- Custom `ChatBot/index.tsx` with form-based interface
- Manual state management for messages
- Custom styling in `ChatBot.module.css`

**After:**
- `ChatBot/index.tsx` uses `@openai/chatkit-react` components
- ChatKit manages conversation state
- Enhanced styling to match Docusaurus theme

### 2. Dependencies Added
- `@openai/chatkit-react` - ChatKit React components
- CDN script in Docusaurus config

### 3. State Management Changes
**Before:**
```typescript
const [messages, setMessages] = useState<Message[]>([]);
const [inputValue, setInputValue] = useState("");
```

**After:**
```typescript
const { control } = useChatKit({...});
// State managed by ChatKit
```

## Backend Changes

### 1. API Endpoint Changes
**Before:**
- `POST /api/chat` - Direct chat endpoint
- Manual request/response handling

**After:**
- `POST /api/chatkit` - ChatKit-compatible endpoint
- ChatKit server implementation
- Preserved original endpoint with authentication

### 2. Thread Management
**Before:**
- Manual thread state in component
- Simple localStorage for persistence

**After:**
- ChatKit MemoryStore implementation
- Full thread/item persistence system
- Enhanced thread history functionality

### 3. Authentication Added
- Token-based authentication system
- Login/logout endpoints
- All API endpoints now require authentication

### 4. AI Model Configuration (ChatKit Optimized)
**Before:**
```python
from agents import OpenAIChatCompletionsModel, AsyncOpenAI

client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)
```

**After (ChatKit Recommended):**
```python
from agents.extensions.models.litellm_model import LitellmModel

model = LitellmModel(
    model="gemini/gemini-2.5-flash",
    api_key=API_KEY,
)
```

The LitellmModel approach is specifically recommended for ChatKit implementations as it includes important fixes for ID collision issues when using non-OpenAI providers like Google Gemini with ChatKit.

## Preserved Functionality

### 1. AI Agent Integration
- OpenAI Agents SDK with `Agent`, `Runner`, and `RunConfig`
- LitellmModel for multi-provider support (with ID collision fix for ChatKit)
- Google Gemini model: `gemini/gemini-2.5-flash`
- Original RAG agent logic and instructions

### 2. RAG Pipeline
- Qdrant vector database integration
- Context chunk retrieval and processing
- Source attribution in responses

## Key Improvements

### 1. Enhanced UI/UX
- Professional chat interface with typing indicators
- Message streaming and loading states
- Thread history and persistence

### 2. Better Error Handling
- Network failure resilience
- Input validation and sanitization
- Rate limiting for rapid submissions

### 3. Security Enhancements
- Authentication for all endpoints
- Input sanitization and length limits
- Thread isolation

## Files Modified

### Frontend
- `frontend/src/components/ChatBot/index.tsx` - Complete rewrite with ChatKit
- `frontend/src/components/ChatBot/ChatBot.module.css` - Updated styles
- `frontend/docusaurus.config.ts` - Added ChatKit CDN script
- `frontend/package.json` - Added ChatKit dependency

### Backend
- `backend/src/backend/main.py` - Added ChatKit server and auth
- `backend/src/backend/chatkit_adapter.py` - New adapter implementation
- `backend/src/backend/store.py` - ChatKit MemoryStore implementation
- `backend/src/backend/CHATKIT_IMPLEMENTATION.md` - Documentation
- `backend/src/backend/MIGRATION_GUIDE.md` - This guide

## Environment Variables Required

Ensure the following environment variables are set:
- `GEMINI_API_KEY` - Google Gemini API key
- `QDRANT_COLLECTION_NAME` - Qdrant collection name (optional, defaults to "book_content")
- `SEARCH_LIMIT` - Search result limit (optional, defaults to 5)
- `SCORE_THRESHOLD` - Similarity score threshold (optional, defaults to 0.7)

## Testing the Migration

1. Verify chat functionality works with new interface
2. Test thread persistence across page refreshes
3. Confirm authentication works for all endpoints
4. Validate RAG responses include context and sources
5. Test error handling scenarios

## Rollback Plan

To rollback to the previous version:
1. Revert the `ChatBot/index.tsx` component to the original implementation
2. Remove ChatKit dependencies from `package.json`
3. Remove authentication from backend endpoints
4. Revert model configuration back to OpenAIChatCompletionsModel
5. Remove ChatKit-specific files and configurations

## Performance Considerations

- ChatKit may introduce slight latency for initial load
- Authentication adds minimal overhead to API calls
- Rate limiting prevents system overload
- Memory usage is optimized with proper cleanup

## Important Notes

This migration specifically uses LitellmModel for the AI integration, which is the ChatKit-recommended approach. This provides better compatibility with ChatKit's threading system and includes fixes for ID collision issues that can occur when using non-OpenAI providers like Google Gemini with ChatKit.