# Quickstart: Qdrant Vector Database Integration

## Prerequisites

- Python 3.12+
- uv package manager
- Qdrant Cloud account with cluster URL and API key
- Google Gemini API key

## Setup

1. **Install dependencies**:
   ```bash
   cd backend
   uv add fastapi uvicorn openai-agents openai google-generativeai qdrant-client psycopg2 pyjwt passlib bcrypt python-dotenv
   ```

2. **Configure environment variables** in `backend/.env`:
   ```env
   QDRANT_URL=https://your-cluster.qdrant.cloud:6333
   QDRANT_API_KEY=your_qdrant_api_key
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_COLLECTION_NAME=book_content
   SEARCH_LIMIT=5
   SCORE_THRESHOLD=0.7
   CHUNK_SIZE=1000
   CHUNK_OVERLAP=200
   ```

## Usage

### 1. Initialize the database connection and collection:

```python
from backend.database import initialize_collection

# Initialize the book_content collection
initialize_collection("book_content")
```

### 2. Ingest book content:

```bash
cd backend
uv run backend/ingest.py --docs_path ../frontend/docs
```

### 3. Start the backend server:

```bash
cd backend
uv run uvicorn src.backend.main:app --reload
```

### 4. Query the RAG system:

Send POST request to `http://localhost:8000/api/chat` with body:
```json
{
  "user_query": "Your question about the book content",
  "selected_text": "Optional selected text context",
  "chat_history": []
}
```

### 5. Use the ChatBot component in Docusaurus:

The ChatBot component is available at `frontend/src/components/ChatBot/index.tsx` and can be used in any Docusaurus page or document:

```jsx
import ChatBot from '@site/src/components/ChatBot';

<ChatBot />
```

## API Endpoints

- `POST /api/chat` - RAG chat endpoint with query and context
- `GET /api/search` - Direct vector search endpoint

## Testing

Run the test suite:
```bash
uv run backend/test_qdrant.py
```

## Frontend Integration

The frontend includes a ChatBot component that displays:
- AI-generated responses to user queries
- Source citations with document links
- Context chunks used in the response
- Loading states during RAG processing
- Error handling for API operations