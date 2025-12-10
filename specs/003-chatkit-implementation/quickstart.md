# Quickstart: ChatKit Implementation for Docusaurus RAG Chatbot

## Prerequisites

- Node.js 18+ with npm
- Python 3.12+
- uv (Python package manager)
- Access to Google Gemini API (for existing RAG functionality)
- Qdrant vector database running

## Setup Instructions

### 1. Environment Configuration
```bash
# Copy environment files
cp backend/.env.example backend/.env
cp frontend/.env.example frontend/.env

# Update with your Gemini API key and Qdrant configuration
```

### 2. Backend Setup
```bash
cd backend
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -e .
uvicorn src.backend.main:app --reload --port 8000
```

### 3. Frontend Setup
```bash
cd frontend
npm install
npm run dev
```

### 4. Ingest Documentation (Required for RAG)
```bash
cd backend
python -m src.ingest
```

## Key Integration Points

### Frontend Integration
- ChatKit components replace custom ChatBot implementation
- Embedded in Docusaurus layout at `/src/components/ChatBot/index.tsx`
- Uses CDN script for ChatKit functionality

### Backend Integration
- New ChatKit-compatible endpoints in `main.py`
- Store implementation in `store.py` handles thread persistence
- Adapter layer connects ChatKit to existing RAG pipeline

### RAG Pipeline Preservation
- Existing Qdrant integration remains unchanged
- OpenAI agent logic preserved in `agents/` directory
- Context chunk and source functionality maintained

## Testing the Integration

1. Navigate to your Docusaurus site (usually http://localhost:3000)
2. Look for the ChatKit widget (floating chat button)
3. Start a conversation about your documentation
4. Verify responses include context chunks and sources as before
5. Test thread persistence by refreshing the page

## Common Issues

### ChatKit CDN Issues
- Ensure the CDN script is included in your HTML
- Check browser console for loading errors

### Thread Persistence
- Verify store implementation is properly configured
- Check that thread IDs are being saved to localStorage

### RAG Functionality
- Confirm Qdrant is properly populated with documentation
- Verify Gemini API key is correctly configured
- Check that agent logic is being called properly