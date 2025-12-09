---
id: 2
title: qdrant-integration-plan
stage: plan
date: 2025-12-08
surface: agent
model: claude-opus-4-5-20251101
feature: qdrant-integration
branch: 001-qdrant-integration
user: Claude User
command: /sp.plan
labels: [qdrant, vector-database, rag, planning, architecture]
links:
  spec: specs/001-qdrant-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-qdrant-integration/plan.md
 - specs/001-qdrant-integration/research.md
 - specs/001-qdrant-integration/data-model.md
 - specs/001-qdrant-integration/quickstart.md
 - specs/001-qdrant-integration/contracts/rag-api.yaml
tests:
 - specs/001-qdrant-integration/contracts/rag-api.yaml
---

## Prompt

$ARGUMENTS

## Response snapshot

# Implementation Plan: Qdrant Vector Database Integration

**Branch**: `001-qdrant-integration` | **Date**: 2025-12-08 | **Spec**: specs/001-qdrant-integration/spec.md
**Input**: Feature specification from `/specs/001-qdrant-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement Qdrant vector database integration to enable RAG (Retrieval Augmented Generation) functionality for book content. This involves setting up Qdrant client connection, creating document ingestion pipeline with chunking and embedding generation using Google Gemini, and implementing search and chat endpoints.

## Technical Context

**Language/Version**: Python 3.12+
**Primary Dependencies**: FastAPI, qdrant-client, google-generativeai, openai-agents, uvicorn
**Storage**: Qdrant vector database (cloud), with metadata storage
**Testing**: pytest for backend components
**Target Platform**: Linux server environment
**Project Type**: Web application with frontend/backend separation
**Performance Goals**: <10s response time for RAG queries, <5s for vector search
**Constraints**: <100MB memory for ingestion process, secure API key handling
**Scale/Scope**: Single-user system with book content indexing

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance
- [X] Service Isolation: Architecture maintains tri-fold separation (Frontend/Backend/Auth Server)
- [X] Guardrail Implementation: AI interactions will pass through guardrail layer using OpenAI Agents SDK
- [X] Profile-Driven: System will inject user profile data into RAG prompts
- [X] Truth in Markdown: Vector DB will synchronize with documentation sources in frontend/docs/
- [X] Identity Propagation: JWT validation will be maintained for secure auth flow
- [X] CORS Configuration: Cross-origin settings will be verified for ports 3000, 3001, 8000

### Technology Standards Compliance
- [X] Stack Requirements: Docusaurus 3.x (frontend), Python 3.12+ (backend), Node.js 20+ (auth server)
- [X] Directory Structure: Confirmed backend/, auth-server/, frontend/ organization
- [X] Security: Secrets will be in .env files, not committed to version control

## Project Structure

### Documentation (this feature)

```text
specs/001-qdrant-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── backend/
│   ├── database.py      # Qdrant client and embedding functions
│   └── ingest.py        # Document ingestion script
├── src/backend/
│   └── main.py          # FastAPI app with RAG endpoints
├── .env                 # Environment variables
└── pyproject.toml       # Python dependencies (uv format)

frontend/
└── docs/                # Book content (Markdown files)
```

**Structure Decision**: Web application structure selected with backend handling RAG functionality and frontend providing book content.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

# Research: Qdrant Vector Database Integration

## Overview
This document captures research findings for implementing Qdrant vector database integration with book content for RAG functionality.

## Decision: Qdrant Cloud vs Self-Hosted
**Rationale**: Qdrant Cloud was selected based on the feature specification which specifically mentions Qdrant Cloud. This provides managed infrastructure, scalability, and reduced operational overhead compared to self-hosted solutions.

**Alternatives considered**:
- Self-hosted Qdrant: Requires infrastructure management but provides more control
- Pinecone: Commercial alternative but less flexibility in implementation
- Weaviate: Open-source alternative with GraphQL interface

## Decision: Google Gemini text-embedding-004 for Embeddings
**Rationale**: The feature specification specifically calls for Google Gemini text-embedding-004 model which produces 768-dim vectors. This model provides good quality embeddings with efficient processing.

**Alternatives considered**:
- OpenAI embeddings: More established but requires different API integration
- Sentence Transformers: Open-source alternatives but potentially slower

## Decision: FastAPI Backend with uv Package Manager
**Rationale**: Feature specification calls for Python 3.12+ with FastAPI and uv package manager. FastAPI provides excellent async support and automatic API documentation.

**Alternatives considered**:
- Flask: Simpler but less performant for async operations
- Django: More complex for this use case

## Decision: 1000-character Chunking with 200-character Overlap
**Rationale**: Based on feature specification, chunking at 1000 characters with 200-character overlap provides good balance between context preservation and retrieval efficiency.

**Alternatives considered**:
- Smaller chunks: More granular but might miss context
- Larger chunks: Less granular but might include too much irrelevant info

## Decision: OpenAI Agents SDK for RAG Implementation
**Rationale**: The feature specification specifically requires using OpenAI Agents SDK for the RAG system implementation, providing built-in guardrails and agent orchestration.

**Alternatives considered**:
- Direct LLM calls: Less structured, no built-in guardrails
- LangChain: Alternative framework but not specified in requirements

## Technical Unknowns Resolved

1. **Qdrant Collection Configuration**: Will use cosine distance with 768 dimensions to match Gemini embedding size
2. **Metadata Schema**: Will store filename, text content, chunk number, and total chunks per document
3. **Search Parameters**: Will use top-5 results with 0.7 score threshold as specified
4. **Environment Variables**: Will use QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY as specified

# Data Model: Qdrant Vector Database Integration

## Entities

### Book Content Chunk
- **Description**: A segment of book content that has been processed and embedded for vector storage
- **Fields**:
  - `id` (string): Unique identifier for the chunk in Qdrant
  - `text` (string): The actual content of the chunk (up to 1000 characters)
  - `filename` (string): Relative path to the source document
  - `chunk_number` (int): Position of this chunk within the document
  - `total_chunks` (int): Total number of chunks in the document
  - `embedding` (vector[768]): 768-dimensional vector embedding of the text content
  - `created_at` (timestamp): When the chunk was created
  - `updated_at` (timestamp): When the chunk was last updated

### Qdrant Collection
- **Description**: Vector database collection storing book content chunks with their embeddings and metadata
- **Fields**:
  - `name` (string): Name of the collection (book_content)
  - `vector_size` (int): Size of vectors (768 for Gemini embeddings)
  - `distance` (string): Distance metric (Cosine)
  - `chunks` (array): List of Book Content Chunk entities
  - `created_at` (timestamp): When the collection was created

### Query Response
- **Description**: The output from the RAG system containing the AI-generated answer and metadata
- **Fields**:
  - `output` (string): The AI-generated response to the user query
  - `context_chunks` (array): List of relevant content chunks used to generate the response
  - `sources` (array): List of source documents referenced in the response
  - `query_embedding` (vector[768]): Embedding of the original user query
  - `retrieval_time` (float): Time taken to retrieve relevant chunks
  - `generation_time` (float): Time taken to generate the response

### Search Result
- **Description**: Result from vector similarity search in Qdrant
- **Fields**:
  - `chunk_id` (string): ID of the matching chunk
  - `score` (float): Similarity score (0.0-1.0)
  - `text` (string): Content of the matching chunk
  - `metadata` (object): Additional metadata (filename, chunk_number, total_chunks)
  - `distance` (float): Distance value from the query

## Validation Rules

1. **Book Content Chunk**:
   - Text must be between 1 and 1000 characters
   - Chunk number must be between 1 and total_chunks
   - Embedding vector must be exactly 768 dimensions
   - Filename must be a valid relative path

2. **Query Response**:
   - Output must not be empty
   - Context chunks must be between 1 and 10
   - Sources must match the documents of context chunks

3. **Search Result**:
   - Score must be between 0.0 and 1.0
   - Distance must be non-negative

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

## API Endpoints

- `POST /api/chat` - RAG chat endpoint with query and context
- `GET /api/search` - Direct vector search endpoint

## Testing

Run the test suite:
```bash
uv run backend/test_qdrant.py
```

openapi: 3.0.0
info:
  title: Qdrant RAG API
  description: API for Retrieval Augmented Generation using Qdrant vector database
  version: 1.0.0
servers:
  - url: http://localhost:8000
    description: Local development server

paths:
  /api/chat:
    post:
      summary: RAG chat endpoint
      description: Process user query with RAG and return contextual response
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - user_query
              properties:
                user_query:
                  type: string
                  description: User's question or query about the book content
                  example: "What are the key concepts in chapter 3?"
                selected_text:
                  type: string
                  description: Optional selected text context from the frontend
                  example: "This is some selected text..."
                chat_history:
                  type: array
                  items:
                    type: object
                    properties:
                      role:
                        type: string
                        enum: [user, assistant]
                      content:
                        type: string
                  description: Previous conversation history
                  example: []
      responses:
        '200':
          description: Successful response with RAG-generated content
          content:
            application/json:
              schema:
                type: object
                required:
                  - output
                  - context_chunks
                  - sources
                properties:
                  output:
                    type: string
                    description: The AI-generated response to the user query
                    example: "Chapter 3 covers the fundamental concepts of..."
                  context_chunks:
                    type: array
                    items:
                      $ref: '#/components/schemas/ContextChunk'
                    description: List of relevant content chunks used for response generation
                  sources:
                    type: array
                    items:
                      $ref: '#/components/schemas/Source'
                    description: List of source documents referenced in the response
        '400':
          description: Bad request - invalid input parameters
        '500':
          description: Internal server error

  /api/search:
    get:
      summary: Direct vector search
      description: Perform direct vector similarity search on book content
      parameters:
        - name: query
          in: query
          required: true
          schema:
            type: string
          description: Search query text
        - name: limit
          in: query
          required: false
          schema:
            type: integer
            default: 5
          description: Maximum number of results to return
        - name: score_threshold
          in: query
          required: false
          schema:
            type: number
            format: float
            default: 0.7
          description: Minimum similarity score threshold
      responses:
        '200':
          description: Successful search results
          content:
            application/json:
              schema:
                type: object
                required:
                  - results
                properties:
                  results:
                    type: array
                    items:
                      $ref: '#/components/schemas/SearchResult'
        '400':
          description: Bad request - invalid query parameters
        '500':
          description: Internal server error

components:
  schemas:
    ContextChunk:
      type: object
      required:
        - id
        - text
        - filename
        - score
      properties:
        id:
          type: string
          description: Unique identifier for the chunk
          example: "chunk-123"
        text:
          type: string
          description: The content of the relevant chunk
          example: "The fundamental concept of this chapter is..."
        filename:
          type: string
          description: Relative path to the source document
          example: "docs/chapter-3.md"
        chunk_number:
          type: integer
          description: Position of this chunk within the document
          example: 2
        total_chunks:
          type: integer
          description: Total number of chunks in the document
          example: 5
        score:
          type: number
          format: float
          description: Similarity score for this chunk
          example: 0.85

    Source:
      type: object
      required:
        - filename
        - url
      properties:
        filename:
          type: string
          description: Name of the source document
          example: "docs/chapter-3.md"
        url:
          type: string
          description: URL to access the source document
          example: "/docs/chapter-3"

    SearchResult:
      type: object
      required:
        - id
        - text
        - filename
        - score
      properties:
        id:
          type: string
          description: Unique identifier for the chunk
          example: "search-result-456"
        text:
          type: string
          description: The content of the search result
          example: "The fundamental concept of this chapter is..."
        filename:
          type: string
          description: Relative path to the source document
          example: "docs/chapter-3.md"
        chunk_number:
          type: integer
          description: Position of this chunk within the document
          example: 2
        total_chunks:
          type: integer
          description: Total number of chunks in the document
          example: 5
        score:
          type: number
          format: float
          description: Similarity score for this result
          example: 0.85

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Qdrant vector database integration with all required artifacts
- 🧪 Tests: Created API contract in OpenAPI format for validation
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and contracts/rag-api.yaml
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan addresses all aspects of Qdrant integration including data models, API contracts, and implementation approach

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All planning artifacts completed with proper structure
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation phase to implement the planned features