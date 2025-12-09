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