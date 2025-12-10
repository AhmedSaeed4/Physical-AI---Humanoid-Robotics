# ChatKit Implementation Guide

This document describes the ChatKit implementation for the Docusaurus RAG Chatbot.

## Overview

The ChatKit implementation replaces the custom React chat interface with OpenAI's ChatKit components while preserving the existing RAG (Retrieval-Augmented Generation) functionality powered by the OpenAI Agents SDK with Google Gemini.

## Architecture

### Backend Components

1. **ChatKit Server** (`main.py`)
   - Implements the ChatKit server interface
   - Processes chat requests through the RAG pipeline
   - Handles thread management and message persistence
   - Uses OpenAI Agents SDK with LitellmModel for multi-provider support

2. **ChatKit Adapter** (`chatkit_adapter.py`)
   - Connects ChatKit with the existing RAG pipeline
   - Processes user messages through Qdrant vector search
   - Formats responses with context chunks and sources
   - Integrates with OpenAI Agents SDK and Google Gemini

3. **Memory Store** (`store.py`)
   - Implements the ChatKit store interface
   - Provides thread and message persistence
   - Supports all 14 required abstract methods

### Frontend Components

1. **ChatBot Component** (`frontend/src/components/ChatBot/index.tsx`)
   - Uses ChatKit React components
   - Integrates with Docusaurus layout
   - Includes thread history functionality

2. **Styling** (`frontend/src/components/ChatBot/ChatBot.module.css`)
   - Custom styles to match Docusaurus theme
   - Overrides default ChatKit styles
   - Thread history UI elements

## AI Agent Integration

### OpenAI Agents SDK with LitellmModel (ChatKit Recommended)
- Uses `agents` module with `Agent`, `Runner`, and `RunConfig`
- Configured with `LitellmModel` for multi-provider support (specifically recommended for ChatKit)
- Google Gemini model: `gemini/gemini-2.5-flash`
- Preserves original RAG agent logic and instructions
- Includes ID collision fix for proper message handling with non-OpenAI providers

### RAG Pipeline
- Maintains existing Qdrant vector database integration
- Preserves context chunk retrieval and source attribution
- Original prompt engineering and agent instructions preserved

## Features

### Core Functionality
- RAG-powered responses with context chunks
- Source attribution in responses
- Thread persistence across page refreshes
- Multi-turn conversation support

### Enhanced Features
- Thread history with localStorage tracking
- Authentication via token-based system
- Error handling and graceful failure modes
- Rate limiting for rapid query submission
- Input validation and sanitization

## API Endpoints

### ChatKit Endpoint
- `POST /api/chatkit` - ChatKit-compatible endpoint with authentication

### Authentication Endpoints
- `POST /api/auth/login` - User login
- `POST /api/auth/logout` - User logout

### Legacy Endpoints (now authenticated)
- `POST /api/chat` - Original chat endpoint (now requires auth)
- `GET /api/search` - Vector search endpoint (now requires auth)

## Error Handling

The system handles several error scenarios:
- Empty queries: Returns a prompt to enter a question
- Network failures: Returns a graceful error message
- No relevant results: Returns an appropriate response
- Long queries: Truncates to 1000 characters for processing
- Rapid submissions: Implements 500ms cooldown per thread

## Configuration

Environment variables used:
- `GEMINI_API_KEY` - API key for Google Gemini
- `QDRANT_COLLECTION_NAME` - Name of the Qdrant collection (default: book_content)
- `SEARCH_LIMIT` - Maximum number of search results (default: 5)
- `SCORE_THRESHOLD` - Minimum similarity score (default: 0.7)

## Migration Notes

This implementation maintains full compatibility with the existing OpenAI Agents SDK code and Qdrant integration while providing a more robust chat interface through ChatKit. The original agent logic, RAG pipeline, and AI model configuration are preserved.

The implementation follows ChatKit-specific best practices by using LitellmModel for multi-provider support, which includes important fixes for ID collision issues when using non-OpenAI providers like Google Gemini with ChatKit.