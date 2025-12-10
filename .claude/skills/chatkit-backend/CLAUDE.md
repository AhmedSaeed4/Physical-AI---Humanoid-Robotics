# ChatKit Backend Skill

## What This Skill Does
Creates a production-ready ChatKit Python backend with FastAPI, OpenAI Agents SDK, and LiteLLM for multi-provider AI support.

## When to Use
- User asks about setting up a ChatKit backend/server
- User wants to create a Python API for ChatKit
- User needs model configuration for Gemini, OpenAI, or Anthropic
- User asks about the `/chatkit` endpoint

## Key Components
1. **FastAPI Server** — Main application setup with CORS
2. **Model Configuration** — LiteLLM setup for different providers
3. **ChatKit Endpoint** — POST `/chatkit` for chat requests
4. **Debug Endpoint** — GET `/debug/threads` to inspect stored items

## Quick Reference
- Port default: 8000
- Use `StreamingResponse` for streaming events
- Always add CORS middleware for frontend access

## Related Skills
- `chatkit-store.skill` — Store implementation (required)
- `chatkit-agent-memory` — Conversation history
- `chatkit-frontend.skill` — React frontend
