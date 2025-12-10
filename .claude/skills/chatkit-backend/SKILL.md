# Skill: ChatKit Backend Setup

## Metadata
- **Name**: chatkit-backend
- **Category**: setup
- **Tags**: chatkit, backend, python, fastapi, gemini, litellm

## Description
Creates a production-ready ChatKit Python backend with FastAPI, OpenAI Agents SDK, and LiteLLM for multi-provider AI support. Includes critical fix for LiteLLM/Gemini ID collision issue.

## Inputs
| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| provider | string | No | gemini | AI provider: gemini, openai, anthropic |
| model | string | No | auto | Specific model ID |
| port | number | No | 8000 | Server port |

## Outputs
- `backend/main.py` - FastAPI server with ChatKit
- `backend/requirements.txt` - Python dependencies
- `.env.example` - Environment template

## Prerequisites
- Python 3.10+
- pip

## Execution Steps

### 1. Create Directory Structure
```
backend/
├── main.py
├── requirements.txt
└── .venv/ (user creates)
```

### 2. Generate requirements.txt
```txt
fastapi==0.115.6
uvicorn[standard]==0.32.1
openai-chatkit<=1.4.0
openai-agents[litellm]>=0.6.2
python-dotenv==1.0.1
```

### 3. Generate main.py

**CRITICAL: Import ThreadItemConverter for proper history handling:**
```python
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter
```

**Template:**
```python
import os
import uuid
from pathlib import Path
from datetime import datetime, timezone
from typing import Any, AsyncIterator
from dataclasses import dataclass, field

from dotenv import load_dotenv
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel

from chatkit.server import ChatKitServer, StreamingResult
from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter

ROOT_DIR = Path(__file__).parent.parent
load_dotenv(ROOT_DIR / ".env")

# [INSERT STORE IMPLEMENTATION - use chatkit-store skill]

# [INSERT MODEL CONFIGURATION - based on provider param]

# [INSERT SERVER IMPLEMENTATION - use chatkit-agent-memory skill]

app = FastAPI(title="ChatKit Server")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

store = MemoryStore()
server = ChatKitServerImpl(store)

@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    result = await server.process(await request.body(), {})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

@app.get("/health")
async def health():
    return {"status": "ok"}

# Debug endpoint to inspect stored items
@app.get("/debug/threads")
async def debug_threads():
    result = {}
    for thread_id, state in store._threads.items():
        items = []
        for item in state.items:
            item_data = {"id": item.id, "type": type(item).__name__}
            if hasattr(item, 'content') and item.content:
                content_parts = []
                for part in item.content:
                    if hasattr(part, 'text'):
                        content_parts.append(part.text)
                item_data["content"] = content_parts
            items.append(item_data)
        result[thread_id] = {"items": items, "count": len(items)}
    return result

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port={{PORT}})
```

### 4. Model Configuration by Provider

**Gemini:**
```python
model = LitellmModel(
    model="gemini/gemini-2.0-flash",
    api_key=os.getenv("GEMINI_API_KEY"),
)
```

**OpenAI:**
```python
model = LitellmModel(
    model="openai/gpt-4o",
    api_key=os.getenv("OPENAI_API_KEY"),
)
```

**Anthropic:**
```python
model = LitellmModel(
    model="anthropic/claude-3-sonnet-20240229",
    api_key=os.getenv("ANTHROPIC_API_KEY"),
)
```

## CRITICAL: LiteLLM/Gemini ID Collision Fix

When using LiteLLM with non-OpenAI providers (Gemini, Anthropic), the `stream_agent_response` function reuses message IDs from the provider's response. These IDs may collide, causing messages to overwrite each other.

**Solution: ID Mapping in respond() method**
```python
async def respond(self, thread, input, context):
    from chatkit.types import (
        ThreadItemAddedEvent, ThreadItemDoneEvent,
        ThreadItemUpdatedEvent, AssistantMessageItem
    )

    # ... setup code ...

    # Track ID mappings to ensure unique IDs
    id_mapping: dict[str, str] = {}

    async for event in stream_agent_response(agent_context, result):
        # Fix potential ID collisions from LiteLLM/Gemini
        if event.type == "thread.item.added":
            if isinstance(event.item, AssistantMessageItem):
                old_id = event.item.id
                if old_id not in id_mapping:
                    new_id = self.store.generate_item_id("message", thread, context)
                    id_mapping[old_id] = new_id
                event.item.id = id_mapping[old_id]
        elif event.type == "thread.item.done":
            if isinstance(event.item, AssistantMessageItem):
                old_id = event.item.id
                if old_id in id_mapping:
                    event.item.id = id_mapping[old_id]
        elif event.type == "thread.item.updated":
            if event.item_id in id_mapping:
                event.item_id = id_mapping[event.item_id]

        yield event
```

## Validation
- [ ] Backend starts without import errors
- [ ] `/health` endpoint returns 200
- [ ] `/chatkit` endpoint accepts POST requests
- [ ] Messages don't overwrite each other (check `/debug/threads`)
- [ ] Agent remembers conversation context

## Related Skills
- `chatkit-store` - Store implementation
- `chatkit-agent-memory` - Conversation history with ID fix
- `chatkit-frontend` - React frontend
