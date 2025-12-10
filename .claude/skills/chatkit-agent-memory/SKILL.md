# Skill: ChatKit Agent Memory (Conversation History)

## Metadata
- **Name**: chatkit-agent-memory
- **Category**: component
- **Tags**: chatkit, agent, memory, conversation, history, litellm, gemini

## Description
Implements conversation history loading for ChatKit agents with LiteLLM/Gemini ID collision fix. This is CRITICAL - without it, the agent only sees the current message and cannot remember previous context.

## The Problem

Two issues need to be solved:

1. **No Conversation Memory**: By default, ChatKit's `simple_to_agent_input(input)` only passes the **current message** to the agent. This means:
   - Agent doesn't know user's name after they said it
   - Agent forgets context from previous messages
   - Agent repeats questions already answered

2. **LiteLLM/Gemini ID Collision**: When using non-OpenAI providers via LiteLLM, the `stream_agent_response` function reuses message IDs from the provider's response. These IDs may collide, causing messages to **overwrite each other** instead of creating new messages.

## The Solution

1. Use `ThreadItemConverter.to_agent_input()` to load and convert full conversation history
2. Map incoming message IDs to unique store-generated IDs

## Implementation

### Complete ChatKitServer with Memory AND ID Fix

```python
from typing import Any, AsyncIterator

from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel

from chatkit.server import ChatKitServer, StreamingResult
from chatkit.store import Store
from chatkit.types import (
    ThreadMetadata, ThreadItem,
    ThreadItemAddedEvent, ThreadItemDoneEvent, ThreadItemUpdatedEvent,
    AssistantMessageItem
)
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter


class ChatKitServerWithMemory(ChatKitServer[dict]):
    """ChatKit server with full conversation memory and LiteLLM ID fix"""

    def __init__(self, data_store: Store, model: LitellmModel, instructions: str):
        super().__init__(data_store)

        self.agent = Agent[AgentContext](
            name="Assistant",
            instructions=instructions,
            model=model,
        )
        # Use ChatKit's official converter for proper item handling
        self.converter = ThreadItemConverter()

    async def respond(
        self,
        thread: ThreadMetadata,
        input: Any,
        context: dict
    ) -> AsyncIterator:
        """Generate response with full conversation context and unique IDs"""

        # Create agent context
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Load all thread items from store
        page = await self.store.load_thread_items(
            thread.id,
            after=None,
            limit=100,
            order="asc",
            context=context
        )
        all_items = list(page.data)

        # Add current input to the conversation
        if input:
            all_items.append(input)

        print(f"[Server] Processing {len(all_items)} items for agent")

        # Convert using ChatKit's official converter
        agent_input = await self.converter.to_agent_input(all_items) if all_items else []

        print(f"[Server] Converted to {len(agent_input)} agent input items")

        # Run agent with full history
        result = Runner.run_streamed(
            self.agent,
            agent_input,
            context=agent_context,
        )

        # CRITICAL: Track ID mappings to fix LiteLLM/Gemini ID collision
        id_mapping: dict[str, str] = {}

        async for event in stream_agent_response(agent_context, result):
            # Fix potential ID collisions from LiteLLM/Gemini
            if event.type == "thread.item.added":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    # Generate unique ID if we haven't seen this response ID
                    if old_id not in id_mapping:
                        new_id = self.store.generate_item_id("message", thread, context)
                        id_mapping[old_id] = new_id
                        print(f"[Server] Mapping ID {old_id} -> {new_id}")
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

### Usage

```python
from agents.extensions.models.litellm_model import LitellmModel

# Create model (Gemini, OpenAI, Anthropic, etc.)
model = LitellmModel(
    model="gemini/gemini-2.0-flash",
    api_key=os.getenv("GEMINI_API_KEY"),
)

# Create store
store = MemoryStore()

# Create server WITH memory AND ID fix
server = ChatKitServerWithMemory(
    data_store=store,
    model=model,
    instructions="You are a helpful assistant. Remember what the user tells you."
)
```

## Key Points

1. **Use `ThreadItemConverter`** - Don't manually build conversation history
2. **Use `order="asc"`** - Messages need chronological order
3. **Map IDs for LiteLLM** - Generate unique IDs to prevent collisions
4. **Handle all event types** - `added`, `done`, and `updated` events need ID mapping
5. **Limit history** - 100 messages is reasonable; adjust as needed

## Why ID Mapping is Required

The `stream_agent_response` function (line 514-521 in chatkit/agents.py) does:
```python
yield ThreadItemAddedEvent(
    item=AssistantMessageItem(
        id=item.id,  # <-- Uses provider's ID, may collide!
        ...
    )
)
```

When using LiteLLM with Gemini/Anthropic, these IDs may be:
- Reused across different responses
- Not unique within a thread
- Causing messages to overwrite each other in the store

## Testing

```
User: My name is Alice
Assistant: Nice to meet you, Alice!

User: What's my name?
Assistant: Your name is Alice.  <-- CORRECT (has memory, separate messages)

# Without ID fix:
# Both responses might overwrite each other, showing only one message
```

## Debug Endpoint

Add this to verify items are stored correctly:
```python
@app.get("/debug/threads")
async def debug_threads():
    result = {}
    for thread_id, state in store._threads.items():
        items = [{"id": i.id, "type": type(i).__name__} for i in state.items]
        result[thread_id] = {"items": items, "count": len(items)}
    return result
```

## Validation
- [ ] Agent remembers user's name after being told
- [ ] Agent recalls previous topics
- [ ] Messages appear separately (not merged)
- [ ] `/debug/threads` shows unique IDs for each message
- [ ] No "Updated existing item" logs for new messages

## Related Skills
- `chatkit-store` - Store implementation
- `chatkit-backend` - Full backend setup
- `chatkit-debug` - Troubleshooting
