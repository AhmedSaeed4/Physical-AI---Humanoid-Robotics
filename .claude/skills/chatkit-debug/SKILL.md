# Skill: ChatKit Debug & Troubleshooting

## Metadata
- **Name**: chatkit-debug
- **Category**: debug
- **Tags**: chatkit, debug, troubleshooting, errors

## Description
Diagnoses and fixes common ChatKit integration issues across backend and frontend.

## Quick Diagnostic Checklist

### Backend Health Check
```bash
# 1. Check server starts
python backend/main.py

# 2. Test health endpoint
curl http://localhost:8000/health

# 3. Check for import errors in logs
```

### Frontend Health Check
```bash
# 1. Check dev server starts
cd frontend && npm run dev

# 2. Open browser console for errors
# 3. Check Network tab for /chatkit requests
```

## Error Database

### Backend Import Errors

| Error | Wrong Code | Correct Code |
|-------|------------|--------------|
| `ModuleNotFoundError: chatkit.stores` | `from chatkit.stores import Store` | `from chatkit.store import Store` |
| `ModuleNotFoundError: chatkit.models` | `from chatkit.models import ...` | `from chatkit.types import ...` |
| `ImportError: Event` | `from chatkit.server import Event` | Remove - doesn't exist |
| `ImportError: ClientToolCallOutputItem` | `from chatkit.types import ClientToolCallOutputItem` | Use `Any` type |
| `ImportError: FilePart` | `from chatkit.types import FilePart` | Use `Any` type |

### Backend Runtime Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `Can't instantiate abstract class` | Missing Store methods | Implement ALL 14 methods including `save_attachment`, `load_attachment`, `delete_attachment` |
| Agent doesn't remember conversation | Only current message passed | Implement `_build_conversation_history()` |
| `TypeError: object is not subscriptable` | Wrong type access | Check item content structure |

### Frontend Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `FatalAppError: Invalid input at api` | Missing domainKey | Add `domainKey: 'localhost'` to api config |
| `Unrecognized key "name"` | Wrong prompt schema | Use `label` instead of `name` |
| `Unrecognized key "icon"` | Invalid property | Remove `icon` from prompts |
| Blank screen / no chat UI | Missing CDN | Add `<script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js" async></script>` |
| History not loading | Thread not persisted | Implement localStorage for thread ID |
| CORS error | Missing middleware | Add CORSMiddleware to FastAPI |

## Diagnostic Scripts

### Check All Imports
```python
# Run this to verify all imports work
try:
    from chatkit.server import ChatKitServer, StreamingResult
    print("✓ chatkit.server imports OK")
except ImportError as e:
    print(f"✗ chatkit.server: {e}")

try:
    from chatkit.store import Store
    print("✓ chatkit.store imports OK")
except ImportError as e:
    print(f"✗ chatkit.store: {e}")

try:
    from chatkit.types import ThreadMetadata, ThreadItem, Page
    print("✓ chatkit.types imports OK")
except ImportError as e:
    print(f"✗ chatkit.types: {e}")

try:
    from chatkit.agents import AgentContext, stream_agent_response
    print("✓ chatkit.agents imports OK")
except ImportError as e:
    print(f"✗ chatkit.agents: {e}")

try:
    from agents import Agent, Runner
    from agents.extensions.models.litellm_model import LitellmModel
    print("✓ agents imports OK")
except ImportError as e:
    print(f"✗ agents: {e}")
```

### Check Store Implementation
```python
# Verify all abstract methods are implemented
import inspect
from chatkit.store import Store

required_methods = [
    'generate_thread_id',
    'generate_item_id',
    'load_thread',
    'save_thread',
    'load_thread_items',
    'add_thread_item',
    'save_item',
    'load_item',
    'delete_thread_item',
    'load_threads',
    'delete_thread',
    'save_attachment',
    'load_attachment',
    'delete_attachment',
]

# Check your store class
from your_module import YourStore

for method in required_methods:
    if hasattr(YourStore, method):
        print(f"✓ {method}")
    else:
        print(f"✗ {method} - MISSING!")
```

### Test Conversation Memory
```python
# Add to your server for debugging
async def _build_conversation_history(self, thread, current_input, context):
    page = await self.store.load_thread_items(thread.id, None, 100, "asc", context)

    print(f"\n=== CONVERSATION DEBUG ===")
    print(f"Thread: {thread.id}")
    print(f"Items in store: {len(page.data)}")

    for i, item in enumerate(page.data):
        item_type = type(item).__name__
        text = self._extract_text(item)[:50] + "..." if len(self._extract_text(item)) > 50 else self._extract_text(item)
        print(f"  {i+1}. [{item_type}] {text}")

    print(f"=========================\n")

    # ... rest of method
```

## Network Debugging

### Check Request/Response
```javascript
// Add to browser console
const originalFetch = window.fetch;
window.fetch = async (...args) => {
  console.log('Fetch:', args[0]);
  const response = await originalFetch(...args);
  console.log('Response:', response.status, response.headers.get('content-type'));
  return response;
};
```

### Expected Response Types
- `POST /chatkit` for streaming: `text/event-stream`
- `POST /chatkit` for JSON: `application/json`

## Common Fixes Summary

### Backend
1. Use `chatkit.store` not `chatkit.stores`
2. Use `chatkit.types` not `chatkit.models`
3. Implement ALL 14 Store methods
4. Build full conversation history for agent

### Frontend
1. Add CDN script to index.html
2. Add `domainKey: 'localhost'`
3. Use `label` not `name` in prompts
4. Remove `icon` property
5. Persist thread ID in localStorage

## Related Skills
- `chatkit-backend` - Backend setup
- `chatkit-frontend` - Frontend setup
- `chatkit-store` - Store implementation
- `chatkit-agent-memory` - Conversation history
