# ChatKit Store Skill

## What This Skill Does
Provides a complete ChatKit Store implementation with all 14 required abstract methods.

## When to Use
- User needs to implement a ChatKit Store
- User gets "Can't instantiate abstract class" errors
- User asks about thread/item persistence
- User needs MemoryStore implementation

## Critical Knowledge
**Correct Import:**
```python
from chatkit.store import Store  # SINGULAR - not 'stores'
from chatkit.types import ThreadMetadata, ThreadItem, Page
```

## All 14 Required Methods
1. `generate_thread_id`
2. `generate_item_id`
3. `load_thread`
4. `save_thread`
5. `load_thread_items`
6. `add_thread_item`
7. `save_item`
8. `load_item`
9. `delete_thread_item`
10. `load_threads`
11. `delete_thread`
12. `save_attachment` ⚠️ Often forgotten
13. `load_attachment` ⚠️ Often forgotten
14. `delete_attachment` ⚠️ Often forgotten

## Quick Reference
- Use `model_copy(deep=True)` for thread/item copies
- Handle pagination with `after` cursor
- Default to in-memory storage (dict)

## Related Skills
- `chatkit-backend.skill` — Full backend setup
- `chatkit-agent-memory` — Conversation history
