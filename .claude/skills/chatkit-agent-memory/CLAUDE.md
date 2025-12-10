# ChatKit Agent Memory Skill

## What This Skill Does
Implements conversation history loading for ChatKit agents. Without this, the agent only sees the current message and cannot remember previous context.

## When to Use
- User asks about making the agent "remember" conversations
- User mentions "conversation history" or "context"
- User encounters LiteLLM/Gemini ID collision issues
- User asks why the agent repeats questions or forgets things

## Key Components
1. **ThreadItemConverter** — Loads and converts full conversation history
2. **ID Mapping** — Fixes LiteLLM/Gemini ID collision issues
3. **ChatKitServerWithMemory** — Server class with memory built-in

## Quick Reference
- Use `order="asc"` when loading thread items
- Limit history to ~100 messages for performance
- Map IDs for `added`, `done`, and `updated` events

## Related Skills
- `chatkit-store.skill` — Store implementation
- `chatkit-backend.skill` — Full backend setup
