# ChatKit Debug Skill

## What This Skill Does
Diagnoses and fixes common ChatKit integration issues across backend and frontend.

## When to Use
- User encounters import errors
- User sees blank screen or UI not loading
- User reports "agent doesn't remember" issues
- User gets CORS errors or connection failures
- User needs diagnostic scripts

## Key Error Categories
1. **Backend Import Errors** — Wrong module names (chatkit.stores vs chatkit.store)
2. **Backend Runtime Errors** — Missing Store methods, type errors
3. **Frontend Errors** — Missing CDN, wrong config keys, blank screen

## Quick Fixes
| Problem | Solution |
|---------|----------|
| `chatkit.stores not found` | Use `chatkit.store` (singular) |
| `FatalAppError: Invalid input at api` | Add `domainKey: 'localhost'` |
| Blank screen | Add CDN script to index.html |
| Agent forgets context | Implement conversation history loading |

## Related Skills
- `chatkit-backend.skill` — Backend setup
- `chatkit-frontend.skill` — Frontend setup
- `chatkit-agent-memory` — Memory fix
