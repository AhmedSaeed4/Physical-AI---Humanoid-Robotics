# ChatKit Frontend Skill

## What This Skill Does
Creates a ChatKit React frontend with TypeScript, dark theme, and conversation persistence.

## When to Use
- User asks about ChatKit frontend/UI
- User wants React chat interface
- User needs popup chat layout
- User asks about Vite setup for ChatKit

## Key Components
1. **index.html** — Must include CDN script
2. **App.tsx** — useChatKit hook configuration
3. **Layouts** — Fullpage or popup (bottom-right)
4. **Theme** — Dark/light with customizable colors

## Critical Requirements
- CDN script: `<script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js" async></script>`
- `domainKey: 'localhost'` for local development
- Use `label` not `name` in prompts
- Do NOT use `icon` property

## Quick Reference
- Default port: 3000
- Persist thread ID in localStorage
- Use `onThreadChange` callback to save thread

## Related Skills
- `chatkit-backend.skill` — Python backend (required)
- `chatkit-store.skill` — Store implementation
