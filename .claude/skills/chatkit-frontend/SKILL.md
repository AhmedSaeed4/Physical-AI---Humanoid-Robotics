# Skill: ChatKit Frontend Setup

## Metadata
- **Name**: chatkit-frontend
- **Category**: setup
- **Tags**: chatkit, frontend, react, typescript

## Description
Creates a ChatKit React frontend with TypeScript, dark theme, and conversation persistence. Template uses Vite but works with any React framework (Next.js, CRA, Docusaurus, etc.).

## Inputs
| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| theme | string | No | dark | Color scheme: dark, light |
| greeting | string | No | Welcome! | Start screen greeting |
| backendUrl | string | No | http://localhost:8000/chatkit | Backend API URL |
| port | number | No | 3000 | Dev server port |
| layout | string | No | fullpage | Layout: fullpage, popup-right, popup-left |

## Outputs
- `frontend/index.html` - HTML with CDN script
- `frontend/package.json` - Dependencies
- `frontend/tsconfig.json` - TypeScript config
- `frontend/vite.config.ts` - Vite config
- `frontend/src/main.tsx` - Entry point
- `frontend/src/App.tsx` - ChatKit component

## Prerequisites
- Node.js 18+
- npm or yarn

## Execution Steps

### 1. Create Directory Structure
```
frontend/
├── index.html
├── package.json
├── tsconfig.json
├── vite.config.ts
└── src/
    ├── main.tsx
    └── App.tsx
```

### 2. Generate package.json
```json
{
  "name": "chatkit-frontend",
  "private": true,
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build",
    "preview": "vite preview"
  },
  "dependencies": {
    "@openai/chatkit-react": "^1.3.0",
    "react": "^18.3.1",
    "react-dom": "^18.3.1"
  },
  "devDependencies": {
    "@types/react": "^18.3.12",
    "@types/react-dom": "^18.3.1",
    "@vitejs/plugin-react": "^4.3.4",
    "typescript": "^5.6.3",
    "vite": "^6.0.3"
  }
}
```

### 3. Generate index.html

**CRITICAL: CDN script required**
```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>ChatKit App</title>
    <script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js" async></script>
    <style>
      * { margin: 0; padding: 0; box-sizing: border-box; }
      html, body, #root { height: 100%; width: 100%; }
      body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }
      .h-full { height: 100%; }
      .w-full { width: 100%; }
    </style>
  </head>
  <body>
    <div id="root"></div>
    <script type="module" src="/src/main.tsx"></script>
  </body>
</html>
```

### 4. Generate App.tsx

**CRITICAL Configuration:**
- `domainKey: 'localhost'` - REQUIRED for local dev
- Use `label` in prompts, NOT `name`
- Do NOT use `icon` property

```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react'
import { useState, useEffect } from 'react'

function App() {
  const [initialThread, setInitialThread] = useState<string | null>(null)
  const [isReady, setIsReady] = useState(false)

  useEffect(() => {
    const savedThread = localStorage.getItem('chatkit-thread-id')
    setInitialThread(savedThread)
    setIsReady(true)
  }, [])

  const { control } = useChatKit({
    api: {
      url: '{{BACKEND_URL}}',
      domainKey: 'localhost',
    },
    initialThread: initialThread,
    theme: {
      colorScheme: '{{THEME}}',
      color: {
        grayscale: { hue: 220, tint: 6, shade: -1 },
        accent: { primary: '#4cc9f0', level: 1 },
      },
      radius: 'round',
    },
    startScreen: {
      greeting: '{{GREETING}}',
      prompts: [
        { label: 'Hello', prompt: 'Say hello and introduce yourself' },
        { label: 'Help', prompt: 'What can you help me with?' },
      ],
    },
    composer: {
      placeholder: 'Type a message...',
    },
    onThreadChange: ({ threadId }) => {
      if (threadId) {
        localStorage.setItem('chatkit-thread-id', threadId)
      }
    },
    onError: ({ error }) => console.error('ChatKit error:', error),
  })

  if (!isReady) return <div>Loading...</div>

  return (
    <div style={{ height: '100vh', width: '100vw', display: 'flex', flexDirection: 'column' }}>
      <header style={{ padding: '1rem 2rem', background: '#16213e', borderBottom: '1px solid #0f3460', color: '#4cc9f0', fontSize: '1.5rem', fontWeight: 'bold', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span>ChatKit App</span>
        <button
          onClick={() => { localStorage.removeItem('chatkit-thread-id'); window.location.reload() }}
          style={{ padding: '0.5rem 1rem', background: '#4361ee', color: 'white', border: 'none', borderRadius: '0.5rem', cursor: 'pointer' }}
        >
          New Chat
        </button>
      </header>
      <main style={{ flex: 1, overflow: 'hidden' }}>
        <ChatKit control={control} className="h-full w-full" />
      </main>
    </div>
  )
}

export default App
```

### 5. Generate vite.config.ts
```ts
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    port: {{PORT}},
  }
})
```

### 6. Generate tsconfig.json
```json
{
  "compilerOptions": {
    "target": "ES2020",
    "useDefineForClassFields": true,
    "lib": ["ES2020", "DOM", "DOM.Iterable"],
    "module": "ESNext",
    "skipLibCheck": true,
    "moduleResolution": "bundler",
    "allowImportingTsExtensions": true,
    "resolveJsonModule": true,
    "isolatedModules": true,
    "noEmit": true,
    "jsx": "react-jsx",
    "strict": true
  },
  "include": ["src"]
}
```

### 7. Generate src/main.tsx
```tsx
import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App'

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
)
```

## Validation
- [ ] `npm install` completes without errors
- [ ] `npm run dev` starts dev server
- [ ] Chat UI renders (not blank)
- [ ] Messages can be sent
- [ ] Thread persists on page refresh

## Common Errors
| Error | Fix |
|-------|-----|
| Blank screen | Add CDN script to index.html |
| `FatalAppError: Invalid input at api` | Add `domainKey: 'localhost'` |
| `Unrecognized key "name"` | Use `label` not `name` |
| `Unrecognized key "icon"` | Remove `icon` property |

## Layout Variants

### Popup Chat (Bottom-Right) - Recommended

For a floating popup chat that appears in the bottom-right corner:

```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react'
import { useState, useEffect } from 'react'

function App() {
  const [initialThread, setInitialThread] = useState<string | null>(null)
  const [isReady, setIsReady] = useState(false)
  const [isChatOpen, setIsChatOpen] = useState(false)

  useEffect(() => {
    const savedThread = localStorage.getItem('chatkit-thread-id')
    setInitialThread(savedThread)
    setIsReady(true)
  }, [])

  const { control } = useChatKit({
    api: {
      url: '{{BACKEND_URL}}',
      domainKey: 'localhost',
    },
    initialThread: initialThread,
    theme: {
      colorScheme: 'dark',
      color: {
        grayscale: { hue: 220, tint: 6, shade: -1 },
        accent: { primary: '#4cc9f0', level: 1 },
      },
      radius: 'round',
    },
    startScreen: {
      greeting: '{{GREETING}}',
      prompts: [
        { label: 'Hello', prompt: 'Say hello' },
        { label: 'Help', prompt: 'What can you help me with?' },
      ],
    },
    onThreadChange: ({ threadId }) => {
      if (threadId) localStorage.setItem('chatkit-thread-id', threadId)
    },
  })

  if (!isReady) return <div>Loading...</div>

  return (
    <div style={{ height: '100vh', width: '100vw', background: '#1a1a2e' }}>
      {/* Your main page content here */}
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%' }}>
        <h1 style={{ color: '#4cc9f0' }}>Your App</h1>
      </div>

      {/* Floating Chat Button (bottom-right) */}
      {!isChatOpen && (
        <button
          onClick={() => setIsChatOpen(true)}
          style={{
            position: 'fixed',
            bottom: '2rem',
            right: '2rem',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            background: 'linear-gradient(135deg, #4361ee, #4cc9f0)',
            border: 'none',
            cursor: 'pointer',
            boxShadow: '0 4px 20px rgba(76, 201, 240, 0.4)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            zIndex: 100
          }}
        >
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat Popup (bottom-right) */}
      {isChatOpen && (
        <>
          {/* Backdrop */}
          <div
            onClick={() => setIsChatOpen(false)}
            style={{
              position: 'fixed',
              inset: 0,
              background: 'rgba(0, 0, 0, 0.3)',
              zIndex: 999
            }}
          />

          {/* Popup Window */}
          <div style={{
            position: 'fixed',
            bottom: '2rem',
            right: '2rem',
            width: '420px',
            height: '600px',
            maxWidth: 'calc(100vw - 4rem)',
            maxHeight: 'calc(100vh - 4rem)',
            background: '#16213e',
            borderRadius: '1rem',
            boxShadow: '0 10px 50px rgba(0, 0, 0, 0.5)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            zIndex: 1000,
            animation: 'popupIn 0.25s ease-out'
          }}>
            {/* Header */}
            <div style={{
              padding: '1rem',
              background: '#0f3460',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}>
              <span style={{ color: '#4cc9f0', fontWeight: 'bold' }}>Assistant</span>
              <div style={{ display: 'flex', gap: '0.5rem' }}>
                <button
                  onClick={() => { localStorage.removeItem('chatkit-thread-id'); window.location.reload() }}
                  style={{ padding: '0.4rem 0.6rem', background: '#4361ee', color: 'white', border: 'none', borderRadius: '0.375rem', cursor: 'pointer', fontSize: '0.7rem' }}
                >
                  New Chat
                </button>
                <button
                  onClick={() => setIsChatOpen(false)}
                  style={{ padding: '0.4rem', background: 'transparent', color: '#a0a0a0', border: '1px solid #a0a0a0', borderRadius: '0.375rem', cursor: 'pointer' }}
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <line x1="18" y1="6" x2="6" y2="18"></line>
                    <line x1="6" y1="6" x2="18" y2="18"></line>
                  </svg>
                </button>
              </div>
            </div>

            {/* Chat Content */}
            <div style={{ flex: 1, overflow: 'hidden' }}>
              <ChatKit control={control} className="h-full w-full" />
            </div>
          </div>
        </>
      )}

      {/* CSS Animation */}
      <style>{`
        @keyframes popupIn {
          from { opacity: 0; transform: scale(0.9) translateY(20px); }
          to { opacity: 1; transform: scale(1) translateY(0); }
        }
      `}</style>
    </div>
  )
}

export default App
```

### Key Popup Layout Features:
- **Floating button**: Fixed position bottom-right, 60x60px circular
- **Popup window**: 420x600px, positioned above the button
- **Backdrop**: Click to close, semi-transparent overlay
- **Header**: Title, "New Chat" button, close button
- **Animation**: Smooth scale + fade entrance

### Positioning Variants:
- **Bottom-right**: `right: '2rem'` (recommended)
- **Bottom-left**: `left: '2rem'`
- Change both the floating button AND popup window positions

## Related Skills
- `chatkit-backend` - Python backend
- `chatkit-store` - Store implementation
