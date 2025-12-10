# Quickstart: Text Selection to Chat Feature

## Overview
This guide helps developers understand and implement the text selection to chat feature that allows users to select text from book content and send it directly to the AI chatbot with additional context.

## Prerequisites
- Node.js 20+ (for frontend/auth)
- Python 3.12+ with `uv` package manager
- Docusaurus 3.x development environment
- Running backend server with RAG capabilities

## Getting Started

### 1. Frontend Components

#### Text Selection Hook
The `useTextSelection` hook handles detection of text selection:

```typescript
// frontend/src/components/TextSelection/useTextSelection.tsx
import { useState, useEffect } from 'react';

export const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [hasSelection, setHasSelection] = useState(false);

  // Implementation details...

  return { selectedText, position, hasSelection };
};
```

#### Selection Popup Component
The `SelectionPopup` displays the contextual button:

```typescript
// frontend/src/components/TextSelection/SelectionPopup.tsx
interface SelectionPopupProps {
  selectedText: string;
  position: { x: number, y: number };
  onAskAI: (text: string) => void;
  onClose: () => void;
}

const SelectionPopup: React.FC<SelectionPopupProps> = ({
  selectedText,
  position,
  onAskAI,
  onClose
}) => {
  // Implementation details...
};
```

### 2. Docusaurus Integration

#### Swizzled DocItem Component
The feature integrates with Docusaurus by wrapping the DocItem component:

```bash
# To swizzle the component if not already done
cd frontend
npm run swizzle @docusaurus/theme-classic DocItem -- --wrap
```

The swizzled component will include the text selection functionality while preserving all original behavior.

### 3. ChatBot Component Updates

#### Enhanced ChatBot Interface
The ChatBot component accepts a new prop and provides an `openWithSelection` method:

```typescript
interface ChatBotProps {
  initialSelectedText?: string;
  // ... other existing props
}

// Method to open chat with selected text
openWithSelection(text: string): void;
```

### 4. Backend API Enhancement

#### Updated Chat Endpoint
The backend `/api/chat` endpoint now accepts an optional `selected_text` parameter:

```python
# backend/src/backend/main.py
@app.post("/api/chat")
async def chat_endpoint(
    user_query: str,
    selected_text: Optional[str] = None,
    chat_history: List[Dict] = [],
    user_profile: Optional[Dict] = None,
    # ... other parameters
):
    # Enhanced processing with selected_text context
```

### 5. Development Workflow

#### Running the Application
1. Start the backend server:
   ```bash
   cd backend
   uv run uvicorn src.backend.main:app --reload
   ```

2. Start the Docusaurus frontend:
   ```bash
   cd frontend
   npm start
   ```

3. Select text on any documentation page and click the "Ask AI about this" button

#### Testing the Feature
- Select text on a documentation page
- Verify the contextual button appears near the selection
- Click the button and confirm the chat opens with selected text as context
- Ask a question about the selected text and verify the AI response addresses it directly

## Key Implementation Details

### Text Selection Detection
- Uses `window.getSelection()` API for cross-browser compatibility
- Calculates position using `getRangeAt(0).getBoundingClientRect()`
- Includes debouncing to avoid excessive re-renders
- Ignores selections within the chat widget itself

### Contextual Button Positioning
- Appears near the selected text with smooth animation
- Uses glassmorphism design matching existing chat widget
- Responsive positioning that avoids going off-screen
- Disappears when user deselects text or clicks elsewhere

### Backend Context Processing
- Combines selected text with user query for enhanced RAG
- Updates system prompt to prioritize selected text context
- Maintains backward compatibility with existing API clients
- Returns metadata indicating if selected text was used