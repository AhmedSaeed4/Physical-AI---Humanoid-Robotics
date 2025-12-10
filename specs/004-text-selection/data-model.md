# Data Model: Text Selection to Chat Feature

## Overview
This document defines the data structures and entities for the text selection to chat feature.

## Entities

### SelectedText
**Description**: The text content that the user has highlighted/selected on the page, including position coordinates

**Fields**:
- `text`: string - The actual selected text content (up to 1000 characters)
- `position`: { x: number, y: number } - Coordinates for positioning the contextual button
- `hasSelection`: boolean - Flag indicating whether text is currently selected
- `elementRect`: { top: number, left: number, width: number, height: number } - Bounding rectangle of the selection

**Validation Rules**:
- Text must not be empty or whitespace-only
- Text length must be between 1 and 1000 characters
- Position coordinates must be valid numbers

### ContextualButton
**Description**: The UI element that appears near the selection offering the "Ask AI" functionality

**Fields**:
- `isVisible`: boolean - Whether the button is currently visible
- `position`: { x: number, y: number } - Current position on screen
- `textContent`: string - Button text (e.g., "Ask AI about this")
- `icon`: string - Icon identifier (e.g., "sparkle", "ai")

**Validation Rules**:
- Must not appear when no text is selected
- Must disappear when user clicks elsewhere or deselects text
- Position must be adjusted to stay within viewport bounds

### ChatContext
**Description**: The data structure that includes both the selected text and user's additional query

**Fields**:
- `selectedText`: string - The text selected by the user (optional)
- `userQuery`: string - The user's question or query
- `chatHistory`: array - Previous messages in the conversation
- `userProfile`: object - User profile data for personalization

**Validation Rules**:
- When selectedText is present, it should be combined with userQuery for better context
- Must maintain backward compatibility when selectedText is not provided
- User profile data must be properly injected into system prompts

## API Request/Response Models

### Chat Request with Selected Text
**Description**: Enhanced request model for the chat endpoint

**Fields**:
- `user_query`: string - The user's question
- `selected_text`: string (optional) - Text selected by user on the page
- `chat_history`: array (optional) - Previous conversation history
- `user_profile`: object (optional) - User profile information

### Chat Response with Selected Text Context
**Description**: Enhanced response model that indicates if selected text was used

**Fields**:
- `output`: string - AI response text
- `context_chunks`: array - Retrieved context chunks from RAG
- `sources`: array - Sources of the information
- `used_selected_text`: boolean - Whether the selected text was used in the response