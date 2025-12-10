# Research: Text Selection to Chat Feature

## Overview
This document captures research findings for implementing the text selection to chat feature, resolving all technical unknowns and clarifying implementation decisions.

## Decision: Text Selection Detection Method
**Rationale**: Using `window.getSelection()` API is the standard and most reliable method for detecting text selection in web browsers. This API provides access to the current text selection and its properties including position coordinates via `getRangeAt(0).getBoundingClientRect()`.

**Alternatives considered**:
- MutationObserver approach: More complex and not suitable for immediate selection detection
- Custom selection tracking: Reinventing the wheel with more potential for bugs
- Third-party libraries: Would add unnecessary dependencies when native APIs are sufficient

## Decision: Contextual Button Positioning
**Rationale**: Using absolute positioning with coordinates from `getBoundingClientRect()` provides the most accurate placement of the contextual button near the selected text. This ensures the button appears in a natural location where users expect it.

**Alternatives considered**:
- Fixed position elements: Would not adapt to selection location
- CSS-based positioning: Less precise control over placement relative to selection
- Pointer position tracking: More complex and less reliable than range-based positioning

## Decision: Component Integration with Docusaurus
**Rationale**: Using Docusaurus swizzling to wrap the DocItem component is the recommended approach for customizing Docusaurus themes. This preserves all existing functionality while allowing the addition of text selection detection.

**Alternatives considered**:
- Global event listeners: Could interfere with other page elements
- Custom MDX components: Would require changing all documentation pages
- Theme override: Would be more complex and potentially break future updates

## Decision: ChatBot Integration Pattern
**Rationale**: Adding an `initialSelectedText` prop and `openWithSelection` method to the existing ChatBot component follows the principle of extending existing functionality rather than duplicating it. This maintains consistency with the existing component API.

**Alternatives considered**:
- Separate component: Would duplicate functionality and create maintenance overhead
- Global state management: More complex than necessary for this use case
- Context API: Overkill for simple prop passing between components

## Decision: Backend API Enhancement
**Rationale**: Extending the existing `/api/chat` endpoint with an optional `selected_text` parameter maintains backward compatibility while adding the new functionality. This follows REST API best practices for extending existing endpoints.

**Alternatives considered**:
- New endpoint: Would create unnecessary complexity and duplicate chat logic
- Query parameters: Less appropriate for potentially large text content
- Separate context endpoint: Would require multiple API calls instead of one

## Decision: RAG Context Augmentation Strategy
**Rationale**: Combining selected text with user query in both embedding generation and system prompt provides the most effective context for the AI response. This ensures the AI directly addresses the selected text while still leveraging the broader knowledge base.

**Alternatives considered**:
- Separate processing: Would require more complex backend logic
- Only system prompt enhancement: Would miss semantic similarity benefits in vector search
- Only embedding enhancement: Would not provide explicit context to the LLM