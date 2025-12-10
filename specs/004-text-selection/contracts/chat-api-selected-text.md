# API Contract: Enhanced Chat Endpoint with Selected Text

## Overview
This contract defines the enhanced `/api/chat` endpoint that accepts selected text as additional context for AI queries.

## Endpoint: POST /api/chat

### Request
**Content-Type**: `application/json`

#### Request Body
```json
{
  "user_query": "string, the user's question or query",
  "selected_text": "string (optional), text selected by user on the page",
  "chat_history": "array (optional), previous conversation history",
  "user_profile": "object (optional), user profile information for personalization"
}
```

#### Example Request
```json
{
  "user_query": "Can you explain how this works?",
  "selected_text": "The QuickSight dashboard provides real-time analytics for user engagement metrics including page views, session duration, and conversion rates.",
  "chat_history": [
    {
      "role": "user",
      "content": "How do I set up the dashboard?"
    },
    {
      "role": "assistant",
      "content": "To set up the dashboard, you need to configure the data sources first..."
    }
  ],
  "user_profile": {
    "education": "intermediate",
    "experience": "developer",
    "interests": ["analytics", "dashboards"]
  }
}
```

### Response
**Content-Type**: `application/json`

#### Response Body
```json
{
  "output": "string, AI response text",
  "context_chunks": "array, retrieved context chunks from RAG",
  "sources": "array, sources of the information",
  "used_selected_text": "boolean, whether the selected text was used in the response"
}
```

#### Example Response
```json
{
  "output": "The selected text explains that QuickSight provides real-time analytics for user engagement. Based on this and the documentation, the dashboard tracks metrics like page views and session duration to help understand user behavior.",
  "context_chunks": [
    {
      "content": "QuickSight dashboard tracks page views, session duration, and conversion rates",
      "source": "docs/analytics/dashboard-setup.md"
    }
  ],
  "sources": [
    "docs/analytics/dashboard-setup.md",
    "docs/analytics/metrics.md"
  ],
  "used_selected_text": true
}
```

### Error Responses
- `400 Bad Request`: Invalid request format or missing required fields
- `401 Unauthorized`: Invalid or missing authentication token
- `500 Internal Server Error`: Server error during processing

## Backward Compatibility
The `selected_text` field is optional to maintain backward compatibility with existing clients. When omitted, the endpoint behaves exactly as before.