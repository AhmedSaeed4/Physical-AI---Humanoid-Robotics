import os
import uuid
from pathlib import Path
from datetime import datetime, timezone
from typing import Any, AsyncIterator, List, Dict
from dataclasses import dataclass, field

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel

from agents import Agent, Runner, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI
import sys
from pathlib import Path
import os

from chatkit.server import ChatKitServer, StreamingResult
from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page
from chatkit.types import AssistantMessageItem
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter

# Add the backend directory to the path so we can import from backend.database
current_dir = Path(__file__).parent  # src/backend/
backend_dir = current_dir.parent.parent  # project root (where backend/ folder is)
database_dir = backend_dir / "backend"  # backend/ directory
sys.path.insert(0, str(database_dir))

# Import using direct path
from database import search_chunks, initialize_collection
import google.generativeai as genai
from qdrant_client import QdrantClient
import time

# Load environment variables
load_dotenv()

# Create the OpenAI client with Gemini's OpenAI-compatible endpoint
API_KEY = os.getenv("GEMINI_API_KEY")
if not API_KEY:
    raise ValueError("GEMINI_API_KEY not found in environment variables")

# Use AsyncOpenAI client with Gemini's OpenAI-compatible endpoint
client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Define the model using OpenAIChatCompletionsModel
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

# Configure the run settings
config = RunConfig(
    model=model,
    model_provider=client,
)

# Import store and adapter
from .store import MemoryStore
from .chatkit_adapter import ChatKitRAGAdapter


from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi import Depends

# Define ChatKit Server Implementation with proper streaming
class ChatKitServerImpl(ChatKitServer):
    def __init__(self, store: MemoryStore, model=None, config=None):
        super().__init__(store)
        self.store = store
        # Use ChatKit's official converter for proper item handling
        self.converter = ThreadItemConverter()
        self.model = model
        self.config = config

    async def respond(self, thread, input, context):
        """Handle ChatKit requests with RAG functionality using proper ChatKit patterns"""
        from chatkit.types import (
            ThreadItemAddedEvent, ThreadItemDoneEvent,
            ThreadItemUpdatedEvent, AssistantMessageItem
        )

        # Track ID mappings to ensure unique IDs (LiteLLM/Gemini fix)
        id_mapping: dict[str, str] = {}

        # Extract user message from input
        user_message = ""
        if hasattr(input, 'content') and input.content:
            for content_item in input.content:
                if hasattr(content_item, 'text') and content_item.text:
                    user_message = content_item.text
                    break
        elif isinstance(input, str):
            user_message = input

        if not user_message:
            # If no message content, return an empty response
            return

        # Search for relevant context from Qdrant
        try:
            collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
            search_results = search_chunks(
                collection_name=collection_name,
                query=user_message,
                limit=int(os.getenv("SEARCH_LIMIT", "5")),
                score_threshold=0.5
            )
        except Exception as e:
            # Fallback if search fails
            search_results = []

        # Build context string for the agent
        if search_results:
            context_str = "\n\n".join([chunk["text"] for chunk in search_results if chunk.get("text")])
            system_prompt = f"""
            You are a helpful assistant that answers questions based on the provided book content.
            Use the following context to answer the user's question:
            {context_str}

            If the context doesn't contain information to answer the question, say so.
            Always cite the source document when providing information from the context.
            """
        else:
            system_prompt = """
            You are a helpful assistant. The documentation search did not return any relevant results.
            Try to provide a helpful response based on general knowledge, and suggest the user rephrase their question if needed.
            """

        # Create the RAG agent with the context
        rag_agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Create agent context
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Load thread items for conversation history
        page = await self.store.load_thread_items(
            thread.id,
            after=None,
            limit=100,
            order="asc",
            context=context
        )
        all_items = list(page.data)

        # Add current input to the conversation
        if input:
            all_items.append(input)

        # Convert using ChatKit's official converter
        agent_input = await self.converter.to_agent_input(all_items) if all_items else []

        # Run agent with full history using streamed response
        result = Runner.run_streamed(
            rag_agent,
            agent_input,
            context=agent_context,
        )

        # Stream the response with ID collision fix
        async for event in stream_agent_response(agent_context, result):
            # Fix potential ID collisions from LiteLLM/Gemini
            if event.type == "thread.item.added":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    if old_id not in id_mapping:
                        new_id = self.store.generate_item_id("message", thread, context)
                        id_mapping[old_id] = new_id
                    event.item.id = id_mapping[old_id]

            elif event.type == "thread.item.done":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    if old_id in id_mapping:
                        event.item.id = id_mapping[old_id]

            elif event.type == "thread.item.updated":
                if event.item_id in id_mapping:
                    event.item_id = id_mapping[event.item_id]

            yield event


import secrets
from typing import Optional

# Simple in-memory user store (in production, use a proper database)
users_db = {}

# Simple in-memory token store (in production, use a proper database with expiration)
tokens_db = {}

# Create FastAPI app
app = FastAPI(title="Qdrant RAG API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Add any additional origins as needed for your frontend
    # For development with Docusaurus frontend
    allow_origin_regex=r"https?://localhost:300[0-1](\..*)?$"  # Allow localhost:3000 and 3001
)

# Simple authentication
security = HTTPBearer()

def create_access_token(user_id: str) -> str:
    """Create a simple access token"""
    token = secrets.token_urlsafe(32)
    tokens_db[token] = {
        "user_id": user_id,
        "created_at": datetime.now(timezone.utc)
    }
    return token

async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Get current user from token"""
    token = credentials.credentials
    if token not in tokens_db:
        raise HTTPException(status_code=401, detail="Invalid authentication token")

    user_data = tokens_db[token]
    return {"id": user_data["user_id"], "name": f"User {user_data['user_id']}"}

# Authentication endpoints
@app.post("/api/auth/login")
async def login(username: str, password: str):
    """Simple login endpoint (in production, properly hash passwords)"""
    # In a real app, verify credentials against a user database
    # For this example, we'll create a user if they don't exist
    user_id = f"user_{username}"
    users_db[user_id] = {
        "username": username,
        "created_at": datetime.now(timezone.utc)
    }

    token = create_access_token(user_id)
    return {"access_token": token, "token_type": "bearer", "user_id": user_id}

@app.post("/api/auth/logout")
async def logout(current_user: dict = Depends(get_current_user)):
    """Simple logout endpoint"""
    # Remove the token from the database
    # In a real app, you would need to get the token from the request
    # For now, we'll just return a success message
    return {"message": f"User {current_user['id']} logged out successfully"}

# Add ChatKit endpoint with authentication (for production)
@app.post("/api/chatkit-auth")
async def chatkit_endpoint_auth(request: Request, current_user: dict = Depends(get_current_user)):
    # Add user context to the server call
    context = {"user": current_user}
    result = await server.process(await request.body(), context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

# Add ChatKit endpoint without authentication (for testing)
@app.post("/api/chatkit")
async def chatkit_endpoint_no_auth(request: Request):
    # Add a default user context to the server call
    context = {"user": {"id": "test_user", "name": "Test User"}}
    result = await server.process(await request.body(), context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

# Initialize store and server
store = MemoryStore()
server = ChatKitServerImpl(store)


# Debug endpoint to inspect stored items
@app.get("/debug/threads")
async def debug_threads():
    result = {}
    for thread_id, state in store._threads.items():
        items = []
        for item in state.items:
            item_data = {"id": item.id, "type": type(item).__name__}
            if hasattr(item, 'content') and item.content:
                content_parts = []
                for part in item.content:
                    if hasattr(part, 'text'):
                        content_parts.append(part.text)
                item_data["content"] = content_parts
            items.append(item_data)
        result[thread_id] = {"items": items, "count": len(items)}
    return result

# Pydantic models for request/response
class ChatRequest(BaseModel):
    user_query: str
    selected_text: str = ""
    chat_history: List[Dict[str, str]] = []

class ContextChunk(BaseModel):
    id: str
    text: str
    filename: str
    chunk_number: int
    total_chunks: int
    score: float

class Source(BaseModel):
    filename: str
    url: str

class ChatResponse(BaseModel):
    output: str
    context_chunks: List[ContextChunk]
    sources: List[Source]

class SearchResponse(BaseModel):
    results: List[ContextChunk]

@app.on_event("startup")
def startup_event():
    """
    Initialize the Qdrant collection when the app starts
    """
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
    initialize_collection(collection_name)

@app.get("/")
async def root():
    """
    Health check endpoint
    """
    return {"message": "Qdrant RAG API is running"}

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest, current_user: dict = Depends(get_current_user)):
    """
    RAG chat endpoint - Process user query with RAG and return contextual response
    Now requires authentication
    """
    try:
        # Search for relevant chunks in the vector database
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        limit = int(os.getenv("SEARCH_LIMIT", 5))
        score_threshold = float(os.getenv("SCORE_THRESHOLD", 0.7))

        search_results = search_chunks(
            collection_name=collection_name,
            query=request.user_query,
            limit=limit,
            score_threshold=score_threshold
        )

        if not search_results:
            # If no relevant chunks found, return a response without context
            return ChatResponse(
                output="I couldn't find any relevant information in the book content to answer your question.",
                context_chunks=[],
                sources=[]
            )

        # Format the context from search results
        context_text = "\n\n".join([f"Source: {chunk['filename']}\nContent: {chunk['text']}"
                                   for chunk in search_results])

        # Prepare the prompt for the AI agent
        prompt = f"""
        You are a helpful assistant that answers questions based on book content provided in the context.
        Use the context to answer the user's query. If the context doesn't contain enough information,
        clearly state that you couldn't find relevant information.

        Context:
        {context_text}

        User's query: {request.user_query}
        """

        # Create the agent with the prompt
        agent = Agent(
            name="RAGBot",
            instructions=f"You are a helpful assistant that answers questions based on provided context. Always cite your sources from the context provided. Context: {context_text}",
            model=model
        )

        # Run the agent with the user query
        result = await Runner.run(agent, request.user_query, run_config=config)
        ai_response = result.final_output

        # Format context chunks for response
        context_chunks = [
            ContextChunk(
                id=chunk["id"],
                text=chunk["text"],
                filename=chunk["filename"],
                chunk_number=chunk["chunk_number"],
                total_chunks=chunk["total_chunks"],
                score=chunk["score"]
            )
            for chunk in search_results
        ]

        # Extract unique sources
        unique_sources = {}
        for chunk in search_results:
            if chunk["filename"] not in unique_sources:
                unique_sources[chunk["filename"]] = {
                    "filename": chunk["filename"],
                    "url": f"/docs/{chunk['filename']}"  # Adjust URL based on your frontend structure
                }

        sources_list = [Source(filename=src["filename"], url=src["url"]) for src in unique_sources.values()]

        return ChatResponse(
            output=ai_response,
            context_chunks=context_chunks,
            sources=sources_list
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.get("/api/search", response_model=SearchResponse)
async def search_endpoint(
    query: str = Query(..., min_length=1, description="Search query text"),
    limit: int = Query(default=5, ge=1, le=20, description="Maximum number of results to return"),
    score_threshold: float = Query(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score threshold"),
    current_user: dict = Depends(get_current_user)
):
    """
    Direct vector search endpoint - Perform direct vector similarity search on book content
    Now requires authentication
    """
    try:
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

        search_results = search_chunks(
            collection_name=collection_name,
            query=query,
            limit=limit,
            score_threshold=score_threshold
        )

        # Format results
        results = [
            ContextChunk(
                id=chunk["id"],
                text=chunk["text"],
                filename=chunk["filename"],
                chunk_number=chunk["chunk_number"],
                total_chunks=chunk["total_chunks"],
                score=chunk["score"]
            )
            for chunk in search_results
        ]

        return SearchResponse(results=results)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing search request: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)