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
from .neon_store import NeonStore
from .store import MemoryStore
from .chatkit_adapter import ChatKitRAGAdapter


from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi import Depends

# Define ChatKit Server Implementation with proper streaming
class ChatKitServerImpl(ChatKitServer):
    def __init__(self, store: NeonStore, model=None, config=None):
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

        # Extract selected text from context if available
        selected_text = None
        if context and isinstance(context, dict):
            # Check if selected text was passed in the context
            selected_text = context.get('selected_text', None)

        # Build the query based on whether selected text is provided
        if selected_text:
            # Combine user query with selected text for better context
            enhanced_query = f"{user_message}\n\nContext from selected text: {selected_text}"
            search_query = enhanced_query
        else:
            search_query = user_message

        # Search for relevant context from Qdrant
        try:
            collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
            search_results = search_chunks(
                collection_name=collection_name,
                query=search_query,
                limit=int(os.getenv("SEARCH_LIMIT", "5")),
                score_threshold=0.5
            )
        except Exception as e:
            # Fallback if search fails
            search_results = []

        # Prepare context string for the agent
        if search_results:
            context_str = "\n\n".join([chunk["text"] for chunk in search_results if chunk.get("text")])
        else:
            context_str = "No relevant information found in the book content."

        # Extract user profile from context if available
        user_profile_context = ""
        if context and isinstance(context, dict):
            user = context.get('user', {})
            if user:
                user_profile_context = f"""
                The user has the following background:
                - Education level: {user.get('educationLevel', user.get('education_level', 'Not specified'))}
                - Programming experience: {user.get('programmingExperience', user.get('programming_experience', 'Not specified'))}
                - Robotics background: {user.get('roboticsBackground', user.get('robotics_background', 'Not specified'))}

                Tailor your explanations to match their experience level.
                """

        # Build system prompt with selected text context if available and user profile if available
        if selected_text:
            system_prompt = f"""
            You are a helpful assistant helping users understand book content.
            {user_profile_context}

            The user has selected this specific text from the book:
            "{selected_text}"

            Use this as the primary context for your response. Also consider the following
            relevant sections from the book: {context_str}

            Answer the user's question clearly and specifically reference the selected text.
            """
        else:
            system_prompt = f"""
            You are a helpful assistant that answers questions based on the provided book content.
            {user_profile_context}
            Use the following context to answer the user's question: {context_str}

            If the context doesn't contain information to answer the question, say so.
            Always cite the source document when providing information from the context.
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
    allow_origins=[
        "https://physical-ai-humanoid-robotics-ai.netlify.app",  # Production frontend
        "http://localhost:3000",  # Frontend (Docusaurus)
        "http://localhost:3001",  # Auth Server
        "http://localhost:8000",  # Backend (self - for frontend to backend calls)
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
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
    """Get current user from token with full profile from database"""
    token = credentials.credentials
    if token not in tokens_db:
        raise HTTPException(status_code=401, detail="Invalid authentication token")

    user_data = tokens_db[token]
    user_id = user_data["user_id"]
    
    # Fetch full user profile from database including learning preferences
    try:
        import psycopg2
        from psycopg2.extras import RealDictCursor
        
        conn = psycopg2.connect(
            dsn=os.getenv('DATABASE_URL'),
            sslmode='require'
        )
        cursor = conn.cursor(cursor_factory=RealDictCursor)
        
        cursor.execute('''
            SELECT id, name, email, "educationLevel", "programmingExperience", 
                   "roboticsBackground", "softwareBackground", "hardwareBackground"
            FROM "user" WHERE id = %s
        ''', (user_id,))
        
        user_row = cursor.fetchone()
        cursor.close()
        conn.close()
        
        if user_row:
            return dict(user_row)
        else:
            return {"id": user_id, "name": f"User {user_id}"}
    except Exception as e:
        print(f"Error fetching user profile: {e}")
        return {"id": user_id, "name": f"User {user_id}"}

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
    # Parse request to extract selected text if present
    request_body = await request.body()

    # Try to parse the body to extract selected_text
    import json
    try:
        request_json = json.loads(request_body)
        selected_text = request_json.get('selected_text', None)
    except json.JSONDecodeError:
        selected_text = None

    # Add user context and selected text to the server call
    context = {"user": current_user}
    if selected_text:
        context['selected_text'] = selected_text

    result = await server.process(request_body, context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

# Add ChatKit endpoint - extracts user from query parameter
@app.post("/api/chatkit")
async def chatkit_endpoint_no_auth(request: Request):
    # Parse request to extract selected text if present
    request_body = await request.body()

    # Try to parse the body to extract selected_text
    import json
    try:
        request_json = json.loads(request_body)
        selected_text = request_json.get('selected_text', None)
    except json.JSONDecodeError:
        selected_text = None

    # Get user ID from query parameter (passed from frontend localStorage)
    user_id = request.query_params.get('userId')
    user_name = "User"
    
    
    
    if user_id:
        # Validate user exists in database and fetch full profile
        try:
            import psycopg2
            from psycopg2.extras import RealDictCursor
            
            conn = psycopg2.connect(
                dsn=os.getenv('DATABASE_URL'),
                sslmode='require'
            )
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            # Fetch full user profile including learning preferences
            cursor.execute('''
                SELECT id, name, email, "educationLevel", "programmingExperience", 
                       "roboticsBackground", "softwareBackground", "hardwareBackground"
                FROM "user" WHERE id = %s
            ''', (user_id,))
            user_row = cursor.fetchone()
            
            if user_row:
                user_name = user_row.get('name') or user_row.get('email', 'User')
                # Store full user profile
                user_profile = dict(user_row)
            else:
                user_id = None
                user_profile = None
            
            cursor.close()
            conn.close()
        except Exception as e:
            print(f"Error validating user: {e}")
            user_id = None
            user_profile = None
    
    # If no valid user, return error
    if not user_id:
        raise HTTPException(
            status_code=401, 
            detail="Authentication required. Please log in to use the chat."
        )
    
    # Build context with full user profile
    context = {"user": user_profile if user_profile else {"id": user_id, "name": user_name}}
    if selected_text:
        context['selected_text'] = selected_text

    result = await server.process(request_body, context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")

# Initialize store and server
store = NeonStore()
server = ChatKitServerImpl(store)


# Debug endpoint to inspect stored items
@app.get("/debug/threads")
async def debug_threads(current_user: dict = Depends(get_current_user)):
    # For NeonStore, we'll load threads for the current user
    result = {}

    # Load threads for the current user
    page = await store.load_threads(limit=100, after=None, order="desc", context={"user": current_user})

    for thread in page.data:
        # Load items for each thread
        items_page = await store.load_thread_items(
            thread_id=thread.id,
            after=None,
            limit=100,
            order="asc",
            context={"user": current_user}
        )

        items = []
        for item in items_page.data:
            item_data = {"id": item.id, "type": type(item).__name__}
            if hasattr(item, 'content') and item.content:
                content_parts = []
                for part in item.content:
                    if hasattr(part, 'text'):
                        content_parts.append(str(part))
                item_data["content"] = content_parts
            items.append(item_data)

        result[thread.id] = {"items": items, "count": len(items)}

    return result

# Pydantic models for request/response
class ChatRequest(BaseModel):
    user_query: str
    selected_text: Optional[str] = None
    chat_history: List[Dict[str, str]] = []
    user_profile: Optional[Dict[str, Any]] = None

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

class ChatResponseWithSelectedText(BaseModel):
    output: str
    context_chunks: List[ContextChunk]
    sources: List[Source]
    used_selected_text: bool

@app.post("/api/chat", response_model=ChatResponseWithSelectedText)
async def chat_endpoint(request: ChatRequest, current_user: dict = Depends(get_current_user)):
    """
    RAG chat endpoint - Process user query with RAG and return contextual response
    Now enhanced to handle selected text as additional context.
    """
    try:
        # Build the query based on whether selected text is provided
        if request.selected_text:
            # Combine user query with selected text for better context
            enhanced_query = f"{request.user_query}\n\nContext from selected text: {request.selected_text}"
            search_query = enhanced_query
        else:
            search_query = request.user_query

        # Search for relevant chunks in the vector database
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        limit = int(os.getenv("SEARCH_LIMIT", 5))
        score_threshold = float(os.getenv("SCORE_THRESHOLD", 0.7))

        search_results = search_chunks(
            collection_name=collection_name,
            query=search_query,
            limit=limit,
            score_threshold=score_threshold
        )

        # Prepare context string for the agent
        if search_results:
            context_str = "\n\n".join([chunk["text"] for chunk in search_results if chunk.get("text")])
        else:
            context_str = "No relevant information found in the book content."

        # Build system prompt with selected text context if available and user profile if available
        # Extract user profile from the authenticated user
        user_profile_context = ""
        if current_user:
            user_profile_context = f"""
            The user has the following background:
            - Education level: {current_user.get('educationLevel', current_user.get('education_level', 'Not specified'))}
            - Programming experience: {current_user.get('programmingExperience', current_user.get('programming_experience', 'Not specified'))}
            - Robotics background: {current_user.get('roboticsBackground', current_user.get('robotics_background', 'Not specified'))}

            Tailor your explanations to match their experience level.
            """

        if request.selected_text:
            system_prompt = f"""
            You are a helpful assistant helping users understand book content.
            {user_profile_context}

            The user has selected this specific text from the book:
            "{request.selected_text}"

            Use this as the primary context for your response. Also consider the following
            relevant sections from the book: {context_str}

            Answer the user's question clearly and specifically reference the selected text.
            """
        else:
            system_prompt = f"""
            You are a helpful assistant that answers questions based on book content provided in the context.
            {user_profile_context}
            Use the following context to answer the user's query: {context_str}

            If the context doesn't contain enough information, clearly state that you couldn't find relevant information.
            """

        # Create the agent with the enhanced prompt
        agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Run the agent with the user query
        result = await Runner.run(agent, request.user_query, run_config=config)
        ai_response = result.final_output

        # Save the chat message and response to the database
        try:
            import psycopg2
            from psycopg2.extras import RealDictCursor
            import os
            from dotenv import load_dotenv
            load_dotenv()

            conn = psycopg2.connect(
                dsn=os.getenv('DATABASE_URL'),
                sslmode='require'
            )

            cursor = conn.cursor()

            # Insert the chat message and response into the database
            insert_query = """
                INSERT INTO "chat_history" ("userId", "message", "response", "selectedText")
                VALUES (%s, %s, %s, %s)
            """

            # Get user ID from the authenticated user
            user_id = current_user.sub if hasattr(current_user, 'sub') else current_user.get('sub') if isinstance(current_user, dict) else None

            cursor.execute(insert_query, (user_id, request.user_query, ai_response, request.selected_text))
            conn.commit()

            cursor.close()
            conn.close()
        except Exception as e:
            # Log the error but don't fail the entire request
            print(f"Error saving chat history: {str(e)}")
            # Optionally, you could add error handling here

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

        return ChatResponseWithSelectedText(
            output=ai_response,
            context_chunks=context_chunks,
            sources=sources_list,
            used_selected_text=request.selected_text is not None and len(request.selected_text) > 0
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


# Add Pydantic models for chat history
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class ChatHistoryItem(BaseModel):
    id: int
    message: str
    response: str
    selectedText: Optional[str] = None
    createdAt: datetime

class ChatHistoryResponse(BaseModel):
    history: List[ChatHistoryItem]
    total: int


@app.get("/api/chat/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    request: Request,
    limit: int = Query(default=50, ge=1, le=100, description="Number of messages to return"),
    offset: int = Query(default=0, ge=0, description="Offset for pagination"),
    current_user: dict = Depends(get_current_user)
):
    """
    Get chat history for the authenticated user
    """
    try:
        # Get user ID from the authenticated user
        user_id = current_user.sub if hasattr(current_user, 'sub') else current_user.get('sub') if isinstance(current_user, dict) else None

        if not user_id:
            raise HTTPException(status_code=401, detail="User not authenticated properly")

        # Connect to database and fetch chat history
        import psycopg2
        from psycopg2.extras import RealDictCursor
        import os
        from dotenv import load_dotenv
        load_dotenv()

        conn = psycopg2.connect(
            dsn=os.getenv('DATABASE_URL'),
            sslmode='require'
        )

        cursor = conn.cursor(cursor_factory=RealDictCursor)

        # Query chat history for the user
        query = """
            SELECT id, message, response, "selectedText", "createdAt"
            FROM "chat_history"
            WHERE "userId" = %s
            ORDER BY "createdAt" DESC
            LIMIT %s OFFSET %s
        """

        cursor.execute(query, (user_id, limit, offset))
        results = cursor.fetchall()

        # Convert results to ChatHistoryItem objects
        history_items = []
        for row in results:
            history_items.append(ChatHistoryItem(
                id=row['id'],
                message=row['message'],
                response=row['response'],
                selectedText=row['selectedText'],
                createdAt=row['createdAt']
            ))

        # Get total count for pagination
        count_query = "SELECT COUNT(*) as total FROM \"chat_history\" WHERE \"userId\" = %s"
        cursor.execute(count_query, (user_id,))
        total = cursor.fetchone()['total']

        cursor.close()
        conn.close()

        return ChatHistoryResponse(
            history=history_items,
            total=total
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching chat history: {str(e)}")


# API endpoints for ChatKit thread management
@app.get("/api/chat/threads")
async def get_user_threads(
    limit: int = Query(default=20, ge=1, le=100, description="Number of threads to return"),
    after: str = Query(default=None, description="Cursor for pagination"),
    order: str = Query(default="desc", regex="^(asc|desc)$", description="Order of threads (by creation date)"),
    current_user: dict = Depends(get_current_user)
):
    """
    Retrieve list of chat threads for the authenticated user
    """
    try:
        # Load threads for the current user using NeonStore
        page = await store.load_threads(limit=limit, after=after, order=order, context={"user": current_user})

        # Format the response to match the API specification
        threads_data = []
        for thread in page.data:
            thread_data = {
                "id": thread.id,
                "userId": current_user.get('id'),
                "metadata": thread.metadata,
                "createdAt": thread.created_at.isoformat() if thread.created_at else None,
                "updatedAt": thread.created_at.isoformat() if thread.created_at else None  # Using created_at as updatedAt for simplicity
            }
            threads_data.append(thread_data)

        return {"threads": threads_data}

    except ValueError as e:
        # Handle store-specific errors (like connection errors)
        raise HTTPException(status_code=503, detail=f"Service temporarily unavailable: {str(e)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching user threads: {str(e)}")


@app.get("/api/chat/threads/{threadId}")
async def get_thread(threadId: str, current_user: dict = Depends(get_current_user)):
    """
    Retrieve a specific chat thread by ID for the authenticated user
    """
    try:
        # Load the specific thread for the current user
        thread = await store.load_thread(threadId, context={"user": current_user})

        # Format the response to match the API specification
        thread_data = {
            "id": thread.id,
            "userId": current_user.get('id'),
            "metadata": thread.metadata,
            "createdAt": thread.created_at.isoformat() if thread.created_at else None,
            "updatedAt": thread.created_at.isoformat() if thread.created_at else None  # Using created_at as updatedAt for simplicity
        }

        return thread_data

    except ValueError as e:
        # Thread not found
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching thread: {str(e)}")


@app.delete("/api/chat/threads/{threadId}")
async def delete_thread(threadId: str, current_user: dict = Depends(get_current_user)):
    """
    Delete a specific chat thread and all its messages
    """
    try:
        # Delete the thread (this will also delete associated messages due to CASCADE)
        await store.delete_thread(threadId, context={"user": current_user})

        return {"message": "Thread deleted successfully"}

    except ValueError as e:
        # Thread not found or not owned by user
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting thread: {str(e)}")


@app.get("/api/chat/threads/{threadId}/messages")
async def get_thread_messages(
    threadId: str,
    limit: int = Query(default=50, ge=1, le=100, description="Number of messages to return"),
    after: str = Query(default=None, description="Cursor for pagination"),
    order: str = Query(default="asc", regex="^(asc|desc)$", description="Order of messages (by creation date)"),
    current_user: dict = Depends(get_current_user)
):
    """
    Retrieve messages from a specific chat thread
    """
    try:
        # Load messages from the thread for the current user
        page = await store.load_thread_items(
            thread_id=threadId,
            after=after,
            limit=limit,
            order=order,
            context={"user": current_user}
        )

        # Format the response to match the API specification
        messages_data = []
        for item in page.data:
            message_data = {
                "id": item.id,
                "threadId": threadId,
                "type": type(item).__name__.replace('MessageItem', '').lower() or 'message',
                "content": getattr(item, 'content', {}),
                "createdAt": getattr(item, 'created_at', datetime.now(timezone.utc)).isoformat()
            }
            messages_data.append(message_data)

        return {"messages": messages_data}

    except ValueError as e:
        # Thread not found or not owned by user
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching thread messages: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)