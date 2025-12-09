import os
from typing import List, Dict, Any
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
from agents import Agent, Runner, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI
import sys
from pathlib import Path
import os

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

client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Define the Gemini model
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

# Configure the run settings
config = RunConfig(
    model=model,
    model_provider=client,
)

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
async def chat_endpoint(request: ChatRequest):
    """
    RAG chat endpoint - Process user query with RAG and return contextual response
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
    score_threshold: float = Query(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score threshold")
):
    """
    Direct vector search endpoint - Perform direct vector similarity search on book content
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