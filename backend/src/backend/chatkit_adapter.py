"""
ChatKit Adapter for connecting ChatKit with existing RAG pipeline
This adapter connects ChatKit's thread model with the existing Qdrant RAG pipeline
"""
import os
from typing import Dict, Any, List
from datetime import datetime, timezone

from chatkit.types import ThreadMetadata, ThreadItem, UserMessageItem, AssistantMessageItem
from agents import Agent, Runner, RunConfig
from agents.extensions.models.litellm_model import LitellmModel
from pydantic import BaseModel
from dotenv import load_dotenv
from database import search_chunks
import google.generativeai as genai

# Load environment variables
load_dotenv()


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


import time
from collections import defaultdict

class ChatKitRAGAdapter:
    """
    Adapter to connect ChatKit with existing RAG pipeline
    """

    def __init__(self, store, model=None, config=None):
        self.store = store
        self.model = model  # Pass the model from main.py
        self.config = config  # Pass the config from main.py
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        self.limit = int(os.getenv("SEARCH_LIMIT", 5))
        self.score_threshold = float(os.getenv("SCORE_THRESHOLD", 0.7))
        # For handling rapid query submission - track last request time per thread
        self.last_request_times = defaultdict(float)
        self.request_cooldown = 0.5  # 500ms cooldown between requests

    async def process_user_message(self, thread_id: str, user_message: str) -> ThreadItem:
        """
        Process a user message through the RAG pipeline and return an assistant response
        """
        # Handle rapid query submission - check if request is too frequent
        current_time = time.time()
        last_request_time = self.last_request_times[thread_id]
        if current_time - last_request_time < self.request_cooldown:
            # Return a response asking user to slow down
            ai_response = "Please slow down and wait a moment before sending another message."
            context_chunks = []
            sources_list = []

            # Create assistant message
            assistant_item = ThreadItem(
                id=self.store.generate_item_id("assistant", await self.store.load_thread(thread_id, {}), {}),
                thread_id=thread_id,
                type="message",
                role="assistant",
                content=[{"type": "output_text", "text": ai_response}],
                created_at=datetime.now(timezone.utc),
                metadata={
                    "context_chunks": context_chunks,
                    "sources": sources_list
                }
            )
            return assistant_item

        # Update last request time
        self.last_request_times[thread_id] = current_time

        # Handle empty query edge case
        if not user_message or user_message.strip() == "":
            ai_response = "Please enter a question or statement for me to help you with."
            context_chunks = []
            sources_list = []
        else:
            # Handle long query edge case - truncate if too long
            if len(user_message) > 1000:  # Limit query length to 1000 characters
                original_length = len(user_message)
                user_message = user_message[:1000] + "... [truncated for processing]"
                print(f"Query was too long ({original_length} chars) and has been truncated for processing")

            # Search for relevant chunks in the vector database with error handling
            try:
                search_results = search_chunks(
                    collection_name=self.collection_name,
                    query=user_message,
                    limit=self.limit,
                    score_threshold=self.score_threshold
                )
            except Exception as e:
                # Handle network failure or other errors in search
                print(f"Error in search_chunks: {e}")
                ai_response = "Sorry, I'm having trouble accessing the knowledge base right now. Please try again later."
                context_chunks = []
                sources_list = []
                search_results = []  # Set to empty to skip the main processing

        if not search_results and user_message and user_message.strip() != "":
            # If no relevant chunks found, return a response without context
            ai_response = "I couldn't find any relevant information in the book content to answer your question."
            context_chunks = []
            sources_list = []
        elif user_message and user_message.strip() != "":
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

            User's query: {user_message}
            """

            # Create the agent with the prompt
            agent = Agent(
                name="RAGBot",
                instructions=f"You are a helpful assistant that answers questions based on provided context. Always cite your sources from the context provided. Context: {context_text}",
                model=self.model
            )

            # Run the agent with the user query
            result = await Runner.run(agent, user_message, run_config=self.config)
            ai_response = result.final_output

            # Format context chunks for response
            context_chunks = [
                {
                    "id": chunk["id"],
                    "text": chunk["text"],
                    "filename": chunk["filename"],
                    "chunk_number": chunk["chunk_number"],
                    "total_chunks": chunk["total_chunks"],
                    "score": chunk["score"]
                }
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

            sources_list = [{"filename": src["filename"], "url": src["url"]} for src in unique_sources.values()]
        else:
            # Empty query case - values already set above
            pass
        # Append source information to the AI response to ensure it's visible in the chat
        if sources_list:
            sources_text = "\n\nSources:\n" + "\n".join([
                f"- {source['filename']}" for source in sources_list
            ])
            ai_response += sources_text

        # Create assistant message with context chunks and sources as metadata
        assistant_item = ThreadItem(
            id=self.store.generate_item_id("assistant", await self.store.load_thread(thread_id, {}), {}),
            thread_id=thread_id,
            type="message",
            role="assistant",
            content=[{"type": "output_text", "text": ai_response}],
            created_at=datetime.now(timezone.utc),
            metadata={
                "context_chunks": context_chunks,
                "sources": sources_list
            }
        )

        return assistant_item

    async def add_user_message_to_thread(self, thread_id: str, user_message: str) -> ThreadItem:
        """
        Add a user message to the thread
        """
        user_item = UserMessageItem(
            id=self.store.generate_item_id("user", await self.store.load_thread(thread_id, {}), {}),
            thread_id=thread_id,
            content=[{"type": "input_text", "text": user_message}],
            created_at=datetime.now(timezone.utc),
            inference_options={}  # Required field - empty dict uses defaults
        )

        await self.store.add_thread_item(thread_id, user_item, {})
        return user_item

    async def process_thread_message(self, thread_id: str, user_message: str) -> ThreadItem:
        """
        Process a complete thread message interaction: add user message and generate assistant response
        """
        # Add user message to thread
        await self.add_user_message_to_thread(thread_id, user_message)

        # Process through RAG and get assistant response
        assistant_response = await self.process_user_message(thread_id, user_message)

        # Add assistant response to thread
        await self.store.add_thread_item(thread_id, assistant_response, {})

        return assistant_response