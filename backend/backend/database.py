import os
from typing import List, Optional
import numpy as np
import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=30
)

# Initialize Google Gemini API
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def get_embedding(text: str) -> List[float]:
    """
    Generate embedding for text using Google Gemini text-embedding-004 model
    Returns a 768-dimensional vector
    """
    try:
        # Using the text-embedding-004 model as specified in the requirements
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=[text],
            task_type="RETRIEVAL_DOCUMENT"
        )
        # The result is a dictionary with key 'embedding' containing a list of embeddings
        # The first embedding is result['embedding'][0] which is a list of 768 floats
        embedding = result['embedding'][0]  # Get the first embedding from the list
        return embedding
    except Exception as e:
        print(f"Error generating embedding: {e}")
        raise

def initialize_collection(collection_name: str = "book_content"):
    """
    Initialize Qdrant collection with cosine distance and 768 dimensions
    """
    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections().collections
        collection_names = [collection.name for collection in collections]

        if collection_name not in collection_names:
            # Create new collection with cosine distance and 768 dimensions
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # 768 dimensions as required by Gemini embeddings
                    distance=models.Distance.COSINE
                )
            )
            print(f"Collection '{collection_name}' created successfully")
        else:
            print(f"Collection '{collection_name}' already exists")
    except Exception as e:
        print(f"Error initializing collection: {e}")
        raise

def upsert_chunks(collection_name: str, chunks: List[dict]):
    """
    Upsert chunks to Qdrant collection
    Each chunk should have: id, text, filename, chunk_number, total_chunks
    """
    try:
        points = []
        for chunk in chunks:
            # Generate embedding for the text
            embedding = get_embedding(chunk['text'])

            # Create a point for Qdrant - ensure ID is a valid format (UUID string)
            point_id = chunk['id']
            # If the ID is not already a valid UUID format, we'll keep it as a string
            # Qdrant accepts string IDs as well, but they need to be in a valid format
            # Let's ensure it's a proper string ID that Qdrant can accept
            if not isinstance(point_id, str):
                point_id = str(point_id)

            # Create a point for Qdrant
            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": chunk['text'],
                    "filename": chunk['filename'],
                    "chunk_number": chunk['chunk_number'],
                    "total_chunks": chunk['total_chunks']
                }
            )
            points.append(point)

        # Upsert the points to the collection
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        print(f"Upserted {len(points)} chunks to collection '{collection_name}'")
    except Exception as e:
        print(f"Error upserting chunks: {e}")
        raise

def search_chunks(collection_name: str, query: str, limit: int = 5, score_threshold: float = 0.7) -> List[dict]:
    """
    Search for relevant chunks in Qdrant collection based on query
    Returns list of chunks with their metadata and scores
    """
    try:
        # Generate embedding for the query
        query_embedding = get_embedding(query)

        # Perform search in Qdrant - using query_points method (newer API)
        search_response = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=limit,
            score_threshold=score_threshold
        )

        # The response is a QueryResponse object with a 'points' attribute
        search_results = search_response.points

        # Format results
        results = []
        for hit in search_results:
            result = {
                "id": hit.id,
                "text": hit.payload["text"],
                "filename": hit.payload["filename"],
                "chunk_number": hit.payload["chunk_number"],
                "total_chunks": hit.payload["total_chunks"],
                "score": hit.score
            }
            results.append(result)

        return results
    except Exception as e:
        print(f"Error searching chunks: {e}")
        raise

# Initialize the default collection when module is loaded
if __name__ != "__main__":
    initialize_collection(os.getenv("QDRANT_COLLECTION_NAME", "book_content"))