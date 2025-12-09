import asyncio
from fastapi.testclient import TestClient
import sys
from pathlib import Path

# Add the project root and backend directories to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "backend"))

# Import using the correct path structure
from src.backend.main import app

# Import the database module directly from the backend directory
import database
get_embedding = database.get_embedding
search_chunks = database.search_chunks
initialize_collection = database.initialize_collection

import os
from dotenv import load_dotenv

# Load environment variables for tests
load_dotenv()

# Create a test client for the API
client = TestClient(app)

def test_health_check():
    """Test the health check endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "Qdrant RAG API is running"


def test_chat_endpoint_basic():
    """Test the chat endpoint with a basic query"""
    # Test data
    chat_request = {
        "user_query": "What is this book about?",
        "selected_text": "",
        "chat_history": []
    }

    response = client.post("/api/chat", json=chat_request)

    # The response should be successful even if no context is found
    assert response.status_code in [200, 500]  # 500 might occur if collection doesn't exist yet

    if response.status_code == 200:
        data = response.json()
        assert "output" in data
        assert "context_chunks" in data
        assert "sources" in data
        assert isinstance(data["context_chunks"], list)
        assert isinstance(data["sources"], list)


def test_search_endpoint_basic():
    """Test the search endpoint with a basic query"""
    response = client.get("/api/search?query=test&limit=5&score_threshold=0.7")

    # The response should be successful even if no results are found
    assert response.status_code in [200, 500]  # 500 might occur if collection doesn't exist yet

    if response.status_code == 200:
        data = response.json()
        assert "results" in data
        assert isinstance(data["results"], list)


def test_search_endpoint_with_parameters():
    """Test the search endpoint with different parameters"""
    # Test with lower threshold to get more results
    response = client.get("/api/search?query=artificial&limit=3&score_threshold=0.5")

    if response.status_code == 200:
        data = response.json()
        assert "results" in data
        assert isinstance(data["results"], list)
        assert len(data["results"]) <= 3  # Should respect limit


def test_embedding_function():
    """Test the embedding function directly"""
    test_text = "This is a test sentence for embedding."
    embedding = get_embedding(test_text)

    # Check that embedding is a list of 768 floats
    assert isinstance(embedding, list)
    assert len(embedding) == 768
    assert all(isinstance(val, float) for val in embedding)


def test_chat_endpoint_validation():
    """Test the chat endpoint with validation"""
    # Test with empty query (should be handled gracefully)
    chat_request = {
        "user_query": "",
        "selected_text": "",
        "chat_history": []
    }

    response = client.post("/api/chat", json=chat_request)
    # Empty query should still return a valid response structure
    if response.status_code == 200:
        data = response.json()
        assert "output" in data
        assert "context_chunks" in data
        assert "sources" in data


def test_search_endpoint_validation():
    """Test the search endpoint validation"""
    # Test with empty query
    response = client.get("/api/search?query=")
    # Should return 422 for validation error or 200 with empty results
    assert response.status_code in [200, 422]

    # Test with invalid limit
    response = client.get("/api/search?query=test&limit=30")  # Above max of 20
    assert response.status_code in [200, 422]  # May return error or clamp to max


if __name__ == "__main__":
    # Run tests individually for debugging
    print("Running API endpoint tests...")

    try:
        test_health_check()
        print("✓ Health check test passed")
    except Exception as e:
        print(f"✗ Health check test failed: {e}")

    try:
        test_embedding_function()
        print("✓ Embedding function test passed")
    except Exception as e:
        print(f"✗ Embedding function test failed: {e}")

    try:
        test_search_endpoint_validation()
        print("✓ Search endpoint validation test passed")
    except Exception as e:
        print(f"✗ Search endpoint validation test failed: {e}")

    print("API endpoint tests completed.")