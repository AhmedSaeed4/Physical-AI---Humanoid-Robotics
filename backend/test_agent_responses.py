#!/usr/bin/env python3
"""
Test script to run the Qdrant RAG API and demonstrate agent responses
"""
import asyncio
import json
import requests
import time
from threading import Thread
from pathlib import Path
import sys
import os

# Add the project root and necessary directories to the path
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / "src"))

from uvicorn import Config, Server
from src.backend.main import app

class APITester:
    def __init__(self):
        self.base_url = "http://localhost:8000"

    def start_server(self):
        """Start the FastAPI server in a separate thread"""
        config = Config(app=app, host="127.0.0.1", port=8000, log_level="info")
        self.server = Server(config=config)

        # Run the server in a separate thread
        self.server_thread = Thread(target=self.server.run, daemon=True)
        self.server_thread.start()

        # Wait a bit for the server to start
        time.sleep(3)
        print("API server started at http://localhost:8000")

    def test_chat_endpoint(self, query):
        """Test the chat endpoint with a query"""
        try:
            response = requests.post(
                f"{self.base_url}/api/chat",
                json={
                    "user_query": query,
                    "selected_text": "",
                    "chat_history": []
                },
                timeout=30
            )
            return response.json()
        except Exception as e:
            print(f"Error calling chat endpoint: {e}")
            return None

    def test_search_endpoint(self, query):
        """Test the search endpoint with a query"""
        try:
            response = requests.get(
                f"{self.base_url}/api/search",
                params={
                    "query": query,
                    "limit": 3,
                    "score_threshold": 0.5
                },
                timeout=30
            )
            return response.json()
        except Exception as e:
            print(f"Error calling search endpoint: {e}")
            return None

def main():
    print("Starting API server...")
    tester = APITester()
    tester.start_server()

    print("\n" + "="*70)
    print("TESTING RAG AGENT RESPONSES")
    print("="*70)

    # Test 1: Query related to the ebook content
    print("\n1. Testing with ebook-related query:")
    print("Query: 'What is ROS architecture?'")
    result = tester.test_chat_endpoint("What is ROS architecture?")

    if result:
        print(f"Response: {result.get('output', 'No response')[:500]}...")
        print(f"Context chunks found: {len(result.get('context_chunks', []))}")
        print(f"Sources: {[source.get('filename') for source in result.get('sources', [])[:3]]}")
    else:
        print("No response received")

    # Test 2: Another ebook-related query
    print("\n2. Testing with another ebook-related query:")
    print("Query: 'Explain VLA systems in robotics'")
    result = tester.test_chat_endpoint("Explain VLA systems in robotics")

    if result:
        print(f"Response: {result.get('output', 'No response')[:500]}...")
        print(f"Context chunks found: {len(result.get('context_chunks', []))}")
        print(f"Sources: {[source.get('filename') for source in result.get('sources', [])[:3]]}")
    else:
        print("No response received")

    # Test 3: Query not related to ebook content
    print("\n3. Testing with non-ebook related query:")
    print("Query: 'What is the weather today?'")
    result = tester.test_chat_endpoint("What is the weather today?")

    if result:
        print(f"Response: {result.get('output', 'No response')[:500]}...")
        print(f"Context chunks found: {len(result.get('context_chunks', []))}")
        print(f"Sources: {[source.get('filename') for source in result.get('sources', [])[:3]]}")
    else:
        print("No response received")

    # Test 4: Another non-ebook query
    print("\n4. Testing with another non-ebook related query:")
    print("Query: 'Who won the World Cup in 2022?'")
    result = tester.test_chat_endpoint("Who won the World Cup in 2022?")

    if result:
        print(f"Response: {result.get('output', 'No response')[:500]}...")
        print(f"Context chunks found: {len(result.get('context_chunks', []))}")
        print(f"Sources: {[source.get('filename') for source in result.get('sources', [])[:3]]}")
    else:
        print("No response received")

    # Test 5: Search endpoint for comparison
    print("\n5. Testing search endpoint with non-ebook query:")
    print("Search: 'weather'")
    search_result = tester.test_search_endpoint("weather")

    if search_result:
        print(f"Search results: {len(search_result.get('results', []))} found")
        for i, result in enumerate(search_result.get('results', [])[:2]):
            print(f"  Result {i+1}: {result.get('filename', 'N/A')[:50]}...")
    else:
        print("No search results received")

    print("\n" + "="*70)
    print("TESTING COMPLETE")
    print("="*70)

    # Note: In a real scenario, you would properly shut down the server
    # For this test, we'll just let it run until the script ends

if __name__ == "__main__":
    main()