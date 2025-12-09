import asyncio
import requests
import json
from pathlib import Path
import sys

# Add the backend directory to the path so we can import from backend
current_dir = Path(__file__).parent
backend_dir = current_dir / "backend"
sys.path.insert(0, str(backend_dir))

# Import the database module directly
import database
search_chunks = database.search_chunks
get_embedding = database.get_embedding

def test_vector_search():
    """Test direct vector search functionality"""
    print("Testing vector search functionality...")

    try:
        # Test search with a query related to ROS
        query = "ROS message types"
        collection_name = "book_content"

        results = search_chunks(
            collection_name=collection_name,
            query=query,
            limit=3,
            score_threshold=0.5
        )

        print(f"Search query: '{query}'")
        print(f"Found {len(results)} results:")

        for i, result in enumerate(results):
            print(f"  Result {i+1}:")
            print(f"    ID: {result['id']}")
            print(f"    Filename: {result['filename']}")
            print(f"    Score: {result['score']:.3f}")
            print(f"    Text snippet: {result['text'][:100]}...")
            print()

        return len(results) > 0
    except Exception as e:
        print(f"Vector search test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_rag_response():
    """Test the RAG response by directly using the same logic as the API"""
    print("Testing RAG response logic...")

    try:
        # Test query similar to what a user might ask
        user_query = "What are ROS message types?"
        collection_name = "book_content"

        # Search for relevant chunks (same as in main.py)
        search_results = search_chunks(
            collection_name=collection_name,
            query=user_query,
            limit=5,
            score_threshold=0.7
        )

        print(f"RAG query: '{user_query}'")
        print(f"Found {len(search_results)} relevant chunks:")

        if search_results:
            # Format the context from search results (same as in main.py)
            context_text = "\\n\\n".join([f"Source: {chunk['filename']}\\nContent: {chunk['text']}"
                                       for chunk in search_results])

            print("Context provided to AI:")
            for i, chunk in enumerate(search_results):
                print(f"  Source {i+1}: {chunk['filename']}")
                print(f"    Score: {chunk['score']:.3f}")
                print(f"    Text: {chunk['text'][:150]}...")
                print()

            # Check if the context contains relevant information
            context_lower = context_text.lower()
            has_ros_info = "msg" in context_lower or "message" in context_lower or "ros" in context_lower

            print(f"Context contains ROS-related information: {has_ros_info}")
            return has_ros_info
        else:
            print("No relevant chunks found in vector database")
            return False

    except Exception as e:
        print(f"RAG response test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_embedding_quality():
    """Test if embeddings are being generated properly"""
    print("Testing embedding generation...")

    try:
        test_text = "Robotics and artificial intelligence"
        embedding = get_embedding(test_text)

        print(f"Test text: '{test_text}'")
        print(f"Embedding length: {len(embedding)}")
        print(f"Sample values: {embedding[:5]}...")

        # Check if embedding has correct dimensions
        is_correct_length = len(embedding) == 768
        print(f"Embedding has correct length (768): {is_correct_length}")

        # Check if embedding contains reasonable values (not all zeros)
        has_variance = any(val != 0.0 for val in embedding)
        print(f"Embedding has variance: {has_variance}")

        return is_correct_length and has_variance
    except Exception as e:
        print(f"Embedding test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all verification tests"""
    print("="*60)
    print("RAG AI FUNCTIONALITY VERIFICATION")
    print("="*60)

    tests = [
        ("Embedding Quality", test_embedding_quality),
        ("Vector Search", test_vector_search),
        ("RAG Response Logic", test_rag_response)
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * len(test_name))
        if test_func():
            passed += 1
            print(f"✓ {test_name} PASSED")
        else:
            print(f"✗ {test_name} FAILED")

    print(f"\n{'='*60}")
    print(f"VERIFICATION SUMMARY: {passed}/{total} tests passed")
    print(f"{'='*60}")

    if passed == total:
        print("✓ All RAG functionality tests passed!")
        print("The system is correctly retrieving and using information from the vector database.")
    else:
        print(f"✗ {total - passed} test(s) failed")
        print("There may be issues with the RAG functionality.")

    return passed == total

if __name__ == "__main__":
    main()