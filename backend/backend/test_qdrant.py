import os
import sys
from pathlib import Path
import uuid

# Add the current directory to the path so we can import directly
sys.path.insert(0, str(Path(__file__).parent))

from database import (
    qdrant_client,
    get_embedding,
    initialize_collection,
    upsert_chunks,
    search_chunks
)

def test_connection():
    """
    Test connection to Qdrant
    """
    try:
        # Test connection by getting collections
        collections = qdrant_client.get_collections()
        print("✓ Qdrant connection successful")
        print(f"Available collections: {[col.name for col in collections.collections]}")
        return True
    except Exception as e:
        print(f"✗ Qdrant connection failed: {e}")
        return False

def test_embedding():
    """
    Test embedding generation using Google Gemini
    """
    try:
        test_text = "This is a test sentence for embedding."
        embedding = get_embedding(test_text)

        if len(embedding) == 768:  # Check if it's 768-dimensional as required
            print("✓ Embedding generation successful")
            print(f"✓ Embedding has correct dimensions: {len(embedding)}")
            return True
        else:
            print(f"✗ Embedding has incorrect dimensions: {len(embedding)}")
            return False
    except Exception as e:
        print(f"✗ Embedding generation failed: {e}")
        return False

def test_collection_initialization():
    """
    Test Qdrant collection initialization
    """
    try:
        collection_name = "test_collection"
        initialize_collection(collection_name)
        print(f"✓ Collection '{collection_name}' initialized successfully")
        return True
    except Exception as e:
        print(f"✗ Collection initialization failed: {e}")
        return False

def test_upsert_and_search():
    """
    Test upserting chunks and searching
    """
    try:
        collection_name = "test_collection"

        # Test data
        test_chunks = [
            {
                "id": str(uuid.uuid4()),  # Generate a proper UUID string
                "text": "This is the first test chunk containing information about Python programming.",
                "filename": "test_doc.md",
                "chunk_number": 1,
                "total_chunks": 2
            },
            {
                "id": str(uuid.uuid4()),  # Generate a proper UUID string
                "text": "This is the second test chunk with more information about machine learning concepts.",
                "filename": "test_doc.md",
                "chunk_number": 2,
                "total_chunks": 2
            }
        ]

        # Upsert test chunks
        upsert_chunks(collection_name, test_chunks)
        print("✓ Chunks upserted successfully")

        # Search for relevant chunks
        results = search_chunks(collection_name, "Python programming", limit=5, score_threshold=0.0)
        print(f"✓ Search completed, found {len(results)} results")

        if len(results) > 0:
            print("✓ Search returned relevant results")
            for result in results:
                print(f"  - ID: {result['id']}, Score: {result['score']:.3f}")

        return True
    except Exception as e:
        print(f"✗ Upsert and search test failed: {e}")
        return False

def main():
    """
    Run all tests
    """
    print("Running Qdrant integration tests...\n")

    tests = [
        ("Connection Test", test_connection),
        ("Embedding Test", test_embedding),
        ("Collection Initialization Test", test_collection_initialization),
        ("Upsert and Search Test", test_upsert_and_search),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * len(test_name))
        if test_func():
            passed += 1
        print()

    print(f"Tests completed: {passed}/{total} passed")

    if passed == total:
        print("\n✓ All tests passed!")
        return True
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        return False

if __name__ == "__main__":
    # Check if required environment variables are set
    required_env_vars = ["QDRANT_URL", "QDRANT_API_KEY", "GEMINI_API_KEY"]
    missing_vars = [var for var in required_env_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Error: Missing required environment variables: {missing_vars}")
        print("Please set these variables in your .env file")
        sys.exit(1)

    main()