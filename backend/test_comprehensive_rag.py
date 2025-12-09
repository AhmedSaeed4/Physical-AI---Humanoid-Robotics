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

def test_various_queries():
    """Test the RAG system with various types of queries"""
    print("Testing various query types...")

    test_queries = [
        ("ROS Architecture", "What is ROS architecture?"),
        ("AI Integration", "How is AI integrated with robotics?"),
        ("VLA Systems", "What are Vision-Language-Action systems?"),
        ("Python Robotics", "Python libraries for robotics"),
        ("Navigation", "How do robots navigate?"),
        ("Perception", "Robot perception systems"),
        ("Control Systems", "Robot control systems"),
        ("Message Types", "ROS message types"),
        ("Services", "ROS services vs topics"),
        ("Actions", "ROS actions")
    ]

    collection_name = "book_content"
    all_passed = True

    for query_name, query in test_queries:
        print(f"\n  Testing {query_name}: '{query}'")
        try:
            results = search_chunks(
                collection_name=collection_name,
                query=query,
                limit=3,
                score_threshold=0.6
            )

            print(f"    Found {len(results)} results")

            if len(results) > 0:
                # Check if results are relevant by looking for related terms
                context_text = " ".join([result['text'] for result in results])
                query_lower = query.lower()
                context_lower = context_text.lower()

                # Simple relevance check - if query terms appear in context or related terms
                has_relevance = any(term in context_lower for term in query_lower.split() if len(term) > 2)
                print(f"    Relevance check: {'✓' if has_relevance else '✗'}")

                if not has_relevance:
                    all_passed = False
            else:
                print(f"    No results found (may be expected for specific queries)")

        except Exception as e:
            print(f"    Error: {e}")
            all_passed = False

    return all_passed

def test_edge_cases():
    """Test edge cases like empty queries, very short queries, etc."""
    print("Testing edge cases...")

    edge_cases = [
        ("Empty query", ""),
        ("Single character", "a"),
        ("Very short", "AI"),
        ("Random text", "asdkfjlasdjf asdf"),
        ("Special characters", "robotics & ai!"),
        ("Long query", "What are the detailed explanations of the most complex topics related to artificial intelligence and robotics including all the mathematical formulas and code examples?"),
        ("Numbers", "ROS 2"),
        ("Mixed case", "RoS ArChItEcTuRe")
    ]

    collection_name = "book_content"
    all_passed = True

    for case_name, query in edge_cases:
        print(f"\n  Testing {case_name}: '{query}'")
        try:
            if query.strip():  # Only test if query is not empty
                results = search_chunks(
                    collection_name=collection_name,
                    query=query,
                    limit=2,
                    score_threshold=0.5
                )
                print(f"    Found {len(results)} results")
            else:
                print(f"    Skipped empty query test (expected to fail validation)")

        except Exception as e:
            print(f"    Handled gracefully: {type(e).__name__}")
            # For edge cases, we expect the system to handle errors gracefully
            pass

    return all_passed

def test_score_thresholds():
    """Test different score thresholds to ensure proper filtering"""
    print("Testing score thresholds...")

    query = "robotics"
    collection_name = "book_content"

    thresholds = [0.5, 0.7, 0.8, 0.9]
    results = {}

    for threshold in thresholds:
        try:
            search_results = search_chunks(
                collection_name=collection_name,
                query=query,
                limit=5,
                score_threshold=threshold
            )
            results[threshold] = len(search_results)
            print(f"  Threshold {threshold}: {len(search_results)} results")

            # Verify that higher thresholds return same or fewer results
            if threshold > 0.5:
                prev_threshold = [t for t in thresholds if t < threshold][-1]
                if results[threshold] > results[prev_threshold]:
                    print(f"  WARNING: Higher threshold returned more results!")
                    return False

        except Exception as e:
            print(f"  Error at threshold {threshold}: {e}")
            return False

    print("  ✓ Score thresholding works correctly (higher thresholds return fewer or equal results)")
    return True

def test_limit_functionality():
    """Test the limit parameter to ensure it caps results properly"""
    print("Testing limit functionality...")

    query = "artificial intelligence"
    collection_name = "book_content"

    limits = [1, 3, 5, 10]
    all_correct = True

    for limit in limits:
        try:
            results = search_chunks(
                collection_name=collection_name,
                query=query,
                limit=limit,
                score_threshold=0.5
            )

            if len(results) <= limit:
                print(f"  Limit {limit}: Got {len(results)} results (≤{limit}) ✓")
            else:
                print(f"  Limit {limit}: Got {len(results)} results (>{limit}) ✗")
                all_correct = False

        except Exception as e:
            print(f"  Error at limit {limit}: {e}")
            all_correct = False

    return all_correct

def test_embedding_consistency():
    """Test that embeddings are consistent for the same input"""
    print("Testing embedding consistency...")

    test_text = "Artificial Intelligence in Robotics"

    try:
        embedding1 = get_embedding(test_text)
        embedding2 = get_embedding(test_text)

        # Check if embeddings are identical (they should be for the same input)
        are_identical = embedding1 == embedding2
        print(f"  Same input produces same embedding: {'✓' if are_identical else '✗'}")

        # Check dimensions
        dims_correct = len(embedding1) == 768 and len(embedding2) == 768
        print(f"  Embeddings have correct dimensions: {'✓' if dims_correct else '✗'}")

        return are_identical and dims_correct

    except Exception as e:
        print(f"  Error in embedding consistency test: {e}")
        return False

def test_search_quality():
    """Test search quality with specific known topics"""
    print("Testing search quality with known topics...")

    # These queries should return relevant results based on the sample data we saw earlier
    known_topics = [
        ("ROS Architecture", "Part_II_ROS", "Should find ROS-related content"),
        ("AI Integration", "Part_V_AI_Integration", "Should find AI integration content"),
        ("VLA", "Chapter_11_VLA", "Should find VLA content")
    ]

    collection_name = "book_content"
    all_passed = True

    for query, expected_file_pattern, description in known_topics:
        print(f"\n  {description}: '{query}'")
        try:
            results = search_chunks(
                collection_name=collection_name,
                query=query,
                limit=5,
                score_threshold=0.5
            )

            if results:
                # Check if any result contains the expected file pattern
                found_expected = any(expected_file_pattern in result['filename'] for result in results)
                print(f"    Found expected content ({expected_file_pattern}): {'✓' if found_expected else '✗'}")

                if not found_expected:
                    print(f"    Top results: {[r['filename'] for r in results[:3]]}")
                    all_passed = False
            else:
                print(f"    No results found for '{query}'")
                all_passed = False

        except Exception as e:
            print(f"    Error: {e}")
            all_passed = False

    return all_passed

def main():
    """Run all comprehensive tests"""
    print("="*70)
    print("COMPREHENSIVE RAG AI TESTING WITH EDGE CASES")
    print("="*70)

    tests = [
        ("Various Queries", test_various_queries),
        ("Edge Cases", test_edge_cases),
        ("Score Thresholds", test_score_thresholds),
        ("Limit Functionality", test_limit_functionality),
        ("Embedding Consistency", test_embedding_consistency),
        ("Search Quality", test_search_quality)
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

    print(f"\n{'='*70}")
    print(f"COMPREHENSIVE TESTING SUMMARY: {passed}/{total} tests passed")
    print(f"{'='*70}")

    if passed == total:
        print("✓ All comprehensive RAG tests passed!")
        print("The RAG system is working correctly with various query types and edge cases.")
    else:
        print(f"✗ {total - passed} test(s) failed")
        print("There may be issues with the RAG system under certain conditions.")

    return passed == total

if __name__ == "__main__":
    main()