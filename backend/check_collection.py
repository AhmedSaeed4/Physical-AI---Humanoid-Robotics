import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import from backend
current_dir = Path(__file__).parent
backend_dir = current_dir / "backend"
sys.path.insert(0, str(backend_dir))

from database import qdrant_client

# Check the book_content collection
collection_name = 'book_content'
try:
    collection_info = qdrant_client.get_collection(collection_name)
    print(f'Collection {collection_name} exists with {collection_info.points_count} points')

    # Get some sample points to see what's in the collection
    if collection_info.points_count > 0:
        # Get the first few points to see sample data
        from qdrant_client.http import models
        records = qdrant_client.scroll(
            collection_name=collection_name,
            limit=3,
            with_payload=True
        )

        print("\nSample records from the collection:")
        records_list = records[0]  # Extract the list of records
        for i, record in enumerate(records_list):
            payload = record.payload
            print(f"Record {i+1}:")
            print(f"  ID: {record.id}")
            print(f"  Filename: {payload.get('filename', 'N/A')}")
            print(f"  Chunk {payload.get('chunk_number', 'N/A')}/{payload.get('total_chunks', 'N/A')}")
            print(f"  Text snippet: {payload.get('text', '')[:100]}...")
            print()
except Exception as e:
    print(f'Collection {collection_name} does not exist or error occurred: {e}')
    import traceback
    traceback.print_exc()