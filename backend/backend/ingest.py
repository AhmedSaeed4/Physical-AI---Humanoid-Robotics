import os
import argparse
from pathlib import Path
from typing import List, Dict
import uuid
import sys

# Add current directory to import database module
sys.path.insert(0, str(Path(__file__).parent))

from database import upsert_chunks, initialize_collection


# -----------------------------------
# CHUNKING FUNCTION
# -----------------------------------
def chunk_text(text: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> List[Dict]:
    """
    Split text into overlapping chunks without blowing up RAM.
    """
    text_len = len(text)
    if text_len <= chunk_size:
        return [{"text": text, "start": 0, "end": text_len}]

    chunks = []
    start = 0

    while start < text_len:
        end = min(start + chunk_size, text_len)

        # Determine logical breakpoints
        if end < text_len:
            search_start = max(end - chunk_overlap, start)
            break_point = end

            # Priority 1: paragraph break
            for i in range(end, search_start, -1):
                if text[i:i+2] == "\n\n":
                    break_point = i + 2
                    break
            else:

                # Priority 2: sentence break
                for i in range(end, search_start, -1):
                    if text[i] in ".!?":
                        break_point = i + 1
                        break
                else:

                    # Priority 3: word boundary
                    for i in range(end, search_start, -1):
                        if text[i] == " ":
                            break_point = i
                            break
                    else:
                        break_point = end

        else:
            break_point = end

        # Append the chunk
        chunks.append({
            "text": text[start:break_point],
            "start": start,
            "end": break_point
        })

        # Move pointer forward
        new_start = break_point - chunk_overlap
        start = max(new_start, break_point)

        # Safety: prevent infinite loop
        if start <= chunks[-1]["start"]:
            start = chunks[-1]["end"]

    return chunks


# -----------------------------------
# READ MARKDOWN FILES
# -----------------------------------
def read_markdown_files(docs_path: str) -> List[Dict]:
    """
    Reads all .md and .mdx files in directory.
    """
    docs_path = Path(docs_path)
    if not docs_path.exists():
        raise FileNotFoundError(f"Docs path does not exist: {docs_path}")

    file_list = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))
    documents = []

    for file_path in file_list:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            relative_path = file_path.relative_to(docs_path)

            documents.append({
                "filename": str(relative_path),
                "content": content,
                "path": str(file_path)
            })

        except Exception as e:
            print(f"Error reading file {file_path}: {e}")

    return documents


# -----------------------------------
# PROCESS DOCUMENTS
# -----------------------------------
def process_documents(docs_path: str, chunk_size: int = 1000, chunk_overlap: int = 200):
    print(f"Reading documents from: {docs_path}")

    documents = read_markdown_files(docs_path)
    print(f"Found {len(documents)} documents")

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
    initialize_collection(collection_name)

    batch_size = 100
    batch_buffer = []

    total_chunks = 0

    for doc_index, doc in enumerate(documents):
        print(f"Processing {doc_index + 1}/{len(documents)} → {doc['filename']}")

        chunks = chunk_text(doc["content"], chunk_size, chunk_overlap)

        for chunk_idx, chunk in enumerate(chunks):
            chunk_record = {
                "id": str(uuid.uuid4()),
                "text": chunk["text"],
                "filename": doc["filename"],
                "chunk_number": chunk_idx + 1,
                "total_chunks": len(chunks)
            }

            batch_buffer.append(chunk_record)
            total_chunks += 1

            # Upsert in batches without storing everything in RAM
            if len(batch_buffer) >= batch_size:
                print(f"Upserting batch ({len(batch_buffer)} chunks)...")
                upsert_chunks(collection_name, batch_buffer)
                batch_buffer = []

    # Final leftover batch
    if batch_buffer:
        print(f"Upserting final batch ({len(batch_buffer)} chunks)...")
        upsert_chunks(collection_name, batch_buffer)

    print(f"✔ Completed ingestion: {total_chunks} chunks uploaded.")


# -----------------------------------
# MAIN
# -----------------------------------
def main():
    parser = argparse.ArgumentParser(description="Ingest documents into Qdrant vector DB")
    parser.add_argument("--docs_path", type=str, required=True)
    parser.add_argument("--chunk_size", type=int, default=1000)
    parser.add_argument("--chunk_overlap", type=int, default=200)

    args = parser.parse_args()
    process_documents(args.docs_path, args.chunk_size, args.chunk_overlap)


if __name__ == "__main__":
    main()
