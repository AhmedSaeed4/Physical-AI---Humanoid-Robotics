# Data Model: Qdrant Vector Database Integration

## Entities

### Book Content Chunk
- **Description**: A segment of book content that has been processed and embedded for vector storage
- **Fields**:
  - `id` (string): Unique identifier for the chunk in Qdrant
  - `text` (string): The actual content of the chunk (up to 1000 characters)
  - `filename` (string): Relative path to the source document
  - `chunk_number` (int): Position of this chunk within the document
  - `total_chunks` (int): Total number of chunks in the document
  - `embedding` (vector[768]): 768-dimensional vector embedding of the text content
  - `created_at` (timestamp): When the chunk was created
  - `updated_at` (timestamp): When the chunk was last updated

### Qdrant Collection
- **Description**: Vector database collection storing book content chunks with their embeddings and metadata
- **Fields**:
  - `name` (string): Name of the collection (book_content)
  - `vector_size` (int): Size of vectors (768 for Gemini embeddings)
  - `distance` (string): Distance metric (Cosine)
  - `chunks` (array): List of Book Content Chunk entities
  - `created_at` (timestamp): When the collection was created

### Query Response
- **Description**: The output from the RAG system containing the AI-generated answer and metadata
- **Fields**:
  - `output` (string): The AI-generated response to the user query
  - `context_chunks` (array): List of relevant content chunks used to generate the response
  - `sources` (array): List of source documents referenced in the response
  - `query_embedding` (vector[768]): Embedding of the original user query
  - `retrieval_time` (float): Time taken to retrieve relevant chunks
  - `generation_time` (float): Time taken to generate the response

### Search Result
- **Description**: Result from vector similarity search in Qdrant
- **Fields**:
  - `chunk_id` (string): ID of the matching chunk
  - `score` (float): Similarity score (0.0-1.0)
  - `text` (string): Content of the matching chunk
  - `metadata` (object): Additional metadata (filename, chunk_number, total_chunks)
  - `distance` (float): Distance value from the query

## Validation Rules

1. **Book Content Chunk**:
   - Text must be between 1 and 1000 characters
   - Chunk number must be between 1 and total_chunks
   - Embedding vector must be exactly 768 dimensions
   - Filename must be a valid relative path

2. **Query Response**:
   - Output must not be empty
   - Context chunks must be between 1 and 10
   - Sources must match the documents of context chunks

3. **Search Result**:
   - Score must be between 0.0 and 1.0
   - Distance must be non-negative