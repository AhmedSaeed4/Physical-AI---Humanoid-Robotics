import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ChatBot.module.css';

interface ContextChunk {
  id: string;
  text: string;
  filename: string;
  chunk_number: number;
  total_chunks: number;
  score: number;
}

interface Source {
  filename: string;
  url: string;
}

interface ChatResponse {
  output: string;
  context_chunks: ContextChunk[];
  sources: Source[];
}

const ChatBot: React.FC = () => {
  const [query, setQuery] = useState<string>('');
  const [response, setResponse] = useState<ChatResponse | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setError(null);

    try {
      const res = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_query: query,
          selected_text: '',
          chat_history: [],
        }),
      });

      if (!res.ok) {
        throw new Error(`API request failed with status ${res.status}`);
      }

      const data: ChatResponse = await res.json();
      setResponse(data);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An unknown error occurred');
      console.error('Chat request error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={clsx('container', styles.chatContainer)}>
      <h2>Book Content Q&A</h2>
      <form onSubmit={handleSubmit} className={styles.chatForm}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about the book content..."
          className={clsx('input', styles.chatInput)}
          disabled={loading}
        />
        <button type="submit" disabled={loading} className={clsx('button button--primary', styles.chatButton)}>
          {loading ? 'Asking...' : 'Ask'}
        </button>
      </form>

      {loading && (
        <div className={clsx('alert alert--info', styles.loadingState)}>
          Processing your query with RAG...
        </div>
      )}

      {error && (
        <div className={clsx('alert alert--error', styles.errorState)}>
          Error: {error}
        </div>
      )}

      {response && (
        <div className={clsx('margin-top--lg', styles.responseContainer)}>
          <div className={styles.responseOutput}>
            <h3>Response:</h3>
            <p>{response.output}</p>
          </div>

          {response.context_chunks.length > 0 && (
            <div className={clsx('margin-top--md', styles.contextChunks)}>
              <h3>Context Chunks Used: {response.context_chunks.length}</h3>
              <ul className="clean-list">
                {response.context_chunks.map((chunk, index) => (
                  <li key={chunk.id} className={clsx('card', styles.chunkItem)}>
                    <div>
                      <p><strong>Source:</strong> {chunk.filename} (Chunk {chunk.chunk_number}/{chunk.total_chunks})</p>
                      <p><strong>Relevance Score:</strong> {chunk.score.toFixed(3)}</p>
                      <p><strong>Content:</strong> {chunk.text}</p>
                    </div>
                  </li>
                ))}
              </ul>
            </div>
          )}

          {response.sources.length > 0 && (
            <div className={clsx('margin-top--md', styles.sources)}>
              <h3>Sources:</h3>
              <ul className="clean-list">
                {response.sources.map((source, index) => (
                  <li key={index}>
                    <a href={source.url} target="_blank" rel="noopener noreferrer">
                      {source.filename}
                    </a>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatBot;