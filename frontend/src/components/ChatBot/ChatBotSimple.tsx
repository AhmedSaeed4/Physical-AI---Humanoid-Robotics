import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import styles from './ChatBot.module.css';

interface ChatBotSimpleProps {
  initialSelectedText?: string;
}

// Simple ChatBot component without authentication
const ChatBotSimple: React.FC<ChatBotSimpleProps> = ({ initialSelectedText }) => {
  const [showThreadHistory, setShowThreadHistory] = useState(false);
  const [initialThread, setInitialThread] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | undefined>(initialSelectedText);

  useEffect(() => {
    const savedThread = localStorage.getItem('chatkit-thread-id');
    setInitialThread(savedThread || null);

    // Listen for the custom event to update selected text
    const handleOpenChatWithSelection = (event: CustomEvent) => {
      setSelectedText(event.detail.text);
    };

    // Add event listener using addEventListener
    const eventListener = (event: Event) => handleOpenChatWithSelection(event as CustomEvent);
    window.addEventListener('openChatWithSelection', eventListener);

    // Cleanup function to remove event listener
    return () => {
      window.removeEventListener('openChatWithSelection', eventListener);
    };
  }, []);

  const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
  const { control } = useChatKit({
    api: {
      url: `${backendUrl}/api/chatkit`,
      domainKey: 'localhost',
    },
    initialThread: initialThread,
    theme: {
      colorScheme: 'light', // Using light theme to match Docusaurus styling
      color: {
        grayscale: { hue: 220, tint: 6, shade: -1 },
        accent: { primary: '#3578e5', level: 1 }, // Using Docusaurus primary color
      },
      radius: 'round',
    },
    startScreen: {
      greeting: 'Ask questions about the book content!',
      prompts: [
        { label: 'What is this book about?', prompt: 'What is this book about?' },
        { label: 'Help me understand', prompt: 'Help me understand a concept from the book' },
        ...(selectedText ? [{ label: `Explain: ${selectedText.substring(0, 30)}...`, prompt: `Explain: ${selectedText}` }] : []),
      ],
    },
    composer: {
      placeholder: selectedText
        ? `Ask about: "${selectedText.substring(0, 60)}${selectedText.length > 60 ? '...' : ''}"`
        : 'Ask a question about the book content...',
    },
    onThreadChange: ({ threadId }) => {
      if (threadId) {
        localStorage.setItem('chatkit-thread-id', threadId);

        // Track this thread in recent threads
        const recentThreads = JSON.parse(localStorage.getItem('chatkit-recent-threads') || '[]');
        if (!recentThreads.includes(threadId)) {
          recentThreads.unshift(threadId);
          // Keep only the 10 most recent threads
          if (recentThreads.length > 10) {
            recentThreads.splice(10);
          }
          localStorage.setItem('chatkit-recent-threads', JSON.stringify(recentThreads));
        }
      }
    },
    onError: ({ error }) => console.error('ChatKit error:', error),
  });

  // Method to open chat with selected text
  const openWithSelection = (text: string) => {
    setSelectedText(text);
    // If the chat is minimized, we might need to expand it here
    // For now, we're just storing the selected text context
  };

  // Function to get recent threads from localStorage or other source
  const getRecentThreads = () => {
    // In a real implementation, this would fetch from the backend
    // For now, we'll just use localStorage to track recent threads
    const recentThreadIds = JSON.parse(localStorage.getItem('chatkit-recent-threads') || '[]');
    return recentThreadIds.map((id: string) => ({
      id,
      title: `Thread ${id.substring(0, 8)}...`,
      lastActive: new Date().toLocaleTimeString()
    }));
  };

  // Function to switch to a different thread
  const switchToThread = (threadId: string) => {
    // Update the initial thread and force a re-render
    setInitialThread(threadId);
    localStorage.setItem('chatkit-thread-id', threadId);
    window.location.reload(); // This is a simple approach, but causes full page reload
  };

  // Function to start a new thread
  const startNewThread = () => {
    localStorage.removeItem('chatkit-thread-id');
    setInitialThread(null);
    window.location.reload(); // This is a simple approach, but causes full page reload
  };

  return (
    <div className={clsx('container', styles.chatContainer)}>
      <div className={styles.chatHeader}>
        <h2>Book Content Q&A</h2>
        <div className={styles.threadControls}>
          <button
            className={styles.newThreadButton}
            onClick={startNewThread}
            title="Start new conversation"
          >
            + New Chat
          </button>
          <button
            className={styles.historyToggleButton}
            onClick={() => setShowThreadHistory(!showThreadHistory)}
            title="Show conversation history"
          >
            📜 History
          </button>
        </div>
      </div>

      {/* Display selected text context if available */}
      {selectedText && (
        <div className={styles.selectedTextContext}>
          <div className={styles.selectedTextHeader}>
            <span>Selected from book:</span>
            <button
              className={styles.clearSelectedText}
              onClick={() => setSelectedText(undefined)}
              title="Clear selected text"
            >
              ×
            </button>
          </div>
          <div className={styles.selectedTextContent}>
            "{selectedText}"
          </div>
        </div>
      )}

      {showThreadHistory && (
        <div className={styles.threadHistoryPanel}>
          <h3>Recent Conversations</h3>
          <div className={styles.threadList}>
            {getRecentThreads().length > 0 ? (
              getRecentThreads().map((thread) => (
                <div
                  key={thread.id}
                  className={styles.threadItem}
                  onClick={() => switchToThread(thread.id)}
                >
                  <div className={styles.threadTitle}>{thread.title}</div>
                  <div className={styles.threadTime}>{thread.lastActive}</div>
                </div>
              ))
            ) : (
              <div className={styles.noThreads}>No recent conversations</div>
            )}
          </div>
        </div>
      )}

      <div className={styles['chatkit-container']}>
        <ChatKit control={control} className={styles['chatkit-container']} />
      </div>
    </div>
  );
};

export default ChatBotSimple;