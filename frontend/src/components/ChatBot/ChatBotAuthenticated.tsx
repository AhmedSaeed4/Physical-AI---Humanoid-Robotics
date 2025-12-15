import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ChatBot.module.css';

interface User {
  id: string;
  email: string;
  name: string;
  educationLevel: string;
  programmingExperience: string;
  roboticsBackground: string;
  softwareBackground?: string;
  hardwareBackground?: string;
}

interface ChatBotAuthenticatedProps {
  initialSelectedText?: string;
  user: User;
}

const ChatBotAuthenticated: React.FC<ChatBotAuthenticatedProps> = ({ initialSelectedText, user }) => {
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

  const { siteConfig } = useDocusaurusContext();
  const backendUrl = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';

  // Prepare personalized greeting based on user profile
  const getPersonalizedGreeting = () => {
    const experienceLevel = user.programmingExperience?.toLowerCase() || 'beginner';
    const educationLevel = user.educationLevel?.toLowerCase() || 'undergraduate';

    if (experienceLevel.includes('beginner') || experienceLevel.includes('no experience')) {
      return `Hello ${user.name}! I'm here to help you learn. Ask me about the book content and I'll explain concepts at a beginner-friendly level.`;
    } else if (experienceLevel.includes('advanced')) {
      return `Welcome back ${user.name}! Ready for some in-depth technical discussions? Ask me anything about the book content.`;
    } else {
      return `Hello ${user.name}! I'm here to help you understand the book content based on your background. Feel free to ask questions.`;
    }
  };

  const { control } = useChatKit({
    api: {
      url: `${backendUrl}/api/chatkit-auth`, // Use authenticated endpoint
      domainKey: 'localhost',
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('auth-token') || ''}`,
      }
    } as any, // Note: headers not supported in ChatKit types, auth handled by backend
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
      greeting: getPersonalizedGreeting(),
      prompts: [
        { label: 'What is this book about?', prompt: 'What is this book about?' },
        { label: 'Help me understand', prompt: `Help me understand a concept from the book as a ${user.educationLevel} level ${user.programmingExperience} with ${user.roboticsBackground} background` },
        ...(selectedText ? [{ label: `Explain: ${selectedText.substring(0, 30)}...`, prompt: `Explain: ${selectedText}` }] : []),
      ],
    },
    composer: {
      placeholder: selectedText
        ? `Ask about: "${selectedText.substring(0, 60)}${selectedText.length > 60 ? '...' : ''}"`
        : `Ask a question (personalized for ${user.educationLevel} level ${user.programmingExperience})...`,
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
        <div className={styles.userProfileInfo}>
          <h2>Personalized Q&A for {user.name}</h2>
          <div className={styles.profileTags}>
            <span className={styles.profileTag}>{user.educationLevel}</span>
            <span className={styles.profileTag}>{user.programmingExperience}</span>
            <span className={styles.profileTag}>{user.roboticsBackground}</span>
          </div>
        </div>
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

export default ChatBotAuthenticated;