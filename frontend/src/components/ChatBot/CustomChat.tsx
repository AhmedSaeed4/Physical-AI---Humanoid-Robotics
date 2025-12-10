import React, { useState, useEffect, useRef } from 'react';
import { useChatService } from '@site/src/services/chatService';
import clsx from 'clsx';
import styles from './ChatBot.module.css';

interface CustomChatProps {
  initialSelectedText?: string;
}

const CustomChat: React.FC<CustomChatProps> = ({ initialSelectedText }) => {
  const [inputMessage, setInputMessage] = useState('');
  const [messages, setMessages] = useState<{ role: string; content: string }[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | undefined>(initialSelectedText);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const chatService = useChatService();

  useEffect(() => {
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

  useEffect(() => {
    // Scroll to bottom of messages when messages change
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = inputMessage.trim();
    setInputMessage('');

    // Add user message to UI immediately
    const newMessages = [...messages, { role: 'user', content: userMessage }];
    setMessages(newMessages);

    setIsLoading(true);
    try {
      // Send message with selected text to backend
      const response = await chatService.sendMessage(userMessage, selectedText);

      // Add AI response to messages
      setMessages(prev => [...prev, { role: 'assistant', content: response.output }]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request.'
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(e as any);
    }
  };

  return (
    <div className={styles.customChatContainer}>
      {/* Selected text context display */}
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

      {/* Messages container */}
      <div className={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div className={styles.emptyState}>
            <p>Ask a question about the book content...</p>
          </div>
        ) : (
          messages.map((msg, index) => (
            <div
              key={index}
              className={clsx(
                styles.message,
                msg.role === 'user' ? styles.userMessage : styles.assistantMessage
              )}
            >
              <div className={styles.messageContent}>{msg.content}</div>
            </div>
          ))
        )}
        {isLoading && (
          <div className={clsx(styles.message, styles.assistantMessage)}>
            <div className={styles.messageContent}>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input area */}
      <form onSubmit={handleSendMessage} className={styles.inputForm}>
        <textarea
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={selectedText
            ? `Ask about: "${selectedText.substring(0, 60)}${selectedText.length > 60 ? '...' : ''}"`
            : 'Ask a question about the book content...'}
          className={styles.messageInput}
          rows={1}
          disabled={isLoading}
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={!inputMessage.trim() || isLoading}
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default CustomChat;