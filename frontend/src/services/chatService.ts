import { useState } from 'react';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
}

interface ChatResponse {
  output: string;
  context_chunks: Array<{
    id: string;
    text: string;
    filename: string;
    chunk_number: number;
    total_chunks: number;
    score: number;
  }>;
  sources: Array<{
    filename: string;
    url: string;
  }>;
  used_selected_text: boolean;
}

export interface ChatService {
  sendMessage: (message: string, selectedText?: string) => Promise<ChatResponse>;
  getChatHistory: () => ChatMessage[];
  clearHistory: () => void;
}

export const useChatService = (): ChatService => {
  const [chatHistory, setChatHistory] = useState<ChatMessage[]>([]);

  const sendMessage = async (message: string, selectedText?: string): Promise<ChatResponse> => {
    // Use production URL if available, fallback to localhost for development
    // This URL is configured at build time via docusaurus.config.ts customFields
    const backendUrl = 'http://localhost:8000'; // Will be overridden in components using useDocusaurusContext
    try {
      const response = await fetch(`${backendUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth-token') || 'test-token'}` // Use test token for now
        },
        body: JSON.stringify({
          user_query: message,
          selected_text: selectedText || null,
          chat_history: chatHistory,
          user_profile: null // Add user profile if available
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Update chat history
      setChatHistory(prev => [
        ...prev,
        { role: 'user', content: message },
        { role: 'assistant', content: data.output }
      ]);

      return data;
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  };

  const getChatHistory = () => chatHistory;

  const clearHistory = () => {
    setChatHistory([]);
  };

  return {
    sendMessage,
    getChatHistory,
    clearHistory
  };
};