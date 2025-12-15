import { useState } from 'react';

interface ChatHistoryItem {
  id: number;
  message: string;
  response: string;
  selectedText?: string;
  createdAt: string; // ISO date string
}

interface ChatHistoryResponse {
  history: ChatHistoryItem[];
  total: number;
}

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
  getChatHistory: (limit?: number, offset?: number) => Promise<ChatHistoryResponse>;
  getLocalChatHistory: () => ChatMessage[];
  clearHistory: () => void;
}

export const useChatService = (): ChatService => {
  const [localChatHistory, setLocalChatHistory] = useState<ChatMessage[]>([]);

  const sendMessage = async (message: string, selectedText?: string): Promise<ChatResponse> => {
    // Use production URL if available, fallback to localhost for development
    const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
    try {
      const response = await fetch(`${backendUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth-token') || ''}`
        },
        body: JSON.stringify({
          user_query: message,
          selected_text: selectedText || null,
          chat_history: localChatHistory,
          user_profile: null // Add user profile if available
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Update local chat history
      setLocalChatHistory(prev => [
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

  const getChatHistory = async (limit: number = 50, offset: number = 0): Promise<ChatHistoryResponse> => {
    const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
    try {
      const response = await fetch(`${backendUrl}/api/chat/history?limit=${limit}&offset=${offset}`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('auth-token') || ''}`
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatHistoryResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error fetching chat history:', error);
      throw error;
    }
  };

  const getLocalChatHistory = () => localChatHistory;

  const clearHistory = () => {
    setLocalChatHistory([]);
  };

  return {
    sendMessage,
    getChatHistory,
    getLocalChatHistory,
    clearHistory
  };
};