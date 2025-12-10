/**
 * ChatKit Service for Docusaurus RAG Chatbot
 * Provides integration layer between ChatKit components and backend API
 */

import { createContext, useContext } from 'react';

// Define types for our service
export interface ChatKitServiceConfig {
  baseUrl: string;
  domainKey: string;
}

export interface ThreadData {
  id: string;
  createdAt: Date;
  metadata?: Record<string, any>;
}

export interface MessageData {
  id: string;
  threadId: string;
  role: 'user' | 'assistant';
  content: string;
  createdAt: Date;
  contextChunks?: any[];
  sources?: any[];
}

class ChatKitService {
  private config: ChatKitServiceConfig;
  private baseUrl: string;

  constructor(config?: Partial<ChatKitServiceConfig>) {
    this.config = {
      baseUrl: config?.baseUrl || 'http://localhost:8000',
      domainKey: config?.domainKey || 'localhost',
    };
    this.baseUrl = this.config.baseUrl;
  }

  /**
   * Initialize a new thread
   */
  async createThread(initialContext?: any): Promise<ThreadData> {
    // For ChatKit, threads are typically created automatically
    // This is a placeholder that returns a basic thread structure
    return {
      id: `thread_${Date.now()}`,
      createdAt: new Date(),
      metadata: initialContext || {}
    };
  }

  /**
   * Send a message to the backend via ChatKit endpoint
   */
  async sendMessage(threadId: string, message: string): Promise<MessageData> {
    try {
      // For ChatKit integration, we use the /api/chatkit endpoint
      const response = await fetch(`${this.baseUrl}/api/chatkit`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          event: 'create.message',
          data: {
            thread_id: threadId,
            content: [{
              type: 'text',
              text: message
            }]
          }
        })
      });

      if (!response.ok) {
        throw new Error(`Failed to send message: ${response.statusText}`);
      }

      // For now, return a basic message object
      // In a real implementation, this would process the ChatKit response
      return {
        id: `msg_${Date.now()}`,
        threadId,
        role: 'assistant',
        content: 'Response from assistant would appear here',
        createdAt: new Date()
      };
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  /**
   * Get messages for a specific thread
   */
  async getThreadMessages(threadId: string): Promise<MessageData[]> {
    // This would connect to the backend to retrieve thread messages
    // Implementation depends on how the backend stores and retrieves messages
    console.warn('getThreadMessages not fully implemented - this would connect to backend API');
    return [];
  }

  /**
   * Get available threads for the user
   */
  async getThreads(): Promise<ThreadData[]> {
    // This would connect to the backend to retrieve user's threads
    // Implementation depends on how the backend stores and retrieves threads
    console.warn('getThreads not fully implemented - this would connect to backend API');
    return [];
  }

  /**
   * Save a thread to backend
   */
  async saveThread(thread: ThreadData): Promise<void> {
    // Implementation to save thread to backend
    console.warn('saveThread not fully implemented - this would connect to backend API');
  }
}

// Create a singleton instance of the service
const chatKitService = new ChatKitService();

// Create React context for the service
const ChatKitServiceContext = createContext<ChatKitService>(chatKitService);

// Custom hook to use the service
export const useChatKitService = () => {
  return useContext(ChatKitServiceContext);
};

// Export the service instance directly as well
export default chatKitService;