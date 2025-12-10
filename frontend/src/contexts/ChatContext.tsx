import React, { createContext, useContext, ReactNode } from 'react';

interface ChatContextType {
  openChatWithSelection: (text: string) => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const openChatWithSelection = (text: string) => {
    // Dispatch a custom event that can be listened to by the ChatBot component
    window.dispatchEvent(new CustomEvent('openChatWithSelection', {
      detail: { text }
    }));
  };

  return (
    <ChatContext.Provider value={{ openChatWithSelection }}>
      {children}
    </ChatContext.Provider>
  );
};

export const useChatContext = () => {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
};