import React, { forwardRef, useImperativeHandle, useState } from 'react';
import clsx from 'clsx';
import styles from './ChatBot.module.css';
import ChatBotSimple from './ChatBotSimple';
import ChatBotAuthenticated from './ChatBotAuthenticated';
import { useAuth } from '../../contexts/AuthContext';

export interface ChatBotHandle {
  openWithSelection: (text: string) => void;
}

interface ChatBotProps {
  initialSelectedText?: string;
}

const ChatBot = forwardRef<ChatBotHandle, ChatBotProps>(({ initialSelectedText }, ref) => {
  const [selectedText, setSelectedText] = useState<string | undefined>(initialSelectedText);
  const { user, loading } = useAuth();

  useImperativeHandle(ref, () => ({
    openWithSelection: (text: string) => {
      setSelectedText(text);
    }
  }));

  // If still loading auth status, show loading
  if (loading) {
    return <div className={styles.chatLoading}>Loading chat...</div>;
  }

  // If user is authenticated, show authenticated chat component
  if (user) {
    return <ChatBotAuthenticated initialSelectedText={selectedText} user={user} />;
  }

  // If user is not authenticated, show simple chat component
  return <ChatBotSimple initialSelectedText={selectedText} />;
});

ChatBot.displayName = 'ChatBot';

export default ChatBot;