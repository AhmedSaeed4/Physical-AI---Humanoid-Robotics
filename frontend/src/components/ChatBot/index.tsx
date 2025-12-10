import React, { forwardRef, useImperativeHandle, useState } from 'react';
import clsx from 'clsx';
import styles from './ChatBot.module.css';
import ChatBotSimple from './ChatBotSimple';

export interface ChatBotHandle {
  openWithSelection: (text: string) => void;
}

interface ChatBotProps {
  initialSelectedText?: string;
}

const ChatBot = forwardRef<ChatBotHandle, ChatBotProps>(({ initialSelectedText }, ref) => {
  const [selectedText, setSelectedText] = useState<string | undefined>(initialSelectedText);

  useImperativeHandle(ref, () => ({
    openWithSelection: (text: string) => {
      setSelectedText(text);
    }
  }));

  return <ChatBotSimple initialSelectedText={selectedText} />;
});

ChatBot.displayName = 'ChatBot';

export default ChatBot;