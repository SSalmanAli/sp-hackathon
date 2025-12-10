// src/components/ChatModal/types.ts
// TypeScript interfaces for the chat modal component

export interface Message {
  id: string;
  content: string;
  sender: 'user' | 'ai';
  timestamp: Date;
  status: 'sent' | 'delivered' | 'error';
}

export interface ChatSession {
  id: string;
  messages: Message[];
  isActive: boolean;
  createdAt: Date;
  lastMessageAt: Date;
}

export interface ChatButtonState {
  isVisible: boolean;
  isMinimized: boolean;
  hasUnread: boolean;
}