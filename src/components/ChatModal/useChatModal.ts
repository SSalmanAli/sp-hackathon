// src/components/ChatModal/useChatModal.ts
// Custom hook for managing chat modal state

import { useState, useCallback, useEffect } from 'react';
import { Message, ChatSession } from './types';

export const useChatModal = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Initialize with a welcome message from AI
  useEffect(() => {
    if (messages.length === 0 && isOpen) {
      const welcomeMessage: Message = {
        id: 'welcome-1',
        content: 'Hello! I\'m your AI assistant for the Physical AI Book. How can I help you with the content today?',
        sender: 'ai',
        timestamp: new Date(),
        status: 'delivered'
      };
      setMessages([welcomeMessage]);
    }
  }, [isOpen, messages.length]);

  const toggleModal = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const openModal = useCallback(() => {
    setIsOpen(true);
  }, []);

  const closeModal = useCallback(() => {
    setIsOpen(false);
  }, []);

  const sendMessage = useCallback(async (content: string) => {
    if (!content.trim()) return;

    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      content: content.trim(),
      sender: 'user',
      timestamp: new Date(),
      status: 'sent'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Call the backend API
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/v1/chat/', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: content,
          timestamp: new Date().toISOString()
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      const aiMessage: Message = {
        id: `ai-${Date.now()}`,
        content: data.response_text || "I'm sorry, I couldn't generate a response.",
        sender: 'ai',
        timestamp: new Date(),
        status: 'delivered'
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: Message = {
        id: `err-${Date.now()}`,
        content: "I'm sorry, I'm having trouble connecting to the server. Please ensure the backend is running.",
        sender: 'ai',
        timestamp: new Date(),
        status: 'error'
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, []);

  const handleInputChange = useCallback((value: string) => {
    setInputValue(value);
  }, []);

  return {
    isOpen,
    messages,
    inputValue,
    isLoading,
    toggleModal,
    openModal,
    closeModal,
    sendMessage,
    handleInputChange
  };
};