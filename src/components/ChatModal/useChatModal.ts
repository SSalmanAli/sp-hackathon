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

  const sendMessage = useCallback((content: string) => {
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

    // Simulate AI response after a short delay
    setIsLoading(true);
    setTimeout(() => {
      const aiMessage: Message = {
        id: `ai-${Date.now()}`,
        content: `I received your message: "${content}". This is a simulated response from the AI assistant about the Physical AI Book content. In a real implementation, this would connect to an actual AI service.`,
        sender: 'ai',
        timestamp: new Date(),
        status: 'delivered'
      };
      setMessages(prev => [...prev, aiMessage]);
      setIsLoading(false);
    }, 1000);
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