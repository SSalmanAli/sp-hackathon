// src/components/ChatModal/ChatModal.tsx
// Main chat modal component with message history and input field

import React, { useRef, useEffect, useCallback } from 'react';
import clsx from 'clsx';
import styles from './ChatModal.module.css';
import { Message } from './types';

interface ChatModalProps {
  isOpen: boolean;
  messages: Message[];
  inputValue: string;
  isLoading: boolean;
  onClose: () => void;
  onInputChange: (value: string) => void;
  onSendMessage: (content: string) => void;
  onClickOutside?: () => void;
}

const ChatModal: React.FC<ChatModalProps> = ({
  isOpen,
  messages,
  inputValue,
  isLoading,
  onClose,
  onInputChange,
  onSendMessage,
  onClickOutside
}) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const modalRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle keyboard events
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  // Focus input when modal opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen]);

  // Manage focus when modal closes
  useEffect(() => {
    if (!isOpen && modalRef.current) {
      // When modal closes, focus should return to the trigger button
      // For now, we'll just ensure document focus is properly managed
      const previouslyFocusedElement = document.activeElement as HTMLElement;
      return () => {
        // In a real implementation, we would return focus to the element that opened the modal
        // For now, we ensure focus management is considered
      };
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSend = () => {
    if (inputValue.trim()) {
      onSendMessage(inputValue);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget && onClickOutside) {
      onClickOutside();
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={`${styles.backdrop} ${isOpen ? '' : styles.hidden}`}
      onClick={handleBackdropClick}
    >
      <div
        ref={modalRef}
        className={`${styles.chatModal} ${isOpen ? styles.open : ''}`}
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-label="AI Assistant Chat"
      >
        <div className={styles.chatModalHeader}>
          <h3 className={styles.chatModalTitle}>AI Assistant</h3>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
            tabIndex={0}
          >
            ×
          </button>
        </div>

        <div className={styles.chatModalBody}>
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  message.sender === 'user' ? styles.userMessage : styles.aiMessage
                )}
                role="log"
                aria-live="polite"
              >
                {message.content}
              </div>
            ))}
            {isLoading && (
              <div className={clsx(styles.message, styles.aiMessage)}>
                <div className={styles.loadingIndicator}>AI is typing...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
        </div>

        <div className={styles.chatModalFooter}>
          <input
            ref={inputRef}
            type="text"
            className={styles.inputField}
            value={inputValue}
            onChange={(e) => onInputChange(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your question about the book..."
            aria-label="Type your message"
            role="textbox"
            tabIndex={0}
          />
          <button
            className={styles.sendButton}
            onClick={handleSend}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
            tabIndex={0}
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M2 21L23 12L2 3V10L17 12L2 14V21Z" fill="white"/>
            </svg>
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatModal;