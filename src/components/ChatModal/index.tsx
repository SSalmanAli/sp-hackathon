// src/components/ChatModal/index.tsx
// Main entry point for the chat modal components

import React, { useRef, useEffect } from 'react';
import ChatModal from './ChatModal';
import ChatButton from './ChatButton';
import { useChatModal } from './useChatModal';

const ChatModalContainer: React.FC = () => {
  const {
    isOpen,
    messages,
    inputValue,
    isLoading,
    toggleModal,
    openModal,
    closeModal,
    sendMessage,
    handleInputChange
  } = useChatModal();

  const triggerButtonRef = useRef<HTMLButtonElement>(null);

  // Track the element that opened the modal to return focus when it closes
  useEffect(() => {
    if (isOpen && triggerButtonRef.current) {
      // Store reference to the trigger element
    } else if (!isOpen && triggerButtonRef.current) {
      // Return focus to the trigger element when modal closes
      setTimeout(() => {
        triggerButtonRef.current?.focus();
      }, 0);
    }
  }, [isOpen]);

  return (
    <>
      <ChatModal
        isOpen={isOpen}
        messages={messages}
        inputValue={inputValue}
        isLoading={isLoading}
        onClose={closeModal}
        onInputChange={handleInputChange}
        onSendMessage={sendMessage}
        onClickOutside={closeModal}
      />
      <ChatButton
        ref={triggerButtonRef}
        onClick={toggleModal}
        hasUnread={messages.some(msg => msg.sender === 'ai' && !msg.status.includes('delivered'))}
        isVisible={true}
      />
    </>
  );
};

export default ChatModalContainer;
export { ChatModal, ChatButton, useChatModal };