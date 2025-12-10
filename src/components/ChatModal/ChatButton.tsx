// src/components/ChatModal/ChatButton.tsx
// Floating chat button component

import React, { forwardRef } from 'react';
import clsx from 'clsx';
import styles from './ChatButton.module.css';

interface ChatButtonProps {
  onClick: () => void;
  hasUnread?: boolean;
  isVisible?: boolean;
}

const ChatButton = forwardRef<HTMLButtonElement, ChatButtonProps>(({
  onClick,
  hasUnread = false,
  isVisible = true
}, ref) => {
  if (!isVisible) {
    return null;
  }

  return (
    <button
      ref={ref}
      className={clsx(
        styles.chatButton,
        hasUnread && styles.hasUnread
      )}
      onClick={onClick}
      aria-label="Open chat with AI assistant"
    >
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M21 11.5C21.0034 12.8199 20.6951 14.1219 20.1 15.3C19.3798 16.7425 18.3052 17.9762 16.974 18.9C16.9607 18.909 16.9473 18.918 16.9338 18.9269C16.5703 19.164 16.1956 19.3832 15.81 19.58C15.4209 19.7916 15.0209 19.9835 14.61 20.15C14.1988 20.3167 13.78 20.46 13.35 20.58C12.92 20.7 12.48 20.8 12 20.88C10.26 21.21 8.46 21.1 6.7 20.6C4.78 20.06 3.12 18.94 1.9 17.4C1.3135 16.558 0.905304 15.5989 0.703653 14.5822C0.502003 13.5655 0.511417 12.515 0.73135 11.5C0.919512 10.6291 1.28968 9.80434 1.81704 9.0817C2.3444 8.35906 3.01716 7.7575 3.79 7.315C4.57 6.87 5.44 6.58 6.33 6.45C7.22 6.32 8.12 6.35 9 6.55C9.88 6.75 10.72 7.11 11.47 7.61C12.22 8.11 12.86 8.74 13.36 9.47C13.86 10.2 14.2 11.02 14.36 11.88C14.52 12.74 14.5 13.62 14.3 14.47C14.1 15.32 13.73 16.12 13.22 16.82C12.71 17.52 12.07 18.11 11.34 18.55C10.61 18.99 9.81 19.27 9 19.37C8.19 19.47 7.39 19.39 6.64 19.14C5.89 18.89 5.21 18.48 4.65 17.94C4.09 17.4 3.67 16.75 3.42 16.04C3.17 15.33 3.1 14.57 3.22 13.82C3.34 13.07 3.64 12.36 4.09 11.74C4.54 11.12 5.13 10.61 5.82 10.25C6.51 9.89 7.27 9.7 8.05 9.69C8.83 9.68 9.6 9.85 10.3 10.18C11 10.51 11.62 11 12.12 11.61C12.62 12.22 12.99 12.93 13.2 13.7C13.41 14.47 13.45 15.28 13.32 16.07C13.19 16.86 12.89 17.61 12.45 18.27C12.01 18.93 11.44 19.48 10.78 19.88C10.12 20.28 9.39 20.52 8.64 20.58C7.89 20.64 7.14 20.52 6.44 20.23C5.74 19.94 5.11 19.49 4.6 18.92C4.09 18.35 3.72 17.68 3.51 16.95C3.3 16.22 3.26 15.45 3.4 14.7" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
      </svg>
      {hasUnread && (
        <span className={styles.unreadBadge}>!</span>
      )}
    </button>
  );
});

ChatButton.displayName = 'ChatButton';

export default ChatButton;