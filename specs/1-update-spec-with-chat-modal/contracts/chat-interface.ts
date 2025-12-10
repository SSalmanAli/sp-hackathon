// Chat Interface Contract
// Defines the interface for the chat modal component

interface Message {
  id: string;
  content: string;
  sender: 'user' | 'ai';
  timestamp: Date;
  status: 'sent' | 'delivered' | 'error';
}

interface ChatSession {
  id: string;
  messages: Message[];
  isActive: boolean;
  createdAt: Date;
  lastMessageAt: Date;
}

interface ChatModalProps {
  // Callback for when a new message is sent
  onSendMessage: (content: string) => void;

  // Callback for when the modal is opened
  onOpen?: () => void;

  // Callback for when the modal is closed
  onClose?: () => void;

  // Initial messages to display
  initialMessages?: Message[];

  // Whether the modal is currently open
  isOpen: boolean;

  // Whether to show the modal
  visible: boolean;
}

interface ChatButtonProps {
  // Callback for when the button is clicked
  onClick: () => void;

  // Whether the chat has unread messages
  hasUnread?: boolean;

  // Whether the button should be visible
  visible?: boolean;
}