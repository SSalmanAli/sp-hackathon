# Data Model: Chat Modal

## Entities

### Message
Represents a single communication unit in the chat

**Fields:**
- id: string (unique identifier)
- content: string (the text content of the message)
- sender: 'user' | 'ai' (distinguishes between user and AI messages)
- timestamp: Date (when the message was sent/received)
- status: 'sent' | 'delivered' | 'error' (for message delivery tracking)

**Validation Rules:**
- content must not be empty or exceed 1000 characters
- sender must be either 'user' or 'ai'
- timestamp must be a valid date

### ChatSession
Represents an active chat interaction between user and AI assistant

**Fields:**
- id: string (unique identifier)
- messages: Message[] (array of messages in the session)
- isActive: boolean (whether the session is currently active)
- createdAt: Date (when the session was created)
- lastMessageAt: Date (timestamp of the last message)

**Validation Rules:**
- messages array must not exceed 100 messages
- isActive must be a boolean value
- createdAt and lastMessageAt must be valid dates

### ChatButtonState
Represents the state of the floating chat button

**Fields:**
- isVisible: boolean (whether the button is visible)
- isMinimized: boolean (whether the chat modal is minimized)
- hasUnread: boolean (whether there are unread messages)

**Validation Rules:**
- All fields must be boolean values

## State Transitions

### Message Status Transitions
- Initial: 'sent' (when user sends a message)
- After successful delivery: 'delivered'
- If delivery fails: 'error'

### Chat Session Transitions
- Initial: isActive = true (when modal is opened)
- When modal is closed: isActive = false (session remains for history)
- When new message is sent: lastMessageAt is updated

## Relationships

- ChatSession has many Message entities
- Each Message belongs to one ChatSession
- ChatButtonState is independent but reflects the state of the associated ChatSession