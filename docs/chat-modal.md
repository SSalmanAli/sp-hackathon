# Chat Modal Component Documentation

## Overview
The Chat Modal component provides an AI assistant interface that allows users to get help with the Physical AI Book content. The component includes a floating button that opens a chat modal with message history and input functionality.

## Features
- Floating chat button in bottom-right corner
- YouTube-inspired color scheme (red, white, black)
- Responsive design for mobile, tablet, and desktop
- Keyboard accessibility (ESC to close, Enter to submit)
- ARIA attributes for accessibility compliance
- Smooth open/close animations
- Click outside to close functionality
- Close (X) button in top-right corner

## Usage

### Basic Implementation
```jsx
import ChatModalContainer from './components/ChatModal';

function App() {
  return (
    <div>
      {/* Your app content */}
      <ChatModalContainer />
    </div>
  );
}
```

### Component Structure
The component consists of:
- `ChatModalContainer`: Main container that manages state and connects button and modal
- `ChatButton`: Floating action button that triggers the modal
- `ChatModal`: The chat interface with message history and input
- `useChatModal`: Custom hook for state management

### Accessibility Features
- Keyboard navigation support (Tab to navigate, Enter to activate, ESC to close)
- ARIA roles and labels for screen readers
- Proper focus management when modal opens/closes
- High contrast color scheme for visibility

### Responsive Behavior
- Desktop: 380px wide modal
- Tablet (≤768px): Full width with 40px margins, 60% viewport height
- Mobile (≤480px): Full width with 20px margins, 50% viewport height

### Customization
The component uses CSS modules for styling. You can customize colors by modifying the CSS variables in `src/css/custom.css`:

```css
:root {
  --ifm-color-primary: #FF0000; /* YouTube red */
  --ifm-color-white: #FFFFFF;
  --ifm-color-black: #000000;
}
```

## API
The main `ChatModalContainer` component doesn't require any props and manages its own state internally.

## State Management
- Tracks modal open/close state
- Manages message history
- Handles input state and loading states
- Simulates AI responses for demonstration