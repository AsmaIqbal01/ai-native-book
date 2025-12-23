import React from 'react';

// Define types for message roles and content
export type MessageRole = 'user' | 'assistant' | 'system';
export interface Message {
  id: string;
  content: string;
  role: MessageRole;
  timestamp: Date;
  // Optional: Different AI persona for styling
  persona?: string;
}

// Props for the MessageBubble component
interface MessageBubbleProps {
  message: Message;
  isCurrentUser?: boolean;
}

const MessageBubble: React.FC<MessageBubbleProps> = ({ message, isCurrentUser = false }) => {
  // Determine bubble alignment and styling based on sender
  const positionClass = isCurrentUser ? 'ml-auto' : 'mr-auto';
  const bgClass = isCurrentUser 
    ? 'bg-blue-500 text-white rounded-br-none' 
    : 'bg-gray-200 dark:bg-gray-700 text-gray-800 dark:text-gray-200 rounded-bl-none';
  
  // Format timestamp
  const formattedTime = message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  return (
    <div className={`flex ${isCurrentUser ? 'justify-end' : 'justify-start'} mb-4 max-w-[85%]`}>
      <div className={`inline-block p-4 rounded-xl shadow-md ${positionClass} ${bgClass}`}>
        {/* Message content */}
        <div className="whitespace-pre-wrap break-words">
          {message.content}
        </div>
        
        {/* Timestamp */}
        <div className={`text-xs mt-1 ${isCurrentUser ? 'text-blue-100' : 'text-gray-500 dark:text-gray-400'}`}>
          {formattedTime}
        </div>
      </div>
    </div>
  );
};

export default MessageBubble;