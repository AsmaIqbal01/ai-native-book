import React, { useEffect, useRef } from 'react';
import MessageBubble, { Message } from './MessageBubble';
import SystemNotification from './SystemNotification';

interface ChatWindowProps {
  messages: Message[];
  isLoading?: boolean;
  className?: string;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ 
  messages, 
  isLoading = false,
  className = '' 
}) => {
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    if (chatContainerRef.current) {
      chatContainerRef.current.scrollTop = chatContainerRef.current.scrollHeight;
    }
  };

  return (
    <div 
      ref={chatContainerRef}
      className={`flex-1 overflow-y-auto p-4 space-y-4 ${className}`}
      aria-live="polite"
      aria-relevant="additions"
    >
      {messages.length === 0 ? (
        <div className="flex items-center justify-center h-full text-gray-500 dark:text-gray-400">
          <p>No messages yet. Start the conversation!</p>
        </div>
      ) : (
        messages.map((message) => {
          if (message.role === 'system') {
            return (
              <SystemNotification 
                key={message.id} 
                message={message.content} 
                timestamp={message.timestamp} 
              />
            );
          }
          
          return (
            <MessageBubble
              key={message.id}
              message={message}
              isCurrentUser={message.role === 'user'}
            />
          );
        })
      )}
      
      {isLoading && (
        <div className="flex justify-start mb-4">
          <div className="inline-block p-4 rounded-xl bg-gray-200 dark:bg-gray-700 text-gray-800 dark:text-gray-200 rounded-bl-none">
            <div className="flex space-x-2">
              <div className="w-2 h-2 rounded-full bg-gray-500 animate-bounce"></div>
              <div className="w-2 h-2 rounded-full bg-gray-500 animate-bounce delay-75"></div>
              <div className="w-2 h-2 rounded-full bg-gray-500 animate-bounce delay-150"></div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWindow;