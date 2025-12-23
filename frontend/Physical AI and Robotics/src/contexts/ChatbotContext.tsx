import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { Message } from '../components/MessageBubble';

/**
 * ChatbotContext - Global State Management for AI Chatbot
 *
 * This context provides agent-controlled visibility and state management
 * for the floating chatbot widget. It ensures the chatbot state persists
 * across route changes and can be controlled by external agents or components.
 *
 * Features:
 * - Global visibility state (isExpanded, isMinimized)
 * - Persistent message history (localStorage backed)
 * - Backend health monitoring
 * - Unread message tracking
 * - Agent-controlled UI state
 */

export interface ChatbotContextType {
  // Visibility State
  isExpanded: boolean;
  isMinimized: boolean;
  setIsExpanded: (expanded: boolean) => void;
  setIsMinimized: (minimized: boolean) => void;
  toggleExpand: () => void;

  // Message State
  messages: Message[];
  setMessages: React.Dispatch<React.SetStateAction<Message[]>>;
  addMessage: (message: Message) => void;
  clearMessages: () => void;

  // UI State
  isLoading: boolean;
  setIsLoading: (loading: boolean) => void;
  unreadCount: number;
  setUnreadCount: (count: number) => void;

  // Backend State
  backendStatus: 'unknown' | 'healthy' | 'unhealthy';
  setBackendStatus: (status: 'unknown' | 'healthy' | 'unhealthy') => void;
}

const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

interface ChatbotProviderProps {
  children: ReactNode;
}

export const ChatbotProvider: React.FC<ChatbotProviderProps> = ({ children }) => {
  // Visibility state - persisted globally
  const [isExpanded, setIsExpanded] = useState<boolean>(false);
  const [isMinimized, setIsMinimized] = useState<boolean>(false);
  const [unreadCount, setUnreadCount] = useState<number>(0);

  // Chat state
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [backendStatus, setBackendStatus] = useState<'unknown' | 'healthy' | 'unhealthy'>('unknown');

  // Initialize with welcome message
  useEffect(() => {
    const welcomeMessage: Message = {
      id: 'welcome-1',
      content: 'Hello! I\'m your AI RAG assistant for the Physical AI and Robotics textbook. Ask me anything about AI-Native Development, ROS2, Digital Twins, or Vision-Language-Action models!',
      role: 'assistant',
      timestamp: new Date(),
    };
    setMessages([welcomeMessage]);
  }, []);

  // Load chat history from localStorage on mount (client-side only)
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const savedMessages = localStorage.getItem('chatHistory');
    if (savedMessages) {
      try {
        const parsed = JSON.parse(savedMessages);
        // Convert timestamp strings back to Date objects
        const messagesWithDates = parsed.map((msg: any) => ({
          ...msg,
          timestamp: new Date(msg.timestamp),
        }));
        setMessages(messagesWithDates);
      } catch (error) {
        console.error('Failed to load chat history:', error);
        // Clear corrupted data
        localStorage.removeItem('chatHistory');
      }
    }
  }, []);

  // Save chat history to localStorage when messages change
  useEffect(() => {
    if (typeof window === 'undefined') return;
    if (messages.length > 1) { // Only save if there are messages beyond welcome
      localStorage.setItem('chatHistory', JSON.stringify(messages));
    }
  }, [messages]);

  // Track unread messages when minimized or collapsed
  useEffect(() => {
    if (!isExpanded && messages.length > 0) {
      const lastMessage = messages[messages.length - 1];
      if (lastMessage.role === 'assistant' && !isMinimized) {
        setUnreadCount(prev => prev + 1);
      }
    }
  }, [messages, isExpanded, isMinimized]);

  // Toggle expand/collapse
  const toggleExpand = () => {
    setIsExpanded(!isExpanded);
    if (!isExpanded) {
      setUnreadCount(0);
      setIsMinimized(false);
    }
  };

  // Add a message to the conversation
  const addMessage = (message: Message) => {
    setMessages(prev => [...prev, message]);
  };

  // Clear all messages and reset to welcome message
  const clearMessages = () => {
    if (typeof window === 'undefined') return;

    const welcomeMessage: Message = {
      id: 'welcome-' + Date.now(),
      content: 'Hello! I\'m your AI RAG assistant. How can I help you today?',
      role: 'assistant',
      timestamp: new Date(),
    };
    setMessages([welcomeMessage]);
    localStorage.removeItem('chatHistory');
  };

  const value: ChatbotContextType = {
    // Visibility
    isExpanded,
    isMinimized,
    setIsExpanded,
    setIsMinimized,
    toggleExpand,

    // Messages
    messages,
    setMessages,
    addMessage,
    clearMessages,

    // UI State
    isLoading,
    setIsLoading,
    unreadCount,
    setUnreadCount,

    // Backend
    backendStatus,
    setBackendStatus,
  };

  return (
    <ChatbotContext.Provider value={value}>
      {children}
    </ChatbotContext.Provider>
  );
};

/**
 * Hook to access chatbot context
 * Must be used within ChatbotProvider
 */
export const useChatbot = (): ChatbotContextType => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
};
