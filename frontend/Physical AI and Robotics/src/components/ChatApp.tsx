import React, { useState, useEffect } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import ChatWindow from './ChatWindow';
import ChatInput from './ChatInput';
import { Message } from './MessageBubble';
import { queryRAG, checkHealth, ApiError } from '../services/api';

const ChatApp: React.FC = () => {
  const { darkMode, toggleDarkMode } = useTheme();
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your AI RAG assistant for the Physical AI and Robotics textbook. Ask me anything about AI-Native Development, ROS2, Digital Twins, or Vision-Language-Action models!',
      role: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [backendStatus, setBackendStatus] = useState<'unknown' | 'healthy' | 'unhealthy'>('unknown');

  // Check backend health on mount
  useEffect(() => {
    const checkBackendHealth = async () => {
      try {
        await checkHealth();
        setBackendStatus('healthy');
      } catch (error) {
        console.error('Backend health check failed:', error);
        setBackendStatus('unhealthy');
        // Add system message about backend status
        const systemMessage: Message = {
          id: 'health-warning',
          content: '⚠️ Warning: Backend API is currently unavailable. Responses may be delayed or unavailable.',
          role: 'system',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, systemMessage]);
      }
    };

    checkBackendHealth();
  }, []);

  // Query the RAG backend
  const queryBackend = async (userMessage: string): Promise<string> => {
    try {
      const response = await queryRAG({
        question: userMessage,
        top_k: 5,
      });

      // Format response with citations if available
      let formattedAnswer = response.answer;

      if (response.citations && response.citations.length > 0) {
        formattedAnswer += '\n\n**Sources:**\n';
        response.citations.forEach((citation, index) => {
          const chapterInfo = citation.chapter ? `Chapter ${citation.chapter}` : '';
          const sectionInfo = citation.section ? `, ${citation.section}` : '';
          formattedAnswer += `${index + 1}. ${chapterInfo}${sectionInfo}\n`;
        });
      }

      return formattedAnswer;
    } catch (error) {
      console.error('Error querying backend:', error);

      if (error instanceof ApiError) {
        if (error.status === 504 || error.status === 503) {
          throw new Error('The backend is currently unavailable or taking too long to respond. Please try again in a moment.');
        } else if (error.status === 429) {
          throw new Error('Rate limit exceeded. Please wait a moment before trying again.');
        } else {
          throw new Error(`Backend error: ${error.detail || error.message}`);
        }
      }

      throw new Error('Failed to get response from the AI assistant. Please check your connection and try again.');
    }
  };

  // Handle sending a message
  const handleSendMessage = async (messageText: string) => {
    // Add user message to chat
    const userMessage: Message = {
      id: Date.now().toString(),
      content: messageText,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Get response from RAG backend
      const response = await queryBackend(messageText);

      // Add AI response to chat
      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: response,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      // Add error message to chat
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: error instanceof Error ? error.message : 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'system',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`flex flex-col h-screen max-w-4xl mx-auto ${darkMode ? 'dark' : ''}`}>
      {/* Header with title, status, and theme toggle */}
      <header className="p-4 border-b border-gray-200 dark:border-gray-700 flex justify-between items-center">
        <div className="flex items-center gap-3">
          <h1 className="text-xl font-bold text-gray-800 dark:text-white">AI RAG Assistant</h1>
          {backendStatus !== 'unknown' && (
            <span className={`text-xs px-2 py-1 rounded-full ${
              backendStatus === 'healthy'
                ? 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-200'
                : 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200'
            }`}>
              {backendStatus === 'healthy' ? '● Connected' : '● Offline'}
            </span>
          )}
        </div>
        <button
          onClick={toggleDarkMode}
          className="p-2 rounded-full bg-gray-200 dark:bg-gray-700 text-gray-700 dark:text-gray-200 hover:bg-gray-300 dark:hover:bg-gray-600 transition-colors"
          aria-label={darkMode ? 'Switch to light mode' : 'Switch to dark mode'}
        >
          {darkMode ? (
            <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" viewBox="0 0 20 20" fill="currentColor">
              <path fillRule="evenodd" d="M10 2a1 1 0 011 1v1a1 1 0 11-2 0V3a1 1 0 011-1zm4 8a4 4 0 11-8 0 4 4 0 018 0zm-.464 4.95l.707.707a1 1 0 001.414-1.414l-.707-.707a1 1 0 00-1.414 1.414zm2.12-10.607a1 1 0 010 1.414l-.706.707a1 1 0 11-1.414-1.414l.707-.707a1 1 0 011.414 0zM17 11a1 1 0 100-2h-1a1 1 0 100 2h1zm-7 4a1 1 0 011 1v1a1 1 0 11-2 0v-1a1 1 0 011-1zM5.05 6.464A1 1 0 106.465 5.05l-.708-.707a1 1 0 00-1.414 1.414l.707.707zm1.414 8.486l-.707.707a1 1 0 01-1.414-1.414l.707-.707a1 1 0 011.414 1.414zM4 11a1 1 0 100-2H3a1 1 0 000 2h1z" clipRule="evenodd" />
            </svg>
          ) : (
            <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" viewBox="0 0 20 20" fill="currentColor">
              <path d="M17.293 13.293A8 8 0 016.707 2.707a8.001 8.001 0 1010.586 10.586z" />
            </svg>
          )}
        </button>
      </header>

      {/* Chat window */}
      <ChatWindow
        messages={messages}
        isLoading={isLoading}
        className="bg-white dark:bg-gray-900"
      />

      {/* Chat input */}
      <div className="p-4 bg-white dark:bg-gray-900 border-t border-gray-200 dark:border-gray-700">
        <ChatInput
          onSendMessage={handleSendMessage}
          placeholder="Type your message..."
          disabled={isLoading}
        />
      </div>
    </div>
  );
};

export default ChatApp;