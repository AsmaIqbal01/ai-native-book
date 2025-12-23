import React, { useEffect } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useChatbot } from '../contexts/ChatbotContext';
import ChatWindow from './ChatWindow';
import ChatInput from './ChatInput';
import { Message } from './MessageBubble';
import { queryRAG, checkHealth, ApiError } from '../services/api';

/**
 * FloatingChatWidget Component
 *
 * A viewport-fixed chatbot widget that appears on all pages in the bottom-left corner.
 * Features:
 * - Viewport-fixed positioning (stays in place during scroll)
 * - Agent-controlled global state (via ChatbotContext)
 * - Expandable/collapsible chat interface
 * - Persistent state across page navigation
 * - Backend health monitoring
 * - Smooth animations
 * - Responsive design
 * - Accessibility support
 */

interface FloatingChatWidgetProps {
  className?: string;
}

const FloatingChatWidget: React.FC<FloatingChatWidgetProps> = ({ className = '' }) => {
  console.log('🤖 FloatingChatWidget component initialized');
  const { darkMode } = useTheme();
  console.log('🤖 Theme context loaded, darkMode:', darkMode);

  // Use global chatbot context instead of local state
  const {
    isExpanded,
    isMinimized,
    unreadCount,
    messages,
    isLoading,
    backendStatus,
    toggleExpand,
    setMessages,
    setIsLoading,
    setBackendStatus,
    setUnreadCount,
  } = useChatbot();

  // Check backend health on mount
  useEffect(() => {
    const checkBackendHealth = async () => {
      try {
        await checkHealth();
        setBackendStatus('healthy');
      } catch (error) {
        console.error('Backend health check failed:', error);
        setBackendStatus('unhealthy');
      }
    };

    checkBackendHealth();
    // Check health every 5 minutes
    const interval = setInterval(checkBackendHealth, 5 * 60 * 1000);
    return () => clearInterval(interval);
  }, [setBackendStatus]);

  // Keyboard accessibility: Close chatbot with Escape key
  useEffect(() => {
    const handleEscapeKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isExpanded) {
        toggleExpand();
      }
    };

    document.addEventListener('keydown', handleEscapeKey);
    return () => document.removeEventListener('keydown', handleEscapeKey);
  }, [isExpanded, toggleExpand]);

  // Query the RAG backend
  const queryBackend = async (userMessage: string): Promise<string> => {
    try {
      const response = await queryRAG({
        question: userMessage,
        top_k: 5,
      });

      // Format response with citations
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
          throw new Error('The backend is currently unavailable. Please try again in a moment.');
        } else if (error.status === 429) {
          throw new Error('Rate limit exceeded. Please wait before trying again.');
        } else {
          throw new Error(`Backend error: ${error.detail || error.message}`);
        }
      }

      throw new Error('Failed to get response. Please check your connection and try again.');
    }
  };

  // Handle sending a message
  const handleSendMessage = async (messageText: string) => {
    const userMessage: Message = {
      id: Date.now().toString(),
      content: messageText,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await queryBackend(messageText);

      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: response,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: error instanceof Error ? error.message : 'Sorry, I encountered an error. Please try again.',
        role: 'system',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Clear chat history
  const handleClearChat = () => {
    if (typeof window === 'undefined') return;
    if (confirm('Clear all chat history?')) {
      const welcomeMessage: Message = {
        id: 'welcome-' + Date.now(),
        content: 'Hello! I\'m your AI RAG assistant. How can I help you today?',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages([welcomeMessage]);
      localStorage.removeItem('chatHistory');
    }
  };

  return (
    <div id="rag-chatbot-container">
      {/* Floating Chat Button (when collapsed) - Icon Only */}
      {!isExpanded && (
        <button
          onClick={toggleExpand}
          className={`fixed bottom-4 left-4 z-[9999] flex items-center justify-center w-16 h-16 rounded-full text-white shadow-2xl hover:shadow-3xl hover:scale-110 active:scale-95 transition-all duration-300 group ${className}`}
          style={{
            background: 'linear-gradient(135deg, #0066ff 0%, #a855f7 100%)',
            boxShadow: unreadCount > 0
              ? '0 0 30px rgba(168, 85, 247, 0.8), 0 8px 32px rgba(0, 0, 0, 0.3)'
              : '0 0 20px rgba(0, 102, 255, 0.6), 0 8px 32px rgba(0, 0, 0, 0.2)',
          }}
          aria-label="Open AI chatbot"
          title="AI Assistant"
        >
          {/* Unread badge */}
          {unreadCount > 0 && (
            <span
              className="absolute -top-1 -right-1 flex items-center justify-center w-6 h-6 text-xs font-bold text-white rounded-full animate-pulse"
              style={{
                background: 'linear-gradient(135deg, #ef4444 0%, #dc2626 100%)',
                boxShadow: '0 0 15px rgba(239, 68, 68, 0.8)',
              }}
            >
              {unreadCount > 9 ? '9+' : unreadCount}
            </span>
          )}

          {/* Backend status indicator */}
          <span
            className={`absolute top-1 right-1 w-3 h-3 rounded-full ${
              backendStatus === 'healthy'
                ? 'bg-green-400'
                : backendStatus === 'unhealthy'
                ? 'bg-red-400'
                : 'bg-yellow-400'
            }`}
            style={{
              boxShadow: backendStatus === 'healthy'
                ? '0 0 10px rgba(52, 211, 153, 0.8)'
                : '0 0 10px rgba(248, 113, 113, 0.8)',
            }}
            title={backendStatus === 'healthy' ? 'Backend online' : 'Backend offline'}
          />

          {/* Robot Emoji - Bold and Simple */}
          <span
            className="text-4xl transition-transform duration-300 group-hover:scale-110"
            role="img"
            aria-label="Robot"
          >
            🤖
          </span>
        </button>
      )}

      {/* Expanded Chat Window - Bold Enterprise Design */}
      {isExpanded && (
        <div
          className={`fixed bottom-4 left-4 z-[9999] flex flex-col w-[400px] h-[600px] max-w-[calc(100vw-2rem)] max-h-[calc(100vh-2rem)] bg-white dark:bg-gray-900 rounded-2xl overflow-hidden animate-slide-in-left ${className}`}
          style={{
            boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3), 0 0 40px rgba(168, 85, 247, 0.3)',
            border: '2px solid rgba(168, 85, 247, 0.5)',
          }}
          role="dialog"
          aria-label="AI Chatbot"
        >
          {/* Header - Icon Only, Bold Design */}
          <div
            className="flex items-center justify-between px-4 py-3 text-white relative overflow-hidden"
            style={{
              background: 'linear-gradient(135deg, #0066ff 0%, #a855f7 100%)',
              boxShadow: '0 4px 20px rgba(168, 85, 247, 0.4)',
            }}
          >
            <div className="flex items-center gap-3">
              <div className="flex items-center justify-center w-12 h-12 rounded-full bg-white/20 backdrop-blur-sm">
                <span className="text-2xl" role="img" aria-label="Robot">
                  🤖
                </span>
              </div>
              <div className="flex items-center gap-2">
                <span
                  className={`w-2 h-2 rounded-full ${
                    backendStatus === 'healthy' ? 'bg-green-400' : 'bg-red-400'
                  }`}
                  style={{
                    boxShadow: backendStatus === 'healthy'
                      ? '0 0 10px rgba(52, 211, 153, 0.8)'
                      : '0 0 10px rgba(248, 113, 113, 0.8)',
                  }}
                  title={backendStatus === 'healthy' ? 'Online' : 'Offline'}
                />
              </div>
            </div>

            <div className="flex items-center gap-2">
              {/* Clear chat button */}
              <button
                onClick={handleClearChat}
                className="p-2 rounded-lg hover:bg-white/20 transition-colors"
                aria-label="Clear chat history"
                title="Clear chat"
              >
                <svg
                  className="w-5 h-5"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16"
                  />
                </svg>
              </button>

              {/* Close button */}
              <button
                onClick={toggleExpand}
                className="p-2 rounded-lg hover:bg-white/20 transition-colors"
                aria-label="Close chatbot"
                title="Close"
              >
                <svg
                  className="w-5 h-5"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M6 18L18 6M6 6l12 12"
                  />
                </svg>
              </button>
            </div>
          </div>

          {/* Chat Window */}
          <div className="flex-1 overflow-hidden">
            <ChatWindow
              messages={messages}
              isLoading={isLoading}
              className="h-full bg-gray-50 dark:bg-gray-900"
            />
          </div>

          {/* Chat Input */}
          <div className="p-4 bg-white dark:bg-gray-900 border-t border-gray-200 dark:border-gray-700">
            <ChatInput
              onSendMessage={handleSendMessage}
              placeholder="Type your message..."
              disabled={isLoading || backendStatus === 'unhealthy'}
            />
          </div>
        </div>
      )}

      {/* Custom CSS for animations */}
      <style>{`
        @keyframes slide-in-left {
          from {
            opacity: 0;
            transform: translateX(-20px) scale(0.95);
          }
          to {
            opacity: 1;
            transform: translateX(0) scale(1);
          }
        }

        .animate-slide-in-left {
          animation: slide-in-left 0.3s ease-out;
        }

        /* Mobile responsiveness */
        @media (max-width: 640px) {
          .z-\[9999\][role="dialog"] {
            bottom: 0 !important;
            left: 0 !important;
            width: 100vw !important;
            height: 100vh !important;
            max-width: 100vw !important;
            max-height: 100vh !important;
            border-radius: 0 !important;
          }
        }

        /* Ensure chatbot is above all Docusaurus elements */
        #rag-chatbot-container {
          isolation: isolate;
        }
      `}</style>
    </div>
  );
};

export default FloatingChatWidget;
