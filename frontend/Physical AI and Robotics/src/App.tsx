import React from 'react';
import { ThemeProvider } from './contexts/ThemeContext';
import ChatApp from './components/ChatApp';

const App: React.FC = () => {
  return (
    <ThemeProvider>
      <div className="h-screen bg-gray-50 dark:bg-gray-900">
        <ChatApp />
      </div>
    </ThemeProvider>
  );
};

export default App;