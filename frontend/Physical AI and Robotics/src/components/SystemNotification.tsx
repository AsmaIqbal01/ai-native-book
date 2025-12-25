import React from 'react';

interface SystemNotificationProps {
  message: string;
  timestamp?: Date;
}

const SystemNotification: React.FC<SystemNotificationProps> = ({ 
  message, 
  timestamp = new Date() 
}) => {
  const formattedTime = timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  
  return (
    <div className="flex justify-center my-4">
      <div className="inline-flex items-center px-4 py-2 bg-gray-100 dark:bg-gray-800 text-gray-600 dark:text-gray-300 rounded-full text-sm border border-gray-200 dark:border-gray-700">
        <span>{message}</span>
        <span className="ml-2 text-xs opacity-70">{formattedTime}</span>
      </div>
    </div>
  );
};

export default SystemNotification;