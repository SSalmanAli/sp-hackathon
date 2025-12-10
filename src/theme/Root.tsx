import React from 'react';
import ChatModalContainer from '../components/ChatModal';

// Default implementation, that you can customize
export default function Root({ children }: { children: React.ReactNode }) {
    return (
        <>
            {children}
            <ChatModalContainer />
        </>
    );
}
