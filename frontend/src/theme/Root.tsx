import React from 'react';
import FloatingChatWidget from '@site/src/components/ChatBot/FloatingChatWidget';

// This component wraps the entire site layout
// See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
export default function Root({ children }) {
    return (
        <>
            {children}
            <FloatingChatWidget />
        </>
    );
}
