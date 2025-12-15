import React from 'react';
import styles from './FloatingChatWidget.module.css';

/**
 * LoginPrompt - Shown in the chat widget when user is not authenticated.
 * Provides a simple CTA to redirect to the auth page.
 */
const LoginPrompt: React.FC = () => {
    const handleLoginClick = () => {
        // Pass current location as redirect param so user returns here after login
        const currentPath = window.location.pathname;
        window.location.href = `/auth?redirect=${encodeURIComponent(currentPath)}`;
    };

    return (
        <div style={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            height: '100%',
            padding: '2rem',
            textAlign: 'center',
            color: 'var(--ifm-color-content)',
        }}>
            <svg
                xmlns="http://www.w3.org/2000/svg"
                width="64"
                height="64"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="1"
                strokeLinecap="round"
                strokeLinejoin="round"
                style={{ marginBottom: '1.5rem', color: '#25c2a0' }}
            >
                <path d="M15 3h4a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2h-4" />
                <polyline points="10 17 15 12 10 7" />
                <line x1="15" y1="12" x2="3" y2="12" />
            </svg>

            <h3 style={{ marginBottom: '1rem', fontSize: '1.2rem', fontWeight: 600 }}>
                Login Required
            </h3>

            <p style={{ marginBottom: '2rem', opacity: 0.8, lineHeight: 1.5 }}>
                Please sign in to access the AI assistant and ask questions about the documentation.
            </p>

            <button
                onClick={handleLoginClick}
                style={{
                    backgroundColor: '#25c2a0',
                    color: 'white',
                    border: 'none',
                    padding: '10px 24px',
                    borderRadius: '20px',
                    fontSize: '1rem',
                    fontWeight: 600,
                    cursor: 'pointer',
                    transition: 'transform 0.1s ease, background-color 0.2s',
                    boxShadow: '0 4px 12px rgba(37, 194, 160, 0.3)',
                }}
                onMouseOver={(e) => e.currentTarget.style.transform = 'scale(1.05)'}
                onMouseOut={(e) => e.currentTarget.style.transform = 'scale(1)'}
            >
                Sign In / Sign Up
            </button>
        </div>
    );
};

export default LoginPrompt;
