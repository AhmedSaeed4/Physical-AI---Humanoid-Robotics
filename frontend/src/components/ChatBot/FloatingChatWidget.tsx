import React, { useState, useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './FloatingChatWidget.module.css';
import { authService } from '../../services/authService';
import LoginPrompt from './LoginPrompt';

/**
 * Floating Chat Widget - appears as a button in the bottom-right corner
 * Opens a popup panel with the ChatKit interface
 */
// Inner component that handles ChatKit logic
const FloatingChatInner: React.FC<{ selectedText: string | null; initialThread: string | null; onThreadChange: (threadId: string) => void }> = ({ selectedText, initialThread, onThreadChange }) => {
    const { siteConfig } = useDocusaurusContext();
    const backendUrl = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';

    // Get user from localStorage synchronously (must be done in initial state to avoid hook timing issues)
    const getUserIdFromStorage = (): string | null => {
        if (typeof window === 'undefined') return null;
        const authUser = localStorage.getItem('auth_user');
        if (authUser) {
            try {
                const user = JSON.parse(authUser);
                return user.id || null;
            } catch (e) {
                console.error('Failed to parse auth_user', e);
            }
        }
        return null;
    };

    const userId = getUserIdFromStorage();


    const domainKey = (siteConfig.customFields?.domainKey as string) || 'localhost';

    // Build API URL with user ID query parameter
    const apiUrl = userId
        ? `${backendUrl}/api/chatkit?userId=${encodeURIComponent(userId)}`
        : `${backendUrl}/api/chatkit`;

    const { control, setComposerValue, focusComposer } = useChatKit({
        api: {
            url: apiUrl,
            domainKey: domainKey,
        },
        initialThread: initialThread,
        theme: {
            colorScheme: 'dark',
            color: {
                grayscale: { hue: 220, tint: 6, shade: -1 },
                accent: { primary: '#25c2a0', level: 1 },
            },
            radius: 'round',
        },
        startScreen: {
            greeting: 'Welcome! Ask me anything about the documentation.',
            prompts: [
                { label: 'Hello', prompt: 'Hello' },
                { label: 'Help', prompt: 'What can you help me with?' },
                ...(selectedText ? [{ label: 'Explain selected', prompt: `Explain this: "${selectedText}"` }] : [
                    { label: 'Documentation', prompt: 'What topics are covered in this documentation?' }
                ]),
            ],
        },
        composer: {
            placeholder: selectedText ? 'Ask about your selection...' : 'Type a message about the documentation...',
        },
        onThreadChange: ({ threadId }) => {
            if (threadId) {
                onThreadChange(threadId);
            }
        },
        onError: ({ error }) => console.error('ChatKit error:', error),
    });

    // Pre-fill composer when selectedText is provided
    useEffect(() => {
        if (selectedText && setComposerValue) {
            setComposerValue({ text: `Explain this: "${selectedText}"` });
            if (focusComposer) {
                focusComposer();
            }
        }
    }, [selectedText, setComposerValue, focusComposer]);

    return (
        <div className={styles.chatBody}>
            <ChatKit control={control} />
        </div>
    );
};

/**
 * Floating Chat Widget - appears as a button in the bottom-right corner
 * Opens a popup panel with the ChatKit interface
 */
const FloatingChatWidget: React.FC = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [initialThread, setInitialThread] = useState<string | null>(null);
    const [selectedText, setSelectedText] = useState<string | null>(null);
    const [chatKey, setChatKey] = useState(0);
    // Direct auth check - no Context needed
    const [isAuthenticated, setIsAuthenticated] = useState<boolean | null>(null);

    useEffect(() => {
        // Get user ID to create user-specific thread storage key
        const authUser = localStorage.getItem('auth_user');
        let savedThread = null;
        if (authUser) {
            try {
                const user = JSON.parse(authUser);
                const threadKey = `chatkit-thread-id-${user.id}`;
                savedThread = localStorage.getItem(threadKey);
            } catch (e) {
                // Fallback to general storage if parsing fails
                savedThread = localStorage.getItem('chatkit-thread-id');
            }
        } else {
            // If not authenticated, use general storage
            savedThread = localStorage.getItem('chatkit-thread-id');
        }
        setInitialThread(savedThread || null);

        // Listen for the custom event to update selected text
        const handleOpenChatWithSelection = (event: CustomEvent) => {
            setSelectedText(event.detail.text);
            // Don't reset chatKey - keep the active thread instead of creating a new one
            setIsOpen(true);
        };

        const eventListener = (event: Event) => handleOpenChatWithSelection(event as CustomEvent);
        window.addEventListener('openChatWithSelection', eventListener);

        return () => {
            window.removeEventListener('openChatWithSelection', eventListener);
        };
    }, []);

    // Check auth status on mount and when widget opens
    useEffect(() => {
        const checkAuth = async () => {
            try {
                const user = await authService.checkAuthStatus();
                setIsAuthenticated(!!user);
            } catch {
                setIsAuthenticated(false);
            }
        };
        checkAuth();

        // Re-check auth when localStorage changes (e.g., after login in another tab/page)
        const handleStorageChange = (e: StorageEvent) => {
            if (e.key === 'auth_user') {
                checkAuth();
                // If user logged out, clear their thread storage
                if (e.newValue === null || e.newValue === 'null') {
                    const oldUser = e.oldValue ? JSON.parse(e.oldValue) : null;
                    if (oldUser && oldUser.id) {
                        localStorage.removeItem(`chatkit-thread-id-${oldUser.id}`);
                    }
                }
            }
        };
        window.addEventListener('storage', handleStorageChange);

        // Listen for auth changes from same page (login/logout dispatch custom event)
        const handleAuthChanged = (e: Event) => {
            checkAuth();
            // Handle logout scenario
            const customEvent = e as CustomEvent;
            if (customEvent.detail && customEvent.detail.user === null) {
                // User logged out, clear their thread storage
                const authUser = localStorage.getItem('auth_user');
                if (authUser) {
                    try {
                        const user = JSON.parse(authUser);
                        localStorage.removeItem(`chatkit-thread-id-${user.id}`);
                    } catch (e) {
                        // If parsing fails, try to clear general storage
                        localStorage.removeItem('chatkit-thread-id');
                    }
                }
            }
        };
        window.addEventListener('auth_changed', handleAuthChanged);

        return () => {
            window.removeEventListener('storage', handleStorageChange);
            window.removeEventListener('auth_changed', handleAuthChanged);
        };
    }, []);

    // Re-check auth every time the widget is opened
    useEffect(() => {
        if (isOpen) {
            const checkAuth = async () => {
                try {
                    const user = await authService.checkAuthStatus();
                    setIsAuthenticated(!!user);
                } catch {
                    setIsAuthenticated(false);
                }
            };
            checkAuth();
        }
    }, [isOpen]);

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const startNewChat = () => {
        // Get user ID to create user-specific thread storage key
        const authUser = localStorage.getItem('auth_user');
        if (authUser) {
            try {
                const user = JSON.parse(authUser);
                const threadKey = `chatkit-thread-id-${user.id}`;
                localStorage.removeItem(threadKey);
            } catch (e) {
                // Fallback to general storage if parsing fails
                localStorage.removeItem('chatkit-thread-id');
            }
        } else {
            // If not authenticated, use general storage
            localStorage.removeItem('chatkit-thread-id');
        }
        setInitialThread(null);
        setSelectedText(null); // Clear selection on new chat
        setChatKey(prev => prev + 1);  // Force ChatKit remount for fresh state
    };

    const handleThreadChange = (threadId: string) => {
        // Get user ID to create user-specific thread storage key
        const authUser = localStorage.getItem('auth_user');
        if (authUser) {
            try {
                const user = JSON.parse(authUser);
                const threadKey = `chatkit-thread-id-${user.id}`;
                localStorage.setItem(threadKey, threadId);
            } catch (e) {
                // Fallback to general storage if parsing fails
                localStorage.setItem('chatkit-thread-id', threadId);
            }
        } else {
            // If not authenticated, use general storage
            localStorage.setItem('chatkit-thread-id', threadId);
        }
    };

    return (
        <>
            {/* Floating button */}
            <button
                className={`${styles.floatingButton} ${isOpen ? styles.floatingButtonHidden : ''}`}
                onClick={toggleChat}
                aria-label="Open chat assistant"
                title="Chat with AI Assistant"
            >
                <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                >
                    <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                </svg>
            </button>

            {/* Chat panel */}
            <div
                className={`${styles.chatPanel} ${isOpen ? styles.chatPanelOpen : ''}`}
            >
                <div className={styles.chatHeader}>
                    <span className={styles.chatTitle}>Documentation Assistant</span>
                    <div className={styles.headerButtons}>
                        <button
                            className={styles.newChatButton}
                            onClick={startNewChat}
                            title="Start new chat"
                        >
                            New Chat
                        </button>
                        <button
                            className={styles.closeButton}
                            onClick={toggleChat}
                            aria-label="Close chat"
                            title="Close chat"
                        >
                            ×
                        </button>
                    </div>
                </div>

                {/* Auth gate: Show LoginPrompt if not authenticated, ChatKit otherwise */}
                {isAuthenticated === null ? (
                    <div className={styles.chatBody} style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                        <span>Loading...</span>
                    </div>
                ) : isAuthenticated ? (
                    <FloatingChatInner
                        key={chatKey}
                        selectedText={selectedText}
                        initialThread={selectedText ? null : initialThread}
                        onThreadChange={handleThreadChange}
                    />
                ) : (
                    <div className={styles.chatBody}>
                        <LoginPrompt />
                    </div>
                )}
            </div>

            {/* Overlay when chat is open on mobile */}
            {isOpen && (
                <div className={styles.overlay} onClick={toggleChat} />
            )}
        </>
    );
};

export default FloatingChatWidget;
