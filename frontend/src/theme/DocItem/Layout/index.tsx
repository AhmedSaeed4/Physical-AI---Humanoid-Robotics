import React from 'react';
import clsx from 'clsx';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import DocItemMetadata from '@theme/DocItem/Metadata';
import DocItemPaginator from '@theme/DocItem/Paginator';
import DocItemFooter from '@theme/DocItem/Footer';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import { useTextSelection } from '@site/src/components/TextSelection/useTextSelection';
import SelectionPopup from '@site/src/components/TextSelection/SelectionPopup';
import type { Props } from '@theme/DocItem/Layout';

// Create a ref to access the ChatBot component
let chatBotRef: any = null;

// Function to register the ChatBot reference globally
export const registerChatBotRef = (ref: any) => {
    chatBotRef = ref;
};

export default function DocItemLayout({ children }: Props): JSX.Element {
    const { frontMatter, metadata } = useDoc();
    const { text, position, hasSelection } = useTextSelection();

    const handleAskAI = (selectedText: string) => {
        if (chatBotRef && typeof chatBotRef.openWithSelection === 'function') {
            chatBotRef.openWithSelection(selectedText);
        } else {
            // Fallback: store in a global context or use event system
            window.dispatchEvent(new CustomEvent('openChatWithSelection', { detail: { text: selectedText } }));
        }
    };

    const handleClosePopup = () => {
        // No need to do anything specific here, the hook handles deselection
    };

    // Add keyboard accessibility - allow users to trigger AI on selected text with keyboard
    React.useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            // Check if user presses Ctrl+Enter or Cmd+Enter to ask AI about selection
            if ((e.ctrlKey || e.metaKey) && e.key === 'Enter' && text) {
                e.preventDefault();
                handleAskAI(text);
            }

            // Check if user presses Escape to close the popup
            if (e.key === 'Escape' && hasSelection) {
                // This will trigger the deselect mechanism in the hook
                window.getSelection()?.empty?.(); // For newer browsers
                window.getSelection()?.removeAllRanges?.(); // For all browsers
            }
        };

        document.addEventListener('keydown', handleKeyDown);
        return () => {
            document.removeEventListener('keydown', handleKeyDown);
        };
    }, [text, hasSelection]);

    return (
        <>
            <DocItemMetadata />
            <div className="container margin-vert--lg">
                <div className="row">
                    <main
                        className={clsx('col', {
                            'col--8': metadata.sidebar,
                        })}
                        itemScope
                        itemType="http://schema.org/Article"
                        data-doc-page
                    >
                        <DocBreadcrumbs />
                        <article className="margin-bottom--xl">
                            {children}
                        </article>
                        <DocItemFooter />
                        <DocItemPaginator />
                    </main>
                </div>
            </div>

            {/* Render the selection popup when there's a selection */}
            {hasSelection && text && (
                <SelectionPopup
                    selectedText={text}
                    position={position}
                    isVisible={hasSelection}
                    onAskAI={handleAskAI}
                    onClose={handleClosePopup}
                />
            )}
        </>
    );
}
