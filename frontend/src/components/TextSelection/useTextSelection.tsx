import { useState, useEffect, useCallback, useRef } from 'react';

interface SelectedText {
  text: string;
  position: { x: number; y: number };
  hasSelection: boolean;
  elementRect?: { top: number; left: number; width: number; height: number };
}

export const useTextSelection = (): SelectedText => {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [hasSelection, setHasSelection] = useState(false);
  const [elementRect, setElementRect] = useState<{ top: number; left: number; width: number; height: number } | undefined>(undefined);
  const debouncedTimeout = useRef<NodeJS.Timeout | null>(null);

  const getSelectedText = useCallback((): string => {
    const selection = window.getSelection();
    return selection ? selection.toString().trim() : '';
  }, []);

  const getSelectionPosition = useCallback((): { x: number; y: number } | null => {
    const selection = window.getSelection();
    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    try {
      // Handle multi-range selections by focusing on the first range
      if (selection.rangeCount === 0) return null;

      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Position the popup near the selection, slightly above it
      return {
        x: rect.left,
        y: rect.top - 40, // 40px above the selection
      };
    } catch (error) {
      // If there's an error getting the range (e.g., selection across multiple elements)
      // fall back to mouse position or current cursor position
      console.warn('Error getting selection position:', error);
      return null;
    }
  }, []);

  const getSelectionRect = useCallback((): { top: number; left: number; width: number; height: number } | undefined => {
    const selection = window.getSelection();
    if (!selection || selection.toString().trim() === '') {
      return undefined;
    }

    try {
      if (selection.rangeCount === 0) return undefined;

      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      return {
        top: rect.top + window.scrollY,
        left: rect.left + window.scrollX,
        width: rect.width,
        height: rect.height,
      };
    } catch (error) {
      console.warn('Error getting selection rect:', error);
      return undefined;
    }
  }, []);

  const handleSelection = useCallback(() => {
    const text = getSelectedText();

    // Handle edge case: empty or whitespace-only selections
    if (!text || text.trim().length === 0) {
      setSelectedText('');
      setHasSelection(false);
      setElementRect(undefined);
      return;
    }

    // Handle edge case: very long text selections (>1000 characters)
    if (text.length > 1000) {
      // Truncate to 1000 characters but try to break at word boundary
      let truncatedText = text.substring(0, 1000);
      // Find the last space to avoid breaking words
      const lastSpaceIndex = truncatedText.lastIndexOf(' ');
      if (lastSpaceIndex > 800) { // Only truncate at word boundary if it's not too short
        truncatedText = truncatedText.substring(0, lastSpaceIndex);
      }
      // Add ellipsis to indicate truncation
      truncatedText += '... [selection truncated]';

      const pos = getSelectionPosition();
      const rect = getSelectionRect();

      if (pos) {
        setSelectedText(truncatedText);
        setPosition(pos);
        setHasSelection(true);
        setElementRect(rect);
      }
      return;
    }

    // Handle normal case
    const pos = getSelectionPosition();
    const rect = getSelectionRect();

    if (pos) {
      setSelectedText(text);
      setPosition(pos);
      setHasSelection(true);
      setElementRect(rect);
    }
  }, [getSelectedText, getSelectionPosition, getSelectionRect]);

  const handleDeselect = useCallback(() => {
    // Check if the current selection is empty
    const currentSelection = window.getSelection();
    if (!currentSelection || currentSelection.toString().trim() === '') {
      setSelectedText('');
      setHasSelection(false);
      setElementRect(undefined);
    }
  }, []);

  useEffect(() => {
    // Debounced handler for mouseup event
    const debouncedHandler = () => {
      if (debouncedTimeout.current) {
        clearTimeout(debouncedTimeout.current);
      }

      debouncedTimeout.current = setTimeout(() => {
        handleSelection();
      }, 100); // 100ms debounce
    };

    // Add event listeners for text selection
    const handleMouseUp = () => {
      debouncedHandler();
    };

    const handleClick = (e: MouseEvent) => {
      // Check if click is outside of any selection
      const target = e.target as HTMLElement;
      if (target.closest('.selection-popup') || target.closest('.chatbot-widget')) {
        // Click is within the popup or chat widget, don't clear selection
        return;
      }

      // If clicked outside of selection context, clear the selection
      if (!target.closest('[data-doc-page]')) {
        setTimeout(handleDeselect, 1);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClick);

    // Cleanup function - properly clean up event listeners and timeouts
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClick);

      if (debouncedTimeout.current) {
        clearTimeout(debouncedTimeout.current);
      }
    };
  }, [handleSelection, handleDeselect]);

  return { text: selectedText, position, hasSelection, elementRect };
};