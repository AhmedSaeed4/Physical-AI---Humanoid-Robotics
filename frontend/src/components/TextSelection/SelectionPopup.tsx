import React, { useEffect, useState } from 'react';
import styles from './SelectionPopup.module.css';

interface SelectionPopupProps {
  selectedText: string;
  position: { x: number; y: number };
  isVisible: boolean;
  onAskAI: (text: string) => void;
  onClose: () => void;
}

const SelectionPopup: React.FC<SelectionPopupProps> = ({
  selectedText,
  position,
  isVisible,
  onAskAI,
  onClose
}) => {
  const [adjustedPosition, setAdjustedPosition] = useState({ x: position.x, y: position.y });
  const [show, setShow] = useState(false);

  // Adjust position to ensure the popup stays within viewport
  useEffect(() => {
    if (isVisible) {
      let newX = position.x;
      let newY = position.y;

      // Adjust X position to stay within viewport
      if (newX < 20) {
        newX = 20;
      } else if (newX + 150 > window.innerWidth - 20) { // 150px is approximate width of popup
        newX = window.innerWidth - 170;
      }

      // Adjust Y position to stay within viewport
      if (newY < 20) {
        newY = 20;
      }

      setAdjustedPosition({ x: newX, y: newY });
      setShow(true);
    } else {
      setShow(false);
      // Delay hiding the element to allow for animation
      const timer = setTimeout(() => {
        if (!isVisible) {
          setAdjustedPosition({ x: position.x, y: position.y });
        }
      }, 300);
      return () => clearTimeout(timer);
    }
  }, [position, isVisible]);

  const handleAskAI = () => {
    onAskAI(selectedText);
    onClose();
  };

  if (!isVisible || !selectedText) {
    return null;
  }

  return (
    <div
      className={`${styles['selection-popup']} ${show ? styles.show : styles.hide}`}
      style={{
        left: `${adjustedPosition.x}px`,
        top: `${adjustedPosition.y}px`,
      }}
      onClick={(e) => e.stopPropagation()}
    >
      <button
        className={styles['ask-ai-button']}
        onClick={handleAskAI}
        title="Ask AI about this text"
      >
        <span className={styles['ai-icon']}>✨</span>
        <span className={styles['button-text']}>Ask AI</span>
      </button>
    </div>
  );
};

export default SelectionPopup;