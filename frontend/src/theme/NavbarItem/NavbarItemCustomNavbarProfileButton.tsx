import React, { useState, useEffect } from 'react';
import { authService } from '../../services/authService';
import styles from './NavbarProfileButton.module.css';

const NavbarItemCustomNavbarProfileButton: React.FC = () => {
  const [user, setUser] = useState<{name?: string; email: string} | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const checkAuth = async () => {
      try {
        const userData = await authService.checkAuthStatus();
        setUser(userData);
      } catch {
        setUser(null);
      } finally {
        setLoading(false);
      }
    };
    checkAuth();

    // Listen for auth changes from same page
    const handleAuthChanged = () => {
      checkAuth();
    };
    window.addEventListener('auth_changed', handleAuthChanged);

    // Listen for storage changes (cross-tab)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'auth_user') {
        checkAuth();
      }
    };
    window.addEventListener('storage', handleStorageChange as EventListener);

    return () => {
      window.removeEventListener('auth_changed', handleAuthChanged);
      window.removeEventListener('storage', handleStorageChange as EventListener);
    };
  }, []);

  if (loading) {
    // Show loading state if auth status is still being determined
    return (
      <div className={styles.navbarProfileButton}>
        <div className={styles.profileAvatar}>
          <span className={styles.profileInitials}>...</span>
        </div>
      </div>
    );
  }

  if (user) {
    // Show profile button when user is logged in
    const initials = user.name
      ? user.name
          .split(' ')
          .map(n => n[0])
          .join('')
          .substring(0, 2)
          .toUpperCase()
      : user.email
          .substring(0, 2)
          .toUpperCase();

    return (
      <a
        href="/profile"
        className={styles.navbarProfileButton}
        title="View Profile"
      >
        <div className={styles.profileAvatar}>
          <span className={styles.profileInitials}>{initials}</span>
        </div>
      </a>
    );
  } else {
    // Show login link when user is not logged in
    return (
      <a
        href="/auth"
        className={styles.navbarLoginLink}
        title="Login"
      >
        Login
      </a>
    );
  }
};

export default NavbarItemCustomNavbarProfileButton;