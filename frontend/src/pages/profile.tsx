import React from 'react';
import Layout from '@theme/Layout';
import ProfileForm from '../components/Auth/ProfileForm';
import { useAuth, AuthProvider } from '../contexts/AuthContext';
import { authService } from '../services/authService';
import styles from '../components/Auth/Profile.module.css';

const ProfilePageContent: React.FC = () => {
  const { user, loading: authLoading } = useAuth();
  const [loggingOut, setLoggingOut] = React.useState(false);

  const handleLogout = async () => {
    setLoggingOut(true);
    try {
      await authService.logout();
      // Redirect to home after logout
      window.location.href = '/';
    } catch (error) {
      console.error('Logout failed:', error);
      setLoggingOut(false);
    }
  };

  if (authLoading) {
    return (
      <Layout title="Profile">
        <div className={styles.profileLoading}>Loading...</div>
      </Layout>
    );
  }

  if (!user) {
    return (
      <Layout title="Profile">
        <div className={styles.notLoggedIn}>
          <h2 className={styles.authTitle}>Access Denied</h2>
          <p>Please log in to access your profile.</p>
          <button
            className={styles.submitButton}
            onClick={() => window.location.href = '/auth'}
          >
            Go to Login
          </button>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Your Profile">
      <div className={styles.profileContainer}>
        <div className={styles.profileHeader}>
          <h1 className={styles.profileTitle}>Your Profile</h1>
          <p>Update your learning preferences to get personalized responses</p>
        </div>

        <div className={styles.profileInfo}>
          <h3 className={styles.sectionTitle}>Current Information</h3>
          <div className={styles.profileDetails}>
            <div className={styles.profileInfoItem}>
              <span className={styles.profileInfoLabel}>Name:</span>
              <span className={styles.profileInfoValue}>{user.name}</span>
            </div>
            <div className={styles.profileInfoItem}>
              <span className={styles.profileInfoLabel}>Email:</span>
              <span className={styles.profileInfoValue}>{user.email}</span>
            </div>
            <div className={styles.profileInfoItem}>
              <span className={styles.profileInfoLabel}>Education Level:</span>
              <span className={styles.profileInfoValue}>{user.educationLevel || 'Not specified'}</span>
            </div>
            <div className={styles.profileInfoItem}>
              <span className={styles.profileInfoLabel}>Programming Experience:</span>
              <span className={styles.profileInfoValue}>{user.programmingExperience || 'Not specified'}</span>
            </div>
            <div className={styles.profileInfoItem}>
              <span className={styles.profileInfoLabel}>Robotics Background:</span>
              <span className={styles.profileInfoValue}>{user.roboticsBackground || 'Not specified'}</span>
            </div>
          </div>
        </div>

        <div className={styles.formSection}>
          <ProfileForm />
        </div>

        <div className={styles.logoutSection}>
          <button
            className={styles.logoutButton}
            onClick={handleLogout}
            disabled={loggingOut}
          >
            {loggingOut ? 'Logging out...' : 'Log Out'}
          </button>
        </div>
      </div>
    </Layout>
  );
};

const ProfilePage: React.FC = () => {
  return (
    <AuthProvider>
      <ProfilePageContent />
    </AuthProvider>
  );
};

export default ProfilePage;