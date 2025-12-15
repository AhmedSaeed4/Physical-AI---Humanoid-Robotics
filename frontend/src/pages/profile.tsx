import React from 'react';
import ProfileForm from '../components/Auth/ProfileForm';
import { useAuth, AuthProvider } from '../contexts/AuthContext';

const ProfilePageContent: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading) {
    return <div className="profile-loading">Loading...</div>;
  }

  if (!user) {
    return (
      <div className="profile-auth-required">
        <h2>Access Denied</h2>
        <p>Please log in to access your profile.</p>
        <button
          className="auth-button"
          onClick={() => window.location.href = '/auth'}
        >
          Go to Login
        </button>
      </div>
    );
  }

  return (
    <div className="profile-page">
      <div className="profile-container">
        <div className="profile-header">
          <h1>Your Profile</h1>
          <p>Update your learning preferences to get personalized responses</p>
        </div>

        <div className="profile-info">
          <h3>Current Information</h3>
          <div className="profile-details">
            <p><strong>Name:</strong> {user.name}</p>
            <p><strong>Email:</strong> {user.email}</p>
            <p><strong>Education Level:</strong> {user.educationLevel || 'Not specified'}</p>
            <p><strong>Programming Experience:</strong> {user.programmingExperience || 'Not specified'}</p>
            <p><strong>Robotics Background:</strong> {user.roboticsBackground || 'Not specified'}</p>
          </div>
        </div>

        <div className="profile-form-section">
          <h3>Update Learning Preferences</h3>
          <ProfileForm />
        </div>
      </div>
    </div>
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