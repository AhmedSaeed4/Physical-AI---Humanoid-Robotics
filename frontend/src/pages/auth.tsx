import React, { useState } from 'react';
import SignupForm from '../components/Auth/SignupForm';
import LoginForm from '../components/Auth/LoginForm';
import { useHistory } from 'react-router-dom';

const AuthPage: React.FC = () => {
  const [isLogin, setIsLogin] = useState(true);
  const history = useHistory();

  const handleSuccess = () => {
    // Check for redirect param
    const params = new URLSearchParams(window.location.search);
    const redirectUrl = params.get('redirect') || '/';

    // Redirect to original page or home (full reload to ensure widget updates)
    window.location.href = redirectUrl;
  };

  return (
    <div className="auth-page">
      <div className="auth-container">
        <div className="auth-toggle">
          <button
            className={isLogin ? 'active' : ''}
            onClick={() => setIsLogin(true)}
          >
            Login
          </button>
          <button
            className={!isLogin ? 'active' : ''}
            onClick={() => setIsLogin(false)}
          >
            Sign Up
          </button>
        </div>

        {isLogin ? (
          <LoginForm onSuccess={handleSuccess} />
        ) : (
          <SignupForm onSuccess={handleSuccess} />
        )}
      </div>
    </div>
  );
};

export default AuthPage;