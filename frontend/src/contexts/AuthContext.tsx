import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authService, User } from '../services/authService';
import { useHistory } from 'react-router-dom';

interface AuthContextType {
    user: User | null;
    loading: boolean;
    login: (email: string, password: string) => Promise<void>;
    signup: (userData: any) => Promise<void>;
    logout: () => Promise<void>;
    updateProfile: (data: any) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
    const context = useContext(AuthContext);
    if (!context) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
};

interface AuthProviderProps {
    children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
    const [user, setUser] = useState<User | null>(null);
    const [loading, setLoading] = useState(true);
    const history = useHistory();

    useEffect(() => {
        // Check auth status on app load
        const checkAuth = async () => {
            try {
                // First check localStorage for cached user
                const cachedUser = await authService.checkAuthStatus();
                if (cachedUser) {
                    // User exists in localStorage, fetch fresh profile from server
                    try {
                        const freshUser = await authService.getProfile();
                        setUser(freshUser);
                        // Update localStorage with fresh data
                        localStorage.setItem('auth_user', JSON.stringify(freshUser));
                    } catch {
                        // If profile fetch fails, use cached data
                        setUser(cachedUser);
                    }
                } else {
                    setUser(null);
                }
            } catch (error) {
                console.error('Auth check failed:', error);
                setUser(null);
            } finally {
                setLoading(false);
            }
        };

        checkAuth();
    }, []);

    useEffect(() => {
        // Set up a timer to periodically check auth status (e.g., every 5 minutes)
        // Only set the interval when user is authenticated
        if (user) {
            const interval = setInterval(async () => {
                try {
                    const updatedUserData = await authService.checkAuthStatus();
                    if (!updatedUserData) {
                        // Session has expired
                        console.log('Session expired, logging out user');
                        setUser(null);
                        history.push('/auth');
                    } else {
                        // Update user data in case it was updated elsewhere
                        setUser(updatedUserData);
                    }
                } catch (error) {
                    console.error('Periodic auth check failed:', error);
                    // If periodic check fails, assume session is invalid
                    setUser(null);
                    history.push('/auth');
                }
            }, 5 * 60 * 1000); // Check every 5 minutes

            // Cleanup interval on unmount or when user becomes null
            return () => clearInterval(interval);
        }
    }, [user, history]);

    const login = async (email: string, password: string) => {
        try {
            const userData = await authService.login({ email, password });
            setUser(userData);
            // Redirect to home or chat page after login
            history.push('/');
        } catch (error) {
            console.error('Login failed:', error);
            throw error;
        }
    };

    const signup = async (userData: any) => {
        try {
            const newUser = await authService.signup(userData);
            setUser(newUser);
            // Redirect to home or chat page after signup
            history.push('/');
        } catch (error) {
            console.error('Signup failed:', error);
            throw error;
        }
    };

    const logout = async () => {
        try {
            await authService.logout();
            setUser(null);
            // Redirect to login page after logout
            history.push('/auth');
        } catch (error) {
            console.error('Logout failed:', error);
            // Even if logout fails, clear local state
            setUser(null);
            history.push('/auth');
        }
    };

    const updateProfile = async (profileData: any) => {
        const updatedUser = await authService.updateProfile(profileData);
        setUser(updatedUser);
    };

    const value = {
        user,
        loading,
        login,
        signup,
        logout,
        updateProfile
    };

    return (
        <AuthContext.Provider value={value}>
            {children}
        </AuthContext.Provider>
    );
};