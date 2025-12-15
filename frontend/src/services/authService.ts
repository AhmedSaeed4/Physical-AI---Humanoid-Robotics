// Auth service for frontend API calls

interface SignupData {
  email: string;
  password: string;
  name: string;
  educationLevel: string;
  programmingExperience: string;
  roboticsBackground: string;
}

interface LoginData {
  email: string;
  password: string;
}

interface UpdateProfileData {
  educationLevel?: string;
  programmingExperience?: string;
  roboticsBackground?: string;
}

interface User {
  id: string;
  email: string;
  name: string;
  educationLevel: string;
  programmingExperience: string;
  roboticsBackground: string;
}

class AuthService {
  // Browser-safe environment variable access (process.env is not available in browser)
  private API_BASE = (typeof process !== 'undefined' && process.env?.REACT_APP_AUTH_URL) || 'http://localhost:3001/api';

  async signup(userData: SignupData): Promise<User> {
    const response = await fetch(`${this.API_BASE}/auth/sign-up/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(userData),
      credentials: 'include', // Include cookies for session management
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || error.statusText || 'Signup failed');
    }

    const data = await response.json();
    // Store user in localStorage for local auth check
    if (data.user) {
      localStorage.setItem('auth_user', JSON.stringify(data.user));
      // Dispatch event for same-page components (storage event only fires for other tabs)
      window.dispatchEvent(new CustomEvent('auth_changed', { detail: { user: data.user } }));
    }
    return data.user;
  }

  async login(credentials: LoginData): Promise<User> {
    const response = await fetch(`${this.API_BASE}/auth/sign-in/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(credentials),
      credentials: 'include', // Include cookies for session management
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || error.statusText || 'Login failed');
    }

    const data = await response.json();
    // Store user in localStorage for local auth check
    if (data.user) {
      localStorage.setItem('auth_user', JSON.stringify(data.user));
      // Dispatch event for same-page components (storage event only fires for other tabs)
      window.dispatchEvent(new CustomEvent('auth_changed', { detail: { user: data.user } }));
    }
    return data.user;
  }

  async logout(): Promise<void> {
    const response = await fetch(`${this.API_BASE}/auth/sign-out`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Include cookies for session management
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || error.statusText || 'Logout failed');
    }
    // Clear localStorage on logout
    localStorage.removeItem('auth_user');
    // Dispatch event for same-page components
    window.dispatchEvent(new CustomEvent('auth_changed', { detail: { user: null } }));
  }

  async getProfile(): Promise<User> {
    const response = await fetch(`${this.API_BASE}/user/profile`, {
      method: 'GET',
      credentials: 'include', // Include cookies for session management
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to get profile');
    }

    const data = await response.json();
    return data.user;
  }

  async updateProfile(profileData: UpdateProfileData): Promise<User> {
    const response = await fetch(`${this.API_BASE}/user/update`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(profileData),
      credentials: 'include', // Include cookies for session management
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to update profile');
    }

    const data = await response.json();
    // Update localStorage with new user data
    if (data.user) {
      localStorage.setItem('auth_user', JSON.stringify(data.user));
      window.dispatchEvent(new CustomEvent('auth_changed', { detail: { user: data.user } }));
    }
    return data.user;
  }

  async checkAuthStatus(): Promise<User | null> {
    // First check localStorage for cached user (fast check)
    const cachedUser = localStorage.getItem('auth_user');
    if (cachedUser) {
      try {
        return JSON.parse(cachedUser);
      } catch {
        localStorage.removeItem('auth_user');
      }
    }
    return null;
  }
}

export const authService = new AuthService();
export type { SignupData, LoginData, UpdateProfileData, User };