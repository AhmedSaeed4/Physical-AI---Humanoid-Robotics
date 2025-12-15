# Quick Start: User Authentication and Persistent Chat History

**Feature**: 005-auth-neon-integration
**Date**: 2025-12-13

## Overview

This feature adds user authentication with Better Auth and persistent RAG chat history using Neon PostgreSQL. It integrates with the existing RAG chatbot system to provide per-user chat persistence, learning profile personalization, and secure session management.

## Prerequisites

### Environment Setup
1. **Neon PostgreSQL Database**: Create a Neon project and get `DATABASE_URL`
2. **Environment Variables**: Update `.env` files in all three services:
   ```bash
   # auth-server/.env
   DATABASE_URL=postgresql://user:password@host/database?sslmode=require
   JWT_SECRET=your-strong-jwt-secret-here

   # backend/.env (add these)
   DATABASE_URL=postgresql://user:password@host/database?sslmode=require
   JWT_SECRET=your-strong-jwt-secret-here  # Same as auth-server

   # frontend/.env (if needed)
   NEXT_PUBLIC_AUTH_URL=http://localhost:3001
   NEXT_PUBLIC_API_URL=http://localhost:8000
   ```

### Service Ports
- **Frontend (Docusaurus)**: Port 3000
- **Auth Server (Node.js)**: Port 3001
- **Backend (FastAPI)**: Port 8000

## Setup Steps

### 1. Database Migrations
```bash
# Run from project root
cd backend
python migrations/migrate.py
```

This creates:
- Better Auth tables (via library)
- Custom user profile columns
- Chat history table with indexes

### 2. Install Dependencies
```bash
# Auth Server
cd auth-server
npm install

# Backend (if new dependencies)
cd backend
uv add python-jose[cryptography]  # For JWT validation
uv add psycopg2-binary  # For PostgreSQL connection

# Frontend
cd frontend
npm install
```

### 3. Start All Services
```bash
# Terminal 1: Frontend
cd frontend
npm start  # Runs on http://localhost:3000

# Terminal 2: Auth Server
cd auth-server
npm run dev  # Runs on http://localhost:3001

# Terminal 3: Backend
cd backend
uv run uvicorn src.backend.main:app --reload --port 8000
```

## Testing the Integration

### 1. Create Account
```bash
curl -X POST http://localhost:3001/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123",
    "name": "Test User",
    "educationLevel": "Undergraduate",
    "programmingExperience": "Beginner",
    "roboticsBackground": "No Experience"
  }'
```

**Expected Response**: 201 Created with user data and Set-Cookie header

### 2. Login
```bash
curl -X POST http://localhost:3001/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123"
  }'
```

**Expected Response**: 200 OK with user data and Set-Cookie header

### 3. Send Chat Message (Authenticated)
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <JWT_TOKEN_FROM_COOKIE>" \
  -d '{
    "user_query": "How do I program a robot arm?",
    "selected_text": "The robot arm has 6 degrees of freedom...",
    "chat_history": [],
    "user_profile": null
  }'
```

**Expected Response**: 200 OK with AI response, context chunks, and sources

### 4. Get Chat History
```bash
curl -X GET "http://localhost:8000/api/chat/history?limit=10" \
  -H "Authorization: Bearer <JWT_TOKEN_FROM_COOKIE>"
```

**Expected Response**: 200 OK with array of chat history items

### 5. Update Profile
```bash
curl -X POST http://localhost:3001/api/user/update \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <JWT_TOKEN_FROM_COOKIE>" \
  -d '{
    "programmingExperience": "Intermediate",
    "educationLevel": "Graduate"
  }'
```

**Expected Response**: 200 OK with updated user data

### 6. Get User Profile
```bash
curl -X GET "http://localhost:3001/api/user/profile" \
  -H "Authorization: Bearer <JWT_TOKEN_FROM_COOKIE>"
```

**Expected Response**: 200 OK with complete user profile including learning preferences

## Frontend Integration

### Auth Context Setup
```typescript
// frontend/src/contexts/AuthContext.tsx
import { createContext, useContext, useState, useEffect } from 'react';
import { authService } from '../services/authService';

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (userData: SignupData) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (data: UpdateProfileData) => Promise<void>;
}

export const AuthContext = createContext<AuthContextType>({...});

// Wrap your app with AuthProvider
```

### Protected Chat Component
```typescript
// frontend/src/components/ChatBot/index.tsx
import { useAuth } from '../../contexts/AuthContext';

export default function ChatBot() {
  const { user, loading } = useAuth();

  if (loading) return <div>Loading...</div>;

  if (!user) {
    return (
      <div className="chat-login-gate">
        <button onClick={() => navigate('/auth')}>
          <ChatIcon /> Login to start chatting
        </button>
      </div>
    );
  }

  return <ChatBotAuthenticated user={user} />;
}
```

### Auth Pages
- `/auth` - Login/Signup page with toggle
- `/profile` - Profile update page with learning preference selectors

## Error Handling

### Common Errors and Solutions

1. **"Invalid email or password" (401)**
   - Check email/password correctness
   - Verify user exists in database

2. **"Email already exists" (409)**
   - Use different email for signup
   - Or login with existing account

3. **"Session expired" (401)**
   - Auto-redirect to login page
   - Clear local storage/cookies

4. **"Unable to connect" (Network Error)**
   - Check if all services are running
   - Verify CORS configuration

5. **"Database connection failed"**
   - Check `DATABASE_URL` in .env
   - Verify Neon PostgreSQL is running
   - Check SSL configuration (`sslmode=require`)

## Development Workflow

### Local Development
1. **Triple-Terminal Setup**: Always run all three services
2. **Database First**: Run migrations before testing auth
3. **CORS Configuration**: Ensure ports 3000, 3001, 8000 are allowed
4. **Environment Variables**: Keep .env files synchronized

### Testing
```bash
# Run auth tests
cd auth-server
npm test

# Run backend tests
cd backend
uv run pytest

# Run frontend tests
cd frontend
npm test
```

### Debugging
1. **Check Console Logs**: All services log to console
2. **Verify Cookies**: Use browser dev tools → Application → Cookies
3. **Network Requests**: Check Network tab for API calls
4. **Database Queries**: Use Neon console to query tables directly

## Production Deployment

### Environment Variables (Production)
```bash
# Production .env
DATABASE_URL=postgresql://prod-user:prod-password@prod-host/prod-db?sslmode=require
JWT_SECRET=strong-production-secret
FRONTEND_URL=https://your-domain.com
BACKEND_URL=https://api.your-domain.com
AUTH_URL=https://auth.your-domain.com
```

### CORS Configuration (Production)
```javascript
// auth-server CORS
app.use(cors({
  origin: [process.env.FRONTEND_URL, process.env.BACKEND_URL],
  credentials: true
}));
```

### Database Backups
- Neon provides automatic backups
- Consider point-in-time recovery for chat history
- Regular export of user data for compliance

## Monitoring and Maintenance

### Key Metrics to Monitor
1. **Auth Success Rate**: Should be >95%
2. **Chat Persistence Rate**: Should be 100%
3. **Session Duration**: Average user session time
4. **Database Performance**: Query latency, connection pool usage

### Regular Maintenance
1. **Database Indexes**: Monitor and optimize as chat history grows
2. **Session Cleanup**: Better Auth handles expired sessions
3. **Security Updates**: Keep Better Auth and dependencies updated
4. **Backup Verification**: Regularly test database restore process

## Troubleshooting Guide

### Issue: "CORS error" in browser console
**Solution**:
1. Check CORS configuration in both auth-server and backend
2. Verify ports match (3000, 3001, 8000)
3. Ensure `credentials: 'include'` in frontend fetch calls

### Issue: "JWT validation failed" in backend
**Solution**:
1. Verify `JWT_SECRET` is identical in auth-server and backend .env files
2. Check token expiration (default 24 hours)
3. Ensure Authorization header format: `Bearer <token>`

### Issue: "Database connection refused"
**Solution**:
1. Check `DATABASE_URL` format and credentials
2. Verify Neon project is active and not paused
3. Check SSL configuration (`sslmode=require`)

### Issue: "Chat history not loading"
**Solution**:
1. Check user ID in JWT matches database records
2. Verify chat_history table exists and has data
3. Check database indexes are created

## Support

For issues not covered here:
1. Check service logs for error details
2. Verify all prerequisites are met
3. Consult research.md for implementation decisions
4. Review data-model.md for database schema
5. Check contracts/openapi.yaml for API specifications