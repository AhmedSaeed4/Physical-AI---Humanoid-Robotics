---
id: 0004
title: Fix Navbar Auth Context Error
stage: red
date: 2025-12-15
surface: agent
model: claude-opus-4-5-20251101
feature: 006-auth-styling
branch: 006-auth-styling
user: adev
command: /sp.analyze
labels: ["navbar", "auth-context", "docusaurus", "bug-fix"]
links:
  spec: /specs/006-auth-styling/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/theme/NavbarItem/NavbarItemCustomNavbarProfileButton.tsx
 - frontend/src/theme/Root.tsx
tests:
 - none
---

## Prompt

i have run into this error now, heres the error with solution :Problem
The
NavbarItemCustomNavbarProfileButton.tsx
 component crashes with error:

useAuth must be used within an AuthProvider
This happens because the navbar in Docusaurus renders outside of any AuthProvider wrapper. The component uses
useAuth()
 from AuthContext, which throws an error when no provider is present.

File to Fix
frontend/src/theme/NavbarItem/NavbarItemCustomNavbarProfileButton.tsx

Solution Approach
Replace the
useAuth()
 hook with direct authService.checkAuthStatus() calls, using the same pattern as
FloatingChatWidget.tsx
.

Reference Implementation
Look at
frontend/src/components/ChatBot/FloatingChatWidget.tsx
 lines 99-129 for how to:

Use authService.checkAuthStatus() instead of
useAuth()
Store auth state with useState
Listen for auth_changed custom events (for same-page auth changes)
Listen for storage events (for cross-tab auth changes)
Required Changes
Remove the
useAuth
 import and hook call
Add imports for:
useState, useEffect from React
authService from ../../services/authService
Add state:
tsx
const [user, setUser] = useState<{name?: string; email: string} | null>(null);
const [loading, setLoading] = useState(true);
Add useEffect to check auth status on mount and listen for changes:
tsx
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
    window.addEventListener('storage', handleStorageChange);
    return () => {
        window.removeEventListener('auth_changed', handleAuthChanged);
        window.removeEventListener('storage', handleStorageChange);
    };
}, []);
Keep the rest of the component logic unchanged (the conditional rendering based on user and loading states)
Expected Result
The navbar profile button should:

Show "..." while loading
Show user initials when logged in (links to /profile)
Show "Login" link when not logged in (links to /auth)
React immediately when user logs in/out (without page refresh)
NOT crash the entire app

## Response snapshot

Fixed the "useAuth must be used within an AuthProvider" error by updating NavbarItemCustomNavbarProfileButton.tsx to use direct authService calls instead of the useAuth hook, following the same pattern as FloatingChatWidget.tsx. The component now manages its own auth state with useState and useEffect, and listens for auth changes via custom events and storage events.

## Outcome

- ✅ Impact: Fixed navbar component crash by removing dependency on AuthProvider context
- 🧪 Tests: none
- 📁 Files: 1 file modified (NavbarItemCustomNavbarProfileButton.tsx) to use authService directly
- 🔁 Next prompts: none
- 🧠 Reflection: Docusaurus navbar components render outside React Router context, requiring direct auth service usage

## Evaluation notes (flywheel)

- Failure modes observed: useAuth hook crashing when used outside AuthProvider context in Docusaurus navbar
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Verify navbar component properly responds to auth state changes across tabs
