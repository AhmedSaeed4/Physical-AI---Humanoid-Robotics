# Better Auth API Reference

## Core Configuration

### Auth Instance Configuration
The main configuration for Better Auth. Create an auth instance with authentication settings.

**Required Fields:**
- `database` (object): Database connection settings
  - `provider` (string): Database provider ('sqlite', 'postgresql', 'mysql')
  - `url` (string): Database connection URL

**Common Optional Fields:**
- `emailAndPassword` (object): Email/password authentication settings
  - `enabled` (boolean): Whether email/password auth is enabled
  - `requireEmailVerification` (boolean): Require email verification
  - `minPasswordLength` (number): Minimum password length
- `socialProviders` (object): OAuth provider configurations
- `session` (object): Session management settings
- `account` (object): Account-related configurations
- `plugins` (array): Additional functionality plugins
- `rateLimit` (object): Rate limiting configuration

### Example Configuration:
```typescript
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "postgresql",
    url: process.env.DATABASE_URL!,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
    minPasswordLength: 8,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    },
  },
  session: {
    expires: 7 * 24 * 60 * 60 * 1000, // 7 days
    updateAge: 24 * 60 * 60 * 1000,   // Refresh every 24 hours
  },
  plugins: [
    // Add plugins here
  ],
  rateLimit: {
    window: 60000, // 1 minute
    max: 10,       // 10 requests per minute
  },
});
```

## Database Adapters

### Supported Databases
- `sqlite`: SQLite database
- `postgresql`: PostgreSQL database
- `mysql`: MySQL database
- `planetscale`: PlanetScale MySQL
- `custom`: Custom database adapter

### Example Database Configuration:
```typescript
database: {
  provider: "postgresql",
  url: process.env.DATABASE_URL!,
  // Optional: custom schema name
  schema: "my_schema",
}
```

## Social Providers

### Available Providers
- `google`: Google OAuth
- `github`: GitHub OAuth
- `facebook`: Facebook OAuth
- `twitter`: Twitter OAuth
- `apple`: Apple Sign In
- `discord`: Discord OAuth
- `twitch`: Twitch OAuth
- `linkedin`: LinkedIn OAuth
- `microsoft`: Microsoft OAuth

### Provider Configuration Options:
- `clientId` (string): OAuth client ID
- `clientSecret` (string): OAuth client secret
- `scope` (string[]): Additional OAuth scopes
- `redirectUri` (string): Custom redirect URI
- `customProvider` (object): Custom OAuth provider configuration

## Session Management

### Session Configuration Options:
- `expires` (number): Session expiration time in milliseconds
- `updateAge` (number): How often to update session age
- `cookieName` (string): Name of the session cookie
- `cookieOptions` (object): Cookie configuration options
- `slidingExpiration` (boolean): Whether to use sliding expiration

### Cookie Options:
- `domain` (string): Cookie domain
- `path` (string): Cookie path
- `secure` (boolean): Whether to use secure cookies (HTTPS)
- `httpOnly` (boolean): Whether cookie is HTTP only
- `sameSite` (string): SameSite attribute ('lax', 'strict', 'none')

## Authentication Methods

### Email/Password Authentication
```typescript
// Sign up with email and password
const user = await authClient.signUp.email({
  email: "user@example.com",
  password: "securePassword",
  name: "John Doe",
});

// Sign in with email and password
const session = await authClient.signIn.email({
  email: "user@example.com",
  password: "securePassword",
});

// Reset password
await authClient.resetPassword.reset({
  token: "reset-token",
  newPassword: "newSecurePassword",
});

// Send password reset email
await authClient.resetPassword.sendResetLink({
  email: "user@example.com",
});
```

### Social Authentication
```typescript
// Sign in with Google
const googleAuthUrl = await authClient.signIn.social({
  provider: "google",
  callbackURL: "/dashboard",
});

// Sign in with GitHub
const githubAuthUrl = await authClient.signIn.social({
  provider: "github",
  callbackURL: "/dashboard",
});
```

## User Management

### User Fields
- `id` (string): Unique user identifier
- `email` (string): User's email address
- `emailVerified` (boolean): Whether email is verified
- `name` (string): User's display name
- `image` (string): User's profile image URL
- `createdAt` (Date): Account creation date
- `updatedAt` (Date): Last update date
- `disabled` (boolean): Whether account is disabled

### User Operations
```typescript
// Get current user session
const { session, user } = await authClient.getSession();

// Update user profile
const updatedUser = await authClient.updateUser({
  name: "New Name",
  image: "https://example.com/new-avatar.jpg",
});

// Change password
await authClient.changePassword({
  newPassword: "newSecurePassword",
  currentPassword: "currentPassword", // required if changing own password
});

// Delete account
await authClient.deleteAccount();
```

## Plugin System

### Available Plugins
- `organization`: Organization and team management
- `twoFactor`: Two-factor authentication
- `saml`: SAML single sign-on
- `admin`: Admin user management
- `account`: Additional account features
- `auditLog`: Audit logging capabilities

### Plugin Configuration:
```typescript
import { betterAuth } from "better-auth";
import { organization } from "better-auth/plugins";

export const auth = betterAuth({
  // ... other config
  plugins: [
    organization({
      allowUserToCreateOrganization: async (user) => {
        // Custom logic to allow/deny org creation
        return user.hasActiveSubscription;
      },
      defaultRole: "member",
      roles: ["owner", "admin", "member"],
    }),
  ],
});
```

## Client API

### Client Configuration
```typescript
import { createAuthClient } from "better-auth/client";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_BASE_URL || "http://localhost:3000",
  // Optional: custom fetch implementation
  fetch: customFetch,
  // Optional: plugins
  plugins: [
    // Add client-side plugins here
  ],
});
```

### Client Methods
- `signIn`: Authentication methods
- `signUp`: Registration methods
- `signOut`: Session termination
- `getSession`: Get current session
- `updateUser`: Update user profile
- `deleteAccount`: Delete user account
- `changePassword`: Password modification
- `resetPassword`: Password reset operations

## Middleware and Route Protection

### Next.js Middleware
```typescript
// middleware.ts
import { auth } from "@/lib/auth";

export default auth.middleware;

export const config = {
  matcher: ["/dashboard/:path*", "/api/auth/:path*"],
};
```

### Custom Middleware Configuration
```typescript
export default auth.middleware.extend({
  matcher: {
    // Include routes that need authentication
    includes: ["/dashboard", "/api/protected"],
    // Exclude routes from authentication
    excludes: ["/api/public", "/health"],
  },
});
```

## Environment Variables

### Required Variables
- `DATABASE_URL`: Database connection string
- `SECRET_KEY`: Secret key for signing sessions (32+ characters)

### Social Provider Variables
- `GOOGLE_CLIENT_ID`: Google OAuth client ID
- `GOOGLE_CLIENT_SECRET`: Google OAuth client secret
- `GITHUB_CLIENT_ID`: GitHub OAuth client ID
- `GITHUB_CLIENT_SECRET`: GitHub OAuth client secret

### Optional Variables
- `SITE_URL`: Public URL of your application
- `NEXT_PUBLIC_BASE_URL`: Base URL for client-side operations

## CLI Commands

### Database Migrations
```bash
# Run migrations
npx @better-auth/cli migrate

# Generate migration files
npx @better-auth/cli generate

# Check migration status
npx @better-auth/cli status
```

### Development Tools
```bash
# Get diagnostic info about your setup
npx @better-auth/cli info
```

## Error Handling

### Common Error Types
- `AUTHORIZATION_ERROR`: Authorization failed
- `VALIDATION_ERROR`: Input validation failed
- `DATABASE_ERROR`: Database operation failed
- `RATE_LIMIT_ERROR`: Rate limit exceeded
- `SESSION_EXPIRED`: Session has expired
- `USER_NOT_FOUND`: User does not exist

### Error Response Format
```json
{
  "error": {
    "message": "Error description",
    "code": "ERROR_CODE",
    "status": 400
  }
}
```

## Hooks and Extensibility

### Available Hooks
- `beforeAuth`: Runs before authentication
- `afterAuth`: Runs after successful authentication
- `beforeSession`: Runs before session creation
- `afterSession`: Runs after session creation
- `beforeUserCreate`: Runs before user creation
- `afterUserCreate`: Runs after user creation

### Hook Configuration
```typescript
export const auth = betterAuth({
  // ... other config
  hooks: {
    beforeUserCreate: async (user) => {
      // Modify user data before creation
      return {
        ...user,
        email: user.email.toLowerCase(),
      };
    },
    afterUserCreate: async (user) => {
      // Send welcome email, etc.
      await sendWelcomeEmail(user.email);
    },
  },
});
```