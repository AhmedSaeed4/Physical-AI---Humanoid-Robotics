# Better Auth for Claude

## When to Use This Skill
Use this skill when users ask about:
- Setting up Better Auth authentication
- Configuring plugins (organization, 2FA, SSO, etc.)
- Integrating with frameworks (Next.js, Remix, Astro, etc.)
- Database configuration and migrations
- Social login providers setup
- Session management and security
- Custom authentication flows
- Migration from other auth providers (Clerk, etc.)

## How to Respond
When a user asks about Better Auth:

1. First, identify their specific authentication requirements
2. Provide relevant code examples and configuration snippets
3. Explain concepts clearly with practical applications
4. Include security best practices and common patterns
5. Suggest appropriate solutions based on their framework and needs

## Key Concepts to Explain

### Authentication Setup
- Better Auth is initialized with the `betterAuth()` function
- Configure providers (email/password, social, etc.)
- Set up database adapters for user storage
- Handle environment variables for secrets

### Plugins Architecture
- Plugins extend Better Auth functionality
- Use `plugins: [pluginName()]` in auth configuration
- Run migrations for plugin-specific database changes
- Access plugin features through client/server APIs

### Framework Integration
- Each framework has specific integration patterns
- Set up API routes/endpoints for auth handlers
- Configure client-side libraries for frontend integration
- Use framework-specific middleware for route protection

## Common Code Patterns

### Basic Auth Setup
```typescript
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "sqlite", // or "postgresql", "mysql"
    url: process.env.DATABASE_URL!,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    },
  },
});
```

### Plugin Configuration
```typescript
import { betterAuth } from "better-auth";
import { organization } from "better-auth/plugins";

export const auth = betterAuth({
  plugins: [
    organization({
      allowUserToCreateOrganization: async (user) => {
        // Custom logic to allow/deny org creation
        return true;
      }
    })
  ]
});
```

### Client Setup
```typescript
import { createAuthClient } from "better-auth/client";
import { organizationClient } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  plugins: [
    organizationClient()
  ]
});
```

### Route Protection (Next.js)
```typescript
// middleware.ts
import { auth } from "@/lib/auth";

export default auth.middleware;

export const config = {
  matcher: ["/dashboard/:path*", "/api/auth/:path*"],
};
```

## Best Practices
- Always use environment variables for sensitive data
- Implement proper error handling for auth operations
- Use plugins to extend functionality instead of custom code when possible
- Follow security best practices for session management
- Run database migrations after installing plugins
- Test authentication flows thoroughly

## Troubleshooting
If users encounter issues:
- Check that environment variables are properly set
- Verify database connection and adapter configuration
- Ensure CLI is installed for running migrations: `npx @better-auth/cli migrate`
- Review plugin-specific documentation for setup requirements
- Confirm framework integration follows recommended patterns
- Check for version compatibility between packages

## Migration Notes
- For Clerk migration, use the official migration guide
- All active sessions will be invalidated during migration
- Plan for data mapping between auth systems
- Test thoroughly in a staging environment first