# Better Auth Skill

## Overview
This skill provides comprehensive knowledge and capabilities for working with Better Auth, a framework-agnostic authentication and authorization library for TypeScript. It enables Claude to help users implement secure authentication, configure plugins, manage user sessions, and integrate with various frameworks and databases.

## Purpose
This skill allows Claude to:
- Explain Better Auth concepts and architecture
- Help create and configure Better Auth instances
- Implement authentication flows (email/password, social, etc.)
- Configure and use plugins (organization, 2FA, SSO, etc.)
- Integrate with different frameworks (Next.js, Remix, etc.)
- Manage database adapters and migrations
- Troubleshoot common authentication issues

## Quick Start
When a user needs help with Better Auth:

1. First, understand their specific authentication requirements
2. Suggest appropriate setup based on their framework (Next.js, Remix, etc.)
3. Provide installation instructions and basic configuration
4. Guide them through authentication flows and security considerations

## Core Capabilities

### Authentication Setup
- Initialize Better Auth instances with various configurations
- Set up email/password authentication
- Configure social login providers
- Implement phone number authentication

### Plugin Management
- Install and configure plugins (organization, 2FA, SSO, etc.)
- Extend functionality with custom plugins
- Handle plugin-specific database migrations
- Manage plugin hooks and customization

### Framework Integration
- Integrate with Next.js, Remix, Astro, and other frameworks
- Set up middleware for route protection
- Configure client-side authentication
- Handle framework-specific patterns and best practices

### Database Management
- Configure database adapters (Prisma, Drizzle, etc.)
- Run database migrations using CLI
- Manage user schema and relationships
- Handle multi-tenant setups

### Security Features
- Implement session management
- Configure secure password hashing
- Set up two-factor authentication
- Manage rate limiting and security policies

## Installation
```bash
# Using npm
npm install better-auth

# Using pnpm
pnpm add better-auth

# Using yarn
yarn add better-auth
```

For CLI tools:
```bash
# Install CLI globally
npm install -g @better-auth/cli

# Or run with npx
npx @better-auth/cli
```

## Usage Examples
- "How do I set up Better Auth with Next.js?"
- "Show me how to add social login providers"
- "Help me configure the organization plugin"
- "I need to implement two-factor authentication"
- "How do I protect routes with Better Auth middleware?"
- "Can you help me migrate from Clerk to Better Auth?"

## Advanced Features
- Plugin architecture for extensibility
- Framework-agnostic design
- TypeScript-first development
- Comprehensive API and client libraries
- Built-in security best practices
- Flexible database adapter system