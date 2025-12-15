import { betterAuth } from 'better-auth';
import { pool } from './database/neon';

export const auth = betterAuth({
  database: pool,
  // debug: true, // Uncomment if debug supports it, but let's try provider change first
  secret: process.env.JWT_SECRET || 'your-default-jwt-secret-change-in-production',
  // Allow requests from frontend
  trustedOrigins: [
    process.env.FRONTEND_URL || 'http://localhost:3000',
    process.env.BACKEND_URL || 'http://localhost:8000',
  ],
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || "",
      clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
    },
  },
  user: {
    // Custom fields for learning preferences
    additionalFields: {
      educationLevel: {
        type: 'string',
        required: false,
      },
      programmingExperience: {
        type: 'string',
        required: false,
      },
      roboticsBackground: {
        type: 'string',
        required: false,
      },
      softwareBackground: {
        type: 'string',
        required: false,
      },
      hardwareBackground: {
        type: 'string',
        required: false,
      },
    },
  },
  session: {
    expiresIn: 24 * 60 * 60, // 24 hours
    updateAge: 24 * 60 * 60, // 24 hours
    cookieCache: {
      enabled: false,
      maxAge: 5 * 60, // 5 minutes
    },
  },
  advanced: {
    // Allow cookies to work across different ports (localhost:3000 -> localhost:3001)
    crossSubDomainCookies: {
      enabled: true,
      domain: 'localhost',
    },
    defaultCookieAttributes: {
      sameSite: 'lax',
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      path: '/',
    },
  },
  email: {
    enabled: true,
    from: process.env.EMAIL_FROM || 'noreply@example.com',
    provider: {
      type: 'smtp',
      host: process.env.SMTP_HOST || 'smtp.gmail.com',
      port: parseInt(process.env.SMTP_PORT || '587'),
      auth: {
        user: process.env.SMTP_USER || '',
        pass: process.env.SMTP_PASS || '',
      },
    },
  },
});