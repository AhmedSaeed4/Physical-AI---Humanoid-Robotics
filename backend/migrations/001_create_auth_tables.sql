-- Better Auth required tables
-- Create the base auth tables, then add custom fields

-- Create user table
CREATE TABLE IF NOT EXISTS "user" (
    id TEXT PRIMARY KEY,
    name TEXT,
    email TEXT UNIQUE NOT NULL,
    "emailVerified" BOOLEAN DEFAULT FALSE,
    image TEXT,
    "createdAt" TIMESTAMP DEFAULT NOW(),
    "updatedAt" TIMESTAMP DEFAULT NOW()
);

-- Create session table
CREATE TABLE IF NOT EXISTS "session" (
    id TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token TEXT UNIQUE NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "ipAddress" TEXT,
    "userAgent" TEXT,
    "createdAt" TIMESTAMP DEFAULT NOW(),
    "updatedAt" TIMESTAMP DEFAULT NOW()
);

-- Create account table
CREATE TABLE IF NOT EXISTS "account" (
    id TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    "accountId" TEXT NOT NULL,
    "providerId" TEXT NOT NULL,
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "accessTokenExpiresAt" TIMESTAMP,
    "refreshTokenExpiresAt" TIMESTAMP,
    scope TEXT,
    password TEXT,
    "createdAt" TIMESTAMP DEFAULT NOW(),
    "updatedAt" TIMESTAMP DEFAULT NOW()
);

-- Create verification table
CREATE TABLE IF NOT EXISTS "verification" (
    id TEXT PRIMARY KEY,
    identifier TEXT NOT NULL,
    value TEXT NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "createdAt" TIMESTAMP DEFAULT NOW(),
    "updatedAt" TIMESTAMP DEFAULT NOW()
);

-- Add custom fields to the user table for learning preferences
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "educationLevel" TEXT;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "programmingExperience" TEXT;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "roboticsBackground" TEXT;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "softwareBackground" TEXT;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "hardwareBackground" TEXT;

-- Create indexes for the custom fields
CREATE INDEX IF NOT EXISTS "idx_user_education_level" ON "user" ("educationLevel");
CREATE INDEX IF NOT EXISTS "idx_user_programming_experience" ON "user" ("programmingExperience");
CREATE INDEX IF NOT EXISTS "idx_user_robotics_background" ON "user" ("roboticsBackground");