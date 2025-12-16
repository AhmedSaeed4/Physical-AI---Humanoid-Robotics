-- Create ChatKit tables for persistent storage following the ChatKit store interface
-- This replaces the in-memory MemoryStore with Neon PostgreSQL storage

-- Create chatkit_thread table to store chat thread information
CREATE TABLE IF NOT EXISTS "chatkit_thread" (
    "id" TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL,
    "metadata" JSONB DEFAULT '{}',
    "createdAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key constraint to user table for data isolation
    CONSTRAINT fk_chatkit_thread_user FOREIGN KEY ("userId") REFERENCES "user"("id") ON DELETE CASCADE
);

-- Create chatkit_thread_item table to store individual messages
CREATE TABLE IF NOT EXISTS "chatkit_thread_item" (
    "id" TEXT PRIMARY KEY,
    "threadId" TEXT NOT NULL,
    "type" TEXT NOT NULL,
    "content" JSONB NOT NULL,
    "createdAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key constraint to chatkit_thread table
    CONSTRAINT fk_chatkit_thread_item_thread FOREIGN KEY ("threadId") REFERENCES "chatkit_thread"("id") ON DELETE CASCADE
);

-- Create chatkit_attachment table (defined for interface compatibility but not used)
CREATE TABLE IF NOT EXISTS "chatkit_attachment" (
    "id" TEXT PRIMARY KEY,
    "threadId" TEXT,
    "content" JSONB NOT NULL,
    "createdAt" TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Foreign key constraint to chatkit_thread table (optional reference)
    CONSTRAINT fk_chatkit_attachment_thread FOREIGN KEY ("threadId") REFERENCES "chatkit_thread"("id") ON DELETE CASCADE
);

-- Create indexes for efficient querying
CREATE INDEX IF NOT EXISTS "idx_chatkit_thread_user" ON "chatkit_thread" ("userId");
CREATE INDEX IF NOT EXISTS "idx_chatkit_thread_created" ON "chatkit_thread" ("createdAt" DESC);
CREATE INDEX IF NOT EXISTS "idx_chatkit_item_thread" ON "chatkit_thread_item" ("threadId");
CREATE INDEX IF NOT EXISTS "idx_chatkit_item_created" ON "chatkit_thread_item" ("createdAt");
CREATE INDEX IF NOT EXISTS "idx_chatkit_attachment_thread" ON "chatkit_attachment" ("threadId");

-- Create a trigger function to update the updatedAt field in chatkit_thread
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updatedAt = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Create a trigger to automatically update the updatedAt field
CREATE TRIGGER update_chatkit_thread_updated_at
    BEFORE UPDATE ON "chatkit_thread"
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();