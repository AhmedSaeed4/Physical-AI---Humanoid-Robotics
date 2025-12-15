-- Create chat history table for storing user conversations
CREATE TABLE IF NOT EXISTS "chat_history" (
    "id" SERIAL PRIMARY KEY,
    "userId" TEXT NOT NULL,
    "message" TEXT NOT NULL,
    "response" TEXT NOT NULL,
    "selectedText" TEXT,
    "createdAt" TIMESTAMP DEFAULT NOW(),

    -- Foreign key constraint to user table
    CONSTRAINT fk_user FOREIGN KEY ("userId") REFERENCES "user"("id") ON DELETE CASCADE
);

-- Create indexes for efficient querying
CREATE INDEX IF NOT EXISTS "idx_chat_history_user_id" ON "chat_history" ("userId");
CREATE INDEX IF NOT EXISTS "idx_chat_history_created_at" ON "chat_history" ("createdAt" DESC);