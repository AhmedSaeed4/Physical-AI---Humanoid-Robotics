-- Chat History table (custom)
CREATE TABLE IF NOT EXISTS "chat_history" (
    "id" SERIAL PRIMARY KEY,
    "userId" TEXT NOT NULL,
    "message" TEXT NOT NULL,
    "response" TEXT NOT NULL,
    "selectedText" TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Add foreign key constraint to user table
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1
        FROM information_schema.table_constraints
        WHERE constraint_name = 'chat_history_user_id_fkey'
        AND table_name = 'chat_history'
    ) THEN
        ALTER TABLE "chat_history"
        ADD CONSTRAINT "chat_history_user_id_fkey"
        FOREIGN KEY ("userId") REFERENCES "user"("id") ON DELETE CASCADE;
    END IF;
END $$;

-- Indexes for performance
CREATE INDEX IF NOT EXISTS "idx_chat_history_user" ON "chat_history" ("userId");
CREATE INDEX IF NOT EXISTS "idx_chat_history_created" ON "chat_history" ("createdAt" DESC);
CREATE INDEX IF NOT EXISTS "idx_chat_history_user_created" ON "chat_history" ("userId", "createdAt" DESC);