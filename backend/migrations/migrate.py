import os
import sys
import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv
import glob

# Load environment variables
load_dotenv()

def get_db_connection():
    """Create a database connection using environment variables"""
    conn = psycopg2.connect(
        dsn=os.getenv('DATABASE_URL'),
        sslmode='require'
    )
    return conn

def run_migrations():
    """Run all migration files in order"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        # Create migrations table if it doesn't exist
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS migrations (
                id SERIAL PRIMARY KEY,
                migration_name VARCHAR(255) UNIQUE NOT NULL,
                executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        conn.commit()

        # Get already executed migrations
        cursor.execute("SELECT migration_name FROM migrations ORDER BY migration_name")
        executed_migrations = [row['migration_name'] for row in cursor.fetchall()]

        # Get all migration files
        migration_files = glob.glob("migrations/*.sql")
        migration_files.sort()  # Sort to ensure order

        print(f"Found {len(migration_files)} migration files")

        for migration_file in migration_files:
            migration_name = os.path.basename(migration_file)

            # Skip if already executed
            if migration_name in executed_migrations:
                print(f"Skipping {migration_name} (already executed)")
                continue

            print(f"Executing migration: {migration_name}")

            # Read and execute migration
            with open(migration_file, 'r') as f:
                sql = f.read()

            cursor.execute(sql)

            # Record migration execution
            cursor.execute(
                "INSERT INTO migrations (migration_name) VALUES (%s)",
                (migration_name,)
            )

            conn.commit()
            print(f"Migration {migration_name} completed successfully")

        cursor.close()
        conn.close()

        print("All migrations completed!")

    except Exception as e:
        print(f"Error running migrations: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    run_migrations()