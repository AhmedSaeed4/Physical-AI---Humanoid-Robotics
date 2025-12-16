import uuid
import json
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from contextlib import contextmanager

import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv
import os

from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page
from chatkit.types import AssistantMessageItem, UserMessageItem

# Load environment variables
load_dotenv()


class NeonStore(Store[dict]):
    """PostgreSQL-based store for ChatKit using Neon database"""

    def __init__(self, connection_string: Optional[str] = None) -> None:
        """Initialize the NeonStore with database connection"""
        self.connection_string = connection_string or os.getenv('DATABASE_URL')
        if not self.connection_string:
            raise ValueError("DATABASE_URL environment variable is required")

    @contextmanager
    def get_connection(self):
        """Context manager for database connections"""
        try:
            conn = psycopg2.connect(
                dsn=self.connection_string,
                sslmode='require'
            )
            yield conn
        except psycopg2.OperationalError as e:
            # Handle connection errors (e.g., database unavailable)
            raise ValueError(f"Database connection failed: {str(e)}")
        except Exception as e:
            # Handle other database errors
            if 'conn' in locals():
                conn.rollback()
            raise e
        finally:
            if 'conn' in locals() and conn:
                conn.close()

    def generate_thread_id(self, context: dict) -> str:
        """Generate unique thread ID"""
        return f"thread_{uuid.uuid4().hex[:12]}"

    def generate_item_id(self, item_type: str, thread: ThreadMetadata, context: dict) -> str:
        """Generate unique item ID with type prefix"""
        return f"{item_type}_{uuid.uuid4().hex[:12]}"

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """Load existing thread or create new one"""
        # Extract user ID from context for data isolation
        user_id = self._get_user_id_from_context(context)

        try:
            with self.get_connection() as conn:
                cursor = conn.cursor(cursor_factory=RealDictCursor)

                # Load thread with user ID check for data isolation
                cursor.execute("""
                    SELECT id, metadata, "createdAt", "updatedAt"
                    FROM chatkit_thread
                    WHERE id = %s AND "userId" = %s
                """, (thread_id, user_id))

                row = cursor.fetchone()

                if row:
                    # Convert metadata from JSONB to dict if needed
                    metadata = row['metadata']
                    if isinstance(metadata, str):
                        metadata = json.loads(metadata)

                    return ThreadMetadata(
                        id=row['id'],
                        metadata=metadata or {},
                        created_at=row['createdAt']
                    )
                else:
                    # Create new thread if it doesn't exist
                    now = datetime.now(timezone.utc)
                    new_thread = ThreadMetadata(
                        id=thread_id,
                        created_at=now,
                        metadata={
                            "title": f"Chat {now.strftime('%Y-%m-%d %H:%M')}",  # Default title with timestamp
                            "creationDate": now.isoformat(),
                            "lastMessageDate": now.isoformat()
                        }
                    )

                    # Insert the new thread
                    cursor.execute("""
                        INSERT INTO chatkit_thread (id, "userId", metadata, "createdAt", "updatedAt")
                        VALUES (%s, %s, %s, %s, %s)
                    """, (
                        thread_id,
                        user_id,
                        json.dumps(new_thread.metadata),
                        now,
                        now
                    ))
                    conn.commit()

                    return new_thread
        except ValueError as e:
            # Re-raise ValueError (like connection errors) as they are expected
            raise e
        except Exception as e:
            # Log the error (in a real app, you'd use proper logging)
            print(f"Error in load_thread: {str(e)}")
            # Raise a generic error that can be handled by the application
            raise ValueError(f"Failed to load thread: {str(e)}")

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        """Save thread metadata"""
        user_id = self._get_user_id_from_context(context)

        # Update the lastMessageDate in metadata to current time
        updated_metadata = thread.metadata.copy() if thread.metadata else {}
        updated_metadata["lastMessageDate"] = datetime.now(timezone.utc).isoformat()

        try:
            with self.get_connection() as conn:
                cursor = conn.cursor()

                # Update thread metadata
                cursor.execute("""
                    UPDATE chatkit_thread
                    SET metadata = %s, "updatedAt" = %s
                    WHERE id = %s AND "userId" = %s
                """, (
                    json.dumps(updated_metadata),
                    datetime.now(timezone.utc),
                    thread.id,
                    user_id
                ))

                # If no rows were updated, the thread doesn't exist for this user
                if cursor.rowcount == 0:
                    # Create the thread if it doesn't exist
                    cursor.execute("""
                        INSERT INTO chatkit_thread (id, "userId", metadata, "createdAt", "updatedAt")
                        VALUES (%s, %s, %s, %s, %s)
                        ON CONFLICT (id) DO UPDATE SET
                            metadata = EXCLUDED.metadata,
                            "updatedAt" = EXCLUDED."updatedAt"
                    """, (
                        thread.id,
                        user_id,
                        json.dumps(updated_metadata),
                        thread.created_at or datetime.now(timezone.utc),
                        datetime.now(timezone.utc)
                    ))

                conn.commit()
        except ValueError as e:
            # Re-raise ValueError (like connection errors) as they are expected
            raise e
        except Exception as e:
            # Log the error (in a real app, you'd use proper logging)
            print(f"Error in save_thread: {str(e)}")
            # Raise a generic error that can be handled by the application
            raise ValueError(f"Failed to save thread: {str(e)}")

    async def load_threads(self, limit: int, after: str | None, order: str, context: dict) -> Page[ThreadMetadata]:
        """List all threads for the current user with pagination"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Base query for threads belonging to the user
            query = """
                SELECT id, metadata, "createdAt", "updatedAt"
                FROM chatkit_thread
                WHERE "userId" = %s
            """
            params = [user_id]

            # Add cursor-based pagination if 'after' is provided
            if after:
                query += " AND id > %s"
                params.append(after)

            # Add ordering and limit - order by updated_at for most recent activity first
            order_direction = "DESC" if order == "desc" else "ASC"
            query += f' ORDER BY "updatedAt" {order_direction} LIMIT %s'
            params.append(limit + 1)  # Fetch one extra to check for has_more

            cursor.execute(query, params)
            rows = cursor.fetchall()

            # Check if there are more results
            has_more = len(rows) > limit
            threads = rows[:limit] if has_more else rows

            # Convert to ThreadMetadata objects
            thread_objects = []
            for row in threads:
                metadata = row['metadata']
                if isinstance(metadata, str):
                    metadata = json.loads(metadata)

                thread_objects.append(ThreadMetadata(
                    id=row['id'],
                    metadata=metadata or {},
                    created_at=row['createdAt']
                ))

            return Page(data=thread_objects, has_more=has_more)

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        """Delete thread and all its items"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor()

            # Delete thread and all related items (CASCADE will handle items)
            cursor.execute("""
                DELETE FROM chatkit_thread
                WHERE id = %s AND "userId" = %s
            """, (thread_id, user_id))

            conn.commit()

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict,
    ) -> Page[ThreadItem]:
        """Load thread items with pagination"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Verify thread belongs to user for data isolation
            cursor.execute("""
                SELECT id FROM chatkit_thread WHERE id = %s AND "userId" = %s
            """, (thread_id, user_id))

            if not cursor.fetchone():
                # Thread doesn't exist for this user
                return Page(data=[], has_more=False)

            # Query items from the thread
            query = """
                SELECT id, type, content, "createdAt", "threadId"
                FROM chatkit_thread_item
                WHERE "threadId" = %s
            """
            params = [thread_id]

            # Add cursor-based pagination if 'after' is provided
            if after:
                query += " AND id > %s"
                params.append(after)

            # Add ordering and limit
            order_direction = "DESC" if order == "desc" else "ASC"
            query += f' ORDER BY "createdAt" {order_direction} LIMIT %s'
            params.append(limit + 1)  # Fetch one extra to check for has_more

            cursor.execute(query, params)
            rows = cursor.fetchall()

            # Check if there are more results
            has_more = len(rows) > limit
            items = rows[:limit] if has_more else rows

            # Convert to ThreadItem objects using model_validate
            item_objects = []
            for row in items:
                content = row['content']
                if isinstance(content, str):
                    content = json.loads(content)

                # Get the stored type
                stored_type = row['type'].lower() if row['type'] else 'unknown'
                
                try:
                    # Try to reconstruct using model_validate with stored data
                    if stored_type in ['assistantmessageitem', 'assistant', 'message']:
                        # For AssistantMessageItem, use model_validate if full data available
                        if isinstance(content, dict) and 'thread_id' in content:
                            item_objects.append(AssistantMessageItem.model_validate(content))
                        else:
                            item_content = content.get('content', content) if isinstance(content, dict) else content
                            item_objects.append(AssistantMessageItem(
                                id=row['id'],
                                thread_id=thread_id,  # Required field
                                created_at=row['createdAt'],
                                content=item_content
                            ))
                    elif stored_type in ['usermessageitem', 'user']:
                        # For UserMessageItem, use model_validate to reconstruct with all fields
                        if isinstance(content, dict) and 'thread_id' in content:
                            # Full stored data available - use model_validate
                            item_objects.append(UserMessageItem.model_validate(content))
                        else:
                            # Partial data - construct with required defaults
                            item_content = content.get('content', []) if isinstance(content, dict) else []
                            item_objects.append(UserMessageItem(
                                id=row['id'],
                                thread_id=thread_id,  # Use the function parameter
                                created_at=row['createdAt'],
                                content=item_content if isinstance(item_content, list) else [],
                                inference_options={}  # Required field with default
                            ))
                    else:
                        # Unknown type - try to reconstruct as best as possible
                        # Skip unknown items to avoid breaking the chat
                        print(f"Skipping unknown item type: {stored_type}")
                        continue
                except Exception as e:
                    print(f"Error reconstructing item {row['id']}: {e}")
                    # Skip items that can't be reconstructed
                    continue

            return Page(data=item_objects, has_more=has_more)

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Add or update item in thread"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor()

            # Verify thread belongs to user
            cursor.execute("""
                SELECT id FROM chatkit_thread WHERE id = %s AND "userId" = %s
            """, (thread_id, user_id))

            if not cursor.fetchone():
                # Thread doesn't exist for this user, create it
                new_thread = ThreadMetadata(
                    id=thread_id,
                    created_at=datetime.now(timezone.utc),
                    metadata={}
                )

                cursor.execute("""
                    INSERT INTO chatkit_thread (id, "userId", metadata, "createdAt", "updatedAt")
                    VALUES (%s, %s, %s, %s, %s)
                """, (
                    thread_id,
                    user_id,
                    json.dumps({}),
                    new_thread.created_at,
                    new_thread.created_at
                ))

            # Store the complete item using model_dump to preserve all required fields
            try:
                # Use model_dump to get the complete item state
                if hasattr(item, 'model_dump'):
                    item_data = item.model_dump()
                else:
                    item_data = {
                        'id': item.id,
                        'content': getattr(item, 'content', {}),
                        'created_at': str(getattr(item, 'created_at', datetime.now(timezone.utc)))
                    }
                
                # Ensure created_at is serializable
                if 'created_at' in item_data and hasattr(item_data['created_at'], 'isoformat'):
                    item_data['created_at'] = item_data['created_at'].isoformat()
                
                content_json = json.dumps(item_data, default=str)
            except Exception as e:
                print(f"Error serializing item: {e}")
                # Fallback to basic serialization
                content_json = json.dumps({
                    'id': item.id,
                    'type': type(item).__name__,
                    'content': [],
                    'created_at': datetime.now(timezone.utc).isoformat()
                })

            # Insert or update the item
            cursor.execute("""
                INSERT INTO chatkit_thread_item (id, "threadId", type, content, "createdAt")
                VALUES (%s, %s, %s, %s, %s)
                ON CONFLICT (id)
                DO UPDATE SET
                    type = EXCLUDED.type,
                    content = EXCLUDED.content,
                    "createdAt" = EXCLUDED."createdAt"
            """, (
                item.id,
                thread_id,
                type(item).__name__.lower(),  # Store type as lowercase
                content_json,
                getattr(item, 'created_at', datetime.now(timezone.utc))
            ))

            conn.commit()

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Alias for add_thread_item"""
        await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        """Load single item by ID"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            # Verify thread belongs to user and get the item
            cursor.execute("""
                SELECT i.id, i.type, i.content, i."createdAt"
                FROM chatkit_thread_item i
                JOIN chatkit_thread t ON i."threadId" = t.id
                WHERE i.id = %s AND i."threadId" = %s AND t."userId" = %s
            """, (item_id, thread_id, user_id))

            row = cursor.fetchone()
            if not row:
                raise ValueError(f"Item {item_id} not found in thread {thread_id}")

            content = row['content']
            if isinstance(content, str):
                content = json.loads(content)

            # Create appropriate item type based on the type field
            stored_type = row['type'].lower() if row['type'] else 'unknown'
            item_content = content.get('content', content) if isinstance(content, dict) else content
            
            if stored_type in ['assistantmessageitem', 'assistant', 'message']:
                # For AssistantMessageItem, use model_validate if full data available
                if isinstance(content, dict) and 'thread_id' in content:
                    return AssistantMessageItem.model_validate(content)
                else:
                    return AssistantMessageItem(
                        id=row['id'],
                        thread_id=thread_id,  # Required field
                        created_at=row['createdAt'],
                        content=item_content
                    )
            else:
                # For UserMessageItem, use model_validate if full data available
                if isinstance(content, dict) and 'thread_id' in content:
                    return UserMessageItem.model_validate(content)
                else:
                    return UserMessageItem(
                        id=row['id'],
                        thread_id=thread_id,  # Use the thread_id from the function parameter
                        created_at=row['createdAt'],
                        content=item_content if isinstance(item_content, list) else [],
                        inference_options={}
                    )

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        """Delete single item"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor()

            # Delete item only if it belongs to the user's thread
            cursor.execute("""
                DELETE FROM chatkit_thread_item
                WHERE id = %s
                AND "threadId" IN (
                    SELECT id FROM chatkit_thread WHERE id = %s AND "userId" = %s
                )
            """, (item_id, thread_id, user_id))

            conn.commit()

    # ==================== Attachment Operations ====================

    async def save_attachment(self, attachment: Any, context: dict) -> None:
        """Save attachment to database"""
        user_id = self._get_user_id_from_context(context)

        with self.get_connection() as conn:
            cursor = conn.cursor()

            # Convert attachment to JSON
            attachment_json = json.dumps({
                'id': attachment.id,
                'data': getattr(attachment, 'data', {}),
                'metadata': getattr(attachment, 'metadata', {})
            })

            cursor.execute("""
                INSERT INTO chatkit_attachment (id, "threadId", content, "createdAt")
                VALUES (%s, %s, %s, %s)
                ON CONFLICT (id)
                DO UPDATE SET
                    content = EXCLUDED.content,
                    "createdAt" = EXCLUDED."createdAt"
            """, (
                attachment.id,
                getattr(attachment, 'threadId', None),
                attachment_json,
                datetime.now(timezone.utc)
            ))

            conn.commit()

    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        """Load attachment by ID"""
        with self.get_connection() as conn:
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            cursor.execute("""
                SELECT id, "threadId", content, "createdAt"
                FROM chatkit_attachment
                WHERE id = %s
            """, (attachment_id,))

            row = cursor.fetchone()
            if not row:
                raise ValueError(f"Attachment {attachment_id} not found")

            content = row['content']
            if isinstance(content, str):
                content = json.loads(content)

            # Return the attachment data as a simple object
            attachment = type('Attachment', (), {})()
            attachment.id = row['id']
            attachment.threadId = row['threadId']
            attachment.data = content.get('data', {})
            attachment.metadata = content.get('metadata', {})
            attachment.created_at = row['createdAt']

            return attachment

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        """Delete attachment"""
        with self.get_connection() as conn:
            cursor = conn.cursor()

            cursor.execute("""
                DELETE FROM chatkit_attachment
                WHERE id = %s
            """, (attachment_id,))

            conn.commit()

    def _get_user_id_from_context(self, context: dict) -> str:
        """Extract user ID from context for data isolation"""
        user = context.get('user', {})
        user_id = user.get('id') if isinstance(user, dict) else getattr(user, 'id', None)

        if not user_id:
            raise ValueError("User ID is required for data isolation. Please ensure authentication context is provided.")

        return user_id