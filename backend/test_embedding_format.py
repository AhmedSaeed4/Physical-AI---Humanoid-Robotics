import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import from backend.database
sys.path.append(str(Path(__file__).parent))

import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Google Gemini API
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def test_embedding_format():
    """
    Test to understand the exact format of the embedding response
    """
    try:
        # Using the text-embedding-004 model as specified in the requirements
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=["This is a test sentence for embedding."],
            task_type="RETRIEVAL_DOCUMENT"
        )

        print("Type of result:", type(result))
        print("Result content:", result)

        if isinstance(result, dict):
            print("Keys in result:", result.keys())
            for key, value in result.items():
                print(f"Key '{key}': type={type(value)}, value={value}")
        elif isinstance(result, list):
            print("Length of result list:", len(result))
            for i, item in enumerate(result):
                print(f"Item {i}: type={type(item)}, value={item}")
        else:
            print("Result is neither dict nor list, it's:", type(result))

        return result
    except Exception as e:
        print(f"Error generating embedding: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    test_embedding_format()