# Gemini LLM Integration with OpenAI Agents SDK

## Overview
This guide explains how to configure and use Google's Gemini models with the OpenAI Agents SDK by leveraging the OpenAI-compatible API endpoint. This allows you to use Gemini models (like gemini-2.5-flash) as drop-in replacements for OpenAI models.

## Prerequisites
- OpenAI Agents SDK installed (`uv add openai-agents`)
- Google AI API key for Gemini access
- Python environment with the `python-dotenv` package

## Setup Process

### 1. Environment Configuration
Create a `.env` file in your project root with your Gemini API key:
```
GEMINI_API_KEY=your_actual_api_key_here
```

### 2. Installation Requirements
```bash
# Install the OpenAI Agents SDK
uv add openai-agents

# Install python-dotenv for environment variable management
uv add python-dotenv
```

### 3. Gemini Configuration Code
Here's the complete configuration code for using Gemini with OpenAI Agents SDK:

```python
import os
from dotenv import load_dotenv
from agents import (
    Agent, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI
)

# Load environment variables from .env file
load_dotenv()

# Get the API key from environment variables
API_KEY = os.environ["GEMINI_API_KEY"]
if not API_KEY:
    raise ValueError("GEMINI_API_KEY not found in .env file")

# Create the OpenAI client with Gemini's OpenAI-compatible endpoint
client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Define the Gemini model without temperature here
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

# Configure the run settings
config = RunConfig(
    model=model,
    model_provider=client,
)

# Create an agent using the Gemini configuration
agent = Agent(
    name="gemini-agent",
    system_prompt="You are a helpful assistant powered by Google's Gemini model.",
    config=config
)
```

## Supported Gemini Models
- `gemini-2.5-flash` - Fast, efficient model for most tasks
- `gemini-2.5-pro` - More capable model for complex tasks
- `gemini-1.5-pro` - Previous generation high-quality model
- `gemini-1.0-pro` - Original Gemini Pro model

## Complete Working Example

```python
import os
from dotenv import load_dotenv
from agents import Agent, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI

# Load environment variables
load_dotenv()

# Validate API key exists
API_KEY = os.environ.get("GEMINI_API_KEY")
if not API_KEY:
    raise ValueError("GEMINI_API_KEY not found in .env file")

# Initialize Gemini-compatible client
client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Configure the Gemini model
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

# Set up the run configuration
config = RunConfig(
    model=model,
    model_provider=client,
)

# Create and use the agent
agent = Agent(
    name="gemini-agent",
    system_prompt="You are a helpful assistant powered by Google's Gemini model.",
    config=config
)

# Example usage
async def run_gemini_agent():
    response = await agent.run("Hello! Can you help me with a question?")
    return response
```

## Key Configuration Points

### Base URL
The base URL `https://generativelanguage.googleapis.com/v1beta/openai/` is crucial - it enables the OpenAI-compatible interface for Gemini.

### Model Selection
- Use `gemini-2.5-flash` for most applications (fast and cost-effective)
- Use `gemini-2.5-pro` for more complex reasoning tasks
- Adjust model selection based on your specific needs

### Client Configuration
The `AsyncOpenAI` client is configured with:
- Your Gemini API key
- The OpenAI-compatible Gemini endpoint
- Standard OpenAI client interface for compatibility

## Troubleshooting

### Common Issues
1. **API Key Error**: Ensure `GEMINI_API_KEY` is properly set in your `.env` file
2. **Model Not Found**: Verify the model name is correct and available in your region
3. **Connection Issues**: Check that the base URL is correctly formatted

### Error Handling
```python
try:
    response = await agent.run("Hello!")
except Exception as e:
    print(f"Error occurred: {e}")
    if "API key" in str(e):
        print("Please check your GEMINI_API_KEY in the .env file")
```

## Benefits of Using Gemini with OpenAI Agents SDK
- Access to Google's advanced Gemini models
- Compatibility with existing OpenAI Agents SDK workflows
- Cost-effective alternatives to OpenAI models
- Potential performance advantages for specific use cases