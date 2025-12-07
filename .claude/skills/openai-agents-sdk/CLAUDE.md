# OpenAI Agents SDK for Claude

## When to Use This Skill
Use this skill when users ask about:
- OpenAI Agents SDK for Python
- Building multi-agent systems
- Creating AI agents with tools and guardrails
- Real-time voice agents
- Agent handoffs and orchestration
- Function tools and custom models

## How to Respond
When a user asks about OpenAI Agents SDK:

1. First, identify their specific use case or problem
2. Provide relevant code examples using the SDK
3. Explain the concepts clearly with practical applications
4. Include best practices and common patterns
5. Suggest appropriate architecture based on their needs

## Key Concepts to Explain

### Agents
- Agents are the fundamental building blocks
- They can interact with tools and models
- Configure with system prompts and model settings
- Manage agent lifecycle and sessions

### Tools
- Use `@function_tool` decorator to create function tools
- The SDK automatically parses function signatures and docstrings
- Handle errors with failure_error_function parameter
- Support file and image inputs/outputs

### Handoffs
- Enable transfer of conversations between specialized agents
- Design workflows with clear handoff points
- Maintain context during handoffs
- Implement proper error handling during transitions

### Real-time Voice
- Use RealtimeRunner for voice conversations
- Handle audio streaming and interruptions
- Process speech-to-text and text-to-speech
- Maintain low-latency interactions

## Common Code Patterns

### Basic Agent Creation
```python
from openai_agents import Agent

agent = Agent(
    name="assistant",
    system_prompt="You are a helpful assistant...",
    model="gpt-4o"
)
```

### Function Tool Creation
```python
from openai_agents import function_tool

@function_tool
def get_weather(location: str) -> str:
    """Get current weather for a location."""
    # Implementation here
    return weather_data
```

### Real-time Voice Processing
```python
from openai_agents import RealtimeRunner

runner = RealtimeRunner(model="gpt-4o-realtime-preview")
session = await runner.run()

async for event in session:
    # Process events like agent_start, tool_call, audio, etc.
```

## Best Practices
- Start with simple examples before complex multi-agent systems
- Use appropriate model settings for the task
- Implement proper error handling and fallbacks
- Consider rate limits and cost implications
- Test thoroughly with realistic scenarios

## Troubleshooting
If users encounter issues:
- Check SDK installation and version compatibility
- Verify API keys and authentication
- Review tool function signatures and types
- Validate model names and availability
- Ensure proper async/await patterns in code

## Environment and Installation Notes
- Recommend using uv for package management: `uv add openai-agents`
- For voice capabilities: `uv add 'openai-agents[voice]'`
- Create virtual environments with `python -m venv .venv` before installing
- uv is often faster than pip and handles dependencies more efficiently

## Gemini LLM Integration
- For using Google's Gemini models with the SDK, see gemini_integration.md
- Requires setting up GEMINI_API_KEY in .env file
- Uses OpenAI-compatible endpoint: https://generativelanguage.googleapis.com/v1beta/openai/
- Configure with AsyncOpenAI client and OpenAIChatCompletionsModel