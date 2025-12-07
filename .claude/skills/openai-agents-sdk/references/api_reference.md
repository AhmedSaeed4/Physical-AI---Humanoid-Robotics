# OpenAI Agents SDK API Reference

## Core Classes

### Agent
The fundamental building block of the OpenAI Agents SDK.

**Constructor Parameters:**
- `name` (str): Name of the agent
- `system_prompt` (str): System prompt that defines agent behavior
- `model` (str): Model identifier (e.g., "gpt-4o", "gpt-4o-realtime-preview")
- `tools` (list): List of available tools for the agent
- `model_settings` (dict): Additional model configuration options

**Example:**
```python
from openai_agents import Agent

agent = Agent(
    name="assistant",
    system_prompt="You are a helpful assistant that specializes in Python programming.",
    model="gpt-4o",
    tools=[my_tool_function]
)
```

### Runner
Manages the execution of agents and handles the session lifecycle.

**Methods:**
- `run_agent(agent, input)`: Execute a single agent with given input
- `run_workflow(workflow)`: Execute a multi-agent workflow
- `create_session()`: Create a new session for conversation

**Example:**
```python
from openai_agents import Runner

runner = Runner()
result = await runner.run_agent(agent=my_agent, input="Hello, how are you?")
```

### RealtimeRunner
Specialized runner for real-time voice interactions.

**Constructor Parameters:**
- `model` (str): Real-time model identifier (e.g., "gpt-4o-realtime-preview")
- `tools` (list): List of available tools for the agent
- `voice` (str): Voice identifier for text-to-speech
- `temperature` (float): Model temperature setting

**Methods:**
- `run()`: Start a real-time session
- `update_session()`: Update session parameters during conversation

**Example:**
```python
from openai_agents import RealtimeRunner

runner = RealtimeRunner(
    model="gpt-4o-realtime-preview",
    voice="alloy"
)
session = await runner.run()
```

## Decorators

### @function_tool
Decorator to convert Python functions into callable tools for agents.

**Parameters:**
- `name` (str, optional): Custom name for the tool
- `description` (str, optional): Description of what the tool does
- `failure_error_function` (callable, optional): Function to handle tool errors

**Example:**
```python
from openai_agents import function_tool

@function_tool
def get_weather(location: str) -> str:
    """Get current weather for a location."""
    # Implementation here
    return f"Weather in {location}: Sunny, 75°F"

@function_tool(
    name="calculate_mortgage",
    description="Calculate monthly mortgage payment based on loan details",
    failure_error_function=lambda e: f"Calculation failed: {str(e)}"
)
def calculate_monthly_mortgage(principal: float, interest_rate: float, years: int) -> float:
    """Calculate monthly mortgage payment."""
    monthly_rate = interest_rate / 12 / 100
    num_payments = years * 12
    payment = principal * (monthly_rate * (1 + monthly_rate)**num_payments) / ((1 + monthly_rate)**num_payments - 1)
    return round(payment, 2)
```

## Event Types in Real-time Sessions

When processing events from RealtimeRunner sessions, handle these event types:

### agent_start
- **Properties:** `agent.name`, `session_id`
- **Description:** Triggered when an agent begins processing

### agent_end
- **Properties:** `agent.name`, `session_id`
- **Description:** Triggered when an agent finishes processing

### tool_start
- **Properties:** `tool.name`, `arguments`, `session_id`
- **Description:** Triggered when a tool begins execution

### tool_end
- **Properties:** `tool.name`, `output`, `session_id`
- **Description:** Triggered when a tool completes execution

### handoff
- **Properties:** `from_agent.name`, `to_agent.name`, `reason`
- **Description:** Triggered when control transfers between agents

### audio
- **Properties:** `audio_data`, `timestamp`
- **Description:** Contains audio data during real-time conversations

### audio_interrupted
- **Properties:** `timestamp`
- **Description:** Triggered when audio playback is interrupted

### error
- **Properties:** `error`, `error_type`, `session_id`
- **Description:** Triggered when an error occurs

### history_updated
- **Properties:** `messages`, `session_id`
- **Description:** Triggered when conversation history is updated

## Model Settings

Configure advanced model behavior with model_settings:

```python
model_settings = {
    "temperature": 0.7,          # Creativity control (0.0-2.0)
    "max_tokens": 1000,          # Maximum response tokens
    "top_p": 1.0,                # Nucleus sampling parameter
    "frequency_penalty": 0.0,    # Penalty for frequent tokens
    "presence_penalty": 0.0,     # Penalty for present tokens
    "response_format": "text",   # Expected response format
    "tool_choice": "auto"        # How tools are selected
}
```

## Context Management

### RunContextWrapper
Manage shared context between agents:

```python
from openai_agents import RunContextWrapper

# Create context with specific preferences
context = RunContextWrapper({
    "language_preference": "english",
    "user_timezone": "UTC-5",
    "session_data": {}
})

# Use context in runner
result = await Runner.run(
    agent=my_agent,
    input="Hello",
    context=context.context
)
```

## Common Patterns

### Tool Error Handling
```python
def handle_tool_error(error):
    return f"Sorry, I couldn't complete that action: {str(error)}. Please try again."

@function_tool(failure_error_function=handle_tool_error)
def risky_operation(data: str):
    # This function might fail
    return process_data(data)
```

### Async Tool Implementation
```python
import aiohttp

@function_tool
async def fetch_external_data(url: str) -> str:
    """Fetch data from an external API."""
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            data = await response.json()
            return str(data)
```

### File and Image Processing Tools
```python
from typing import Union
from pathlib import Path

@function_tool
def process_file(file_path: str) -> str:
    """Process a file and return its content summary."""
    path = Path(file_path)
    if path.suffix.lower() in ['.jpg', '.png', '.gif']:
        return f"Image file: {path.name}, size: {path.stat().st_size} bytes"
    elif path.suffix.lower() == '.txt':
        with open(path, 'r') as f:
            content = f.read()
            return f"Text file: {path.name}, length: {len(content)} characters"
    else:
        return f"File: {path.name}, type: {path.suffix}"
```

## Best Practices

### 1. Type Hints
Always use proper type hints for function tools to ensure correct schema generation:

```python
# Good
@function_tool
def add_numbers(a: int, b: int) -> int:
    return a + b

# Better with more complex types
from typing import List, Dict, Optional

@function_tool
def process_user_data(
    name: str,
    age: int,
    hobbies: List[str],
    metadata: Optional[Dict[str, str]] = None
) -> Dict[str, str]:
    return {"status": "processed", "name": name}
```

### 2. Descriptive Docstrings
Include clear descriptions in docstrings for better tool understanding:

```python
@function_tool
def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the distance between two geographic coordinates.

    Args:
        lat1: Latitude of first point
        lon1: Longitude of first point
        lat2: Latitude of second point
        lon2: Longitude of second point

    Returns:
        Distance in kilometers between the two points
    """
    # Implementation here
    pass
```

### 3. Proper Async Handling
Use async/await patterns appropriately for I/O operations:

```python
import asyncio

@function_tool
async def delayed_response(message: str, delay: int = 2) -> str:
    """Return a message after a specified delay."""
    await asyncio.sleep(delay)
    return f"Delayed response: {message}"
```