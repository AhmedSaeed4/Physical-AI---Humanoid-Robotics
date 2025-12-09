---
name: openai-agents-sdk
description: |
  Complete reference for OpenAI Agents SDK (Python). Organized into:
  - Part A: Core Basics (Agents, Tools)
  - Part B: Advanced Workflows (Handoffs, Guardrails, Structured Outputs)
  - Part C: Realtime & Voice (Voice Agents)
  - Part D: Integration & Deployment (Gemini, FastAPI, Errors, Production)
  
  Use this skill to implement specific components based on user request.
---

# OpenAI Agents SDK - Master Skill Reference

This document is organized into 4 distinct parts. Use the relevant part based on the user's specific request.

---

## Part A: Core Basics

### 1. Installation
```bash
# Recommended (uv)
uv add openai-agents
# Optional: uv add 'openai-agents[voice]'

# Pip
pip install openai-agents
```

### 2. Basic Agent (Hello World)
The simplest agent with just instructions and a model.
```python
from agents import Agent, Runner

agent = Agent(
    name="Assistant",
    instructions="You are a helpful assistant.",
    model="gpt-4o"
)

async def main():
    await Runner.run(agent, "Hello, who are you?")
```

### 3. Basic Function Tools
Tools allow agents to perform actions. Use the `@function_tool` decorator.
```python
from agents import function_tool

@function_tool
def get_weather(location: str) -> str:
    """Get the current weather for a specific location."""
    # Logic to fetch weather
    return "Sunny, 25C"

agent = Agent(
    name="WeatherBot",
    tools=[get_weather],
    model="gpt-4o"
)
```

---

## Part B: Advanced Workflows & Guardrails

### 1. Multi-Agent Handoffs
Delegate tasks between specialized agents.
```python
from agents import Agent

# Specialized agents
billing_agent = Agent(name="Billing", instructions="Handle refunds.")
tech_agent = Agent(name="TechSupport", instructions="Handle technical issues.")

# Triage agent that can hand off to others
triage_agent = Agent(
    name="Triage", 
    instructions="Route users to the right department.", 
    handoffs=[billing_agent, tech_agent]
)
```

### 2. Guardrails (Safety & Validation)
Validate inputs and outputs using specialized guardrail agents or functions.

```python
from agents import input_guardrail, output_guardrail, GuardrailFunctionOutput

@input_guardrail
async def safety_check(context, agent, input_text: str) -> GuardrailFunctionOutput:
    if "unsafe" in input_text:
        return GuardrailFunctionOutput(tripwire_triggered=True)
    return GuardrailFunctionOutput(tripwire_triggered=False)

agent = Agent(
    name="SafeBot",
    input_guardrails=[safety_check]
)
```

### 3. Structured Outputs
Ensure the agent returns JSON-like structured data using Pydantic.
```python
from pydantic import BaseModel

class SentimentAnalysis(BaseModel):
    sentiment: str
    confidence: float

agent = Agent(
    name="Analyst",
    output_type=SentimentAnalysis
)
```

---

## Part C: Realtime & Voice

### 1. Realtime Runner
For low-latency voice-to-voice interactions.

```python
from agents import RealtimeRunner

runner = RealtimeRunner(
    instructions="You are a helpful voice assistant.",
    model="gpt-4o-realtime-preview",
)

# Connect and run session
# Requires handling audio streams in production environment
```

---

## Part D: Integration & Deployment

### 1. Gemini (Google) Integration & Custom Clients
Use Google's Gemini models via the OpenAI-compatible endpoint.

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

# Use with agent
agent = Agent(name="GeminiBot", model=model)

# Run the agent output
response = await Runner.run(agent, user_input, run_config=config)
print(response.final_output)
```

### 2. Common Error Handling
Always handle these exceptions in production.

```python
from agents import OutputGuardrailTripwireTriggered

try:
    await Runner.run(agent, user_input)
except OutputGuardrailTripwireTriggered:
    print("Response blocked by safety guardrails.")
except Exception as e:
    print(f"Agent execution failed: {e}")
```

### 3. Production Checklist
- [ ] Set `GEMINI_API_KEY` or `OPENAI_API_KEY` securely.
- [ ] Implement rate limiting on your API endpoint.
- [ ] Use `AsyncOpenAI` for non-blocking I/O.
- [ ] Add `input_guardrails` to prevent prompt injection.
