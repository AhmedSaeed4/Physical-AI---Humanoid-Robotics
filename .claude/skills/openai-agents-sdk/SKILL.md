# OpenAI Agents SDK Python Skill

## Overview
This skill provides comprehensive knowledge and capabilities for working with the OpenAI Agents SDK for Python. It enables Claude to help users build multi-agent workflows, create tools, manage handoffs, implement guardrails, and develop sophisticated agentic AI applications.

## Purpose
This skill allows Claude to:
- Explain OpenAI Agents SDK concepts and architecture
- Help create and configure agents with various LLMs
- Design multi-agent workflows with handoffs
- Implement tools and guardrails
- Debug and optimize agent performance
- Integrate voice capabilities for real-time agents

## Quick Start
When a user needs help with OpenAI Agents SDK:

1. First, understand their specific use case
2. Suggest appropriate agent architecture patterns
3. Provide code examples using the SDK
4. Explain best practices for their implementation

## Core Capabilities

### Agent Creation
- Define agents with custom system prompts
- Configure model settings and parameters
- Set up agent lifecycles and sessions

### Tool Development
- Create function tools with proper schemas
- Implement error handling for tools
- Handle file and image inputs/outputs
- Parse function signatures automatically

### Multi-Agent Workflows
- Design handoff patterns between agents
- Implement guardrails and safety measures
- Manage shared context between agents
- Create orchestration logic

### Voice & Real-time Capabilities
- Set up real-time voice conversations
- Handle audio streaming and interruptions
- Implement voice pipeline patterns
- Process speech-to-text and text-to-speech

## Installation
```bash
# Using pip
pip install openai-agents
# For voice capabilities:
pip install 'openai-agents[voice]'

# Using uv (recommended)
uv add openai-agents
# For voice capabilities:
uv add 'openai-agents[voice]'
```

## Environment Setup with uv
When working with the OpenAI Agents SDK, it's recommended to use uv for package management and virtual environment creation:
```bash
# Create a new project directory
mkdir my_agent_project
cd my_agent_project

# Create a virtual environment
python -m venv .venv

# Install the SDK using uv
uv add openai-agents
```

## Usage Examples
- "How do I create an agent that can call external APIs?"
- "Show me how to implement a handoff between two specialized agents"
- "Help me build a real-time voice agent with interruption handling"
- "I need to create tools that can process files and return results"

## Advanced Features
- Tracing and debugging capabilities
- Model configuration and optimization
- Custom transport mechanisms
- MCP (Model Context Protocol) integration
- Gemini LLM integration (see gemini_integration.md)