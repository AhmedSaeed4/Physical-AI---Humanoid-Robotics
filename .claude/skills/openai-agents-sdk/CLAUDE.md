# OpenAI Agents SDK Skill
(Context: You have access to expert knowledge about OpenAI Agents SDK in SKILL.md)

## ⚠️ CRITICAL INSTRUCTION: TARGETED OUTPUT ONLY
**DO NOT** generate the entire codebase or all examples at once.
**DO NOT** provide installation instructions or basics unless specifically asked.
**ONLY** implement the specific components requested by the user.

## Interaction Guidelines

1. **Identify the "Part"**: Map the user's request to one of the **Skill Parts** below.
2. **Isolate Context**: Ignore information from other parts.
3. **Concise Implementation**: Provide *only* the code snippet necessary for that part.
4. **No Fluff**: Skip generic introductions. Go straight to the solution.

## Skill Parts (Use Independently)

### Part A: Core Basics
*Triggers*: "create agent", "simple assistant", "basic tool", "how to start"
*Focus*:
- `Agent` class initialization
- Basic `@function_tool`
- Simple `Runner.run()`

### Part B: Advanced Workflows & Guardrails
*Triggers*: "multi-agent", "handoff", "router", "safety", "validation", "guardrail"
*Focus*:
- `input_guardrail` / `output_guardrail`
- Handoff logic (agent-to-agent transfers)
- Structured outputs with Pydantic

### Part C: Realtime & Voice
*Triggers*: "voice", "audio", "speech", "realtime", "conversation"
*Focus*:
- `RealtimeRunner`
- Voice-specific tools
- Audio buffer handling

### Part D: Integration & Deployment
*Triggers*: "fastapi", "gemini", "deploy", "server", "env vars"
*Focus*:
- Gemini / Reference implementation details (only if requested)
- `AsyncOpenAI` client setup
- Deployment patterns

## Response Format
When answering:
1. **Selection**: "I will implement the [Part Name] for you."
2. **Code**: [The specific snippet]
3. **Note**: Brief usage comment (1 line).