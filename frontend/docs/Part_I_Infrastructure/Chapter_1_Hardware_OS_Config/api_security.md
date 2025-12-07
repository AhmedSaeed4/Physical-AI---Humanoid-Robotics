---
sidebar_position: 4
title: "API Security & Rate Limiting"
---

# API Security & Rate Limiting Best Practices

## Overview

This guide covers essential security practices and rate limiting strategies for AI integration in robotics applications. When connecting robots to cloud AI services like OpenAI, proper security measures are critical to prevent unauthorized access, excessive costs, and potential safety issues.

## API Key Security

### Environment Variables

Never hardcode API keys in source code. Use environment variables:

```bash
# Create a .env file (and add it to .gitignore)
echo "OPENAI_API_KEY=your_actual_key_here" >> .env
echo "ANTHROPIC_API_KEY=your_actual_key_here" >> .env
```

In your Python code:
```python
import os
from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file

openai_api_key = os.getenv("OPENAI_API_KEY")
```

### Secure Storage

For production deployments, use secure key management:

- **Docker secrets**: For containerized deployments
- **HashiCorp Vault**: For enterprise environments
- **Cloud key management**: AWS Secrets Manager, Azure Key Vault, GCP Secret Manager
- **Hardware security modules (HSM)**: For maximum security

## Rate Limiting Strategies

### Token Consumption Management

AI APIs are billed per token. Implement consumption tracking:

```python
import time
from dataclasses import dataclass
from typing import Optional

@dataclass
class TokenUsage:
    input_tokens: int = 0
    output_tokens: int = 0
    timestamp: float = 0.0

class TokenTracker:
    def __init__(self, max_tokens_per_minute: int = 100000):
        self.max_tokens_per_minute = max_tokens_per_minute
        self.usage_log = []

    def can_make_request(self, estimated_tokens: int) -> bool:
        current_time = time.time()
        # Remove usage data older than 1 minute
        self.usage_log = [usage for usage in self.usage_log
                         if current_time - usage.timestamp < 60]

        total_recent_tokens = sum(usage.input_tokens + usage.output_tokens
                                 for usage in self.usage_log)

        return (total_recent_tokens + estimated_tokens) <= self.max_tokens_per_minute

    def log_usage(self, usage: TokenUsage):
        self.usage_log.append(usage)
```

### Push-to-Talk Logic

For voice processing, implement push-to-talk to minimize token usage:

```python
import pyaudio
import wave
import openai
import threading
import time

class PushToTalkManager:
    def __init__(self, api_key: str, silence_threshold: float = 0.01):
        self.api_key = api_key
        self.silence_threshold = silence_threshold
        self.is_listening = False
        self.listening_thread = None

    def start_listening(self):
        """Start listening for audio input"""
        self.is_listening = True
        self.listening_thread = threading.Thread(target=self._record_audio)
        self.listening_thread.start()

    def stop_listening(self):
        """Stop listening and process audio"""
        self.is_listening = False
        if self.listening_thread:
            self.listening_thread.join()

    def _record_audio(self):
        """Internal method to record audio until silence is detected"""
        # Implementation would use pyaudio to record until silence
        pass
```

## Circuit Breaker Pattern

Implement circuit breakers to handle API failures gracefully:

```python
import time
from enum import Enum
from typing import Callable, Any

class CircuitState(Enum):
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Failure threshold exceeded
    HALF_OPEN = "half_open" # Testing if service is recovered

class CircuitBreaker:
    def __init__(self, failure_threshold: int = 5,
                 timeout: int = 60, recovery_timeout: int = 30):
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.recovery_timeout = recovery_timeout

        self.failure_count = 0
        self.last_failure_time = None
        self.state = CircuitState.CLOSED

    def call(self, func: Callable, *args, **kwargs) -> Any:
        if self.state == CircuitState.OPEN:
            if time.time() - self.last_failure_time > self.recovery_timeout:
                self.state = CircuitState.HALF_OPEN
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = func(*args, **kwargs)
            self._on_success()
            return result
        except Exception as e:
            self._on_failure()
            raise e

    def _on_success(self):
        self.failure_count = 0
        self.state = CircuitState.CLOSED

    def _on_failure(self):
        self.failure_count += 1
        self.last_failure_time = time.time()

        if self.failure_count >= self.failure_threshold:
            self.state = CircuitState.OPEN
```

## Retry Logic with Exponential Backoff

Implement intelligent retry mechanisms:

```python
import random
import time
from functools import wraps

def retry_with_backoff(max_retries: int = 3, base_delay: float = 1.0,
                      max_delay: float = 60.0, backoff_factor: float = 2.0):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            delay = base_delay

            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_retries - 1:  # Last attempt
                        raise e

                    # Add jitter to prevent thundering herd
                    jitter = random.uniform(0, 0.1 * delay)
                    time.sleep(delay + jitter)
                    delay = min(delay * backoff_factor, max_delay)

            return None
        return wrapper
    return decorator

# Usage
@retry_with_backoff(max_retries=3, base_delay=1.0)
def call_openai_api(prompt: str):
    # Your API call here
    pass
```

## Request Validation

Validate all requests before sending to AI services:

```python
import re
from typing import Dict, Any

class RequestValidator:
    @staticmethod
    def validate_prompt(prompt: str) -> bool:
        """Validate that a prompt is safe and appropriate"""
        if len(prompt) > 4096:  # OpenAI's limit
            return False

        # Check for potentially harmful patterns
        harmful_patterns = [
            r"ignore previous instructions",
            r"system prompt",
            r"you are now",
            r"jailbreak",
        ]

        for pattern in harmful_patterns:
            if re.search(pattern, prompt, re.IGNORECASE):
                return False

        return True

    @staticmethod
    def sanitize_input(user_input: str) -> str:
        """Sanitize user input to prevent injection attacks"""
        # Remove potentially harmful characters
        sanitized = re.sub(r'[<>"\']', '', user_input)
        return sanitized.strip()
```

## Monitoring and Alerting

Set up monitoring for API usage:

```python
import logging
from datetime import datetime
from dataclasses import dataclass

@dataclass
class APIMonitoringEvent:
    timestamp: datetime
    service: str
    tokens_used: int
    cost_estimate: float
    response_time: float

class APIMonitor:
    def __init__(self, alert_threshold: float = 10.0):  # $10 daily budget
        self.daily_cost = 0.0
        self.alert_threshold = alert_threshold
        self.logger = logging.getLogger(__name__)

    def log_request(self, event: APIMonitoringEvent):
        self.daily_cost += event.cost_estimate

        # Log for monitoring
        self.logger.info(f"API Request: {event.service}, "
                        f"Tokens: {event.tokens_used}, "
                        f"Cost: ${event.cost_estimate:.4f}, "
                        f"Total Daily: ${self.daily_cost:.4f}")

        # Check for budget alerts
        if self.daily_cost > self.alert_threshold:
            self.logger.warning(f"Daily budget threshold exceeded: ${self.daily_cost:.4f}")
            # Send alert (email, Slack, etc.)
            self.send_alert()

    def send_alert(self):
        """Send alert when budget threshold is exceeded"""
        # Implementation would send notification
        pass
```

## Secure Communication

Use HTTPS and validate certificates:

```python
import ssl
import certifi
import openai

# Configure SSL context
ssl_context = ssl.create_default_context(cafile=certifi.where())

# Configure OpenAI client with secure settings
client = openai.OpenAI(
    api_key=os.getenv("OPENAI_API_KEY"),
    http_client=httpx.Client(
        verify=certifi.where(),  # Use proper certificate bundle
        timeout=30.0  # Set appropriate timeout
    )
)
```

## Best Practices Summary

1. **Never commit API keys** to version control
2. **Use environment variables** or secure vaults
3. **Implement rate limiting** to control costs
4. **Validate all inputs** before sending to AI services
5. **Use circuit breakers** to handle failures gracefully
6. **Monitor usage** and set budget alerts
7. **Log and audit** all API interactions
8. **Implement proper error handling** for network issues
9. **Use appropriate timeouts** to prevent hanging requests
10. **Sanitize outputs** before using in robot control

## Robot Safety Considerations

When AI APIs are unavailable, robots should:
- Fall back to safe default behaviors
- Stop executing complex actions
- Wait for human intervention
- Log the failure for later analysis

```python
class SafeRobotController:
    def __init__(self):
        self.ai_available = True

    def execute_command(self, command: str):
        if not self.ai_available:
            # Fallback to safe behavior
            self.execute_safe_fallback()
            return

        try:
            # Attempt AI-enhanced command execution
            result = self.ai_process_command(command)
            self.send_to_robot(result)
        except APIException:
            # Switch to safe mode
            self.ai_available = False
            self.execute_safe_fallback()
```

This security framework ensures that your AI-integrated robotics system remains safe, cost-effective, and resilient to API failures.