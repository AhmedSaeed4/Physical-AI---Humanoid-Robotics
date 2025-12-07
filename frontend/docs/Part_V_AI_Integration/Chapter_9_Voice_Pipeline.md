---
sidebar_position: 1
title: "Chapter 9: The Ears (Voice Pipeline)"
---

# Chapter 9: The Ears (Voice Pipeline)

## Overview

This chapter explores the voice processing pipeline for humanoid robots, focusing on how to integrate speech recognition and natural language processing to enable voice-based interaction. We'll cover the integration of ReSpeaker microphone arrays, OpenAI Whisper for speech-to-text, and push-to-talk logic to optimize API usage.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up ReSpeaker USB microphone arrays for humanoid robots
- Integrate OpenAI Whisper API for speech recognition
- Implement push-to-talk logic to minimize API costs
- Process voice commands and convert them to robot actions
- Handle voice processing errors and fallback scenarios

## Voice Processing Architecture

### Overview of the Voice Pipeline

The voice pipeline for our humanoid robot consists of several components:

```
Microphone Array → Audio Processing → Speech-to-Text → NLP Processing → Robot Action
```

Each component must work seamlessly to provide natural voice interaction with the robot.

### System Components

1. **Audio Input**: ReSpeaker USB microphone array
2. **Audio Processing**: Noise reduction, voice activity detection
3. **Speech Recognition**: OpenAI Whisper API
4. **Natural Language Processing**: LLM for command interpretation
5. **Action Execution**: Convert commands to robot actions

## ReSpeaker USB Microphone Integration

### Hardware Setup

The ReSpeaker USB microphone array provides high-quality audio input for the humanoid robot:

```python
# Example setup for ReSpeaker microphone
import pyaudio
import wave
import numpy as np
import threading
import queue

class ReSpeakerMicrophone:
    """
    ReSpeaker USB microphone interface for humanoid robot.
    """

    def __init__(self, device_index=None, sample_rate=16000, chunk_size=1024):
        """
        Initialize the ReSpeaker microphone interface.

        Args:
            device_index: Index of the ReSpeaker device (None for default)
            sample_rate: Audio sample rate
            chunk_size: Size of audio chunks to process
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.device_index = device_index
        self.audio = pyaudio.PyAudio()

        # Find ReSpeaker device if index not specified
        if device_index is None:
            self.device_index = self.find_respeaker_device()

        # Audio stream parameters
        self.stream = None
        self.is_recording = False
        self.audio_queue = queue.Queue()

    def find_respeaker_device(self):
        """
        Find the ReSpeaker device index.

        Returns:
            Device index for ReSpeaker or default device
        """
        for i in range(self.audio.get_device_count()):
            device_info = self.audio.get_device_info_by_index(i)
            if "ReSpeaker" in device_info['name']:
                return i

        # If ReSpeaker not found, return default input device
        return self.audio.get_default_input_device_info()['index']

    def start_recording(self):
        """
        Start recording audio from the microphone.
        """
        if self.stream is not None:
            return

        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk_size
        )

        self.is_recording = True

        # Start recording thread
        self.record_thread = threading.Thread(target=self._record_audio)
        self.record_thread.start()

    def stop_recording(self):
        """
        Stop recording audio.
        """
        self.is_recording = False
        if self.record_thread.is_alive():
            self.record_thread.join()
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

    def _record_audio(self):
        """
        Internal method to record audio chunks.
        """
        while self.is_recording:
            try:
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                self.audio_queue.put(data)
            except Exception as e:
                print(f"Error recording audio: {e}")
                break

    def get_audio_chunk(self):
        """
        Get the next audio chunk from the queue.

        Returns:
            Audio chunk as bytes or None if queue is empty
        """
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None
```

### ALSA Configuration for ReSpeaker

For Linux systems, proper ALSA configuration is essential:

```bash
# Example ALSA configuration (.asoundrc)
pcm.reSpeaker {
    type hw
    card 1
    device 0
}

ctl.reSpeaker {
    type hw
    card 1
}

pcm.!default {
    type asym
    playback.pcm {
        type plug
        slave.pcm "dmix:CARD=0,DEVICE=0"
    }
    capture.pcm {
        type plug
        slave.pcm "dsnoop:CARD=1,DEVICE=0"
    }
}
```

## OpenAI Whisper Integration

### Whisper API Integration

The OpenAI Whisper API provides high-quality speech-to-text conversion:

```python
import openai
import asyncio
import aiofiles
from typing import Optional
import tempfile
import os
import wave

class WhisperSTT:
    """
    OpenAI Whisper API integration for speech-to-text.
    """

    def __init__(self, api_key: str, model: str = "whisper-1"):
        """
        Initialize Whisper STT interface.

        Args:
            api_key: OpenAI API key
            model: Whisper model to use (default: whisper-1)
        """
        openai.api_key = api_key
        self.model = model

    async def transcribe_audio(self, audio_path: str) -> Optional[str]:
        """
        Transcribe audio file using OpenAI Whisper API.

        Args:
            audio_path: Path to audio file to transcribe

        Returns:
            Transcribed text or None if transcription failed
        """
        try:
            with open(audio_path, "rb") as audio_file:
                transcript = await openai.Audio.atranscribe(
                    model=self.model,
                    file=audio_file,
                    response_format="verbose_json",
                    timestamp_granularities=["segment"]
                )

            return transcript.text

        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None

    def save_audio_chunk(self, audio_data: bytes, sample_rate: int = 16000) -> str:
        """
        Save audio chunk to temporary WAV file.

        Args:
            audio_data: Raw audio data
            sample_rate: Sample rate for the audio

        Returns:
            Path to temporary WAV file
        """
        # Create temporary WAV file
        temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
        temp_file.close()

        # Write audio data to WAV file
        with wave.open(temp_file.name, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data)

        return temp_file.name
```

### Local Whisper Alternative

For edge deployment, consider using a local Whisper model:

```python
# Local Whisper model using transformers
from transformers import pipeline
import torch

class LocalWhisperSTT:
    """
    Local Whisper model for speech-to-text (edge deployment).
    """

    def __init__(self, model_name: str = "openai/whisper-tiny"):
        """
        Initialize local Whisper STT.

        Args:
            model_name: Name of the Whisper model to use
        """
        # Check if CUDA is available
        device = 0 if torch.cuda.is_available() else -1

        self.transcriber = pipeline(
            "automatic-speech-recognition",
            model=model_name,
            device=device
        )

    def transcribe_audio(self, audio_path: str) -> Optional[str]:
        """
        Transcribe audio using local Whisper model.

        Args:
            audio_path: Path to audio file to transcribe

        Returns:
            Transcribed text or None if transcription failed
        """
        try:
            result = self.transcriber(audio_path)
            return result["text"]
        except Exception as e:
            print(f"Error transcribing audio locally: {e}")
            return None
```

## Push-to-Talk Logic

### Implementing Cost-Effective Voice Recognition

The push-to-talk logic minimizes API usage while maintaining responsiveness:

```python
import threading
import time
import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional

@dataclass
class VoiceCommand:
    """
    Data class for voice commands.
    """
    text: str
    confidence: float
    timestamp: float
    device: str = "ReSpeaker"

class PushToTalkManager:
    """
    Manages push-to-talk logic to minimize API usage.
    """

    def __init__(self,
                 stt_engine,  # STT engine instance
                 silence_threshold: float = 0.01,
                 min_voice_duration: float = 0.5,
                 max_recording_duration: float = 10.0):
        """
        Initialize push-to-talk manager.

        Args:
            stt_engine: Speech-to-text engine instance
            silence_threshold: Threshold for detecting silence
            min_voice_duration: Minimum duration of voice to process (seconds)
            max_recording_duration: Maximum recording duration (seconds)
        """
        self.stt_engine = stt_engine
        self.silence_threshold = silence_threshold
        self.min_voice_duration = min_voice_duration
        self.max_recording_duration = max_recording_duration

        self.is_listening = False
        self.is_recording = False
        self.recording_start_time = 0
        self.audio_buffer = []
        self.command_callback: Optional[Callable[[VoiceCommand], None]] = None

        # Threading for non-blocking operation
        self.listening_thread = None
        self.recording_thread = None

    def start_listening(self):
        """
        Start listening for voice activity.
        """
        if self.listening_thread and self.listening_thread.is_alive():
            return

        self.is_listening = True
        self.listening_thread = threading.Thread(target=self._listen_for_voice)
        self.listening_thread.start()

    def stop_listening(self):
        """
        Stop listening for voice activity.
        """
        self.is_listening = False
        if self.listening_thread and self.listening_thread.is_alive():
            self.listening_thread.join()

    def set_command_callback(self, callback: Callable[[VoiceCommand], None]):
        """
        Set callback function for processed voice commands.

        Args:
            callback: Function to call when voice command is processed
        """
        self.command_callback = callback

    def _listen_for_voice(self):
        """
        Internal method to listen for voice activity.
        """
        microphone = ReSpeakerMicrophone()
        microphone.start_recording()

        voice_detected = False
        voice_start_time = 0

        try:
            while self.is_listening:
                chunk = microphone.get_audio_chunk()
                if chunk is None:
                    time.sleep(0.01)  # Small delay to prevent busy waiting
                    continue

                # Convert bytes to numpy array for analysis
                audio_array = np.frombuffer(chunk, dtype=np.int16)

                # Calculate audio level
                audio_level = np.sqrt(np.mean(audio_array.astype(np.float32) ** 2))

                if audio_level > self.silence_threshold and not voice_detected:
                    # Voice activity detected
                    voice_detected = True
                    voice_start_time = time.time()
                    self._start_recording(microphone, voice_start_time)

                elif audio_level <= self.silence_threshold and voice_detected:
                    # Voice stopped, check if it was long enough
                    voice_duration = time.time() - voice_start_time
                    if voice_duration >= self.min_voice_duration:
                        # Process the recorded audio
                        self._process_recording()

                    voice_detected = False
                    self._stop_recording()

        finally:
            microphone.stop_recording()

    def _start_recording(self, microphone, start_time):
        """
        Start recording audio after voice detection.

        Args:
            microphone: Microphone instance
            start_time: Time when voice was detected
        """
        self.is_recording = True
        self.recording_start_time = start_time
        self.audio_buffer = []

        # Start recording thread
        self.recording_thread = threading.Thread(
            target=self._record_until_silence,
            args=(microphone,)
        )
        self.recording_thread.start()

    def _record_until_silence(self, microphone):
        """
        Record audio until silence is detected or max duration is reached.

        Args:
            microphone: Microphone instance
        """
        while self.is_recording and (time.time() - self.recording_start_time) < self.max_recording_duration:
            chunk = microphone.get_audio_chunk()
            if chunk:
                self.audio_buffer.append(chunk)
            time.sleep(0.01)

    def _stop_recording(self):
        """
        Stop recording audio.
        """
        self.is_recording = False
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join()

    def _process_recording(self):
        """
        Process the recorded audio through STT.
        """
        if not self.audio_buffer:
            return

        try:
            # Combine audio chunks
            full_audio = b''.join(self.audio_buffer)

            # Save to temporary file for STT processing
            temp_path = self._save_audio_buffer(full_audio)

            # Transcribe using STT engine
            text = self.stt_engine.transcribe_audio(temp_path)

            if text and text.strip():
                command = VoiceCommand(
                    text=text.strip(),
                    confidence=0.9,  # Placeholder confidence
                    timestamp=time.time()
                )

                # Call the command callback if set
                if self.command_callback:
                    self.command_callback(command)

            # Clean up temporary file
            if os.path.exists(temp_path):
                os.remove(temp_path)

        except Exception as e:
            print(f"Error processing recording: {e}")

    def _save_audio_buffer(self, audio_data: bytes) -> str:
        """
        Save audio buffer to temporary WAV file.

        Args:
            audio_data: Raw audio data to save

        Returns:
            Path to temporary WAV file
        """
        import wave
        import tempfile

        temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
        temp_file.close()

        with wave.open(temp_file.name, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(16000)  # 16kHz
            wav_file.writeframes(audio_data)

        return temp_file.name
```

## Voice Command Processing

### Processing Voice Commands to Robot Actions

Converting voice commands to robot actions:

```python
import json
from typing import Dict, Any

class VoiceCommandProcessor:
    """
    Processes voice commands and converts them to robot actions.
    """

    def __init__(self):
        """
        Initialize the voice command processor.
        """
        self.command_patterns = {
            "move_forward": ["move forward", "go forward", "walk forward", "step forward"],
            "move_backward": ["move backward", "go backward", "walk backward", "step backward"],
            "turn_left": ["turn left", "rotate left", "pivot left"],
            "turn_right": ["turn right", "rotate right", "pivot right"],
            "stop": ["stop", "halt", "freeze", "pause"],
            "dance": ["dance", "perform dance", "do dance"],
            "wave": ["wave", "wag", "hello"],
            "sit": ["sit", "sit down", "take a seat"],
            "stand": ["stand", "stand up", "get up"]
        }

    def process_command(self, voice_command: VoiceCommand) -> Optional[Dict[str, Any]]:
        """
        Process a voice command and return a robot action.

        Args:
            voice_command: Voice command to process

        Returns:
            Robot action as dictionary or None if command not recognized
        """
        text = voice_command.text.lower().strip()

        # Match command to known patterns
        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                if pattern in text:
                    return self._create_robot_action(action, text)

        # If no pattern matches, return None
        return None

    def _create_robot_action(self, action_type: str, original_text: str) -> Dict[str, Any]:
        """
        Create a robot action from the command type.

        Args:
            action_type: Type of action to create
            original_text: Original voice command text

        Returns:
            Robot action dictionary
        """
        action_map = {
            "move_forward": {
                "action_type": "navigate",
                "parameters": {
                    "x": 1.0,
                    "y": 0.0,
                    "theta": 0.0
                }
            },
            "move_backward": {
                "action_type": "navigate",
                "parameters": {
                    "x": -1.0,
                    "y": 0.0,
                    "theta": 0.0
                }
            },
            "turn_left": {
                "action_type": "rotate",
                "parameters": {
                    "angle": 90.0
                }
            },
            "turn_right": {
                "action_type": "rotate",
                "parameters": {
                    "angle": -90.0
                }
            },
            "stop": {
                "action_type": "stop",
                "parameters": {}
            },
            "dance": {
                "action_type": "dance",
                "parameters": {
                    "style": "default"
                }
            },
            "wave": {
                "action_type": "gesture",
                "parameters": {
                    "type": "wave"
                }
            },
            "sit": {
                "action_type": "posture",
                "parameters": {
                    "pose": "sit"
                }
            },
            "stand": {
                "action_type": "posture",
                "parameters": {
                    "pose": "stand"
                }
            }
        }

        return action_map.get(action_type, {
            "action_type": "unknown",
            "parameters": {"original_text": original_text}
        })
```

## Voice Pipeline Integration

### Complete Voice Pipeline Example

Putting it all together in a complete voice pipeline:

```python
import asyncio
from typing import Optional

class VoicePipeline:
    """
    Complete voice pipeline integrating all components.
    """

    def __init__(self,
                 openai_api_key: str,
                 use_local_whisper: bool = False,
                 model_name: str = "whisper-1"):
        """
        Initialize the complete voice pipeline.

        Args:
            openai_api_key: OpenAI API key for Whisper API
            use_local_whisper: Whether to use local Whisper model
            model_name: Whisper model name
        """
        # Initialize STT engine
        if use_local_whisper:
            self.stt_engine = LocalWhisperSTT(model_name)
        else:
            self.stt_engine = WhisperSTT(openai_api_key, model_name)

        # Initialize push-to-talk manager
        self.ptt_manager = PushToTalkManager(
            stt_engine=self.stt_engine,
            silence_threshold=0.01,
            min_voice_duration=0.3,
            max_recording_duration=8.0
        )

        # Initialize command processor
        self.command_processor = VoiceCommandProcessor()

        # Callback for processed commands
        self.command_callback: Optional[Callable[[Dict[str, Any]], None]] = None

    def start_pipeline(self):
        """
        Start the voice pipeline.
        """
        # Set up command callback
        self.ptt_manager.set_command_callback(self._on_voice_command)

        # Start listening
        self.ptt_manager.start_listening()

    def stop_pipeline(self):
        """
        Stop the voice pipeline.
        """
        self.ptt_manager.stop_listening()

    def set_action_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Set callback for processed robot actions.

        Args:
            callback: Function to call when robot action is generated
        """
        self.command_callback = callback

    def _on_voice_command(self, voice_command: VoiceCommand):
        """
        Handle a recognized voice command.

        Args:
            voice_command: Recognized voice command
        """
        print(f"Recognized: {voice_command.text} (confidence: {voice_command.confidence})")

        # Process the command
        robot_action = self.command_processor.process_command(voice_command)

        if robot_action:
            print(f"Generated action: {robot_action}")

            # Call the action callback if set
            if self.command_callback:
                self.command_callback(robot_action)
        else:
            print(f"Command not recognized: {voice_command.text}")

    async def run_pipeline(self):
        """
        Run the voice pipeline continuously.
        """
        self.start_pipeline()

        try:
            # Keep the pipeline running
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Stopping voice pipeline...")
        finally:
            self.stop_pipeline()

# Example usage
async def main():
    """
    Example main function demonstrating voice pipeline usage.
    """
    # Initialize voice pipeline
    pipeline = VoicePipeline(
        openai_api_key="your-openai-api-key-here",
        use_local_whisper=False  # Use API for better accuracy
    )

    # Set up action callback
    def handle_robot_action(action):
        print(f"Executing robot action: {action}")
        # Here you would send the action to your robot control system

    pipeline.set_action_callback(handle_robot_action)

    # Run the pipeline
    await pipeline.run_pipeline()

if __name__ == "__main__":
    # Only run if in appropriate environment
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Pipeline stopped by user")
```

## Error Handling and Fallbacks

### Robust Error Handling

Implementing error handling for voice processing:

```python
import logging
from enum import Enum

class VoiceProcessingError(Enum):
    """
    Types of voice processing errors.
    """
    MICROPHONE_ERROR = "microphone_error"
    STT_ERROR = "stt_error"
    NETWORK_ERROR = "network_error"
    API_ERROR = "api_error"
    AUDIO_QUALITY_ERROR = "audio_quality_error"

class VoicePipelineWithFallbacks:
    """
    Voice pipeline with comprehensive error handling and fallbacks.
    """

    def __init__(self, openai_api_key: str):
        """
        Initialize voice pipeline with fallbacks.

        Args:
            openai_api_key: OpenAI API key
        """
        self.openai_api_key = openai_api_key
        self.fallback_stt = LocalWhisperSTT("openai/whisper-tiny.en")
        self.voice_processing_errors = []

        # Set up logging
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

        # Error handling configuration
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds

    def handle_error(self, error_type: VoiceProcessingError, error_message: str):
        """
        Handle voice processing errors with appropriate fallbacks.

        Args:
            error_type: Type of error that occurred
            error_message: Error message
        """
        self.logger.error(f"{error_type.value}: {error_message}")
        self.voice_processing_errors.append({
            'type': error_type.value,
            'message': error_message,
            'timestamp': time.time()
        })

        # Implement fallback based on error type
        if error_type == VoiceProcessingError.NETWORK_ERROR:
            self.logger.info("Falling back to local STT processing")
            # Switch to local processing
            pass
        elif error_type == VoiceProcessingError.API_ERROR:
            self.logger.info("API error - using cached response or default action")
            # Use cached response or default action
            pass

    def retry_on_failure(self, func, *args, **kwargs):
        """
        Retry a function on failure.

        Args:
            func: Function to retry
            *args: Arguments to pass to function
            **kwargs: Keyword arguments to pass to function

        Returns:
            Result of function call
        """
        for attempt in range(self.max_retries):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                self.logger.warning(f"Attempt {attempt + 1} failed: {e}")
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay * (attempt + 1))  # Exponential backoff
                else:
                    raise e
```

## Performance Optimization

### Optimizing Voice Processing

To optimize performance and reduce latency:

1. **Audio Preprocessing**: Apply noise reduction and voice activity detection
2. **Buffer Management**: Efficiently manage audio buffers to minimize memory usage
3. **Threading**: Use separate threads for audio capture, processing, and network calls
4. **Caching**: Cache frequently used models and responses when possible

## Summary

In this chapter, we've explored the voice pipeline for humanoid robots, covering:
- ReSpeaker microphone array integration
- OpenAI Whisper API for speech recognition
- Push-to-talk logic to minimize API costs
- Voice command processing and conversion to robot actions
- Error handling and fallback mechanisms

The voice pipeline is a critical component for natural human-robot interaction, enabling users to control the humanoid robot through spoken commands. Proper implementation of push-to-talk logic and error handling ensures a responsive and reliable voice interface while managing API costs effectively.

In the next chapter, we'll explore the brain of the robot - the LLM action planning system that interprets voice commands and generates appropriate robot behaviors.