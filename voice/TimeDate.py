"""
Raspberry Pi Status System with Voice Control
Speaks current time and date via keyword or LLM trigger
"""

import pyttsx3
from datetime import datetime
import speech_recognition as sr
import anthropic
import os

# Initialize text-to-speech engine
tts_engine = pyttsx3.init()
tts_engine.setProperty('rate', 150)  # Speed of speech

def status():
    """
    Speaks the current time and date in a natural format.
    Example: "It is 3:42 PM on October 2, 2025."
    """
    now = datetime.now()
    
    # Format time (12-hour with AM/PM)
    time_str = now.strftime("%I:%M %p").lstrip('0')  # Remove leading zero
    
    # Format date (Month Day, Year)
    date_str = now.strftime("%B %d, %Y").replace(' 0', ' ')  # Remove leading zero from day
    
    # Create status message
    message = f"It is {time_str} on {date_str}."
    
    print(f"Status: {message}")
    tts_engine.say(message)
    tts_engine.runAndWait()
    
    return message

def keyword_mode():
    """
    Keyword-based voice control mode.
    Listens for "time" or "date" keywords to trigger status().
    """
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    print("\n=== KEYWORD MODE ===")
    print("Say 'time' or 'date' to hear the status")
    print("Say 'exit' to quit\n")
    
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
    
    while True:
        try:
            print("Listening...")
            with mic as source:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
            
            text = recognizer.recognize_google(audio).lower()
            print(f"You said: {text}")
            
            # Check for keywords
            if "time" in text or "date" in text:
                status()
            elif "exit" in text or "quit" in text:
                print("Exiting keyword mode...")
                break
            else:
                print("Say 'time' or 'date' to hear the status")
                
        except sr.WaitTimeoutError:
            continue
        except sr.UnknownValueError:
            print("Could not understand audio")
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def llm_mode():
    """
    LLM-based voice control mode using Claude API.
    Claude decides when to call the status tool based on user intent.
    """
    # Check for API key
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        print("Error: ANTHROPIC_API_KEY environment variable not set")
        print("Set it with: export ANTHROPIC_API_KEY='your-key-here'")
        return
    
    client = anthropic.Anthropic(api_key=api_key)
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    print("\n=== LLM MODE ===")
    print("Ask about time or date in natural language")
    print("Say 'exit' to quit\n")
    
    # Define the status tool for Claude
    tools = [{
        "name": "status",
        "description": "Gets the current time and date and speaks it aloud. Use this when the user asks about the current time, date, or what time it is.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    }]
    
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
    
    while True:
        try:
            print("Listening...")
            with mic as source:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
            
            text = recognizer.recognize_google(audio)
            print(f"You said: {text}")
            
            if "exit" in text.lower() or "quit" in text.lower():
                print("Exiting LLM mode...")
                break
            
            # Send to Claude with tool
            response = client.messages.create(
                model="claude-sonnet-4-20250514",
                max_tokens=1000,
                tools=tools,
                messages=[{
                    "role": "user",
                    "content": text
                }]
            )
            
            # Process response
            for block in response.content:
                if block.type == "tool_use" and block.name == "status":
                    print("Claude triggered status()...")
                    status()
                elif block.type == "text":
                    print(f"Claude: {block.text}")
                    tts_engine.say(block.text)
                    tts_engine.runAndWait()
                    
        except sr.WaitTimeoutError:
            continue
        except sr.UnknownValueError:
            print("Could not understand audio")
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def demo_mode():
    """
    Demo mode: Tests three different phrases that should trigger status.
    """
    print("\n=== DEMO MODE ===")
    print("Testing three phrases that trigger status():\n")
    
    test_phrases = [
        "What time is it?",
        "Tell me the date",
        "What's the current time and date?"
    ]
    
    for i, phrase in enumerate(test_phrases, 1):
        print(f"\nTest {i}: '{phrase}'")
        input("Press Enter to hear the response...")
        status()
        print("-" * 50)
    
    print("\nDemo complete!")

def main():
    """Main menu for the status system."""
    print("=" * 60)
    print("RASPBERRY PI TIME/DATE STATUS SYSTEM")
    print("=" * 60)
    
    while True:
        print("\nSelect mode:")
        print("1. Keyword Mode (say 'time' or 'date')")
        print("2. LLM Mode (natural language with Claude)")
        print("3. Demo Mode (test three phrases)")
        print("4. Test status() function once")
        print("5. Exit")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            keyword_mode()
        elif choice == "2":
            llm_mode()
        elif choice == "3":
            demo_mode()
        elif choice == "4":
            print("\nTesting status() function:")
            status()
        elif choice == "5":
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1-5.")

if __name__ == "__main__":
    main()
