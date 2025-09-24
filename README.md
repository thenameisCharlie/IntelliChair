# IntelliChair
## Description
IntelliChair is a prototype smart wheelchair built on the Yahboom G1 Tank platform.It uses a Raspberry Pi and sensors to demonstrate obstacle avoidance and lays the foundation for voice control and LiDAR-based navigation. The goal is to create a low-cost, AI-driven mobility aid.

##Features
Ultrasonic Backup Detection: An HC-SR04 sensor that provides secondary close-range obstacle detection in case LiDAR data is not available or unreliable.
LiDAR-Based Obstacle Avoidance: Primary obstacle detection and mapping will rely on LiDAR to create real-time awareness of the environment.
Voice Command Interface: Users will be able to give spoken instructions (e.g., “Take me to the kitchen”).
AI-Driven Navigation: Integrated AI will interpret natural-language voice commands, translate them into navigation goals, and direct the wheelchair to specific rooms or areas.
Adaptive Navigation: Integration of SLAM (Simultaneous Localization and Mapping) for path planning and environment learning.

## Installation
### Dependencies
- Python 3.10+
- RPi.GPIO (for GPIO control)
- rplidar
- vosk
- pyttsx3
- openai
- flask (option if interface is created)

### Setup



