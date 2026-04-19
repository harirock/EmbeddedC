📡 STM32 Accelerometer LED Indicator
📖 Overview

This project demonstrates how to interface an SPI-based accelerometer with an STM32 microcontroller and visualize motion using onboard LEDs.

The system reads acceleration data (X and Y axes) and lights up different LEDs based on the tilt direction.

🚀 Features
SPI communication with accelerometer (LIS3DSH / LIS302DL)
Real-time tilt detection
LED indication for direction:
🔴 Right
🔵 Left
🟢 Forward
🟠 Backward
Built using STM32 HAL (CubeIDE)
🧰 Hardware Used
STM32F407 Discovery Board
Onboard Accelerometer (LIS3DSH / LIS302DL)
Onboard LEDs (PD12–PD15)
⚙️ Software & Tools
STM32CubeIDE
STM32 HAL Drivers
C Programming Language
🔌 Pin Configuration
Function	Pin
SPI1_SCK	PA5
SPI1_MISO	PA6
SPI1_MOSI	PA7
CS (Chip Select)	PE3
LEDs	PD12–PD15
🧠 How It Works
Initialize system (HAL, clock, GPIO, SPI)
Detect accelerometer using WHO_AM_I register
Configure sensor
Continuously read X and Y axis data
Compare values with threshold
Turn ON corresponding LED
📊 Logic
if(x > threshold)   → RIGHT (RED LED)
if(x < -threshold)  → LEFT (BLUE LED)
if(y > threshold)   → FORWARD (GREEN LED)
if(y < -threshold)  → BACKWARD (ORANGE LED)
▶️ How to Run
Open project in STM32CubeIDE
Connect STM32 board via USB
Build the project
Flash to the board
Tilt the board and observe LEDs
⚠️ Important Notes
SPI must be configured with:
CPOL = HIGH
CPHA = 2nd Edge
Use slower baud rate for stable communication
Ensure correct CS pin handling

📌 Future Improvements
    Edge AI
👨‍💻 Author 

Nellore Harikrishna<nharikrishna6231@gmail.com>

⭐ Acknowledgment

Based on STM32 HAL libraries and onboard sensor documentation.

📜 License

This project is open-source and free to use.