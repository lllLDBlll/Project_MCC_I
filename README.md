# Project_MCC_I
Project_MCC_I - Change this name after

## Main Functions
### Features:
- Read Temperature, Humidity and Ambient Light
- Detecting Objects and Gesture Motions
- Display datas on LCD
 
### How it works:
- Switch the Display's data for each parameter
- Detect if someone is in front of the Display and Stop the Finite State Machine (FSM)
- Each gesture will be a command

### Commands:
- Servo motors will rotate 180° horizontal and vertical
- Send the command to the MQTT Broker (for show act on a Web Application or Change something)
