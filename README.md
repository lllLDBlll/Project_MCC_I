# Project_MCC_I
Project_MCC_I - Change this name after<p>
Suggest: Temperature Chamber

## ESP-02 MODBUS-MQTT
SSID: EOR_...<p>
Password: esp-open-rtos<p>
IP: 172.16.0.1<p>
WiFi config -> WiFi station<p>

## Main Functions

## Hardware
- Humidity and Temperature Sensor SHT21
- Temperature Sensor with NTC
- Cooler

### Features:
- Temperature Read
- Humidity Read
- Display datas on LCD
- Control overheat by a Cooler
- Control underheat by a Peltier
- <s>Ambient Light</s>
- <s>RGB Color Sensing</s>
- <s>Gesture Detection</s>

### How it works:
- Switch the Display's between date and data
- Detect if someone is in front of the Display and Stop the Finite State Machine (FSM)
- Each gesture will be a command

### Commands:
- Servo motors will rotate 180° horizontal and vertical
- Send the command to the MQTT Broker (for show act on a Web Application or Change something)
- Turn On/Off 4 different LEDs



### References:
https://community.createlabz.com/knowledgebase/1-2-switching-leds-using-gesture-sensor-ap05-9960-module/
https://www.youtube.com/watch?v=MCYJAyIXBdA


