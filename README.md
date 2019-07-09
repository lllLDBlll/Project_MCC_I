# Project_MCC_I
Project_MCC_I - Change this name after<p>
Target: Temperature Chamber

## ESP-02 MODBUS-MQTT
SSID: EOR_...<p>
Password: esp-open-rtos<p>
IP: 172.16.0.1<p>
WiFi config -> WiFi station<p>

## Main Functions

## Hardware
- Humidity and Temperature Sensor SHT21
- <s>Redundance Temperature with NTC</s>
- Cooler

### Features:
- Temperature Read
- Humidity Read
- Display datas on LCD
- Control overheat by a Cooler
- <s>Control underheat by a Peltier</s>
- <s>Ambient Light</s>
- <s>RGB Color Sensing</s>
- <s>Gesture Detection</s>

### How it works:
- Switch Display between SHT21 datas and MQTT publishes
- <s>Detect if someone is in front of the Display and Stop the Finite State Machine (FSM)</s>

### Commands:
- Turn On/Off Cooler

## Notes
MQTT Explorer http://mqtt-explorer.com/ <p>
mostquitto - UFSC <p>
Broker: 150.162.29.25 <p>
User: alunos <p>
Pwd: iFsC@2018 <p>
Port: 1883 <p>
Topic - esp_02 xx (leonardo) <p>
Publicando um Valor <p>
esp_02/atuador_0 -> Broker -> Application

### References:
https://community.createlabz.com/knowledgebase/1-2-switching-leds-using-gesture-sensor-ap05-9960-module/
https://www.youtube.com/watch?v=MCYJAyIXBdA


