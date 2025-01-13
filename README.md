# Flood Detection System

## Prerequisites
- [Install Arduino IDE 2](https://docs.arduino.cc/software/ide/#ide-v2)
- [Install ESP8266 NodeMCU Board in Arduino IDE 2](https://randomnerdtutorials.com/installing-esp8266-nodemcu-arduino-ide-2-0/)

### Hardware Support
- ESP8266 board
- LED
- 330 ohmi rezistor
- Buzzer
- BME280

### Dependencies (HTTP server)

```
pip install flask
```

## How to run HTTP server

```
python server.py
```

## MQTT Web Application URL
https://ana-mirza.github.io/web-app.html

## How to setup node
Upload *ESP_MQTT_Server.ino* on the ESP8266 board.

## Application
The application running on the server allows the user to control the LED on the board.
