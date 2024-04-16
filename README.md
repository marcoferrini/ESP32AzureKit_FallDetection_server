# ESP32AzureKit_FallDetection_client

Fall detection system based on ESP32 Azure IoT kit. 
This repository is the esp-idf project forlder used to develop the server of the fall detection system.

## Source code
The main folder contains the sorce code.

- `main.c` contains main task and fall detection task 
- `sensors.c` constains the i2c config functions and the function to interface the magnetometer and the accelerometer
- `bt_functions.c` contains the BLE gatt and gap functions
- `button.c` API for user-defined button
- `ssd1306.c` API for esp32 azure iot kit display ssd1306
- `fonts.c` structures definition for ssd1306


## Author 
Marco Ferrini
