Broadcasting Bluetooth services using ESP32 Devkit V1 <a name="TOP"></a>
===================

## Table of Contents
* [Task Details](#Task-Details)
* [Hardware Components Used](#Hardware-Components-Used)
* [Tools Used](#Tools-Used)
* [Microcontroller Interfacing](#Microcontroller-Interfacing)
* [Schematic](#Schematic)
* [Complete Hardware Assembly](#Complete-Hardware-Assembly)
* [Firmware](#Firmware)
* [Project Overview Video](#Project-Overview-Video)
* [Issues Faced & Resolution](#Issues-Faced-&-Resolution)
* [Outputs](#Outputs)
* [nRF Connect Application interface](#nRF-Connect-Application-interface)
* [References](#References)

## Task Details
#### Services to be broadcasted:
* Temperature Measurement
* Humidity

#### Details about the service to broadcast over Bluetooth
* Service UUID: 00000002-0000-0000-FDFD-FDFDFDFDFDFD.
* Characteristics:
  * Temperature Measurement: Standard BLE characteristic - GATT Char UUID: 0x2A1C.
  * Humidity: Standard BLE characteristic - GATT Char UUID: 0x2A6F.
* Supported operations: Both characteristics support read and notify.

## Hardware Components Used
1. Espressif System's ESP32 Devkit V1 development board
2. DHT11 Temperature and Humidity sensor
3. 10K Ohm Resistor (Pull-up)
4. USB to MicroUSB cable
5. Breadboard
6. Jumpers

## Tools Used
#### Arduino IDE 
Arduino Integrated Development Environment or Arduino Software (IDE) contains a text editor for writing code, a message area, a text console, a toolbar with buttons for common functions and a series of menus. It connects to the Arduino hardware to upload programs and communicate with them. 
* For more details refer: <https://www.arduino.cc/en/software>

#### Fritzing
Fritzing is an open-source hardware initiative that makes electronics accessible as creative material for anyone. We offer a software tool, a community website and services in the spirit of Processing and Arduino, fostering a creative ecosystem that allows users to document their prototypes, share them with others, teach electronics in a classroom, and layout and manufacture professional PCBs. 
* For more details refer: <https://fritzing.org/>

## Microcontroller Interfacing
* Used OneWire protocol to interface the microcontroller to the DHT11 sensor:
* Used BLE protocol for Connectivity with the nRF Connect Mobile Application.

## Schematic
![Untitled Sketch 2_bb](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/321e2724-a62f-460c-bd77-e2680af8002b)

## Complete Hardware Assembly
![IMG_20240711_034424029](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/3f9e5228-5c99-4f26-be10-5f659cd05d99)
![IMG_20240711_034431219](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/186f0934-2f9e-417e-a4ab-6e76f5d279ac)

## Firmware
#### Link (Check the link below or directly download the .ino file or Word document that has the raw code)
- 

## Project Overview Video
#### Link
- https://drive.google.com/file/d/1l5og78Y-2ZODacyJ4YPPP19z49ZOXAzD/view?usp=sharing

## Issues Faced & Resolution
### ESP32 Not Advertising after disconnection
#### Approach 1: Used the callback function to restart advertising within the loop itself.
* Result = Successfully connected and reconnected multiple times with no issues (Check at the end of the [Project Overview Video](#Project-Overview-Video)).

### BLE Data transmission problem
#### Approach 1: Started with using custom characteristic UUIDs and tried to send the data as String values (I had worked on this before).
* Result = Successfully received both values Temperature Measurement & Humidity in Strings.
![Screenshot_20240709-181209](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/f7b39095-ee23-4b6a-a411-83da69e248e8)
* Comment = This was not asked in the task. It was mandatory to use the default GATT UUIDs.

#### Approach 2: Set the UUIDs to the required UUIDs (Temperature Measurement = 0x2A1C; Humidity = 0x2A6F) and tried to send the data as String values.
* Result = Error: Invalid Data Syntax @ nRF Application for Temperature Measurement value & Garbage value for Humidity value.
![Screenshot_20240710-001504](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/bcaded46-4741-448d-a486-981c6b312db2)
* Comment = This was not asked in the task. Getting the Humidity in % and Temperature Measurement in Celsius was mandatory.

#### Approach 3: Changed the Temperature characteristic UUID to default temperature (Temperature = 0x2AE6) and tried to send the data as unsigned 16-bit int. 
* Result = Successfully received both values Temperature Measurement & Humidity in unsigned 16-bit int values.
![Screenshot_20240709-200443](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/b02fc88f-02a1-4c9d-9f13-209e9dd9ec41)
* Comment = Checked the BLE documentation for the UUIDs (Temperature = 0x2AE6; Humidity = 0x2A6F) and their required data syntax type required.

#### Approach 4: Reset the Temperature characteristic UUID (Temperature Measurement = 0x2A1C) and tried to send the data as unsigned 16-bit int. 
* Result = Error: Invalid Data Syntax @ nRF Application for Temperature Measurement value, but successfully received values for Humidity in unsigned 16-bit int values.
![Screenshot_20240709-204813](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/7d2f5718-a76a-4721-b9be-b9850d86d69d)
* Comment = Checked the BLE documentation for the Temperature Measurement UUID (Temperature Measurement = 0x2A1C) and their required data syntax type required.

#### Approach 5: Tried to convert the temperature data to 32-bit float and send the data along with the humidity data as unsigned 16-bit int. 
* Result = Successfully received both values Temperature Measurement & Humidity in unsigned 16-bit int values, but Temperature Measurement unit was in Fahrenheit scale.
* Comment = Checked the BLE documentation for the Temperature Measurement UUID (Temperature Measurement = 0x2A1C) and their required data syntax type required and flags.
* Comment = Checked the conversion requires an IEEE11073 32-bit float data stream with 5 bytes (1 byte for C/F and 4 bytes for the actual value).

#### Approach 6: Changed the IEEE11073 32-bit float data stream with 5 bytes (1 byte for C/F and 4 bytes for the actual value) and set it to 0x00 flag for Celsius unit scale.
* Result = Successfully received both values Temperature Measurement & Humidity values on the nRF Connect Application with correct unit scales.
![Screenshot_20240710-001846](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/0b22935e-afef-471d-9152-78d379f40003)
* Comment = Checked the BLE documentation for the Temperature Measurement UUID (Temperature Measurement = 0x2A1C) and their required data syntax type required and flags.

## Outputs
## Serial Monitor messages
![Screenshot_2](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/1b3069f8-4238-404f-ab51-b2a7502a59ef)
![Screenshot_3](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/bc76e729-1961-4b9a-b3f5-209d3fdaec1c)
![Screenshot_4](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/ef43aa4c-d27b-4dfd-8d17-5964937cada9)
![Screenshot_5](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/c49f85a7-982e-4904-aac3-c5bad0dc3ebd)
![Screenshot_6](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/bf7b4575-6db3-4121-b900-6389dc6dca00)

## nRF Connect Application interface
![Screenshot_20240710-001846](https://github.com/Nirvan007/ESP32_BLE/assets/127144315/66f06f8a-154c-4137-beaa-8b890b05ff05)

## References
 - [1] https://github.com/nkolban/ESP32_BLE_Arduino
 - [2] https://github.com/RuiSantosdotme
 - [3] https://randomnerdtutorials.com/esp32-ble-server-environmental-sensing-service/
 - [4] https://github.com/makerhero/esp32-ble-dht11
 - [5] https://github.com/atc1441/ATC_MiThermometer/issues/150
 - [6] https://stackoverflow.com/questions/28899195/converting-two-bytes-to-an-ieee-11073-16-bit-sfloat-in-c-sharp/28901867#28901867
 - [7] https://stackoverflow.com/questions/60841331/dart-convert-ieee-11073-32-bit-float-to-a-simple-double/71732994#71732994
 - [8] https://docs.bosch-iot-suite.com/edge/edge-services/API/modules/com.prosyst.mbs.bluetooth.le.driver.api/allclasses-noframe.html
 - [9] https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.temperature_measurement.xml
