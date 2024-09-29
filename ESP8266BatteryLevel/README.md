# ESP8266-Based Remote Battery Monitoring and Low-Level Indication

This project aims to remotely monitor the battery level of a device by fetching data from a server and indicating when the battery is low. The ESP8266 WiFi module is used to periodically connect to a server via HTTP and retrieve the battery level. If the battery level is low, an LED will turn on, signaling the need for charging.

## Why This Project?

Remotely monitoring battery levels is valuable for IoT applications where the device lacks a display or is not frequently checked. This project can be extended to include hardware that enables automatic charging of the client device (coming soon in a future project).

## Platform and Hardware

- Developed using PlatformIO IDE in Visual Studio Code.
- WiFi module: NodeMCU ESP8266 12-E.
- Arduino Framework is used.

## Dependencies

To set up the project in PlatformIO:

1. Set your board to Espressif ESP8266 ESP-12E (or an equivalent board). This will automatically add the necessary libraries.
2. Add the [ArduinoJson](https://www.arduino.cc/reference/en/libraries/arduinojson/) library to the project:
   - Navigate to the PlatformIO tab > Libraries.
   - Search for "ArduinoJson" and add the "ArduinoJson" library by Benoit Blanchon.

## Server API Response

For each HTTP request, your web server should return a JSON-encoded response in the following format:

```json
{ "battery": 75, "param2": "value2" }
```

The MCU will look for the "battery" parameter, which should be an integer value.

## Setup Instructions

1. Open the `src/main.cpp` file and add your Wi-Fi credentials:

```
const char *wifi_ssid = "YOUR_WIFI_SSID"; // Replace with your values
const char *wifi_password = "YOUR_WIFI_PASSWORD"; // Replace with your values
```

2. Provide the URL path to your web server:

```
const char *hostURL = "http://YOUR_SERVER_DOMAIN/PATH"; // Replace with your values
```
3. If your HTTP request requires authentication via an API key/token, add your key value:

```
String httpRequestData = "auth=YOUR_API_KEY&deviceId=esp8266_nmcu_12e";
```
Replace `YOUR_API_KEY` with your own key and modify deviceId as needed.






