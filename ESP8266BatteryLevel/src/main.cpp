#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <SPI.h>

#define LED_D1_WIFI_ON 5 // Turns on when WIFI connected. Flashes when connection lost
#define LED_D2_B_LOW 4 // Turns on when battery Low

const char *wifi_ssid = "YOUR_WIFI_SSD"; //Replace with your values
const char *wifi_password = "YOUR_WIFI_PASSWORD"; //Replace with your values

// Provide the server Domain URL or Ip address below
const char *hostURL = "YOUR_DOMAIN";

// Indicate battery-Low threshod below
const int BATTERY_LOW_THRESHOLD = 15; //15% is our battery Low level

unsigned long lastCheckTime = 0;
const unsigned long timerDelay = 5 * 60 * 1000; // 5 minutes
bool wifiConnected = false;
bool batteryLow = false;
unsigned int remoteBatteryLevel = 999;
char spi_output_buffer[5];

// Timer for Interrupt
os_timer_t myTimer;

char getAsciiDigit(char number){
  switch (number){
    case 0: return 48;
    case 1: return 49;
    case 2: return 50;
    case 3: return 51;
    case 4: return 52;
    case 5: return 53;
    case 6: return 54;
    case 7: return 55;   
    case 8: return 56;
    case 9: return 57;   
    default: return 63; //?
  }
}

void IRAM_ATTR onTimerRoutine() { 
  // Called every 10000ms

  //get battery level digits
  spi_output_buffer[0] = 'A'; //start byte

  if( remoteBatteryLevel >= 0 && remoteBatteryLevel < 101){
  
    spi_output_buffer[1] = getAsciiDigit( remoteBatteryLevel / 100);
    spi_output_buffer[2] = getAsciiDigit( remoteBatteryLevel/ 10);
    spi_output_buffer[3] = getAsciiDigit( remoteBatteryLevel % 10);
  
  }else{
    spi_output_buffer[1] = '?';
    spi_output_buffer[2] = '?';
    spi_output_buffer[3] = '?';
  }

  spi_output_buffer[4] = 'Z'; //end byte

 // Serial.println(spi_output_buffer);

	for( unsigned int i = 0; i < sizeof spi_output_buffer; i++){
    //Transfer data to slave device
    Serial.print(spi_output_buffer[i]);
	  SPI.transfer( spi_output_buffer[i] );
  }

  Serial.print('\n');

}

// ________________ START _________________//

void setup(){

  pinMode(LED_D1_WIFI_ON, OUTPUT);  // set pin as output
  pinMode(LED_D2_B_LOW, OUTPUT); // set pin as output

  Serial.begin(9600); // Enable serial output
  SPI.begin();  // begin SPI

  delay(100);

  WiFi.begin(wifi_ssid, wifi_password); // Connect to WIFI

  Serial.println("Connecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  wifiConnected = true;

  // Set up the timer
  os_timer_setfn( &myTimer, (os_timer_func_t*)onTimerRoutine, NULL);
  
  // Configure the timer to trigger every 1000ms, and repeat (true)
  os_timer_arm( &myTimer, 10000, true);  
  

}

// Our infinite Loop function for MCU
void loop(){
  // Check if we have waited long enough for delay duration since last http connect
  
  long time_diff = millis() - lastCheckTime;

  //NOTE: We are using abs() function below because millis() will eventually overflow
  // back to zero after ~49 days

  if ( abs(time_diff) > timerDelay || lastCheckTime == 0 ){
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client;
      HTTPClient http;

      wifiConnected = true;

      http.begin(client, hostURL);

      // Specify content-type header
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      // Our POST data payload
      String httpRequestData = "auth=YOUR_TOKEN&deviceId=esp8266_nmcu_12e";

      // Send HTTP POST request
      int responseCode = http.POST(httpRequestData);

      if (responseCode == 200)
      {
        String response = http.getString();
        Serial.println(response);

        // Parse JSON Response
        JsonDocument jsonDoc;

        // JSON input string.
        const char *json = response.c_str();

        // Deserialize the JSON document
        DeserializationError parseError = deserializeJson(jsonDoc, json);

        // Test if parsing succeeds.
        if (parseError)
        {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(parseError.f_str());
        }
        else
        {
          unsigned int batteryLevel = jsonDoc["battery"];
          // Note : jsonDoc['battery'] above will return 0 if the value is not present in response 
          
          remoteBatteryLevel = batteryLevel;

          if( batteryLevel > 0 && batteryLevel < BATTERY_LOW_THRESHOLD ){
            
            batteryLow = true;

          }else{
            
            batteryLow = false;
          
          }
        }
      }
      else
      {
        Serial.print("Error code: ");
        Serial.println(responseCode);
      }
      // Free resources
      http.end();
    }
    else
    {
      wifiConnected = false;
      Serial.println("WiFi Disconnected");
    }

    lastCheckTime = millis();
  }

  if( batteryLow ){
    // Turn On battery Low LED indication
    digitalWrite(LED_D2_B_LOW, HIGH);

  }else{

    digitalWrite(LED_D2_B_LOW, LOW);
  
  }

  if( wifiConnected ){
    // Keep D1 LED ON
    digitalWrite(LED_D1_WIFI_ON, HIGH);
    
  }else{
    //Flash D1 LED
  
    digitalWrite(LED_D1_WIFI_ON, HIGH); 
    delay(1000);
    digitalWrite(LED_D1_WIFI_ON, LOW); 
  }

  delay(2000);

}
