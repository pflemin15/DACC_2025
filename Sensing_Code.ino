#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "SparkFun_ENS160.h"


/*****      Preamble for ESP NOW Use *****/
// Structure to send data
typedef struct Message {
    char text[25];
} Message;

Message outgoingMessage;

// Peer MAC address 
uint8_t receiverMAC[] = {0x40, 0x4C, 0xCA, 0x57, 0xF5, 0xF0}; 

// Callback when message is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void sendMessage(const char *msg) {
    strcpy(outgoingMessage.text, msg);
    esp_now_send(receiverMAC, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
    Serial.print("Sent: ");
    Serial.println(msg);
}

//To track the mode to ensure the microprocessors are synced
int curOpMode = 0;
int lastOpMode = 0;
const int startButton = 2;
unsigned long startTime = 0;

/**** Definitions for Thermistor calculations    ****/
#define ADS_GAIN 1        // Gain = 1 (±4.096V range)
#define V_REF 3.3         // Reference voltage 
#define RESISTOR_FIXED 2000  // Series resistor value (2kΩ)
#define THERMISTOR_NOMINAL 10000  // Resistance at 25°C (10kΩ)
#define BETA 3978         // Beta value
#define T0 298.15         // 25°C in Kelvin

// Create ADS1115 and ENS160 objects
Adafruit_ADS1115 ads; 
SparkFun_ENS160 myENS1;
SparkFun_ENS160 myENS2;
SparkFun_ENS160 myENS3;
SparkFun_ENS160 myENS4;    
int ensStatus1, ensStatus2, ensStatus3, ensStatus4; 

/*****    Initialize functions      *****/

float getTemperature(int channel);
void initSensors();
int Adsorb1_Desorb2(float CO2_cham1_in, float CO2_cham1_out, float CO2_cham2_in, float temp_cham2);
int Mix_Chambers(float temp_cham1, float temp_cham2);
int Desorb1_Adsorb2(float CO2_cham2_in, float CO2_cham2_out, float CO2_cham1_in, float temp_cham1);

void setup() {

    Serial.begin(115200);
    Wire.begin(6,7);

    initSensors();


    //Set up esp now to send data to partner
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    pinMode(startButton, INPUT_PULLUP);

    //Only leave the setup function if the button is pushed
      while (digitalRead(startButton) != LOW)
  {
      delay(100); //For CPU load
  }
  curOpMode = 1;
  startTime = millis();
}

void loop() {

  //Get Temperature measurements
  float temp1 = getTemperature(0);
  float temp2 = getTemperature(1);
  float temp3 = getTemperature(2);
  float temp4 = getTemperature(3);
  float temp_cham1 = (temp1 + temp2)/2;
  float temp_cham2 = (temp3 + temp4)/2;
  float CO2_cham1_in, CO2_cham1_out, CO2_cham2_in, CO2_cham2_out;

  //Get CO2 measurements
  if( myENS1.checkDataStatus() )
  CO2_cham1_in = myENS1.getECO2();

  if( myENS2.checkDataStatus() )
  CO2_cham1_out = myENS2.getECO2();
  
  if( myENS2.checkDataStatus() )
  CO2_cham2_in = myENS3.getECO2();

  if( myENS4.checkDataStatus() )
  CO2_cham2_out = myENS4.getECO2();

   switch(curOpMode){

    case 1:
      curOpMode = Adsorb1_Desorb2(CO2_cham1_in, CO2_cham1_out, CO2_cham2_in, temp_cham1);
      break;
    
    case 2:
      curOpMode = Mix_Chambers(temp_cham1, temp_cham2);
      break;

    case 3:
      curOpMode = Desorb1_Adsorb2(CO2_cham2_in, CO2_cham2_out, CO2_cham1_in, temp_cham2);
      break;
  }

  delay(10000); //No need to update more than once every 10 seconds 
}

int Adsorb1_Desorb2(float CO2_cham1_in, float CO2_cham1_out, float CO2_cham2_in, float temp_cham2){
  int nextMode = 1;
  static bool tempReached2 = false;

  //If CO2 input and output levels are similar outside of initial few minutes then sorbent is saturated
  if (((CO2_cham1_in-CO2_cham1_out) < 100) && ((millis()-startTime > (10*60*1000))))
  {
    //Signal to actuator microprocessor that 
    nextMode = 2;
    tempReached2 = false;
    sendMessage("Exit");
  }
  //
  else if(temp_cham2 > 200)
  {
    sendMessage("Heat off");
    tempReached2 = true;
  }
  else if((temp_cham2 < 190) && (tempReached2 == true))
    sendMessage("Heat on");
  
  //Desorption process has completed (This may happen multiple times within a cycle)
  if(tempReached2 == true && CO2_cham2_in > 5000)
  {
    sendMessage("Extract");
  }

  lastOpMode = 1;
  return nextMode;
}

int Mix_Chambers(float temp_cham1, float temp_cham2){
  int nextMode = 2;
  //If within a satisfactory value, heat transfer has occured and next step can be taken
  if ((temp_cham1-temp_cham2) < 50 || (temp_cham2-temp_cham1) < 50)
  {
    if(lastOpMode == 1){
      nextMode = 3;
    }
    else 
    {
      nextMode = 1;
    }
    sendMessage("Exit");
  }
  //Start a timer for control
  startTime = millis();
  return nextMode;
}

int Desorb1_Adsorb2(float CO2_cham2_in, float CO2_cham2_out, float CO2_cham1_in, float temp_cham1){
  int nextMode = 3;
  static bool tempReached1 = false;

  //If CO2 input and output levels are similar outside of initial few minutes then sorbent is saturated
  if (((CO2_cham2_in-CO2_cham2_out) < 100) && ((millis()-startTime > (10*60*1000))))
  {
    //Signal to actuator microprocessor that 
    nextMode = 2;
    tempReached1 = false;
    sendMessage("Exit");
  }
  else if(temp_cham1 > 200)
  {
    sendMessage("Heat off");
    tempReached1 = true;
  }
  else if((temp_cham1 < 190) && (tempReached1 == true))
    sendMessage("Heat on");
  
  //Desorption process has completed (This may happen multiple times within a cycle)
  if(tempReached1 == true && CO2_cham1_in > 5000)
  {
    sendMessage("Extract");
  }

  lastOpMode = 3;
  return nextMode;
}

float getTemperature(int channel) {
    /* Gets temperature based on reading from adc */

    // Read and convert ADC value to voltage
    int16_t raw_value = ads.readADC_SingleEnded(channel);  
    float voltage = (raw_value * V_REF) / 32767.0;   

    // Calculate thermistor resistance
    float resistance = RESISTOR_FIXED * (V_REF / voltage - 1);

    // Use Steinhart-Hart equation
    float temperature_kelvin = 1.0 / ((1.0 / T0) + (log(resistance / THERMISTOR_NOMINAL) / BETA));
    
    //Kelvin -> Celsius
    float temperature_celsius = temperature_kelvin - 273.15; 

    return temperature_celsius; // Return temperature
}

void initSensors()
{
  // Initialize Sensors
  if( !myENS1.begin() )
  {
    Serial.println("Could not communicate with the ENS160 (Sensor 1), check wiring.");
    while(1); // Infinite loop to stop further execution
  }

  if( !myENS2.begin() )
  {
    Serial.println("Could not communicate with the ENS160 (Sensor 2), check wiring.");
    while(1); // Infinite loop to stop further execution
  }

  if( !myENS3.begin() )
  {
    Serial.println("Could not communicate with the ENS160 (Sensor 3), check wiring.");
    while(1); // Infinite loop to stop further execution
  }

  if( !myENS4.begin() )
  {
    Serial.println("Could not communicate with the ENS160 (Sensor 4), check wiring.");
    while(1); // Infinite loop to stop further execution
  }
  // Reset and set all sensors to standard operation
  myENS1.setOperatingMode(SFE_ENS160_RESET);
  delay(100);
  myENS1.setOperatingMode(SFE_ENS160_STANDARD);

  myENS2.setOperatingMode(SFE_ENS160_RESET);
  delay(100);
  myENS2.setOperatingMode(SFE_ENS160_STANDARD);

  myENS3.setOperatingMode(SFE_ENS160_RESET);
  delay(100);
  myENS3.setOperatingMode(SFE_ENS160_STANDARD);

  myENS4.setOperatingMode(SFE_ENS160_RESET);
  delay(100);
  myENS4.setOperatingMode(SFE_ENS160_STANDARD);

  // Get status for all sensors
  ensStatus1 = myENS1.getFlags();
  ensStatus2 = myENS2.getFlags();
  ensStatus3 = myENS3.getFlags();
  ensStatus4 = myENS4.getFlags();
  
  Serial.print("Sensor 1 Gas Sensor Status: ");
  Serial.println(ensStatus1);
  Serial.print("Sensor 2 Gas Sensor Status: ");
  Serial.println(ensStatus2);
  Serial.print("Sensor 3 Gas Sensor Status: ");
  Serial.println(ensStatus3);
  Serial.print("Sensor 4 Gas Sensor Status: ");
  Serial.println(ensStatus4);
}
