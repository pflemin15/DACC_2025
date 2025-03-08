#include <esp_now.h>
#include <WiFi.h>

/****       Pin Definitions        *****/
  // Define pin variable names
const int fanPin1 = 7;
const int fanPin2 = 0;
const int heatPin1 = 10;
const int heatPin2 = 11;
const int pumpPin = 6;
const int qSolPin1 = 4;
const int qSolPin2 = 5;
const int SolPin1 = 23;
const int SolPin2 = 22;
const int SolPin3 = 21;
const int SolPin4 = 20;
const int SolPin5 = 19;
const int SolPin6 = 18;

int lastState = 0;
int operatingMode = 0;


/****           ESP_NOW prep        *****/

// Message structure - MUST BE THE SAME AS SENDER MESSAGE STRUCTURE
typedef struct Message {
    char text[25];  // Holds the message string
} Message;

//Variables for receivinf data from other esp
Message incomingMessage;
char latestRecept[25] = "Exit";

// Callback function for receiving data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
    Serial.print("Received Message: ");
    strcpy(latestRecept, incomingMessage.text);
    Serial.println(latestRecept);
    

}

/****         Function Initialization       ****/

void initPins();
void Exit_Sequence();
int Idle();
int Adsorb1_Desorb2();
int Mix_Chambers();
int Desorb1_Adsorb2();

void setup() {

  //Start up serial communication
  Serial.begin(115200);

  // Initialize pins to ensure they are in output mode
  initPins();

  //Set up ESP NOW communication
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
  
  //Set up receiving funciton 
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

}

void loop() {
  
  switch(operatingMode){
    case 0:
      operatingMode = Idle();
      break;

    case 1:
      operatingMode = Adsorb1_Desorb2();
      break;
    
    case 2:
      operatingMode = Mix_Chambers();
      break;

    case 3:
      operatingMode = Desorb1_Adsorb2();
      break;
  }


}



/*****       FUNCTIONS        *****/

int Idle(){
  //Serial.println("0"); //Debugging purposes
  //Display "Idle mode, press button to start process"
  int i=0;
  while ((strcmp(latestRecept, "Exit") == 0))){
    delay(1000); 
  }

  Exit_Sequence();
  return 1;
}

int Adsorb1_Desorb2() {
  Serial.println("1");

  // Turn on the fan
  digitalWrite(fanPin1, HIGH);
  Serial.println("Fan 1 on");

  int nextState = 1;  // Init the next state
  digitalWrite(heatPin1, HIGH);
  Serial.println("Heater 1 ON");

  //Turn on solenoids to allow for air to blow thorugh chamber 1
  digitalWrite(SolPin1, HIGH);
  digitalWrite(SolPin2, HIGH);
  Serial.println("Chamber 1 airflow");

  while (1) {
    if (strcmp(latestRecept, "Exit") == 0) {  // Exit condition received
      Serial.println("Exiting function...");
      nextState = 2;
      break;
    } 
    else if (strcmp(latestRecept, "Heat off") == 0) {  // Turn OFF heater
      digitalWrite(heatPin1, LOW);
      Serial.println("Heater OFF");
    } 
    else if (strcmp(latestRecept, "Heat on") == 0) {  // Turn ON heater
      digitalWrite(heatPin1, HIGH);
      Serial.println("Heater ON");
    } 
    else if (strcmp(latestRecept, "Extract") == 0) {  // Activate pump and solenoid 2
      digitalWrite(pumpPin, HIGH);
      digitalWrite(qSolPin2, HIGH);
      digitalWrite(heatPin1, LOW);
      Serial.println("Pump and qSolenoid2 ON");
    }

    // Clear the received message to prevent repeated processing
    latestRecept[0] = '\0';

    delay(100);  // Short delay to prevent CPU overload
  }

  Exit_Sequence();

  lastState = 1;
  return nextState;  // Return the next state based on the received command
}

int Mix_Chambers(){

  Serial.println("2");

   //Turn on solenoids to allow for mixing of chambers
  digitalWrite(SolPin5, HIGH);
  digitalWrite(SolPin6, HIGH);

  //Enter a loop that will only be exitted when esp receives an exit message
  while (strcmp(latestRecept, "Exit") != 0)
  {
    delay(100);
  }

  Exit_Sequence();

  if(lastState == 1)
    return 3;
  else if(lastState == 3)
    return 1;
  else
    return 0;
}

int Desorb1_Adsorb2() {
  Serial.println("3");

  // Turn on the fan
  digitalWrite(fanPin2, HIGH);
  Serial.println("Fan 2 ON");

  int nextState = 1;  // Init the next state
  digitalWrite(heatPin2, HIGH);
  Serial.println("Heater 2 ON");

  //Turn on solenoids to allow for air to blow thorugh chamber 1
  digitalWrite(SolPin3, HIGH);
  digitalWrite(SolPin4, HIGH);
  Serial.println("Chamber 2 airflow");

  while (1) {
    if (strcmp(latestRecept, "Exit") == 0) {  // Exit condition received
      Serial.println("Exiting function...");
      nextState = 2;
      break;
    } 
    else if (strcmp(latestRecept, "Heat off") == 0) {  // Turn OFF heater
      digitalWrite(heatPin2, LOW);
      Serial.println("Heater 2 OFF");
    } 
    else if (strcmp(latestRecept, "Heat on") == 0) {  // Turn ON heater
      digitalWrite(heatPin2, HIGH);
      Serial.println("Heater 2 ON");
    } 
    else if (strcmp(latestRecept, "Extract") == 0) {  // Activate pump and solenoid 2
      digitalWrite(pumpPin, HIGH);
      digitalWrite(qSolPin2, HIGH);
      digitalWrite(heatPin1, LOW);
      Serial.println("Pump and qSolenoid 2 ON");
    }

    // Clear the received message to prevent repeated processing
    latestRecept[0] = '\0';

    delay(100);  // Short delay to prevent CPU overload
  }

  Exit_Sequence();
  lastState = 3;
  return nextState;  // Return the next state based on the received command
}

void initPins(){
  /* Function to initialize all of the pins for the program */

  //Set as output pin
  pinMode(fanPin1, OUTPUT);
  pinMode(fanPin2, OUTPUT);
  pinMode(heatPin1, OUTPUT);
  pinMode(heatPin2, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(qSolPin1, OUTPUT);
  pinMode(qSolPin2, OUTPUT);
  pinMode(SolPin1, OUTPUT);
  pinMode(SolPin2, OUTPUT);
  pinMode(SolPin3, OUTPUT);
  pinMode(SolPin4, OUTPUT);
  pinMode(SolPin5, OUTPUT);
  pinMode(SolPin6, OUTPUT);

  //Set all pins to low in case one was left high on shutoff
  digitalWrite(fanPin1, LOW);
  digitalWrite(fanPin2, LOW);
  digitalWrite(heatPin1, LOW);
  digitalWrite(heatPin2, LOW);
  digitalWrite(pumpPin, LOW);
  digitalWrite(qSolPin1, LOW);
  digitalWrite(qSolPin2, LOW);
  digitalWrite(SolPin1, LOW);
  digitalWrite(SolPin2, LOW);
  digitalWrite(SolPin3, LOW);
  digitalWrite(SolPin4, LOW);
  digitalWrite(SolPin5, LOW);
  digitalWrite(SolPin6, LOW);

}

void Exit_Sequence(){
    //Turn off all peripherals before heading to next stage (Keeps project safer)
    //If something needs to be on in next sequence it will only be off for a very short time
  digitalWrite(fanPin1, LOW);
  digitalWrite(fanPin2, LOW);
  digitalWrite(heatPin1, LOW);
  digitalWrite(heatPin2, LOW);
  digitalWrite(pumpPin, LOW);
  digitalWrite(qSolPin1, LOW);
  digitalWrite(qSolPin2, LOW);
  digitalWrite(SolPin1, LOW);
  digitalWrite(SolPin2, LOW);
  digitalWrite(SolPin3, LOW);
  digitalWrite(SolPin4, LOW);
  digitalWrite(SolPin5, LOW);
  digitalWrite(SolPin6, LOW);

  latestRecept[0] = '\0';
  Serial.println("Exiting stage, all peripherals off and received message reset");
}
