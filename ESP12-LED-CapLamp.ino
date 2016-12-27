/*  
 *   For a Capacitive sensor with a metal object clipped to the sensor pin and an LED driver at the ledPin.
 *   Code by Thomas Friberg (https://github.com/tomtheswede)
 *   Updated 10/06/2016
 */

// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
 
//INPUT CONSTANTS
const char* ledID = "LED005"; //Name of sensor
const char* buttonID = "BUT005"; //Name of sensor
const char* deviceDescription = "Hallway Lamp";
const char* ssid = "TheSubway"; //Enter your WiFi network name here in the quotation marks
const char* password = "vanillamoon576"; //Enter your WiFi pasword here in the quotation marks
unsigned int localPort = 5007;  //UDP send port
const char* ipAdd = "192.168.0.100"; //Server address
byte packetBuffer[512]; //buffer for incoming packets
const unsigned int switchPin=2; //(0 on esp12)
const unsigned int sensorPin=4; //Must be an interupt pin (4 on esp12)
const unsigned int ledPin=5; // (5 on esp12)
const unsigned int sensorSamples=300; //10000 samples gives a value return rate of 2 per second. 1000 is good for continuous monitor
const unsigned int calibrateSamples=40; //These will be quicksorted so don't exceed ~100
const float thresholdSensitivity=3.4;
const unsigned int sensorArrayLength=5;
const unsigned int defaultFadeSpeed=10;
const unsigned int longPressLength=600; //Time in milliseconds for a long press
const unsigned int longerPressLength=3000; //Time in milliseconds for a longer press (all off)
const unsigned int PWMTable[101] = {0,1,2,3,5,6,7,8,9,10,12,13,14,16,18,20,21,24,26,28,31,33,36,39,42,45,49,52,56,60,64,68,72,77,82,87,92,98,103,109,115,121,128,135,142,149,156,164,172,180,188,197,206,215,225,235,245,255,266,276,288,299,311,323,336,348,361,375,388,402,417,432,447,462,478,494,510,527,544,562,580,598,617,636,655,675,696,716,737,759,781,803,826,849,872,896,921,946,971,997,1023}; //0 to 100 values for brightnes
const unsigned int reTriggerDelay=80; //minimum time in millis between button presses
const unsigned long tooLongPressLength=15000;
const unsigned long calibrateTimer=21600000; //21600000 is 6 hours

//GLOBAL VARIABLES
String data = "";
bool switchState=false; //default to start low
unsigned long switchTime;
unsigned long sensorTime;
unsigned int sampleCounter=0;
unsigned int sensorSum=0;
unsigned int sensorVal=0;
int calibrateSet[calibrateSamples];
bool calibrateTrigger=true;
unsigned int calibrateCount=0;
unsigned int minValue;
unsigned int maxValue;
unsigned int medValue;
unsigned int triggerPoint;
unsigned int fadeSpeed=defaultFadeSpeed;
bool triggerArray[4];
unsigned long pressTime=0;
bool recordAvail=false;
bool triggered=false;
bool messageAvail=false;
bool longPressTriggered;
bool longerPressTriggered;
bool pressed=false;
bool released=false;
unsigned int ledPinState = 0; //Default boot state of LEDs and last setPoint of the pin between 0 and 100
unsigned int ledSetPoint = 0;
unsigned int brightness = 100; //last 'on' setpoint for 0-100 scale brightness
long lastFadeTime = 0;
unsigned int calibrationFrequency=600000;
long calibrationTime=0;
bool calibratePrimer=false;
bool primeLongTrig=false;
bool primeLongerTrig=false;
int timerCount=0;
bool timerPrimer=false;

WiFiUDP Udp; //Instance to send packets

//-----------------------------------------------------------------------------

void setup() {
  delay(0);
  setupLines();
}

//-----------------------------------------------------------------------------

void loop() {
  SumSensorSamples();
  SetLedStates();
  FadeLEDs();
  RecalibrateCheck();
  delay(0);
}

//-----------------------------------------------------------------------------

void sensorTimer() { //What happens when the sensor pin changes value
  sensorTime=micros()-switchTime;
  recordAvail=true;
}

//-----------------------------------------------------------------------------

void RecalibrateCheck() {
  if (!calibrateTrigger && ((calibratePrimer && (millis()-calibrationTime>calibrationFrequency)) || (millis()-pressTime>calibrateTimer))) {
    Serial.println("Recallibrating...");
    calibratePrimer=false;
    calibrateTrigger=true;
    pressTime=millis();
  }
}

void setupLines() {
  pinMode(switchPin,OUTPUT);
  pinMode(sensorPin,INPUT);
  pinMode(ledPin,OUTPUT);
  Serial.begin(115200); //--------------COMMENT OUT WHEN SENDING TO PRODUCTION
  digitalWrite(switchPin,switchState); //Start low
  analogWrite(ledPin,0); //Start off
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with IP: ");
  // Print the IP address
  Serial.println(WiFi.localIP());

  //Open the UDP monitoring port
  Udp.begin(localPort);
  Serial.print("Udp server started at port: ");
  Serial.println(localPort);
  
  //Register on the network with the server after verifying connect
  delay(2000); //Time clearance to ensure registration
  SendUdpValue("REG",ledID,String(deviceDescription)); //Register LED on server
  digitalWrite(ledPin, HIGH); //Turn off LED while connecting
  delay(20); //A flash of light to confirm that the lamp is ready to take commands
  digitalWrite(ledPin, LOW); //Turn off LED while connecting
  ledSetPoint=0; // input a setpoint for fading as we enter the loop
  
  Serial.println("Capactitive sensor going online...");
  delay(500); //To make sure the state is set before an interrupt is applied
  //Set pin to start interrupts
  attachInterrupt(digitalPinToInterrupt(sensorPin),sensorTimer,CHANGE);
  //delay(0); //To make sure the state is set before an interrupt is applied
  //recordAvail=true;
  switchState= !switchState;
  switchTime=micros();
  digitalWrite(switchPin,switchState);
}

//-----------------------------------------------------------------------------

void SumSensorSamples() {
  if (micros()-switchTime>6000) { //This resets the timer if an interupt stuffs up a state change.
    sensorTime=micros()-switchTime;
    //Serial.println("Switch pin value timed out and reset");
    recordAvail=true;
  }
  if (recordAvail) {
    //Serial.println("test in recordAvail");
    recordAvail=false;
    sampleCounter++;
    sensorSum=sensorSum+sensorTime;
    if (sampleCounter>=sensorSamples) {
      sensorVal=sensorSum;
      sampleCounter=0;
      sensorSum=0;
      codeBetweenReadings();
    }
    //Reset pin for interruption
    switchState= !switchState;
    switchTime=micros();
    digitalWrite(switchPin,switchState);
  }
}

void codeBetweenReadings() {
  CalibrateSensor();  
  DetectTrigger();
  Serial.println(sensorVal); //------------Uncomment this if you want to see the values coming out of the sensor---------
  data=ParseUdpPacket(); //Code for receiving UDP messages
  if (data!="") {
    ProcessLedMessage(data);//Conditionals for switching based on LED signal
  }
  CheckTimer();
}

void CalibrateSensor() {
  if (calibrateTrigger) {
    calibrateSet[calibrateCount]=sensorVal;
    calibrateCount++;
    //Serial.print("Calibrate test");
  }
  if (calibrateTrigger && (calibrateCount>=calibrateSamples)) {
    Serial.println("QuickSorting calibration data...");
    quickSort(calibrateSet,0,calibrateSamples-1);
    minValue=calibrateSet[(calibrateSamples/8)-1]; //Take the 5th number in the set
    maxValue=calibrateSet[calibrateSamples-(calibrateSamples/8)-1]; //take the 5th last number in the set
    medValue=calibrateSet[calibrateSamples/2]; //take the middle of the set
    triggerPoint=medValue+(maxValue-minValue)*thresholdSensitivity;
    calibrateTrigger=false;
    calibrateCount=0;
    Serial.print("Calibrated with min, med, max and trigger point of : ");
    Serial.print(minValue);
    Serial.print(", ");
    Serial.print(medValue);
    Serial.print(", ");
    Serial.print(maxValue);
    Serial.print(", ");
    Serial.println(triggerPoint);
    triggered=false; //reset to trigger off if not already
  }
}

void DetectTrigger() {
  if (!calibrateTrigger) {
    triggerArray[0]=triggerArray[1];
    triggerArray[1]=triggerArray[2];
    triggerArray[2]=triggerArray[3];
    if (sensorVal>triggerPoint) {
      triggerArray[3]=true;
    }
    else {
      triggerArray[3]=false;
    }
    
    if (!triggered && triggerArray[0] && triggerArray[1] && triggerArray[2] && triggerArray[3] && (millis()-pressTime>reTriggerDelay)) {
      triggered=true;
      pressed=true;
      primeLongTrig=true;
      primeLongerTrig=true;
      pressTime=millis();
      calibratePrimer=false;
      Serial.println("Pressed button.");
    }
    else if (triggered && !triggerArray[0] && !triggerArray[1] && !triggerArray[2] && !triggerArray[3]) {
      triggered=false;
      released=true;
      Serial.println("Released button.");
    }
    //if the button has been held long enough, make a long trigger event
    if (triggered && primeLongTrig && (millis()-pressTime>longPressLength)) {
      longPressTriggered=true;
      primeLongTrig=false;
      Serial.println("Long press triggered.");
    }
    //if the button has been held long enough, make a longer trigger event
    if (triggered && primeLongerTrig && (millis()-pressTime>longerPressLength)) {
      longerPressTriggered=true;
      primeLongerTrig=false;
      Serial.println("Longer press triggered.");
    }
    //If the sensor is triggered for too long, re-calibrate because something is up...
    if (triggered && (millis()-pressTime>tooLongPressLength)) {
      calibrateTrigger=true;
      Serial.println("Too long press triggered.");
    }
  }
}

String ParseUdpPacket() {
  int noBytes = Udp.parsePacket();
  String udpData = "";
  if ( noBytes ) {
    Serial.print("---Packet of ");
    Serial.print(noBytes);
    Serial.print(" characters received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,noBytes); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i=1;i<=noBytes;i++) {
      udpData = udpData + char(packetBuffer[i - 1]);
    } // end for
    Serial.println("---Data reads: " + udpData);
  } // end if
  return udpData;
}

void ProcessLedMessage(String dataIn) {
  String devID = "";
  String message = "";

  devID=dataIn.substring(0,6); //Break down the message in to it's parts
  //Serial.println("DevID reads after processing: " + devID);
  message=dataIn.substring(7);
  //Serial.println("Message reads after processing: " + message);
  
  if (devID==ledID) { //Only do this set of commands if there is a message for an LED device
    //Enables slow fading
    if (message.startsWith("fade")) { 
      int messagePos=message.indexOf(" ");
      //Serial.println("Position of space is " + messagePos);
      String fadeVal=message.substring(4,messagePos);
      //Serial.println("Fade value is " + fadeVal);
      fadeSpeed=atoi(fadeVal.c_str());
      //Serial.println("Fade speed set to " + fadeSpeed);
      message=message.substring(messagePos+1); //Cutting 'fade' from the message
      Serial.println("Custom fade increment speed of " + fadeVal + " miliseconds trigged");
      Serial.println("Message trimmed to : " + message);
    }
    else {
      fadeSpeed=defaultFadeSpeed;
    }
    //Enables timed commands
    if (message.startsWith("timer")) { 
      int messagePos=message.indexOf(" ");
      //Serial.println("Position of space is " + messagePos);
      String timerVal=message.substring(5,messagePos);
      //Serial.println("Fade value is " + fadeVal);
      timerCount=atoi(timerVal.c_str());
      //Serial.println("Fade speed set to " + fadeSpeed);
      message=message.substring(messagePos+1); //Cutting 'timer' from the message
      Serial.println("Custom timer of " + timerVal + " seconds set");
      Serial.println("Message trimmed to: " + message);
    }
    else {
      timerCount=0;
    }
    //Enables instant toggling
    if (((message=="instant toggle")||(message=="instant on")||(message=="instant 100")) && (ledPinState==0)) { //Only turn on if already off
      SendUdpValue("LOG",ledID,String(100));
      ledPinState=100;
      ledSetPoint=100;
      digitalWrite(ledPin, HIGH);
      Serial.println("---Instant on triggered");
    }
    else if (((message=="instant toggle")  || (message=="instant off") || (message=="instant 0")) && (ledPinState>0)) { //Only turn off if already on
      SendUdpValue("LOG",ledID,String(0));
      ledPinState=0;
      ledSetPoint=0;
      digitalWrite(ledPin, LOW);
      Serial.println("---Instant off triggered");
    }
    //Enables regular dimming
    if (((message=="toggle")||(message=="on")) && (ledPinState==0)) { //Only turn on if already off
      SendUdpValue("LOG",ledID,String(brightness));
      ledSetPoint=brightness; // input a setpoint for fading
      Serial.println("---On triggered");
      //Serial.println("LED state is now has set point of " + String(ledSetPoint));
    }
    else if (((message=="toggle") || (message=="off") || (message=="0")) && (ledPinState>0)) { //Only turn off if already on
      SendUdpValue("LOG",ledID,String(0));
      ledSetPoint=0; // input a setpoint for fading
      Serial.println("---Off triggered");
      //Serial.println("LED state is now has set point of " + String(ledSetPoint));
    }
    else if (message=="hold") { //For stopping the fade
      brightness=ledPinState;
      SendUdpValue("LOG",ledID,String(brightness));
      ledSetPoint=brightness;
      Serial.println("LED state is now has set point of " + String(ledSetPoint));
    }
    else if ((atoi(message.c_str())>0) && (atoi(message.c_str())<=1023) && (ledPinState!=atoi(message.c_str()))) { //Change brightness
      brightness=atoi(message.c_str());
      SendUdpValue("LOG",ledID,String(brightness));
      ledSetPoint=brightness; // input a setpoint for fading
      //Serial.print("---PWM trigger: ");
      Serial.println("LED state is now has set point of " + String(ledSetPoint));
    }
  }
}

void CheckTimer() {
  
  if(millis() % 1000 == 0 && timerPrimer) {
    if(timerCount==0) {
      //Do nothing
    }
    else if (timerCount>1) {
      timerCount=timerCount-1;
      //Serial.println("Timer value reduced to " + String(timerCount) + "");
    }
    else {
      timerCount=timerCount-1;
      SendUdpValue("FWD",ledID,"off");
    }
    timerPrimer=false;
  }
  if(millis() % 1000 == 1) {
    timerPrimer=true;
  }
}

//-----------------------------------------------------------------------------

void SetLedStates() {
  if (pressed && (ledPinState==0)) { //Switch on
    SendUdpValue("LOG",buttonID,"toggle");
    Serial.println("Toggle On Triggered. ");
  }
  else if (pressed && (ledPinState>0) && (ledPinState!=ledSetPoint)) { //Hold
    SendUdpValue("LOG",buttonID,"hold");
    Serial.println("Hold Triggered. ");
  }
  else if (pressed && (ledPinState>0)) { //Off
    calibrationTime=millis();
    calibratePrimer=true;
    SendUdpValue("LOG",buttonID,"toggle");
    Serial.println("Toggle Off Triggered. ");
  }
  else if (longPressTriggered && (ledPinState==0)) { // Full power (or all off)
    longPressTriggered=false;
    SendUdpValue("LOG",buttonID,"100");
    Serial.println("Full Power Triggered. ");
  }
  else if (longPressTriggered && (ledPinState>0)) { // Full power
    longPressTriggered=false;
    SendUdpValue("LOG",buttonID,"100");
    Serial.println("Full Power Triggered. ");
  }
  else if (longerPressTriggered) { // Full power
    longerPressTriggered=false;
    SendUdpValue("LOG",buttonID,"all off");
    Serial.println("All Off Triggered. ");
  }
  pressed=false;
  
}

void FadeLEDs() {
  if ((millis()-lastFadeTime>fadeSpeed) && (ledPinState<ledSetPoint)) {
    ledPinState = ledPinState + 1;
    analogWrite(ledPin, PWMTable[ledPinState]);
    //Serial.println("LED state is now set to " + String(ledPinState));
    lastFadeTime=millis();
  }
  else if ((millis()-lastFadeTime>fadeSpeed) && (ledPinState > ledSetPoint)) {
    ledPinState = ledPinState - 1;
    analogWrite(ledPin, PWMTable[ledPinState]);
    //Serial.println("LED state is now set to " + String(ledPinState));
    lastFadeTime=millis();
  }
}

void SendUdpValue(String type, String sensorID, String value) {
  //Print GPIO state in serial
  Serial.print("-Value sent via UDP: ");
  Serial.println(type + "," + sensorID + "," + value);

  // send a message, to the IP address and port
  Udp.beginPacket(ipAdd,localPort);
  Udp.print(type);
  Udp.write(",");
  Udp.print(sensorID);
  Udp.write(",");
  Udp.print(value); //This is the value to be sent
  Udp.endPacket();
}

void quickSort(int arr[], int left, int right) { //Takes a list and arranges the items from left to right in ascending order
  int i = left, j = right;
  int tmp;
  int pivot = arr[(left + right) / 2];
  
  /* partition */
  while (i <= j) {
       while (arr[i] < pivot)
             i++;
       while (arr[j] > pivot)
             j--;
       if (i <= j) {
             tmp = arr[i];
             arr[i] = arr[j];
             arr[j] = tmp;
             i++;
             j--;
       }
  };
  
  /* recursion */
  if (left < j)
       quickSort(arr, left, j);
  if (i < right)
       quickSort(arr, i, right);
}

