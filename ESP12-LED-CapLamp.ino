/*  
 *   For a Capacitive sensor with a metal object clipped to the sensor pin and an LED driver at the ledPin.
 *   Code by Thomas Friberg (https://github.com/tomtheswede)
 *   Updated 10/06/2016
 */
 
//INPUT CONSTANTS
const unsigned int switchPin=15; //(15 on esp12)
const unsigned int sensorPin=4; //Must be an interupt pin (4 on esp12)
const unsigned int ledPin=5; // (5 on esp12)
const unsigned int sensorSamples=300; //10000 samples gives a value return rate of 2 per second. 1000 is good for continuous monitor
const unsigned int calibrateSamples=60; //These will be quicksorted so don't exceed ~100
const float thresholdSensitivity=3.4;
const unsigned int sensorArrayLength=5;
const unsigned int defaultFadeSpeed=10;
const unsigned int longPressLength=900; //Time in milliseconds for a long press
const unsigned int PWMTable[101] = {0,1,2,3,5,6,7,8,9,10,12,13,14,16,18,20,21,24,26,28,31,33,36,39,42,45,49,52,56,60,64,68,72,77,82,87,92,98,103,109,115,121,128,135,142,149,156,164,172,180,188,197,206,215,225,235,245,255,266,276,288,299,311,323,336,348,361,375,388,402,417,432,447,462,478,494,510,527,544,562,580,598,617,636,655,675,696,716,737,759,781,803,826,849,872,896,921,946,971,997,1023}; //0 to 100 values for brightnes
const unsigned int reTriggerDelay=80;

//GLOBAL VARIABLES
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
bool lastTriggered;
unsigned int ledPinState = 0; //Default boot state of LEDs and last setPoint of the pin between 0 and 100
unsigned int ledSetPoint = 0;
unsigned int brightness = 100; //last 'on' setpoint for 0-100 scale brightness
long lastFadeTime = 0;
unsigned int calibrationFrequency=600000;
long calibrationTime=0;
bool calibratePrimer=false;
bool primeLongTrig=false;
long tooLongPressLength=10000;

//-----------------------------------------------------------------------------

void setup() {
  delay(0);
  setupLines();
}

//-----------------------------------------------------------------------------

void loop() {
  sumSensorSamples();
  setLedStates();
  FadeLEDs();
  calibrateTimer();
  delay(0);
}

//-----------------------------------------------------------------------------

void sensorTimer() { //What happens when the sensor pin changes value
  sensorTime=micros()-switchTime;
  recordAvail=true;
}

//-----------------------------------------------------------------------------

void calibrateTimer() {
  if (!calibrateTrigger && calibratePrimer && (millis()-calibrationTime>calibrationFrequency) /*&& (millis()-calibrationTime<calibrationFrequency-10)**/) {
    Serial.println("Recallibrating...");
    calibratePrimer=false;
    calibrateTrigger=true;
  }
}

void setupLines() {
  pinMode(switchPin,OUTPUT);
  pinMode(sensorPin,INPUT);
  pinMode(ledPin,OUTPUT);
  Serial.begin(115200); //--------------COMMENT OUT WHEN SENDING TO PRODUCTION
  digitalWrite(switchPin,switchState); //Start low
  analogWrite(ledPin,0); //Start off
  Serial.println("Going online...");
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

void sumSensorSamples() {
  if (micros()-switchTime>90) { //This resets the timer if an interupt stuffs up a state change.
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
    triggered=false; //reset to off
    ledSetPoint=0; //reset to off
  }

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
    
    lastTriggered=triggered;
    if (!triggered && triggerArray[0] && triggerArray[1] && triggerArray[2] && triggerArray[3] && (millis()-pressTime>reTriggerDelay)) {
      triggered=true;
      primeLongTrig=true;
      pressTime=millis();
      calibratePrimer=false;
      Serial.print("Pressed button. ");
    }
    else if (triggered && !triggerArray[0] && !triggerArray[1] && !triggerArray[2] && !triggerArray[3]) {
      triggered=false;
      Serial.println("Released button. ");
    }
    
    //if the button has been held long enough, make a long trigger event
    if (triggered && primeLongTrig && (millis()-pressTime>longPressLength)) {
      longPressTriggered=true;
      primeLongTrig=false;
      Serial.println("Long press triggered.");
    }
    //If the sensor is triggered for too long, re-calibrate because something is up...
    if (triggered && (millis()-pressTime>tooLongPressLength)) {
      calibrateTrigger=true;
      Serial.println("Too long press triggered.");
    }
    if (lastTriggered!=triggered) { 
      messageAvail=true;
      //Serial.print("Trigger value is now: ");
      //Serial.println(triggered);
    }
  }
  
  //Serial.println(sensorVal); //------------Uncomment this if you want to see the values coming out of the sensor
}



void setLedStates() {
  if (messageAvail && triggered && (ledPinState==0)) { //Switch on
    ledSetPoint=brightness;
    Serial.print("CMD 1 Triggered. ");
  }
  else if (messageAvail && triggered && (ledPinState>0) && (ledPinState!=ledSetPoint)) { //Hold
    ledSetPoint=ledPinState;
    brightness=ledPinState;
    Serial.print("CMD 2 Triggered. ");
  }
  else if (messageAvail && triggered && (ledPinState>0)) { //Off
    ledSetPoint=0;
    calibrationTime=millis();
    calibratePrimer=true;
    Serial.print("CMD 3 Triggered. ");
  }
  else if (messageAvail && triggered && ledPinState>0 && ledPinState!=ledSetPoint) { //Hold
    ledSetPoint=ledPinState;
    Serial.print("CMD 4 Triggered. ");
  }
  else if (longPressTriggered && (ledPinState==0)) { // Full power (or all off)
    brightness=100;
    ledSetPoint=brightness;
    Serial.print("CMD 5 Triggered. ");
  }
  else if (longPressTriggered && (ledPinState>0)) { // Full power
    brightness=100;
    ledSetPoint=brightness;
    Serial.print("CMD 6 Triggered. ");
  }
  messageAvail=false;
  longPressTriggered=false;
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

void quickSort(int arr[], int left, int right) {
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

