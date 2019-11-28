/*
  DolphinControl

  By: Greg Cummines
  Date: 9/17/2019

  Dolphin is a hydroponics control system.

  One of the most essential functions of Dolphin is to provide
  an automated system to empty and fill the hydroponics plant container
  from a nutrient container.

  Other functions provide pH control, data logging, and alerts.
  Users can interact with Dolphin via a 4 button panel with display,
  or via a Android Bluetooth app via BLE peripheral support.

  The circuit:
  - Arduino Nano 33 IoT connected to pumps, solenoids, water level sensors,
  temperature sensors, and humidity sensor, pH probe.

*/

/* The <Arduino.h> library defines min and so does the C++ library, which causes an error
  https://stackoverflow.com/questions/41093090/esp8266-error-macro-min-passed-3-arguments-but-takes-just-2
*/
#undef min
#undef max

#include <ArduinoBLE.h>
#include <Adafruit_SHT31.h>
#include <SparkFunSX1509.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <OneWire.h>
#include <math.h>

/******************************************************
   Hardware variables
*/
SX1509 gpioExpander; // Create an SX1509 object
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)

OneWire ds(4);  // on pin 4 (a 4.7K resistor is necessary)

Adafruit_PCD8544 display = Adafruit_PCD8544(13, 11, 12, 10, 9);

Adafruit_SHT31 sht31 = Adafruit_SHT31();

/******************************************************
   Display and Menu variables
*/
const int MenuId_Drain = 1;
const int MenuId_Fill = 2;
const int MenuId_Refill = 3;
const int MenuId_TestMode = 4;

struct MenuItem {
  int menuId;
  String menuChoice;
  int state;
};

MenuItem menuItems[] = {
  { MenuId_Drain, "Drain", 0 },
  { MenuId_Fill, "Fill", 0 },
  { MenuId_Refill, "Refill", 0 },
  { MenuId_TestMode, "Test", 0 }
};

boolean backlight = true;
int contrast = 10;

int page = 1;
int displayWindowIndex = 0;
int displaySelectionIndex = 0;
const int LineCountForDisplay = 3;
int sizeOfMenuItemsArray = 0;
volatile boolean upButtonPressed = false;
volatile boolean downButtonPressed = false;
volatile boolean selectButtonPressed = false;
volatile boolean displayMenu = false;

int downButtonState = 0;
int upButtonState = 0;
int selectButtonState = 0;
int displayMenuButtonState = 0;
int lastDownButtonState = 0;
int lastSelectButtonState = 0;
int lastUpButtonState = 0;
int lastDisplayMenuButtonState = 0;

boolean invalidateDisplay = true;

/******************************************************
   Bluetooth GATT variables
*/
BLEService dolphinLowLevelControlService("C240FACB-9C37-4D25-8CB5-64B8CB4EDFA2"); // BLE Dolphin Low Level Control Service
BLEService dolphinHighLevelControlService("4B28FA61-2D6C-4BE5-A11F-BEE8E83F1FA3"); // BLE Dolphin High Level Control Service
BLEService dolphinAutomateControlService("AC3F15EA-DE6C-4938-AA89-F89BCE3647A2"); // BLE Dolphin Automate Control Service

// BLE  Characteristics for Low Level Control - custom 128-bit UUID, read and writable by central
//
// Waste Pump
BLEByteCharacteristic wastePumpCharacteristic("C240FACC-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);
// Waste Solenoid Valve
BLEByteCharacteristic wasteSolenoidValveCharacteristic("C240FACD-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);
// Fill Pump
BLEByteCharacteristic fillPumpCharacteristic("C240FACE-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);
// Fill Solenoid Valve
BLEByteCharacteristic fillSolenoidValveCharacteristic("C240FACF-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);
// Septic Solenoid Valve
BLEByteCharacteristic septicSolenoidValveCharacteristic("C240FAD0-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);
// Septic Divert Back to Nutrient Tank Solenoid Valve
BLEByteCharacteristic septicDivertSolenoidValveCharacteristic("C240FAD1-9C37-4D25-8CB5-64B8CB4EDFA2", BLERead | BLEWrite | BLENotify);

// BLE  Characteristics for High Level Control - custom 128-bit UUID, read and writable by central
//
// Waste
BLEByteCharacteristic emptyCharacteristic("4B28FA62-2D6C-4BE5-A11F-BEE8E83F1FA3", BLERead | BLEWrite | BLENotify);
// Fill
BLEByteCharacteristic fillCharacteristic("4B28FA63-2D6C-4BE5-A11F-BEE8E83F1FA3", BLERead | BLEWrite | BLENotify);
// Water Level (Inches)
BLEByteCharacteristic waterLevelCharacteristic("4B28FA64-2D6C-4BE5-A11F-BEE8E83F1FA3", BLERead | BLENotify);

// BLE  Characteristics for Automated Control - custom 128-bit UUID, read and writable by central
//
// Automate Waste and Fill (Refill)
BLEByteCharacteristic refillCharacteristic("AC3F15EB-DE6C-4938-AA89-F89BCE3647A2", BLERead | BLEWrite | BLENotify);
// Test Mode (diverts waste back to nutrient tank)
BLEByteCharacteristic testModeEnabledCharacteristic("AC3F15EC-DE6C-4938-AA89-F89BCE3647A2", BLERead | BLEWrite | BLENotify); 

/******************************************************
   Pump and solenoid hardware variables
*/
bool turnOnWastePumpTrigger = false;
bool turnOffWastePumpTrigger = false;

bool turnOnWasteSolenoidValveTrigger = false;
bool turnOffWasteSolenoidValveTrigger = false;

bool turnOnFillPumpTrigger = false;
bool turnOffFillPumpTrigger = false;

bool turnOnFillSolenoidValveTrigger = false;
bool turnOffFillSolenoidValveTrigger = false;

bool turnOnSepticOutValveTrigger = false;
bool turnOffSepticOutValveTrigger = false;

bool turnOnDivertBackToNutrientTankValveTrigger = false;
bool turnOffDivertBackToNutrientTankValveTrigger = false;

bool emptyTrigger = false;
bool emptyStarted = false;
unsigned long emptyLoopCheckLevelMilli;
unsigned long emptyLoopHitZeroMilli;
unsigned long emptyLoopCheckTimeoutMilli;
bool waterLevelHitEmptyTrigger = false;
bool emptyCompleted = false;

bool fillTrigger = false;
bool fillStarted = false;
unsigned long fillLoopCheckMilli;
unsigned long fillLoopStopMilli;

bool refillTrigger = false;

unsigned long waterLevelLastReadMilli;
// Amount of time to check the water level again to avoid glitches
// This will avoid fast state changes on a sensor and limit changes
// to once every time period
static const unsigned long WATER_LEVEL_GLITCH_STABILIZER_INTERVAL = 500;

bool isSystemBusy = false;
bool cancel = false;
bool testModeEnabled = false;

const int GpioExpanderPinFillSolenoid = 8;
const int GpioExpanderPinWasteSolenoid = 9;
const int GpioExpanderPinDivertSolenoid = 10;
const int GpioExpanderPinSepticSolenoid = 11;
const int GpioExpanderPinWastePump = 12;
const int GpioExpanderPinFillPump = 13;

unsigned long sensorReadFastLoopDelayMilli;
unsigned long sensorReadSlowLoopDelayMilli;

// This variable sets the amount of time to wait after the empty cycle has been started
// This variable is calculated at runtime based on the current water level
int timeToWaitForWasteEmpty = 0;

const int TimeToWaitAfterHitZero = 90000;       // 90 seconds

// This variable is based on the current water level and is only used as a maximum
// timeout if the water level change is not detected
int timeToWaitForNutrientFill = 0;

const float TargetLevelEmpty = 0;
const float TargetLevelFull = 7;

/******************************************************
   Misc variables
*/
unsigned long lastTimeTempSensorsRead = 0;
unsigned long lastTimeDHT31SensorRead = 0;
unsigned long lastTimeWaterLevelSensorRead = 0;
unsigned long lastTimeAlarmSignalOutputToggled = 0;

static const unsigned long REFRESH_INTERVAL_WATER_LEVEL_SENSOR = 300; // ms
static const unsigned long REFRESH_INTERVAL_TEMP_SENSOR = 60000; // 60 seconds
static const unsigned long REFRESH_INTERVAL_DHT31_SENSOR = 15000; // 15 seconds
static const unsigned long REFRESH_INTERVAL_ALARM_SIGNALOUTPUT_TOGGLE = 150;    // 150mS + whatever amount of time the loop is taking + the amount of overhead in the firmware to call the loop

// controls whether the alarm is on or off
bool alarmIsSounding = false;
// controls the duty cycle of the alarm so the main loop is kept running
bool alarmSignalOutputToggle = false;

byte tempSensor1Address[8] = { 0x28, 0xFF, 0x38, 0x7C, 0x71, 0x16, 0x04, 0x2B };
byte tempSensor2Address[8] = { 0x28, 0xFF, 0xF2, 0x3B, 0x71, 0x16, 0x05, 0x8C };
byte tempSensor3Address[8] = { 0x28, 0xE4, 0xAB, 0x79, 0x97, 0x07, 0x03, 0x4D };

float temp1 = -300.0;
float temp2 = -300.0;
float tempInsideControllerBox = -300.0;
float humidity = -300.0;
float pressure = -300.0;
int waterLevel = 0;

unsigned long tempConversionStartedMilli = 0;
static const unsigned long DS18B20_TEMP_CONVERSION_TIME = 750;

const bool debug = false;

unsigned long internalLoopDelayForBLEMilli;

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }
  }

  log("Dolphin 2.0 bootup!");

  printDallasTempSensors();

  initializeBLE();
  initializeGpioExpander();
  initializeDHT31Sensor();
  initializeDisplay();
  initializePanelControlButtonInputs();
  initializeWaterLevelSensor();

  forceStartTempConversion();
  forceProcessDHT31Sensor();
  forceProcessWaterLevelSensor();
}

void loop() {
  processBLECommands();
  internalLoop();
}

// This is the main internal loop and is called either when central is connected or not
// It is CRITICAL that all functions in this loop execute FAST (< 200mS)
void internalLoop() {
  processSolenoidAndPumpCommands();
  processControlButtonCommands();
  startTempConversion();
  readTempSensorsAfterConversion();
  processDHT31Sensor();
  processWaterLevelSensor();
  processAlarm();
  processLowWaterLevel();
  updateDisplay();
}

void resetPumpsAndSolenoids() {
  turnOffAllOutPumpAndSolenoidValves();
  turnOffInPumpAndSolenoidValve();

  turnOffSepticOutValve();
  turnOffDivertBackToNutrientTankValve();

  // Reset all action flags
  emptyTrigger = false;
  emptyStarted = false;
  fillStarted = false;
  fillTrigger = false;
  refillTrigger = false;
  isSystemBusy = false;
  cancel = false;
}

void processSolenoidAndPumpCommands() {
  // Check for cancel of any operations in progress and
  // stop them (add any code needed here for future operations)
  if (cancel) {
    // Turn off all pumps/solenoid valves
    log("Cancelling process...");
    resetPumpsAndSolenoids();
  }

  processEmptyCommand();
  processFillCommand();

  /* The following triggers are for manual use and testing and will not be
     normally used */

  if (!fillStarted && !emptyStarted && !isSystemBusy) {

    if (turnOnWastePumpTrigger == true) {
      turnOnWastePump();
      turnOnWastePumpTrigger = false;
    }

    if (turnOffWastePumpTrigger == true) {
      turnOffWastePump();
      turnOffWastePumpTrigger = false;
    }

    if (turnOnWasteSolenoidValveTrigger == true) {
      turnOnWasteSolenoidValve();
      turnOnWasteSolenoidValveTrigger = false;
    }

    if (turnOffWasteSolenoidValveTrigger == true) {
      turnOffWasteSolenoidValve();
      turnOffWasteSolenoidValveTrigger = false;
    }

    if (turnOnFillPumpTrigger == true) {
      turnOnFillPump();
      turnOnFillPumpTrigger = false;
    }

    if (turnOffFillPumpTrigger == true) {
      turnOffFillPump();
      turnOffFillPumpTrigger = false;
    }

    if (turnOnFillSolenoidValveTrigger == true) {
      turnOnFillSolenoidValve();
      turnOnFillSolenoidValveTrigger = false;
    }

    if (turnOffFillSolenoidValveTrigger == true) {
      turnOffFillSolenoidValve();
      turnOffFillSolenoidValveTrigger = false;
    }

    if (turnOnSepticOutValveTrigger == true) {
      turnOnSepticOutValve();
      turnOnSepticOutValveTrigger = false;
    }

    if (turnOffSepticOutValveTrigger == true) {
      turnOffSepticOutValve();
      turnOffSepticOutValveTrigger = false;
    }

    if (turnOnDivertBackToNutrientTankValveTrigger == true) {
      turnOnDivertBackToNutrientTankValve();
      turnOnDivertBackToNutrientTankValveTrigger = false;
    }

    if (turnOffDivertBackToNutrientTankValveTrigger == true) {
      turnOffDivertBackToNutrientTankValve();
      turnOffDivertBackToNutrientTankValveTrigger = false;
    }
  }
}

void processEmptyCommand() {
  // If the empty trigger was set (by a button click, BLE command, etc.)
  // Note: Once the empty trigger is set, it will remain set until complete so that
  // other loop operations can check status and make sure it is completed.
  if (emptyTrigger || refillTrigger) {
    // Start the empty cycle if the system is not performing another operation
    if (!emptyStarted && !isSystemBusy) {

      log("Empty cycle started");

      // Prevent other operations from occurring, except cancel
      isSystemBusy = true;
      emptyStarted = true;
      emptyCompleted = false;
      waterLevelHitEmptyTrigger = false;

      stateChanged();

      // Set the timeout based on current level
      timeToWaitForWasteEmpty = getMaxTimeToWaitForEmptyBasedOnLevel();
      log("Max time to wait for empty is " + String(timeToWaitForWasteEmpty) + "mS");
      log("Empty cycle is turning on pump and solenoids...");
      turnOnAllOutPumpAndSolenoidValves();

      // Remember the time so we can check back on the level from time to time
      emptyLoopCheckLevelMilli = millis();
      // Remember the time so we can check to see if our timeout has expired
      emptyLoopCheckTimeoutMilli = millis();

    } else {
      // Empty cycle is running, so check status...

      // Check empty status every second if we have not already detected empty
      if ((millis() - emptyLoopCheckLevelMilli > 1000) && !waterLevelHitEmptyTrigger && emptyStarted) {
        emptyLoopCheckLevelMilli = millis();

        //log("Checking empty status...");

        // If we hit zero, set a trigger so we can wait to turn off pump/solenoid valve
        // until some time later because system is not completely empty yet
        if (waterLevel == TargetLevelEmpty) {
          log("Water level has reached 0...");
          // We are setting the flag so if it registers not empty next time
          // we are still treating the trigger as an empty from now on each
          // time loop gets called
          waterLevelHitEmptyTrigger = true;
          // We detected water level as empty, so set a time variable that we can use to
          // keep the pump running just a little longer to empty the tank
          emptyLoopHitZeroMilli = millis();
        }
      }

      // If we detect a water level of 0, wait a little longer before shutting pump/solenoid valve off
      // because we may not be completely empty yet.
      if (waterLevelHitEmptyTrigger && emptyStarted && (millis() - emptyLoopHitZeroMilli > TimeToWaitAfterHitZero)) {
        log("Timeout after water level 0 has been set, ending empty cycle...");
        turnOffAllOutPumpAndSolenoidValves();

        emptyStarted = false;
        emptyCompleted = true;
        emptyTrigger = false;
        isSystemBusy = false;
        waterLevelHitEmptyTrigger = false;

        stateChanged(); 
        
      }

      // If we reached the maximum time allowed regardless of the water level, stop emptying
      if (((millis() - emptyLoopCheckTimeoutMilli) > timeToWaitForWasteEmpty) &&
          emptyStarted) {
        log("Max timeout has occurred, ending empty cycle...");
        turnOffAllOutPumpAndSolenoidValves();

        emptyStarted = false;
        emptyCompleted = true;
        emptyTrigger = false;
        isSystemBusy = false;

        stateChanged();
      }
    }
  }
}

void processFillCommand() {
  // If the fill trigger was set (by a button click, BLE command, etc.)
  // Note: Once the fill trigger is set, it will remain set until complete so that
  // other loop operations can check status and make sure it is completed.
  if (fillTrigger || (refillTrigger && emptyCompleted)) {
    if (!fillStarted && !isSystemBusy) {
      // Start the fill cycle
      log("Fill started");
      fillStarted = true;
      isSystemBusy = true;

      // even though the empty process turns off pumps and solenoids, make one last
      // effort here just in case something went wrong
      turnOffAllOutPumpAndSolenoidValves();

      // Set the timeout based on current level
      timeToWaitForNutrientFill = getMaxTimeToWaitForFillBasedOnLevel();

      turnOnInPumpAndSolenoidValve();

      fillLoopCheckMilli = millis();
      fillLoopStopMilli = millis();

      stateChanged();
    } else {
      // Fill cycle has started, check for completion

      // Check fill status every 100 mS
      if (millis() - fillLoopCheckMilli > 1000) {
        fillLoopCheckMilli = millis();

        // If the system is full or a max timeout has occurred, stop the cycle
        if (waterLevel == TargetLevelFull || (millis() - fillLoopStopMilli > timeToWaitForNutrientFill)) {
          turnOffInPumpAndSolenoidValve();

          fillStarted = false;
          fillTrigger = false;
          refillTrigger = false;
          isSystemBusy = false;

          stateChanged();
        }
      }
    }
  }
}

void updateMenuState(int id, int state) {
  int menuIndex = getMenuIndexById(id);
  menuItems[menuIndex].state = state;
}

int getMaxTimeToWaitForEmptyBasedOnLevel() {
  // The higher the current water level is the longer we need to wait to
  // shut off the pump and solenoids for water output.
  const int MinuteInMilliseconds = 60000;
  return 10 * MinuteInMilliseconds;    // wait for 10 minutes until we do the calculations
}

int getMaxTimeToWaitForFillBasedOnLevel() {
  int timeToWaitInSeconds = 510;  // 8:30 
  // subtract 60 seconds for each level
  if (waterLevel == 7) {
    timeToWaitInSeconds = 0;
  } else {
    timeToWaitInSeconds -= (waterLevel * 60);
  }
  return (timeToWaitInSeconds * 1000);
}

void turnOnAllOutPumpAndSolenoidValves() {
  turnOnWastePump();
  turnOnWasteSolenoidValve();

  if (testModeEnabled) {
    turnOnDivertBackToNutrientTankValve();
    turnOffSepticOutValve();
  } else {
    turnOnSepticOutValve();
    turnOffDivertBackToNutrientTankValve();
  }
}

void turnOffAllOutPumpAndSolenoidValves() {
  turnOffWasteSolenoidValve();
  turnOffWastePump();

  turnOffSepticOutValve();
  turnOffDivertBackToNutrientTankValve();
}

void turnOnInPumpAndSolenoidValve() {
  turnOnFillPump();
  turnOnFillSolenoidValve();
}

void turnOffInPumpAndSolenoidValve() {
  turnOffFillSolenoidValve();
  turnOffFillPump();
}

void turnOnWastePump() {
  gpioExpander.digitalWrite(GpioExpanderPinWastePump, HIGH);
  wastePumpCharacteristic.writeValue(1);
  delay(10);
}

void turnOffWastePump() {
  gpioExpander.digitalWrite(GpioExpanderPinWastePump, LOW);
  wastePumpCharacteristic.writeValue(0);
  delay(10);
}

void turnOnWasteSolenoidValve() {
  gpioExpander.digitalWrite(GpioExpanderPinWasteSolenoid, HIGH);
  wasteSolenoidValveCharacteristic.writeValue(1);
  delay(10);
}

void turnOffWasteSolenoidValve() {
  gpioExpander.digitalWrite(GpioExpanderPinWasteSolenoid, LOW);
  wasteSolenoidValveCharacteristic.writeValue(0);
  delay(10);
}

void turnOnFillPump() {
  gpioExpander.digitalWrite(GpioExpanderPinFillPump, HIGH);
  fillPumpCharacteristic.writeValue(1);
  delay(10);
}

void turnOffFillPump() {
  gpioExpander.digitalWrite(GpioExpanderPinFillPump, LOW);
  fillPumpCharacteristic.writeValue(0);
  delay(10);
}

void turnOnFillSolenoidValve() {
  gpioExpander.digitalWrite(GpioExpanderPinFillSolenoid, HIGH);
  fillSolenoidValveCharacteristic.writeValue(1);
  delay(10);
}

void turnOffFillSolenoidValve() {
  gpioExpander.digitalWrite(GpioExpanderPinFillSolenoid, LOW);
  fillSolenoidValveCharacteristic.writeValue(0);
  delay(10);
}

void turnOnSepticOutValve() {
  gpioExpander.digitalWrite(GpioExpanderPinSepticSolenoid, HIGH);
  septicSolenoidValveCharacteristic.writeValue(1);
  
  delay(10);
}

void turnOffSepticOutValve() {
  gpioExpander.digitalWrite(GpioExpanderPinSepticSolenoid, LOW);
  septicSolenoidValveCharacteristic.writeValue(0);
  delay(10);
}

void turnOnDivertBackToNutrientTankValve() {
  gpioExpander.digitalWrite(GpioExpanderPinDivertSolenoid, HIGH);
  septicDivertSolenoidValveCharacteristic.writeValue(1);
  delay(10);
}

void turnOffDivertBackToNutrientTankValve() {
  gpioExpander.digitalWrite(GpioExpanderPinDivertSolenoid, LOW);
  septicDivertSolenoidValveCharacteristic.writeValue(0);
  delay(10);
}

void startTempConversion() {
  // Force a temperature sensor measurement every interval
  if (millis() > lastTimeTempSensorsRead + REFRESH_INTERVAL_TEMP_SENSOR) {
    lastTimeTempSensorsRead = millis();

    forceStartTempConversion();
  }
}

void processDHT31Sensor() {
  // Force a DHT31 sensor measurement every interval
  if (millis() > lastTimeDHT31SensorRead + REFRESH_INTERVAL_DHT31_SENSOR) {
    lastTimeDHT31SensorRead = millis();

    forceProcessDHT31Sensor();
  }
}

void processWaterLevelSensor() {

  // Force a water level sensor measurement every interval
  if (millis() > lastTimeWaterLevelSensorRead + REFRESH_INTERVAL_WATER_LEVEL_SENSOR) {
    lastTimeWaterLevelSensorRead = millis();

    forceProcessWaterLevelSensor();
    //log("Analog: " + String(analogVal) + " Moving average " + String(movingAvgCalc.GetAverage()));          // debug value
  }
}

void printDallasTempSensors() {
  byte addr[8];
  byte i;
  log("Finding temp sensors: ");

  while ( ds.search(addr)) {
    Serial.print("Address=");
    for ( i = 0; i < 8; i++) {
      Serial.print(addr[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
  ds.reset_search();
}

void readTempSensorsAfterConversion() {
  // If the time period for the temperature sensors to convert
  // the temperature has gone by, we can now read the temperature sensors
  if (millis() > tempConversionStartedMilli + DS18B20_TEMP_CONVERSION_TIME) {
    
    temp1 = readTemperatureInFahrenheit(tempSensor1Address);
    temp2 = readTemperatureInFahrenheit(tempSensor2Address);
    tempInsideControllerBox = readTemperatureInFahrenheit(tempSensor3Address);

    invalidateDisplay = true;
  }
}

void forceStartTempConversion() {
  byte i;
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;

  ds.reset();
  ds.select(tempSensor1Address);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  ds.reset();
  ds.select(tempSensor2Address);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  ds.reset();
  ds.select(tempSensor3Address);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  tempConversionStartedMilli = millis();
}

void forceProcessDHT31Sensor() {
  humidity = sht31.readHumidity();

  invalidateDisplay = true;
}

void forceProcessWaterLevelSensor() {
  // Only allow water level changes every interval to prevent glitches when
  // the water is on the threshold.
  if (millis() > waterLevelLastReadMilli + WATER_LEVEL_GLITCH_STABILIZER_INTERVAL) {

    waterLevelLastReadMilli = millis();

    int oldWaterLevel = waterLevel;
    int tmpWaterLevel = 0;
    // s1 is the sensor at the lowest level a
    byte s1 = gpioExpander.digitalRead(1);
    byte s2 = gpioExpander.digitalRead(2);
    byte s3 = gpioExpander.digitalRead(3);
    byte s4 = gpioExpander.digitalRead(4);
    byte s5 = gpioExpander.digitalRead(5);
    byte s6 = gpioExpander.digitalRead(6);
    byte s7 = gpioExpander.digitalRead(7);
    if (s7 == 0) {
      tmpWaterLevel = 7;
    } else if (s6 == 0) {
      tmpWaterLevel = 6;
    } else if (s5 == 0) {
      tmpWaterLevel = 5;
    } else if (s4 == 0) {
      tmpWaterLevel = 4;
    } else if (s3 == 0) {
      tmpWaterLevel = 3;
    } else if (s2 == 0) {
      tmpWaterLevel = 2;
    } else if (s1 == 0) {
      tmpWaterLevel = 1;
    } else {
      tmpWaterLevel = 0;
    }
    if (tmpWaterLevel != oldWaterLevel) {
      log("Water level changed from " + String(oldWaterLevel) + " to " + String(tmpWaterLevel));
      setWaterLevel(tmpWaterLevel);
    }
  }
}

// Call this method after conversion is complete
float readTemperatureInFahrenheit(byte address[8]) {
  byte i;
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;

  ds.reset();
  ds.select(address);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];

  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(fahrenheit);
  //log(" F");

  return fahrenheit;
}

void processControlButtonCommands() {
  downButtonState = digitalRead(2);
  selectButtonState = digitalRead(1);
  upButtonState =   digitalRead(0);
  displayMenuButtonState = digitalRead(3);

  // These functions check the current state against the last known state
  // and also provide debouce support
  checkIfDownButtonIsPressed();
  checkIfUpButtonIsPressed();
  checkIfSelectButtonIsPressed();
  checkIfDisplayMenuButtonIsPressed();

  if (upButtonPressed) {
    upButtonPressed = false;
    processUpButtonClick();
  }

  if (downButtonPressed) {
    downButtonPressed = false;
    processDownButtonClick();
  }

  if (selectButtonPressed) {
    selectButtonPressed = false;
    processSelectButtonClick();
  }
}

void processDownButtonClick() {
  if (sizeOfMenuItemsArray > LineCountForDisplay &&
      displaySelectionIndex == (LineCountForDisplay - 1) &&
      displayWindowIndex < (sizeOfMenuItemsArray - LineCountForDisplay)) {
    displayWindowIndex++;
    invalidateDisplay = true;
  } else if (displaySelectionIndex < (min(LineCountForDisplay, sizeOfMenuItemsArray) - 1)) {
    displaySelectionIndex++;
    invalidateDisplay = true;
  }

  //log("displayWindowIndex:" + String(displayWindowIndex) + ", displaySelectionIndex:" + String(displaySelectionIndex));
}

void processUpButtonClick() {
  if (sizeOfMenuItemsArray > LineCountForDisplay &&
      displaySelectionIndex == 0 &&
      displayWindowIndex > 0) {
    displayWindowIndex--;
    invalidateDisplay = true;
  } else if (displaySelectionIndex > 0) {
    displaySelectionIndex--;
    invalidateDisplay = true;
  }
  //log("displayWindowIndex:" + String(displayWindowIndex) + ", displaySelectionIndex:" + String(displaySelectionIndex));
}

void processSelectButtonClick() {
  int menuIndex = displayWindowIndex + displaySelectionIndex;
  log("processing " + menuItems[menuIndex].menuChoice);

  // First cancel any running operation and set menu state to off
  // for menu items that were not selected and clicked
  cancel = true;
  processSolenoidAndPumpCommands();
  
  // Turn off other menu items that are related to each other, if one is active
  // For example, if we are filling, and try to empty, we must stop filling! 
  // So if the user chooses a menu item, the previous is set to 0 before setting
  // the new menu item to 1 or active, but only for "Empty", "Fill", and "Refill". 
  stopAlreadyRunningOtherFillEmptyRefillCommands(menuIndex);

  // Convert the menu index to a change in the state 
  // todo: Create a state machine instead of keeping state tied to 
  // the menu system
  processMenuCommand(menuIndex);
  
  // Since some state has changed, update the display
  invalidateDisplay = true;
}

void processMenuCommand(int menuIndex) {
  // Process menu command
  switch (menuItems[menuIndex].menuId) {
    case MenuId_Drain:
      if (menuItems[menuIndex].state == 0) {
        setEmptyTrigger(true);
      } else {
        setEmptyTrigger(false);
      }
      break;
    case MenuId_Fill:
      if (menuItems[menuIndex].state == 0) {
        setFillTrigger(true);
      } else {
        setFillTrigger(false);
      }
      break;
    case MenuId_Refill:
      if (menuItems[menuIndex].state == 0) {
        setRefillTrigger(true);
      } else {
        setRefillTrigger(false);
      }
      break;
    case MenuId_TestMode:
      if (menuItems[menuIndex].state == 0) {
        setTestModeEnabled(true);
      } else {
        setTestModeEnabled(false);
      }
      break;
  }
  invalidateDisplay = true;
}

void setWaterLevel(int tmpWaterLevel) {
  waterLevel = tmpWaterLevel;
  waterLevelCharacteristic.writeValue(tmpWaterLevel);
  invalidateDisplay = true;
}

void setRefillTrigger(bool enabled) {
  log("setRefillTrigger called with " + String(enabled));
  if (enabled) {
    refillTrigger = true;
    emptyCompleted = false;
    refillCharacteristic.writeValue(1);   
  } else {
    cancel = true; // This will reset the refillTrigger variable too
    refillCharacteristic.writeValue(0);
  }
  stateChanged();
}

void setEmptyTrigger(bool enabled) {
  if (enabled) {
    emptyTrigger = true;
    emptyCompleted = false;
    emptyCharacteristic.writeValue(1);
  } else {
    cancel = true; // This will reset the refillTrigger variable too
    emptyCharacteristic.writeValue(0);
  }
  stateChanged();
}

void setFillTrigger(bool enabled) {
  if (enabled) {
    fillTrigger = true;
    fillCharacteristic.writeValue(1);
  } else {
    cancel = true; // This will reset the refillTrigger variable too
    fillCharacteristic.writeValue(0);
  }
  stateChanged();
}

void setTestModeEnabled(bool enabled) {
  if (enabled) {
    testModeEnabled = true;
    testModeEnabledCharacteristic.writeValue(1);
  } else {
    testModeEnabled = false;
    testModeEnabledCharacteristic.writeValue(0);
  }
  stateChanged();
}

void stopAlreadyRunningOtherFillEmptyRefillCommands(int menuIndex) {
  for (int i = 0; i < sizeOfMenuItemsArray; i++) {
    if (menuItems[i].state == 1 && 
        i != menuIndex && 
        menuItems[i].menuId <= MenuId_Refill) {
      menuItems[i].state = 0;
    }
  }
}

int getMenuIndexById(int id) {
  for (int i = 0; i < sizeOfMenuItemsArray; i++) {
    if (menuItems[i].menuId == id) {
      return i;
    }
  }
  return 0;
}

void processBLECommands() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    log("Connected to central: " + central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the GPIO:
      if (wastePumpCharacteristic.written()) {
        if (wastePumpCharacteristic.value()) {   // any value other than 0
          turnOnWastePumpTrigger = true;
        } else {                              // a 0 value
          turnOffWastePumpTrigger = true;
        }
        stateChanged();
      }

      if (wasteSolenoidValveCharacteristic.written()) {
        if (wasteSolenoidValveCharacteristic.value()) {   // any value other than 0
          turnOnWasteSolenoidValveTrigger = true;
        } else {                              // a 0 value
          turnOffWasteSolenoidValveTrigger = true;
        }
        stateChanged();
      }

      if (fillPumpCharacteristic.written()) {
        if (fillPumpCharacteristic.value()) {   // any value other than 0
          turnOnFillPumpTrigger = true;
        } else {                              // a 0 value
          turnOffFillPumpTrigger = true;
        }
        stateChanged();
      }

      if (fillSolenoidValveCharacteristic.written()) {
        if (fillSolenoidValveCharacteristic.value()) {   // any value other than 0
          turnOnFillSolenoidValveTrigger = true;
        } else {                              // a 0 value
          turnOffFillSolenoidValveTrigger = true;
        }
        stateChanged();
      }

      if (septicSolenoidValveCharacteristic.written()) {
        if (septicSolenoidValveCharacteristic.value()) {   // any value other than 0
          turnOnSepticOutValveTrigger = true;
        } else {                              // a 0 value
          turnOffSepticOutValveTrigger = true;
        }
        stateChanged();
      }

      if (septicDivertSolenoidValveCharacteristic.written()) {
        if (septicDivertSolenoidValveCharacteristic.value()) {   // any value other than 0
          turnOnDivertBackToNutrientTankValveTrigger = true;
        } else {                              // a 0 value
          turnOffDivertBackToNutrientTankValveTrigger = true;
        }
        stateChanged();
      }

      if (emptyCharacteristic.written()) {
        if (emptyCharacteristic.value()) {   // any value other than 0
          setEmptyTrigger(true);
        } else {
          setEmptyTrigger(false);
        }
      }

      if (fillCharacteristic.written()) {
        if (fillCharacteristic.value()) {   // any value other than 0
          setFillTrigger(true);
        } else {
          setFillTrigger(false);
        }
      }

      if (refillCharacteristic.written()) {
        if (refillCharacteristic.value()) {   // any value other than 0
          setRefillTrigger(true);
        } else {                              // a 0 value
          setRefillTrigger(false);
        }
      }

      if (testModeEnabledCharacteristic.written()) {
        if (testModeEnabledCharacteristic.value()) {   // any value other than 0
          setTestModeEnabled(true);
        } else {                              // a 0 value
          setTestModeEnabled(false);
        }
      }
      
      internalLoop();
    }

    // when the central disconnects, print it out:
    log("Disconnected from central: " + central.address());
  } // if (central)
}

void initializeWaterLevelSensor() {
  setWaterLevel(0);
}

void initializeDisplay() {
  sizeOfMenuItemsArray = sizeof(menuItems) / sizeof(menuItems[0]);

  pinMode(7, OUTPUT);

  digitalWrite(7, HIGH); //Turn Backlight ON

  display.begin();
  display.setContrast(contrast); //Set contrast to 50
  display.clearDisplay();
  display.display();
}

void updateDisplay() {
  if (invalidateDisplay) {
    if (displayMenu) {
      drawMenu();
    } else {
      drawHomeScreen();
    }
    invalidateDisplay = false;
  }
}

void initializeBLE() {
  // begin initialization
  if (!BLE.begin()) {
    log("starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("DolphinControl");
  BLE.setAdvertisedService(dolphinLowLevelControlService);
  BLE.setAdvertisedService(dolphinHighLevelControlService);
  BLE.setAdvertisedService(dolphinAutomateControlService);

  // add the characteristics to the service
  dolphinLowLevelControlService.addCharacteristic(wastePumpCharacteristic);
  dolphinLowLevelControlService.addCharacteristic(wasteSolenoidValveCharacteristic);
  dolphinLowLevelControlService.addCharacteristic(fillPumpCharacteristic);
  dolphinLowLevelControlService.addCharacteristic(fillSolenoidValveCharacteristic);
  dolphinLowLevelControlService.addCharacteristic(septicSolenoidValveCharacteristic);
  dolphinLowLevelControlService.addCharacteristic(septicDivertSolenoidValveCharacteristic);

  dolphinHighLevelControlService.addCharacteristic(emptyCharacteristic);
  dolphinHighLevelControlService.addCharacteristic(fillCharacteristic);
  dolphinHighLevelControlService.addCharacteristic(waterLevelCharacteristic);

  dolphinAutomateControlService.addCharacteristic(refillCharacteristic);
  dolphinAutomateControlService.addCharacteristic(testModeEnabledCharacteristic);
  // add services
  BLE.addService(dolphinLowLevelControlService);
  BLE.addService(dolphinHighLevelControlService);
  BLE.addService(dolphinAutomateControlService);

  // set the initial values for the characeristics:
  wastePumpCharacteristic.writeValue(0);
  wasteSolenoidValveCharacteristic.writeValue(0);
  fillPumpCharacteristic.writeValue(0);
  fillSolenoidValveCharacteristic.writeValue(0);
  septicSolenoidValveCharacteristic.writeValue(0);
  septicDivertSolenoidValveCharacteristic.writeValue(0);

  emptyCharacteristic.writeValue(0);
  fillCharacteristic.writeValue(0);
  waterLevelCharacteristic.writeValue(0);

  refillCharacteristic.writeValue(0);
  testModeEnabledCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();
}

void initializeDHT31Sensor() {
  sht31.begin(0x44);
}

void initializeGpioExpander() {
  if (!gpioExpander.begin(SX1509_ADDRESS))
  {
    log("Error: Can't initialize SX1509");
    while (1);
  }

  //log("Setting pin mode...");
  gpioExpander.pinMode(0, OUTPUT);
  gpioExpander.digitalWrite(0, LOW);
  delay(10);
  gpioExpander.pinMode(1, INPUT_PULLUP);
  gpioExpander.pinMode(2, INPUT_PULLUP);
  gpioExpander.pinMode(3, INPUT_PULLUP);
  gpioExpander.pinMode(4, INPUT_PULLUP);
  gpioExpander.pinMode(5, INPUT_PULLUP);
  gpioExpander.pinMode(6, INPUT_PULLUP);
  gpioExpander.pinMode(7, INPUT_PULLUP);
  gpioExpander.pinMode(8, OUTPUT);
  gpioExpander.digitalWrite(8, LOW);
  delay(10);
  gpioExpander.pinMode(9, OUTPUT);
  gpioExpander.digitalWrite(9, LOW);
  delay(10);
  gpioExpander.pinMode(10, OUTPUT);
  gpioExpander.digitalWrite(10, LOW);
  delay(10);
  gpioExpander.pinMode(11, OUTPUT);
  gpioExpander.digitalWrite(11, LOW);
  delay(10);
  gpioExpander.pinMode(12, OUTPUT);
  gpioExpander.digitalWrite(12, LOW);
  delay(10);
  gpioExpander.pinMode(13, OUTPUT);
  gpioExpander.digitalWrite(13, LOW);
  delay(10);
}

void checkIfDisplayMenuButtonIsPressed()
{
  if (displayMenuButtonState != lastDisplayMenuButtonState)
  {
    lastDisplayMenuButtonState = displayMenuButtonState;
    if (displayMenuButtonState == 0)
    {
      //log("Menu button pressed");

      displayMenu = !displayMenu;
      if (displayMenu) {
        // re-initialize the menu anytime we enter
        page = 1;
        displayWindowIndex = 0;
        displaySelectionIndex = 0;
      }

      invalidateDisplay = true;
    }
    delay(50);
  }
}

void checkIfDownButtonIsPressed()
{
  if (downButtonState != lastDownButtonState)
  {
    lastDownButtonState = downButtonState;
    if (downButtonState == 0)
    {
      downButtonPressed = true;

      invalidateDisplay = true;
    }
    delay(50);
  }
}

void checkIfUpButtonIsPressed()
{
  if (upButtonState != lastUpButtonState)
  {
    lastUpButtonState = upButtonState;

    if (upButtonState == 0) {
      upButtonPressed = true;

      invalidateDisplay = true;
    }
    delay(50);
  }

}

void setAlarmIsSounding(bool isSounding) {
  if (isSounding) {
    alarmIsSounding = true;
  } else {
    alarmIsSounding = false;
    gpioExpander.digitalWrite(0, LOW);
    alarmSignalOutputToggle = false;
  }
}

void processAlarm() {
  if (alarmIsSounding) {
    if (millis() > lastTimeAlarmSignalOutputToggled + REFRESH_INTERVAL_ALARM_SIGNALOUTPUT_TOGGLE) {
      lastTimeAlarmSignalOutputToggled = millis();
      if (alarmSignalOutputToggle) {
        gpioExpander.digitalWrite(0, HIGH);
      } else {
        gpioExpander.digitalWrite(0, LOW);
      }
      alarmSignalOutputToggle = !alarmSignalOutputToggle;
    }
  }
}

void processLowWaterLevel() {
  if (!isSystemBusy && waterLevel <= 1) {
    setAlarmIsSounding(true);
  } else {
    setAlarmIsSounding(false);
  }
  
}

void checkIfSelectButtonIsPressed()
{
  if (selectButtonState != lastSelectButtonState)
  {
    lastSelectButtonState = selectButtonState;

    if (selectButtonState == 0) {
      selectButtonPressed = true;
      invalidateDisplay = true;
    }
    delay(50);
  }
}

void drawHomeScreen()
{
  String temp1String = "NA";
  String temp2String = "NA";
  String humidityString = "NA";
  String waterLevelString = "NA";
  if (temp1 > -300) {
    temp1String = String(round(temp1));
  }
  if (temp2 > -300) {
    temp2String = String(round(temp2));
  }
  if (humidity > -300) {
    humidityString = String(round(humidity));
  }
  if (waterLevel > -300) {
    waterLevelString = String(round(waterLevel));
  }
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(10, 0);
  display.print("Dolphin 2.0");
  display.drawFastHLine(0, 10, 83, BLACK);
  display.setCursor(0, 15);
  display.print(temp1String + "F " + temp2String + "F " + humidityString + "%");
  display.setCursor(0, 25);
  display.print("WaterLevel: " + waterLevelString);
  display.setCursor(0, 35);
  display.print("pH: NA");
  display.display();
}

int min(int a, int b) {
  if (a > b) {
    return b;
  } else {
    return a;
  }
}

int max(int a, int b) {
  if (a < b) {
    return b;
  } else {
    return a;
  }
}

void drawMenuHeader() {
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(15, 0);
  display.print("MAIN MENU");
  display.drawFastHLine(0, 10, 83, BLACK);
}

void drawMenu()
{
  drawMenuHeader();

  int rowPosition = 15;
  display.setCursor(0, rowPosition);

  for (int i = displayWindowIndex; i < min(sizeOfMenuItemsArray, (LineCountForDisplay + displayWindowIndex)); i++ ) {
    if (displaySelectionIndex == (i - displayWindowIndex)) {
      display.setTextColor(WHITE, BLACK);
    } else {
      display.setTextColor(BLACK, WHITE);
    }
    
    String displayState = menuItems[i].state == 0 ? "Ready" : "Active";
    if (menuItems[i].menuId == MenuId_TestMode) {
      displayState = menuItems[i].state == 0 ? "Off": "On";
    }
    display.print(">" + menuItems[i].menuChoice + ":" + displayState);
    rowPosition += 10;
    display.setCursor(0, rowPosition);
  }
  display.display();
}

void turnBacklightOn()
{
  digitalWrite(7, HIGH);
}

void turnBacklightOff()
{
  digitalWrite(7, LOW);
}

void initializePanelControlButtonInputs() {
  // Initialize panel control buttons
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
}

stateChanged() {
  if (refillTrigger) {
    updateMenuState(MenuId_Refill, 1);
  } else if (emptyTrigger) {
    updateMenuState(MenuId_Drain, 1);
  } else if (fillTrigger) {
    updateMenuState(MenuId, Fill, 1);
  } else {
    updateMenuState(MenuId_Drain, 0);
    updateMenuState(MenuId_Fill, 0);
    updateMenuState(MenuId_Empty, 0);
  }
  if (testModeEnabled) {
    updateMenuState(MenuId_TestMode, 1);
  } else {
    updateMenuState(MenuId_TestMode, 0);
  }
  invalidateDisplay = true;
}

void log(String logMsg) {
  Serial.println("[" + String(millis()) + "]: " + logMsg);
}
