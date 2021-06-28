/*
 * Project AquaMaster-Fountain - V3 Hardware - For our Backyard Fountain
 * Description: Watering program for the back patio fountain
 * Author: Chip McClelland
 * Date: 1/10/21
 */


// v0.10 - Initial test Motor control
// v0.20 - Testing to see if we are at operational state - no sensors
// v0.25 - Updated with minor bug fixes
// v0.30 - Fixed watchdog issue and rate limited publishes
// v0.31 - Bug fixes and added day of the week mode
// v0.32 - More fixes for time and control
// v0.33 - Fixed the issue with setting new off time for weekend or weekdays
// v0.34 - Allowed for 24 hour operations
// v0.35 - Moved to a new model with PWM on the pump and the option to enable and disable the fountain
// v0.40 - Fixed time managment
// v0.41 - Fixed messaging - also made Sunday part of the weekend
// v0.42 - Fixed more messaging and logic issues
// v0.43 - Fixed minor errors
// v0.44 - Fixed weekday error
// v0.45 - Pump turned off too early - Timezone not set so added a variable.  Fixed issue with LED status

STARTUP(System.enableFeature(FEATURE_RESET_INFO));                      // Track why we reset
SYSTEM_THREAD(ENABLED);
#define DSTRULES isDSTusa
const char releaseNumber[8] = "0.45";                                   // Displays the release on the menu 

namespace EEPROMaddr {                                                  // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                                       // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                                       // Where we store the system status data structure
    currentStatusAddr     = 0x50                                        // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                                        // Increment this number each time the memory map is changed

struct systemStatus_structure {                                         // currently 14 bytes long
  uint8_t structuresVersion;                                            // Version of the data structures (system and data)
  uint8_t placeholder;                                                  // available for future use
  uint8_t connectedStatus;
  uint8_t verboseMode;
  int resetCount;                                                       // reset count of device (0-256)
  float timezone;                                                       // Time zone value -12 to +12
  float dstOffset;                                                      // How much does the DST value change?
  int pumpOnHour;                                                       // When do we turn on the pump
  int ledOnHour;                                                        // When do we turn on the leds
  int powerOffHour;                                                     // When do we turn things off for the night
  int weekendOffHour;                                                   // Off hour for the week ends
  int weekdayOffHour;                                                   // Same for the week days
  int pumpPWMvalue;                                                      // PWM value for the pump 
  int fountainEnabled;                                                  // Whether or not we will enable the pump
  unsigned long lastHookResponse;                                       // Last time we got a valid Webhook response
} sysStatus;

struct currentCounts_structure {                                        // Current values on the device - NOT STORED IN EEPROM TO AVOID WEAR
  uint8_t ledPower;                                                     // ledPowe - Controls the water pump 6V - center pin negative
  uint8_t pumpPower;                                                    // pumpPower - Controls the LEDs on the fountain
  unsigned long lastEventTime;                                          // When did we record our last event
  int temperature;                                                      // Current Temperature - Not currently used
  int lightLevel;                                                       // Current Light Level - Not currently used
  int alertCount;                                                       // What is the current alert count
} current;


// Prototypes and System Mode calls
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));                              // Use this line to enable the external WiFi Antenna
SYSTEM_MODE(AUTOMATIC);                                                 // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                                                 // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, CONTROL_STATE};
char stateNames[5][14] = {"Initialize", "Error", "Idle", "Control"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Constants for Pins
const int ledPowerPin = D7;                                             // Pin that controls the MOSFET that turn on the water
const int pumpPowerPin = D2;                                            // Used for debugging, can see when water is ON
const int donePin = D6;                                                 // Pin the Electron uses to "pet" the watchdog
const int wakeUpPin = WKP;                                              // This is the Particle Photon WKP pin

// Timing Variables
unsigned long webhookWait = 45000;                                      // How long we will wair for a webhook response
unsigned long resetWait = 30000;                                        // Honw long we will wait before resetting on an error
unsigned long webhookTimeStamp = 0;                                     // When did we send the webhook
unsigned long resetTimeStamp = 0;                                       // When did the error condition occur

// Program Variables
volatile bool watchdogFlag = false;
char SignalString[64];                                                  // Used to communicate Wireless RSSI and Description
char currentOffsetStr[10];                                              // What is our offset from UTC
int currentHourlyPeriod = 0;                                            // Need to keep this separate from time so we know when to report
bool systemStatusWriteNeeded = false;
char ledPowerStr[16] = "Led Off";                                       // What is the state of the output pin
char pumpPowerStr[16] = "Pump Off";                                     // What is the state of the output pin


void setup()                                                            // Note: Disconnected Setup()
{
  pinMode(ledPowerPin,OUTPUT);                                          // LED control Pin
  pinMode(pumpPowerPin,OUTPUT);                                         // Pump control Pin
  pinMode(wakeUpPin,INPUT);                                             // This pin is active HIGH
  pinMode(donePin,OUTPUT);                                              // Allows us to pet the watchdog

  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("ledPower", ledPowerStr);                           // "On" or "Off"
  Particle.variable("pumpPower", pumpPowerStr);                         // "On" or "Off"
  Particle.variable("Led On Hour",sysStatus.ledOnHour);
  Particle.variable("Pump On Hour",sysStatus.pumpOnHour);
  Particle.variable("Power Off Hour",sysStatus.powerOffHour);
  Particle.variable("Weekend Off Hour",sysStatus.weekendOffHour);
  Particle.variable("Weekday Off Hour",sysStatus.weekdayOffHour);
  Particle.variable("PWMvalue",sysStatus.pumpPWMvalue);
  Particle.variable("Fountain Enabled",sysStatus.fountainEnabled);
  Particle.variable("TimeZone", currentOffsetStr);

  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-PumpOn", setPumpOnHour);
  Particle.function("Set-LedOn", setLedOnHour);
  Particle.function("Set-WeekendOff",setWeekendOffHour);
  Particle.function("Set-WeekdayOff",setWeekdayOffHour);
  Particle.function("Set-PWMvalue",setPWMvalue);
  Particle.function("EnableFountain",setEnableFountain);

  EEPROM.get(EEPROMaddr::systemStatusAddr, sysStatus);

  checkSystemValues();                                                  // Make sure System values are all in valid range

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                     // If so, store incremented number - watchdog must have done This
  }
  
  Time.setDSTOffset(sysStatus.dstOffset);                               // Set the value from FRAM if in limits
  DSTRULES() ? Time.beginDST() : Time.endDST();                         // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                        // Set the Time Zone for our device

  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  if (Time.weekday() >= 6) {                                                // Weekend!
    sysStatus.powerOffHour = sysStatus.weekendOffHour;                // Set for weekend off hour
  }
  else sysStatus.powerOffHour = sysStatus.weekdayOffHour;             // Set for weekday off hour
  
  petWatchdog();
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                      // The watchdog timer will signal us and we have to respond

  takeMeasurements();

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (Time.hour() != currentHourlyPeriod) state = CONTROL_STATE;    // We want to report on the hour
    break;

  case CONTROL_STATE: 
    // In this mode, the pump hours are assumed to be longer than the led hours
    // Pump off can be 0 to 24
    // Pump on can be 0 to 24 as well   

    if (sysStatus.verboseMode && state != oldState) publishStateTransition();

    currentHourlyPeriod = Time.hour();

    if (Particle.connected()) {
      if (Time.hour() == 0) dailyCleanup();                             // Once a day, clean house
      takeMeasurements();                                               // Update Temp, Battery and Signal Strength values
      state = IDLE_STATE;                                               // Wait for Response
    }
    else {
      resetTimeStamp = millis();
      state = ERROR_STATE;
    }

    if (sysStatus.fountainEnabled == 0) {
      current.pumpPower = current.ledPower = false;
      digitalWrite(pumpPowerPin,LOW);
      strncpy(pumpPowerStr, "Pump Off", sizeof(pumpPowerStr));

      digitalWrite(ledPowerPin, LOW);
      strncpy(ledPowerStr , "Led Off", sizeof(pumpPowerStr));
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Control","Fountain Not Enabled Turned Everything Off",PRIVATE);
      break;
    }

    // Take care of the pump

    if (Time.hour() >= sysStatus.pumpOnHour && Time.hour() < sysStatus.powerOffHour) {                // Pump should be on - is it?
      if (!current.pumpPower) {                                                                       // Not on - we need to fix this
        current.pumpPower = true;
        analogWrite(pumpPowerPin , sysStatus.pumpPWMvalue);
        strncpy(pumpPowerStr , "Pump On", sizeof(pumpPowerStr));
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("Control", "Turned on the Pump",PRIVATE);
      }
    }
    else {                                                                                            // Pump should be off - is it?
      if (current.pumpPower) {                                                                        // Is is on - we need to fix this
        current.pumpPower = false;
        digitalWrite(pumpPowerPin,LOW);
        strncpy(pumpPowerStr, "Pump Off", sizeof(pumpPowerStr));
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("Control", "Turned off the Pump",PRIVATE);
      }
    }

    // Now look at the LED
    if (Time.hour() >= sysStatus.ledOnHour && Time.hour() < sysStatus.powerOffHour) {                 // Time for the LED to be on - is it?
      if (!current.ledPower) {                                                                        // Not on - we need to fix this
        current.ledPower = true;
        digitalWrite(ledPowerPin,HIGH);
        strncpy(ledPowerStr,"Led On",sizeof(ledPowerStr));                                            // LED needs to be turned on
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("Control", "Turned on the LEDs",PRIVATE);
      }
    }
    else  {                                                                                           // Time for the LED to be off - is it?
      if (current.ledPower) {                                                                         // It is on - we need to fix this
        current.ledPower = false;
        digitalWrite(ledPowerPin, LOW);
        strncpy(ledPowerStr , "Led Off", sizeof(ledPowerStr));                                       // LED needs to be turned off
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("Control","Turned off the LEDs",PRIVATE);
      }
    }
    state = IDLE_STATE;
    break;

  case ERROR_STATE:                                                     // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                  // First try simple reset
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) {       //It has been more than two hours since a sucessful hook response
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Took long since last webhook response - Reset", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        System.reset();
      }
      else {                                                            // If we have had 3 resets - time to do something more
        waitUntil(meterParticlePublish);
        if (Particle.connected()) Particle.publish("State","Error State - Too many resets", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                       // Zero the ResetCount
        EEPROM.put(EEPROMaddr::systemStatusAddr,sysStatus);
        System.reset();                                                 // Full Modem reset and reboots
      }
    }
    break;
  }
  // Main loop housekeeping
  if (watchdogFlag) petWatchdog();                                      // Pet the watchdog if needed

  if (systemStatusWriteNeeded) {                                        // Batch write updates to FRAM
    EEPROM.put(EEPROMaddr::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
}

// Take measurements
void takeMeasurements() {                                               // For clarity - 0 = no alert, 1 = alert (), 2 = 75% full and is flashing
 
  getSignalStrength();                                                  // Test signal strength if the cellular modem is on and ready

  // currentStatusWriteNeeded = true;                                   // Not needed until we add more sensors
}

void getSignalStrength() {
  WiFiSignal sig = WiFi.RSSI();
  snprintf(SignalString,sizeof(SignalString), "WiFi Quality: %.02f%% Strength: %.02f%%", sig.getQuality(), sig.getStrength());
}

void petWatchdog() {
  digitalWrite(donePin, HIGH);                                          // Pet the watchdog
  digitalWrite(donePin, LOW);
  watchdogFlag = false;
}

// Here is were we will put the timer and other ISRs
void watchdogISR() {
  watchdogFlag = true;
}


bool meterParticlePublish(void)
{
  static unsigned long lastPublish = 0;
  if(millis() - lastPublish >= 1000) {
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

/* These are the particle functions that allow you to configure and run the device
 * They are intended to allow for customization and control during installations
 * and to allow for management.
*/

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}


void dailyCleanup() {                                                   // Called from Reporting State ONLY - clean house at the end of the day
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Daily Cleanup","Running", PRIVATE);               // Make sure this is being run
  sysStatus.verboseMode = false;
  sysStatus.resetCount = 0;
  Particle.syncTime();                                                // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                               // Wait for up to 30 seconds for the SyncTime to complete
  systemStatusWriteNeeded = true;
  if (Time.weekday() == 1 || Time.weekday() >= 6) {                           // Weekend! (1 = Sunday, 6 /7 = Friday / Saturday)
    sysStatus.powerOffHour = sysStatus.weekendOffHour;                // Set for weekend off hour
  }
  else sysStatus.powerOffHour = sysStatus.weekdayOffHour;             // Set for weekday off hour
}

void loadSystemDefaults() {                                             // Default settings for the device - connected, not-low power and always on
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.timezone = -5;                                              // Default is East Coast Time
  sysStatus.dstOffset = 1;
  EEPROM.put(EEPROMaddr::systemStatusAddr,sysStatus);                   // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                              // Checks to ensure that all system values are in reasonable range
  if (sysStatus.verboseMode < 0 || sysStatus.verboseMode > 1) sysStatus.verboseMode = false;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.pumpOnHour < 0 || sysStatus.pumpOnHour > 24) sysStatus.pumpOnHour = 12;
  if (sysStatus.ledOnHour < 0 || sysStatus.ledOnHour > 24) sysStatus.ledOnHour = 18;
  if (sysStatus.powerOffHour < 0 || sysStatus.powerOffHour > 24) sysStatus.powerOffHour = 20;
  if (sysStatus.weekdayOffHour < 0 || sysStatus.weekdayOffHour > 24) sysStatus.weekdayOffHour = 20;
  if (sysStatus.weekendOffHour < 0 || sysStatus.weekendOffHour > 24) sysStatus.weekendOffHour = 23;
  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}


void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  waitUntil(meterParticlePublish);
  if (sysStatus.verboseMode) if (Particle.connected()) Particle.publish("State Transition",stateTransitionString,PRIVATE);
}


// The following functions will change the time when the pump or led turns on or off and will precipitate a change to the control state.

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  Particle.syncTime();                                                  // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                 // Wait for up to 30 seconds for the SyncTime to complete
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                 // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) || (tempTimeZoneOffset > 12)) return 0; // Make sure it falls in a valid range or send a "fail" result
  sysStatus.timezone = (float)tempTimeZoneOffset;
  Time.zone(sysStatus.timezone);
  systemStatusWriteNeeded = true;                                       // Need to store to FRAM back in the main loop
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
    waitUntil(meterParticlePublish);
    if (Particle.connected()) Particle.publish("Time",Time.timeStr(Time.now()), PRIVATE);
  }
  state = CONTROL_STATE;
  return 1;
}


int setDSTOffset(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempDSTOffset = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempDSTOffset < 0) || (tempDSTOffset > 2)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  Time.setDSTOffset((float)tempDSTOffset);                              // Set the DST Offset
  sysStatus.dstOffset = (float)tempDSTOffset;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "DST offset %2.1f",sysStatus.dstOffset);
  if (Time.isValid()) isDSTusa() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
    if (Particle.connected()) Particle.publish("Time",Time.timeStr(t), PRIVATE);
  }
  state = CONTROL_STATE;
  return 1;
}

int setPumpOnHour (String command) {                                    // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempHour = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempHour < 0) || (tempHour > 24)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.pumpOnHour = tempHour;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "Pump On Hour set to %i",sysStatus.pumpOnHour);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}

int setLedOnHour (String command) {                                     // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempHour = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempHour < 0) || (tempHour > 23)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.ledOnHour = tempHour;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "Led On Hour set to %i",sysStatus.ledOnHour);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}

int setWeekendOffHour (String command) {                                  // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempHour = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempHour < 0) || (tempHour > 24)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.weekendOffHour = tempHour;
  if (Time.weekday() == 1 || Time.weekday() >= 6) sysStatus.powerOffHour = sysStatus.weekendOffHour; // Set if we are on the weekend
  else sysStatus.powerOffHour = sysStatus.weekdayOffHour;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "Weekend Power Off Hour set to %i",sysStatus.weekendOffHour);
  if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}

int setWeekdayOffHour (String command) {                                  // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempHour = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempHour < 0) || (tempHour > 24)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.weekdayOffHour = tempHour;
  if (Time.weekday() >1 && Time.weekday() < 6) sysStatus.powerOffHour = sysStatus.weekdayOffHour; // Set if we are in the week
  else sysStatus.powerOffHour = sysStatus.weekendOffHour;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "Weekday Power Off Hour set to %i",sysStatus.weekdayOffHour);
  if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}

int setPWMvalue (String command) {                                  // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int tempValue = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 255)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.pumpPWMvalue = tempValue;
  systemStatusWriteNeeded = true;
  if (current.pumpPower) analogWrite(pumpPowerPin , sysStatus.pumpPWMvalue);      // If the pump is on, we can refresh the value here
  snprintf(data, sizeof(data), "Pump PWM value set to %i",sysStatus.pumpPWMvalue);
  if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}

int setEnableFountain (String command) {                                  // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempValue = strtol(command,&pEND,10);                           // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 1)) return 0;                       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.fountainEnabled = tempValue;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "Fountain %s", (sysStatus.fountainEnabled) ? "Enabled" : "Not Enabled");
  if (Particle.connected()) Particle.publish("Control",data, PRIVATE);
  state = CONTROL_STATE;
  return 1;
}


bool isDSTusa() {
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}