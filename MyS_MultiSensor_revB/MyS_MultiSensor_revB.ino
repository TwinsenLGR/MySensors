/**
MySensors Multi Sensor Node

Design by scalz: https://www.openhardware.io/view/75/MyMultisensors

Rev B: Change to non-interrupt sleep routine, interrupts handled separately
       Sensors now report at least every n cycles
       WDT used while sensor is awake to reset on lockups
       Added minimum high threshold for OPT sensor to prevent sunrise spam
       Lux thresholds changed to logarithmic
Rev A: Initial release
       MyS sleep with wake on interrupt
       Problems with sensor locking up and draining battery
       Linear threshold scaling for Lux sensor

Cary Wintle
July 2017
*/

#define SN "MultiSensor"
#define SV "0.16"

// Per-unit Configuration
// ----------------------
// VCC Calibration
//   ID   |    VCC_CAL
// -------------------
//   10   |   1122009L
//   11   |   1159092L
//   12   |   1126000L
//   13   |   1115458L
//   14   |   1148884L
#define MY_NODE_ID 10
#define VCC_CAL 1122009L
// To calibrate VCC reading, measure battery voltage with a meter, compare it to the value reported:
// VCC_CAL = Vmeter / Vreported * 1125300

// MySensors Configuration
// -----------------------
//#define MY_DEBUG 
#define MY_RADIO_RFM69
#define MY_RFM69_RST_PIN A2
#define MY_RFM69_NETWORKID 137
#define MY_RFM69_ENABLE_ENCRYPTION
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#include <MySensors.h>  

// Software Configuration
// -----------------------
#define SLEEP_TIME 600 // Sleep time interval between sensor reads (in seconds, 600s = 10 min)
#define BAT_UPDATE_INTERVAL 144 // Interval between battery updates (multiples of SLEEP_TIME) (144 * 10 min = 24 hours)
#define BATV_MIN 2.0 // Minimum battery voltage for 0% reading
#define BATV_MAX 3.0 // Maximum battery voltage for 100% reading
#define PIR_TRIGGER_COUNT 5 // PIR pulses required to trigger
#define PIR_SETTLING_TIME 30000 // Time(ms) to wait after power up for PIR to settle
#define PIR_PRETRIGGER_TIME 13000 // Max time(ms) in ms since last PIR pulse before counter resets (MyS library sleep is inaccurate, 13000 is about 5 seconds)
#define PIR_HOLDOFF_TIME 60000 // Time(ms) after a PIR trigger when new PIR triggers are supressed
#define MAX_INTERRUPTS 30 // Maximum number of interrupts before a compulsory sensor read cycle is triggered (Stops continuous interrupts from silencing other sensors)
#define LUX_THRESHOLD 50 // Change in lux to trigger new message (low is percentage, high is 1/low)
#define FORCE_CYCLES 18 // Force sensor updates after 12 intervals without one (18 * 10 min = 3 hours)

// Other Libraries
// ----------------------
#include <avr/wdt.h>
#include <OneWire.h>

// Temp & Humidity Sensor
#include "Adafruit_Si7021.h"
Adafruit_Si7021 thSensor = Adafruit_Si7021();

// Lux Sensor
#include "Opt3001.h"
Opt3001 opt; 
#define OPT_INTERRUPT_PIN A1

// Pin change interrupts
#define EI_NOTEXTERNAL // External interrupts managed by built-in routines
#include <EnableInterrupt.h>

// Global Variables
// ----------------
volatile byte pirInt = 0, optInt = 0; // Interrupt flags
uint8_t nNoUpdatesTemp, nNoUpdatesHum, nNoUpdatesLux;
bool pirHoldoff = false; // Flag that the PIR sensor is in holdoff mode (has been triggered recently)
bool cycleComplete = true; // Flag that the sleep time completed so we can start a new sensor cycle
bool pirPreTrigger = false; // Flag that the pir has received a pulse
bool pirSettling = true; // Initial settling time for PIR sensor
byte pirCount = 0; // Count of triggers on PIR inputs
byte IRQsSinceLastCycle; // Counts interrupts received since a full sleep cycle was completed
int cycleCount = BAT_UPDATE_INTERVAL; // Send battery update immediately
int32_t cycleTimer; // Accumulator for cycle time in ms
const uint8_t forceUpdates = FORCE_CYCLES;

// MyS Messages
// ------------
MyMessage tempMsg(0, V_TEMP);
MyMessage humMsg(0, V_HUM);
MyMessage luxMsg(0, V_LEVEL);
MyMessage pirMsg(0, V_TRIPPED);
MyMessage doorMsg(0, V_STATUS);
MyMessage batVMsg(0, V_VOLTAGE);

// ---------------------------------------
void presentation() {
  // Send the sketch and version information to the gateway and controller
  sendSketchInfo(SN, SV);

  // Present sensor to controller
  present(0, S_CUSTOM, "Multi Sensor");
}

// ----------------------------------------
void setup() {
  Wire.setClock(400000L); // Fast I2C bus
  thSensor.begin();
  opt.begin(DEFAULT_CONFIG_100_OS); // Start the sensor - quick one-shot measurement
  attachInterrupt(digitalPinToInterrupt(3), doorIRQ, CHANGE);
}

//-------------------------------------------
void pirIRQ() {
  pirInt++;
  _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
}

void optIRQ() {
  optInt++;
  _wokeUpByInterrupt = 0xFD; // Dirty hack to get out of MySensors sleep loop
}

void doorIRQ() {
  //doorInt++;
  _wokeUpByInterrupt = 0xFC; // Dirty hack to get out of MySensors sleep loop
}

// --------------------------------------------
void loop() {

  static float lastTemperature, lastLux;
  static byte lastHumidity;
  static bool lastDoor;
  bool doorState;
  const int32_t sleepTime = (int32_t)SLEEP_TIME * 1000;

  if(!pirHoldoff && pirInt != 0) {
    pirCount += pirInt;
    pirInt = 0;
    pirPreTrigger = true;
    #ifdef MY_DEBUG
      Serial.print("PIR Count: ");
      Serial.println(pirCount);
    #endif
    if(pirCount > PIR_TRIGGER_COUNT) {
      disableInterrupt(6);
      disableInterrupt(7);
      send(pirMsg.set(1));
      pirCount = 0;
      pirPreTrigger = false;
      pirHoldoff = true;
    }
  }

  if(optInt != 0) {
    optInt = 0;
    optSendAndSetThresholds();
  }
  
  if(cycleComplete || IRQsSinceLastCycle > MAX_INTERRUPTS) {
    
    #ifdef MY_DEBUG
      Serial.println("-- New Cycle --");
    #endif

    IRQsSinceLastCycle = 0;
    cycleTimer = sleepTime; // Reload cycle time accumulator
    
    // Fetch and round temperature to one decimal
    float temperature = (float)((int)(thSensor.readTemperature() * 10.)) / 10.;
    #ifdef MY_DEBUG
      Serial.print("Temp: "); Serial.println(temperature);
    #endif
    // Only send data if temperature has changed
    if (lastTemperature != temperature || nNoUpdatesTemp == forceUpdates) {
      // Send in the new temperature
      send(tempMsg.set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature=temperature;
      nNoUpdatesTemp = 0;
    } else {
      nNoUpdatesTemp++;
    }
  
    // Fetch and round humidity to whole value
    byte  humidity = (byte)thSensor.readHumidity();
    #ifdef MY_DEBUG
      Serial.print("Hum: "); Serial.println(humidity);
    #endif
    // Only send data if temperature has changed
    if (lastHumidity != humidity|| nNoUpdatesHum == forceUpdates) {
      // Send in the new temperature
      send(humMsg.set(humidity));
      // Save new temperatures for next compare
      lastHumidity=humidity;
      nNoUpdatesHum = 0;
    } else {
      nNoUpdatesHum++;
    }

    // Check if it's been too long since we sent a Lux value
    if(nNoUpdatesLux >= forceUpdates) {
      optSendAndSetThresholds();
    }
    
    if (cycleCount >= BAT_UPDATE_INTERVAL) {
      cycleCount = 0;
      //int batV = readVCC();
      //float batVolts = batV / 1000.0;
      float batVolts = readVCC() / 1000.0;
      #ifdef MY_DEBUG
        Serial.print("BatV: "); Serial.println(batVolts);
      #endif
      send(batVMsg.set(batVolts, 2));
      //float batPercent = constrain(map(batVolts, (float)BATV_MIN, (float)BATV_MAX, 0.0, 100.0), 0.0, 100.0);
      float batPercent = (batVolts - BATV_MIN) / (BATV_MAX - BATV_MIN) * 100.0;
      batPercent = constrain(batPercent,0,100);
      #ifdef MY_DEBUG
        Serial.print("Bat%: "); Serial.println(batPercent);
      #endif
      sendBatteryLevel((int)batPercent);

      // Take a Lux reading in case the sensor has gotten confused
      // (This is also where we start the sensor on first run)
      optSendAndSetThresholds();
      opt.begin(DEFAULT_CONFIG_800); // Start the sensor free-running
      enableInterrupt(OPT_INTERRUPT_PIN, optIRQ, FALLING);
    }
    cycleCount++;
    nNoUpdatesLux++;
  }

  IRQsSinceLastCycle++;
  cycleComplete = false;

  doorState = digitalRead(3);
  if(doorState != lastDoor) {
    send(doorMsg.set(doorState));
    lastDoor = doorState;
  }

  wdt_disable(); // Disable WDT before sleeping (sleep uses WDT anyway)
  
  // On first startup we sleep for a bit to let the PIR settle
  if(pirSettling) {
    sleep(PIR_SETTLING_TIME);
    if(_wokeUpByInterrupt == INVALID_INTERRUPT_NUM) { // If sleep cycle completed
      #ifdef MY_DEBUG
        Serial.println("- PIR Settled -");
      #endif
      pirSettling = false;
      enableInterrupt(6, pirIRQ, RISING);
      enableInterrupt(7, pirIRQ, RISING);
      cycleTimer -= PIR_SETTLING_TIME;
      if (cycleTimer < 0)
        cycleComplete = true; // Flag that a full sleep cycle was completed
    }
  } else if(pirPreTrigger) {
    sleep(PIR_PRETRIGGER_TIME);
    if(_wokeUpByInterrupt == INVALID_INTERRUPT_NUM) { // If sleep cycle completed
      #ifdef MY_DEBUG
        Serial.println("- PIR Pretrigger Reset -");
      #endif
      pirPreTrigger = false;
      pirCount = 0;
      pirInt = 0;
      cycleTimer -= PIR_PRETRIGGER_TIME;
      if (cycleTimer < 0)
        cycleComplete = true; // Flag that a full sleep cycle was completed
    } else {
      cycleTimer -= PIR_PRETRIGGER_TIME / 2;
      if (cycleTimer < 0)
        cycleComplete = true; // Flag that a full sleep cycle was completed
    }
  } else if(pirHoldoff) {
    sleep(PIR_HOLDOFF_TIME);
    if(_wokeUpByInterrupt == INVALID_INTERRUPT_NUM) { // If sleep cycle completed
      #ifdef MY_DEBUG
        Serial.println("- PIR Holdoff Expired -");
      #endif
      pirHoldoff = false;
      pirCount = 0;
      pirInt = 0;
      enableInterrupt(6, pirIRQ, RISING);
      enableInterrupt(7, pirIRQ, RISING);
      cycleTimer -= PIR_HOLDOFF_TIME;
      if (cycleTimer < 0)
        cycleComplete = true; // Flag that a full sleep cycle was completed
    } else {
      cycleTimer -= PIR_HOLDOFF_TIME / 2;
      if (cycleTimer < 0)
        cycleComplete = true; // Flag that a full sleep cycle was completed
    }
  } else {
    sleep((uint32_t)cycleTimer);
    if(_wokeUpByInterrupt == INVALID_INTERRUPT_NUM) { // If sleep cycle completed
      #ifdef MY_DEBUG
        Serial.println("- Wake From Full Sleep Cycle -");
      #endif
      cycleComplete = true; // Flag that a full sleep cycle was completed
    } else {
      cycleTimer -= cycleTimer / 4; // If we get woken from normal sleep by interrupt, 1/4 off the remaining cycle time
    }
  }
  _wokeUpByInterrupt = INVALID_INTERRUPT_NUM; // Fix dirty hack on MyS sleep loop

  wdt_enable(WDTO_2S); // Enable WDT for the duration of processing

  doorState = digitalRead(3);
  if(doorState != lastDoor) {
    send(doorMsg.set(doorState));
    lastDoor = doorState;
  }
} // end of loop()

// -----------------------------------
int readVCC() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  wait(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  int result = (high<<8) | low;

  result = VCC_CAL / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// ----------------------------------------
void optSendAndSetThresholds() {
  // Read the optical sensor, send the value to the controller, and reset the thresholds

  // const float upScale = 1 + ((float)LUX_THRESHOLD / 100);
  const float dnScale = 1 - ((float)LUX_THRESHOLD / 100);
  const float upScale = 1 / dnScale; // Logarithmic scale
  
  // Fetch and send light level
  uint16_t rawLux = opt.readRawResult();
  float lux = calcLux(rawLux);
  #ifdef MY_DEBUG
    Serial.print("Lux: "); Serial.println(lux);
  #endif

  // Send Lux with variable precision based on value
  if(lux > 650.0) send(luxMsg.set(lux, 0));
  else if (lux > 80.0) send(luxMsg.set(lux, 1));
  else send(luxMsg.set(lux, 2));

  nNoUpdatesLux = 0;
  
  // Calculate the new upper and lower limit thresholds and set the registers
  uint16_t expLux = rawLux & 0xF000; // Mask off the fractional, leaving the exponent
  rawLux &= 0x0FFF; // Mask off the exponent, leaving the fractional
  uint16_t limitFrac = rawLux * upScale; // Scale the fractional
  if(limitFrac < 4096) {
    limitFrac |= expLux; // Add back the exponent
    if(limitFrac < 100) limitFrac = 100; //If the high limit is small (<1 lux), just set it to 1 (prevents sunrise spam)
  } else { //If the fractional exceeds full scale, jump to next higher range
    limitFrac /= 2;
    limitFrac |= expLux + 0x1000; // Add back the exponent one range higher
  }
  opt.setHighLimit(limitFrac); // Set the limit register
  #ifdef MY_DEBUG
    Serial.print("H-Th: "); Serial.println(calcLux(limitFrac));
  #endif
  limitFrac = rawLux * dnScale; //Scale the fractional
  limitFrac |= expLux; // Add back the exponent
  if(limitFrac < 30) limitFrac = 0; // If the lower limit is really small (<0.3 lux), just set it to zero
  opt.setLowLimit(limitFrac); // Set the limit register
  #ifdef MY_DEBUG
    Serial.print("L-Th: "); Serial.println(calcLux(limitFrac));
  #endif
}

float calcLux(uint16_t raw) {
  return 0.01 * (1 << ((raw & 0xF000)>>12)) * (raw & 0xFFF);
}
