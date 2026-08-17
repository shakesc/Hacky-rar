/*
Hacky Monitor Dv1.0
*/
char monitor_version[] = "Hacky Limited";

#define Baud_Rate 19200
#define ESP32

//#define simulator

// https://learn.adafruit.com/thermistor/using-a-thermistor
// https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
// https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// comment out if the debug is not required


#define accelerometer
#define turbo_button

//#define debug
//#define debugSpeed 1
//#define debug_hi
//#define debug_throttle
//#define debug_therm
//#define debug_batt
//#define debug_diff
//#define debug_accel
//#define debug_speed

#include <Arduino.h>
#include <millisDelay.h>
#include <loopTimer.h>  // check speed of main loop for debug
#include <Wire.h>


#include <Adafruit_Sensor.h>

//#include <MPU6050_tockn.h>
#include <MPU6050_light.h>


#include "hackyhacky.h"

// ESP32 serial UART library
#include <HardwareSerial.h>

#define ADC_current_sensor 0
#define ADC_steer 1
#define ADC_temp_sensor 2
#define ADC_throttle 3

#define PIN_sck 18  // SCK
#define PIN_miso 19
#define PIN_mosi 23  // SDA

#define PIN_vsupply 14

#define PIN_NTC_feed 2  //27

#define PIN_vdiff 27

// Digital pins
#define PIN_speedo 4
#define PIN_tft_blk 36
#define PIN_reset 39
#define PIN_turbo_boost 26

#define encoderCLK 34
#define encoderDT 35
#define encoderRotC 32

#define PIN_Throttle_Control 36

#define ledCurrentPin 25
#define ledTempPin 33

#define DAC_throttle_left MCP4728_CHANNEL_C
#define DAC_throttle_right MCP4728_CHANNEL_D


// ==============================================================================================================
// Throttle Map.  Row Throttle , Column Steering
static int const throttle_map[10][11] = {
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
  { 70, 70, 70, 80, 90, 100, 90, 80, 70, 70, 70 },
  { 70, 70, 70, 80, 90, 100, 90, 80, 70, 70, 70 },
  { 70, 70, 70, 80, 90, 100, 90, 80, 70, 70, 70 },
  { 80, 80, 85, 90, 90, 100, 90, 85, 80, 80, 80 },
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
  { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
};

// ==============================================================================================================
// Throttle Map.  Row Throttle , Column Steering
static int const throttle_map_2[10][2] = {
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
  { 100, 20 },
};


#define throttleLower 4000
#define throttleUpper 25000
#define throttleDiff 21000
#define throttlemultiplier 2100

#define speed_adjust 1                      // set to zero if no differential
uint16_t steering_position_centre = 30000;  // over this speed kmh then do not adjust based on steering angle
#define steeringLower 412                   // left max value
#define steeringUpper 24642                 // right max value
#define steeringmultiplier 2203             // right - left / 11
#define steering_mid_value 12195            // 2.29V

#define speed_max 27  // vmax

uint8_t throttle_mapping_case = 0;  // which routing for throttle mapping

uint8_t throttle_index = 0;
uint8_t throttle_multiple = 0;

uint16_t diff_multiplier_left = 0;
uint16_t diff_multiplier_right = 0;
uint16_t diff_delta_left = 0;
uint16_t diff_delta_right = 0;
uint16_t diff_offset = 0;
uint16_t diff_offset_left = 0;
uint16_t diff_offset_right = 0;

uint16_t dac_throttle = 0;
uint16_t dac_throttle_short = 0;
uint16_t dac_throttle_left = 0;
uint16_t dac_throttle_right = 0;

uint16_t dac_throttle_left_short = 0;
uint16_t dac_throttle_right_short = 0;

uint8_t turbo_boost_state = 0;

uint8_t voltage_monitor_state = 0;
uint8_t voltage_monitor = 0;

uint8_t steering_index = 0;
uint8_t diff_multiplier = 0;
long dac_throttle_temp = 0;

// ==============================================================================================================
/* Assign a unique ID to this sensor at the same time */

MPU6050 mpu(Wire);

// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(42);
float xAccl, yAccl, zAccl;
float xAccl_offset, yAccl_offset, zAccl_offset;
float xAccl_norm, yAccl_norm, zAccl_norm;

float xGyro, yGyro, zGyro;
float xGyro_offset, yGyro_offset, zGyro_offset;
float xGyro_norm, yGyro_norm, zGyro_norm;

// ==============================================================================================================
// ADC library and class declaration
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
const int ADCintPin = 2;

// ==============================================================================================================
// MCP4728 4-Channel 12-bit I2C DAC
#include <Adafruit_MCP4728.h>
Adafruit_MCP4728 mcp;

// ==============================================================================================================

#include <TFT_eSPI.h>
#include <SPI.h>
TFT_eSPI tft = TFT_eSPI();
#define SCREEN_X 320
#define SCREEN_Y 240
#define SCREEN_X_HALF SCREEN_X / 2
#define SCREEN_Y_HALF SCREEN_Y / 2

#define BAR_WIDTH 280

// ==============================================================================================================
// Register bank

uint16_t throttle_current = 0, throttle_previous = 0, throttle_average = 0, throttle_peak = 0;
float speed_current = 0, speed_previous = 0, speed_average = 0, speed_peak = 0;
float current_inst = 0, current_peak = 0, current_average = 0, current_fuse_loss = 0;

float adc_current_average, adc_steer_average, adc_tcouple_average, adc_throttle_average;

uint16_t current_zero = 0;

float temp_inst = 0, temp_peak = 0, temp_average = 0, temp_fuse_loss = 0;  // temperature when fuse power went

// ==============================================================================================================

// array for averaging
uint8_t adc_index = 0;  // the index of the current reading
const int AvADCReadings = 10;
uint16_t adc_Readings[4][AvADCReadings];  // the readings from the analog input
unsigned long adc_total[4];               // the running total
uint16_t adc_average[4];                  // the running average
short int adc_raw[4];                     // the raw adc
float adc_volts[4];                       // the raw adc

#define ZERO_CURRENT_WAIT_TIME 3000  // wait for 3 seconds to allow zero current measurement
#define READ_DELAY 50                // 20 ms between rearings

// ==============================================================================================================
// WCS Sensitivity constants
/* mV/A
  11.0,  //WCS1500
  22.0,  //WCS1600
  33.0,  //WCS1700
  66.0,  //WCS1800
  70.0   //WCS2800

33mV/A and a bi-directional 70A sensor (140A range) is 0.033volt*140= 4.62volt span on a 5volt supply.
ADC1115 A/D (65535) on 2/3x gain +/- 6.144V, 1 bit = 0.1875mV.   Number of bits per Amp = 33/0.1875 = 176 bits per Amp
ADC1115 A/D (65535) on 1x gain +/- 4.096V  1 bit = 0.125mV.  Number of bits per Amp = 33/0.125 = 264 bits per Amp

Current calc = (ADC Value - ADC Zero) / 176
*/
#define current_resolution 176;

// Current sensor variables
// voltage reading from fuse 3.2v x 12S minimum to 4.2v x 12S minimum
// use voltage divider of 10K and 100K
// =(12S x Nominal x divider) * 65535 / VDD. 3.2 nominal = 45755 , 4.2 nominal = 60053
const float low_voltage_threshold = 40;  // Volts

// ==============================================================================================================
// Indicator LED i/o
#define current_warning_lower 35       // turn on LED if over threshold
#define current_warning_higher 45      // turn on LED if over threshold
#define temperature_warning_lower 40   // turn on LED if over threshold
#define temperature_warning_higher 60  // turn on LED if over threshold
uint16_t vbatt_reading = 0;
uint8_t temp_led_status = 1;
uint8_t current_led_status = 1;

// ==============================================================================================================
// Setup encoder
// https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
// Rotary Encoder Inputs

bool counterChange = true;
int encoderCounter = 0;
int currentStateCLK;
int lastStateCLK;
const int EncoderCounterMax = 6;  // number of use cases for display

// ==============================================================================================================
#define FULL_VREF_RAW_VALUE 4095
unsigned int supply_reading;

// ==============================================================================================================
// https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
// https://learn.adafruit.com/thermistor/using-a-thermistor
// https://circuitdigest.com/microcontroller-projects/interfacing-Thermistor-with-arduino

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

float ThermVolts = 0;
float logThermVolts;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// ==============================================================================================================
#define DISPLAY_UPDATE_DELAY 1000  // milliseconds
millisDelay displayDelay;

#define ADC_UPDATE_DELAY 100  // milliseconds
millisDelay adcDelay;

#define LED_UPDATE_DELAY 200  // milliseconds
millisDelay ledDelay;

#define TURBO_BOOST_DELAY 2000      // milliseconds
#define TURBO_BOOST_MAX_DELAY 6500  // milliseconds
millisDelay boostDelay;

// ==============================================================================================================
uint16_t bar_position[3];     // the readings from the analog input
uint16_t bar_readings[3][8];  // Previous and Current readings

char bar_label[2][15];

// ==============================================================================================================
// wheel circumference in mm * 1000000  * pi = 310 * pi * 1000000 = 973 893 722

// 6 0's were used in scaling up radius and pi, 6 places are divided in the end
// and the units work out. You can use integers more accurate than float on
// Arduino at greatly faster speed. Both type of long can hold any 9-digits.
// Arduino variable type long long can hold any 19 digits is 19 place accuracy.
// if you work in Small Units and scale back later, integers are plenty accurate.
// remember, this value has to be divided by microseconds per turn.

#define wheel_circumference_mm 973.893722  // 310 * pi
const unsigned long wheel_circumference = wheel_circumference_mm * 1000000UL;
// wheel circumference gets divided by microseconds, 1,000,000/sec (usec or us).
// wheel turns once for 94248000 mm/100 in 1000000 usecs =

unsigned long PrevSpeed = 0;

unsigned long DistanceTravelled = 0;
unsigned long DistanceTravelledMeter = 0;
unsigned long DistanceLoopCount = 0;

volatile byte hall_rising = 0;  // interrupt flag
volatile unsigned long irqMicros;

unsigned long startMicros;
unsigned long elapsedMicros;

unsigned long irqWheelTimeStart;
const unsigned long irqWheelTimeWait = 70;

unsigned long displayStartMillis;
const unsigned long displayWaitMillis = 200;

void wheel_IRQ() {
  irqMicros = micros();
  hall_rising = 1;
  DistanceLoopCount++;
}

void boost_IRQ() {
  if (turbo_boost_state < 1) {
    turbo_boost_state = 1;
  }
}


// ==============================================================================================================

void setup() {
  Serial.begin(Baud_Rate);
  Serial.println(monitor_version);

  // Indicator LEDs
  pinMode(ledTempPin, OUTPUT);
  pinMode(ledCurrentPin, OUTPUT);
  digitalWrite(ledTempPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  digitalWrite(ledCurrentPin, HIGH);

  pinMode(PIN_Throttle_Control, INPUT);  // How throttle is being controlled

#ifdef debug
  Serial.println("LEDS on");
#endif

  tft.init();
  Serial.println(F("Initialized"));
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  display_Banner();

  // setup external ADC
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }

  if (!mcp.begin()) {
    Serial.println("Failed to find DAC chip");
    while (1)
      ;
  }

  /* Initialise the sensor */
  // Try to initialize!

#ifdef accelerometer

MPU6050 mpu(Wire);

#endif

#ifdef debug
  Serial.println("Set initial DAC values");
#endif

  // Set channel initial values
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.saveToEEPROM();

  // ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads.setGain(GAIN_TWOTHIRDS);           // +/- 6.144V  1 bit = 0.1875mV (default)
  ads.setDataRate(RATE_ADS1115_250SPS);  //< 32 samples per second
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);  // set adc reading to continuous mode

  delay(ZERO_CURRENT_WAIT_TIME);
  //***************************************

  initialise_adc_arrays();

  pinMode(PIN_reset, INPUT);      // soft reset pin
  pinMode(PIN_NTC_feed, OUTPUT);  // Setup Thermistor
  digitalWrite(PIN_NTC_feed, HIGH);

  Serial.println("NTC ON");  // debug value

  //pinMode(ADCintPin, INPUT_PULLUP);
  // The convention is ready on the falling edge of a pulse at the ALERT/RDY pin.
  // attachInterrupt(ADCintPin, newDataReady, FALLING);

  Serial.println("Setup decoders");  // debug value

  pinMode(encoderCLK, INPUT);  // Setup decoder
  pinMode(encoderDT, INPUT);
  pinMode(encoderRotC, INPUT_PULLUP);
  lastStateCLK = digitalRead(encoderCLK);  // Read the initial state of CLK
  attachInterrupt(digitalPinToInterrupt(encoderCLK), updateEncoder, CHANGE);

  // Kick off the timers for reading sensors
  displayDelay.start(DISPLAY_UPDATE_DELAY);
  // adcDelay.start(ADC_UPDATE_DELAY);
  ledDelay.start(LED_UPDATE_DELAY);

  // Determine the quiescent current for substration later
  setZeroCurrent();

#ifdef accelerometer
  // Determine the offsets for Accel
  setZeroAccel();
#endif 

#ifdef debug
  Serial.println((String) "CURRENT ZERO: " + current_zero);
#endif

#ifdef turbo_button
  pinMode(PIN_turbo_boost, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_turbo_boost), boost_IRQ, FALLING);
    Serial.println((String) "turbo" );
#endif

pinMode(PIN_vdiff, INPUT_PULLDOWN); //Setup pin to monitor voltage

  // Turn LEDs off to indicate ready
  digitalWrite(ledTempPin, LOW);
  digitalWrite(ledCurrentPin, LOW);
  tft.fillScreen(TFT_BLACK);

  // Setup speedo
  pinMode(PIN_speedo, INPUT_PULLUP);                // pin # is tied to the interrupt
  attachInterrupt(PIN_speedo, wheel_IRQ, FALLING);  // pin 2 looks for HIGH to LOW change

#ifdef debug
  Serial.println(wheel_circumference);
#endif
}

//=========================================================================================

void loop() {
  // loopTimer.check(Serial);
  static unsigned long loopCount = 0;  // I expect loop() runs > 33KHz

  if (ads.conversionComplete() == true) {

    //   Serial.println("Complete");
    ads.getLastConversionResults();
    read_adc_values();
    convert_thermistor();

#ifdef accelerometer
    readGyro();
#endif

    if (digitalRead(PIN_reset) == LOW) {
      Serial.println("RESET PUSHED");
    }

    calculate_ground_speed();

    current_inst = (adc_current_average - current_zero) / current_resolution;  // remove quiescent current & adjust for sensor range
    current_peak = (current_inst > current_peak) ? current_inst : current_peak;

    temp_peak = (temp_inst > temp_peak) ? temp_inst : temp_peak;
    speed_peak = (speed_current > speed_peak) ? speed_current : speed_peak;
  }
  throttle_management();
  //  adcDelay.repeat();
  //check_vbatt_monitor();

  
  if (displayDelay.justFinished()) {
    updateDisplay();
  }
  if (ledDelay.justFinished()) {
    updateLED();
  }

  updateBoost();
}
// end of loop


void updateBoost() {
  if (turbo_boost_state == 1) {
    boostDelay.start(TURBO_BOOST_DELAY);
    turbo_boost_state++;
  }
  if ((turbo_boost_state == 2) && boostDelay.justFinished()) {
    boostDelay.start(TURBO_BOOST_MAX_DELAY);
    turbo_boost_state++;
  }
  if ((turbo_boost_state == 3) && boostDelay.justFinished()) {
    turbo_boost_state = 0;
  }
}

void updateDisplay() {

  if (counterChange == true) {
    counterChange = false;
    tft.fillScreen(TFT_BLACK);
  }
  switch (encoderCounter) {

    case 0:
      MainScreen();
      break;

    case 1:
      displayFourRows("Amp P", current_peak, 1, "Amp B", current_fuse_loss, 1, "Temp P", temp_peak, 0, "Temp B", temp_fuse_loss, 0);
      break;

    case 2:
      displayFourRows("Speed", speed_current, 1, "Dist", DistanceTravelledMeter, 0, "ROTS", DistanceLoopCount, 0, "STEER", adc_steer_average, 0);
      break;

    case 3:
      displaySingleRow("Inst", current_inst, 1, 0);
      displaySingleRow("Avg", current_average, 1, 1);
      displaySingleRow("Peak", current_peak, 1, 2);
      displaySingleRow("Volt", vbatt_reading, 1, 3);
      displaySingleRow("Temp", temp_average, 1, 4);
      break;

    case 4:
      displaySingleRow("ADC0", adc_volts[0], 2, 0);
      displaySingleRow("ADC1", adc_volts[1], 2, 1);
      displaySingleRow("ADC2", adc_volts[2], 2, 2);
      displaySingleRow("ADC3", adc_volts[3], 2, 3);
      displaySingleRow("DACl", dac_throttle_left, 0, 4);
      displaySingleRow("DACr", dac_throttle_right, 0, 5);
      break;

    case 5:
      displaySingleRow("Current ", current_inst, 2, 0);
      displaySingleRow("Steer   ", adc_steer_average, 2, 1);
      displaySingleRow("Temp    ", temp_average, 2, 2);
      displaySingleRow("Rots    ", DistanceLoopCount, 2, 3);

      displaySingleRow("Throttle", adc_throttle_average, 2, 4);

      displaySingleRow("DAC L   ", dac_throttle_left, 2, 5);
      displaySingleRow("DAC R   ", dac_throttle_right, 2, 6);

      displaySingleRow("Throttle I", throttle_index, 2, 7);
      displaySingleRow("Steer I", steering_index, 2, 8);

      displaySingleRow("Diff mux", diff_multiplier, 2, 9);

displaySingleRow("Monitor", vbatt_reading, 2, 10);

      break;

    case 6:
      displaySingleRow("X Accel ", xAccl, 2, 0);
      displaySingleRow("Y Accel ", yAccl, 2, 1);
      displaySingleRow("Z Accel ", zAccl, 2, 2);

      displaySingleRow("X Gyro  ", xGyro, 2, 4);
      displaySingleRow("Y Gyro  ", yGyro, 2, 5);
      displaySingleRow("Z Gyro  ", zGyro, 2, 6);

      break;
  }

  displayDelay.repeat();  // Start the timer again without drift
}

void updateLED() {
  if (temp_inst > current_warning_lower) {
    current_led_status = current_led_status ^ 1;
    digitalWrite(ledTempPin, current_led_status);  // Switch LEDs on to indicate startup of monitor
  }
  if (current_inst > current_warning_higher) {
    digitalWrite(ledTempPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  }
  if (current_inst < current_warning_lower) {
    digitalWrite(ledTempPin, LOW);  // Switch LEDs on to indicate startup of monitor
  }
  if (temp_inst > temperature_warning_lower) {
    temp_led_status = temp_led_status ^ 1;
    digitalWrite(ledCurrentPin, temp_led_status);  // Switch LEDs on to indicate startup of monitor
  }
  if (temp_inst > temperature_warning_higher) {
    digitalWrite(ledCurrentPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  }
  if (temp_inst < temperature_warning_lower) {
    digitalWrite(ledCurrentPin, LOW);  // Switch LEDs on to indicate startup of monitor
  }

  ledDelay.repeat();  // Start the timer again without drift
}


void readGyro() {
  mpu.update();
  xAccl = mpu.getAccX();
  yAccl = mpu.getAccY();
  zAccl = mpu.getAccZ();

  xGyro = mpu.getGyroX();
  yGyro = mpu.getGyroY();
  zGyro = mpu.getGyroZ();


#ifdef debug_accel
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
#endif

}


void calculate_ground_speed() {
  unsigned long SpeedTimeDelta = 0;

  if (hall_rising == 1) {
    elapsedMicros = irqMicros - startMicros;
    startMicros = irqMicros;
    hall_rising = 2;
    irqWheelTimeStart = micros();

  } else if (hall_rising == 2) {
    SpeedTimeDelta = micros() - irqWheelTimeStart;
    if (SpeedTimeDelta >= irqWheelTimeWait) {
      hall_rising = 0;
    }
    DistanceTravelled = (DistanceLoopCount * wheel_circumference_mm);
    DistanceTravelledMeter = DistanceTravelled / 1000;
    DistanceLoopCount++;
  }

  if (elapsedMicros != 0) {
    speed_current = (wheel_circumference * 0.0036) / elapsedMicros;
    speed_previous = speed_current;

    //. here i added a coefficient that means i'm converting to km/h so it's micrometers *3600 / 10^6 )
  }

  if ((micros() - irqMicros) > 2000000) {
    speed_current = 0;
  }

#ifdef debug_speed
  Serial.println(" dist: " + String(DistanceTravelled) + " km/h: " + String(speed_current));  // this now shows mm/sec with no remainder
  Serial.println(" rots: " + String(DistanceLoopCount) + " Delta: " + String(SpeedTimeDelta) + " Elasped: " + String(elapsedMicros));
  Serial.println("----------");
#endif
}

void initialise_adc_arrays() {
  // Initialize adc array using first reading. This means average normalizes quicker

  for (int thisReading = 0; thisReading < AvADCReadings; thisReading++) {
    adc_total[0] += adc_raw[0];
    adc_total[1] += adc_raw[1];
    adc_total[2] += adc_raw[2];
    adc_total[3] += adc_raw[3];

    adc_Readings[0][thisReading] = adc_raw[0];
    adc_Readings[1][thisReading] = adc_raw[1];
    adc_Readings[2][thisReading] = adc_raw[2];
    adc_Readings[3][thisReading] = adc_raw[3];
  }
}

void initialise_variables() {
  speed_current = 0;
  speed_previous = 0;
  speed_average = 0;
  speed_peak = 0;
  current_inst = 0;
  current_peak = 0;
  current_average = 0;
  current_fuse_loss = 0;

  adc_current_average = 0;
  adc_steer_average = 0;
  adc_tcouple_average = 0;
  adc_throttle_average = 0;

  temp_inst = 0;
  temp_peak = 0;
  temp_average = 0;
  temp_fuse_loss = 0;

  DistanceTravelled = 0;
  DistanceTravelledMeter = 0;
  DistanceLoopCount = 0;
}

void read_adc_values() {

  for (int adcChannel = 0; adcChannel < 4; adcChannel++) {
    adc_raw[adcChannel] = ads.readADC_SingleEnded(adcChannel);  // Remove last, now stale, reading
    if (adc_raw[adcChannel] < 0) {
      adc_raw[adcChannel] = 0;
    }

    adc_volts[adcChannel] = ads.computeVolts(adc_raw[adcChannel]);
    adc_total[adcChannel] = adc_total[adcChannel] - adc_Readings[adcChannel][adc_index];
    adc_Readings[adcChannel][adc_index] = adc_raw[adcChannel];
    adc_total[adcChannel] = adc_total[adcChannel] + adc_Readings[adcChannel][adc_index];  // Update last reading with current adc value
    adc_average[adcChannel] = adc_total[adcChannel] / AvADCReadings;

    adc_volts[adcChannel] = ads.computeVolts(adc_average[adcChannel]);
  }

  adc_throttle_average = adc_average[ADC_throttle];
  adc_current_average = adc_average[ADC_current_sensor];
  adc_steer_average = adc_average[ADC_steer];
  adc_tcouple_average = adc_average[ADC_temp_sensor];

  adc_index += 1;
  if (adc_index >= AvADCReadings) {
    adc_index = 0;
  }


  //Serial.println("ADC 3: " + String(ads.readADC_SingleEnded(3)) + " " + adc_raw[3]);

#ifdef debug_hi
  Serial.println("ADC 0: " + String(adc_raw[0]) + ", ADC 1: " + String(adc_raw[1]) + ", ADC 2: " + String(adc_raw[2]) + ", ADC 3: " + String(adc_raw[3]));
  Serial.println("ADCV 0: " + String(adc_volts[0]) + ", ADC 1: " + String(adc_volts[1]) + ", ADC 2: " + String(adc_volts[2]) + ", ADC 3: " + String(adc_volts[3]));
  // Serial.println("ADC Av0: " + String(adc_average[0]) + ", Av1: " + String(adc_average[1]) + ", Av2: " + String(adc_average[2]) + ", Av3: " + String(adc_average[3]));
  // Serial.println("DAC Left: " + String(dac_throttle_left) + ", DAC Right: " + String(dac_throttle_right));
  // Serial.println("==============================");
#endif
}

void check_vbatt_monitor() {

voltage_monitor = digitalRead(PIN_vdiff);

  if ((voltage_monitor == 0) && (voltage_monitor_state == 0)) {  // record values when fuse lost
    current_fuse_loss = current_inst;
    temp_fuse_loss = temp_inst;
    voltage_monitor_state = 1;
    encoderCounter = 1;
  }
#ifdef debug_batt
  Serial.println("VOLTAGE MONITOR STATE: " + String(voltage_monitor_state) + " Volt Pin: " + String(voltage_monitor));
  Serial.println("==============================");
#endif

}

void updateEncoder() {
  // Read the current state of CLK
  currentStateCLK = digitalRead(encoderCLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(encoderDT) != currentStateCLK) {
      encoderCounter--;
      counterChange = true;
    } else {
      // Encoder is rotating CW so increment
      encoderCounter++;
      counterChange = true;
    }

    if (encoderCounter < 0) {
      encoderCounter = EncoderCounterMax;
    }
    if (encoderCounter > EncoderCounterMax) {
      encoderCounter = 0;
    }
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
};


void setZeroCurrent() {
  int iteration = 10;
  long current_accumulator = 0;

  for (int i = 0; i < iteration; i++) {
    current_accumulator += ads.readADC_SingleEnded(ADC_current_sensor);
    delay(READ_DELAY);
  }
  current_zero = current_accumulator / iteration;  // Use this for calcs as quicker and less error than float
  current_inst = current_zero;
}

void setZeroAccel() {
  mpu.begin();
  //mpu.calcGyroOffsets(true);
    mpu.calcOffsets(true,true); // gyro and accelero
}

void convert_thermistor() {
  float ThermRes, logR2;
  // ThermRes = SERIESRESISTOR * ((4.78 / adc_volts[ADC_temp_sensor]) - 1);
  ThermRes = SERIESRESISTOR * ((3.25 / adc_volts[ADC_temp_sensor]) - 1);

  logR2 = log(ThermRes);
  temp_inst = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273;
  temp_average = temp_inst;

#ifdef debug_therm
  Serial.println("Temp V: " + String(adc_tcouple_average) + "Temp RES: " + String(ThermRes) + ", Log: " + String(logR2) + ", Temp Inst: " + String(temp_inst));
  Serial.println("-------------------------------------------------------");
#endif
}


void throttle_management() {

  // ADC1115 A/D (65535) on 2/3x gain +/- 6.144V, 1 bit = 0.1875mV.
  // Range 0 - 5V, 0 - 26666 (5v / 0.1875 mV).
  // Throttle 0.84 - 4.2V. 4483 - 22900
  // DAC 0 - 4095.  5V equivalent = 4095
  // Conversion = 4095 / 26666 = 0.1536

  // If the control is straight through simply send raw value to DAC

  throttle_mapping_case = (digitalRead(PIN_Throttle_Control));  //True = Left

  dac_throttle = adc_throttle_average;

  dac_throttle = (dac_throttle > throttleLower) ? dac_throttle : throttleLower;  // Check for over and under thresholds
  dac_throttle = (dac_throttle < throttleUpper) ? dac_throttle : throttleUpper;

  throttle_index = (dac_throttle - throttleLower) / throttlemultiplier;
  throttle_index = (throttle_index < 9) ? throttle_index : 9;

  if (throttle_mapping_case) {  // Left
    diff_multiplier_left = throttle_map_2[throttle_index][0];
    diff_multiplier_right = throttle_map_2[throttle_index][1];
  } else {
    diff_multiplier_left = throttle_map_2[throttle_index][1];
    diff_multiplier_right = throttle_map_2[throttle_index][0];
  }

#ifdef turbo_button
  if ((turbo_boost_state == 2) && (temp_inst < temperature_warning_lower)) {
    diff_multiplier_left = 100;
    diff_multiplier_right = 100;
  }
#endif

  diff_offset = dac_throttle - throttleLower;

  diff_offset_left = (diff_multiplier_left * diff_offset) / 100;
  diff_offset_right = (diff_multiplier_right * diff_offset) / 100;

  dac_throttle_left = throttleLower + diff_offset_left;
  dac_throttle_right = throttleLower + diff_offset_right;

  dac_throttle_left_short = dac_throttle_left * 0.1536;    // move from 16bit to 12bit
  dac_throttle_right_short = dac_throttle_right * 0.1536;  // move from 16bit to 12bit

#ifdef debug_diff
  Serial.println("CASE: " + String(throttle_mapping_case) + " TBO: " + String(turbo_boost_state) + " TAC: " + String(dac_throttle) + " IDX: " + String(throttle_index) + ", DML: " + String(diff_multiplier_left) + ", DMR: " + String(diff_multiplier_right) + " D_OFF: " + String(diff_offset) + " DAC L: " + String(dac_throttle_left) + " DAC R: " + String(dac_throttle_right) + " DL: " + String(dac_throttle_left_short) + ", DR: " + String(dac_throttle_right_short) + " DfL: " + String(diff_offset_left) + " DfR: " + String(diff_offset_right));

// Serial.println("Throttle: " + String(adc_throttle_average) + " Steering: " + String(adc_steer_average));
#endif

  mcp.setChannelValue(DAC_throttle_left, dac_throttle_left_short);    // left
  mcp.setChannelValue(DAC_throttle_right, dac_throttle_right_short);  // right
  mcp.saveToEEPROM();
}

void display_Banner() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.drawString(monitor_version, 0, 0, 4);
  tft.drawString("Monitor", 0, 30, 4);
  tft.drawString("Reading Zero", 0, 80, 4);

  if (digitalRead(PIN_Throttle_Control)) {
    tft.drawString("Left", 0, 160, 4);
  } else {
    tft.drawString("Right", 0, 160, 4);
  }
}

void MainScreen() {
  char stringBuffer1[10] = { 0 };
  char stringBuffer2[10] = { 0 };
  char stringBuffer3[10] = { 0 };
  char stringBuffer4[10] = { 0 };
  int text_colour = TFT_WHITE;

  dtostrf(current_inst, 6, 1, stringBuffer1);
  dtostrf(current_peak, 6, 1, stringBuffer2);
  dtostrf(speed_current, 6, 1, stringBuffer3);
  dtostrf(temp_average, 6, 1, stringBuffer4);

  //tft.fillScreen(TFT_BLACK);

  //240RGBx320
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0, 4);
  tft.print("Amps Inst");
  tft.setCursor(SCREEN_X_HALF, 0, 4);
  tft.print("Amps Peak");
  tft.setCursor(0, SCREEN_Y_HALF, 4);
  tft.print("Speed KmH");
  tft.setCursor(SCREEN_X_HALF, SCREEN_Y_HALF, 4);
  tft.print("Temp *C");

  tft.fillRect(0, 40, SCREEN_X, 75, TFT_BLACK);
  tft.fillRect(0, 160, SCREEN_X, SCREEN_Y, TFT_BLACK);

  tft.setTextColor(TFT_WHITE);
  if (current_inst > current_warning_lower) {
    text_colour = TFT_ORANGE;
  }
  if (current_inst > current_warning_higher) {
    text_colour = TFT_RED;
  }

  tft.setCursor(0, 40, 7);
  tft.print(stringBuffer1);
  tft.setCursor(SCREEN_X_HALF, 40, 7);

  tft.setTextColor(TFT_WHITE);
  tft.print(stringBuffer2);
  tft.setCursor(0, 160, 7);
  tft.print(stringBuffer3);
  tft.setCursor(SCREEN_X_HALF, 160, 7);

  tft.setTextColor(TFT_WHITE);
  if (temp_average > temperature_warning_lower) {
    text_colour = TFT_ORANGE;
  }
  if (temp_average > temperature_warning_higher) {
    text_colour = TFT_RED;
  }
  tft.print(stringBuffer4);
}

void displayLabel(const char displayLabel[], const uint8_t disp_row) {
  char stringLabelBuffer[12] = { 0 };

  strncpy(stringLabelBuffer, displayLabel, 10);
  tft.setTextSize(2);
  tft.setCursor(0, disp_row, 1);
  tft.print(displayLabel);
}

int calcBarPosition(uint8_t coordPercentage) {
  const int max_bar_width = BAR_WIDTH;  //adjusted for percentage
  uint16_t x_bar_calc = 0;
  x_bar_calc = round(coordPercentage * max_bar_width / 100);
  if (x_bar_calc < 0) {
    x_bar_calc = 0;
  }
  return x_bar_calc;
}

void drawBar(int Bar_Number) {

  // bar_readings[0][0] = 50;     // new
  // bar_readings[0][1] = 50;     // previous
  // bar_readings[0][2] = 24800;  // max adc

  int Bar_Colour = TFT_WHITE;

  uint16_t bar_width_start = 0;
  uint16_t bar_width_end = 0;

  /*
  if (nPer > 75) {
    Bar_Colour = TFT_RED;
  }
  */

  if (bar_readings[Bar_Number][0] > bar_readings[Bar_Number][4]) {
    Bar_Colour = bar_readings[Bar_Number][5];
  }
  if (bar_readings[Bar_Number][0] > bar_readings[Bar_Number][6]) {
    Bar_Colour = bar_readings[Bar_Number][7];
  }

  if (bar_readings[Bar_Number][0] < bar_readings[Bar_Number][1]) {  // if new < previous
    bar_width_start = calcBarPosition(bar_readings[Bar_Number][0]);
    bar_width_end = calcBarPosition(bar_readings[Bar_Number][1] - bar_readings[Bar_Number][0] + 1);
    tft.fillRect(20 + bar_width_start, bar_position[Bar_Number], bar_width_end, 30, TFT_BLACK);
  } else {
    bar_width_start = calcBarPosition(bar_readings[Bar_Number][1] - 1);
    bar_width_end = calcBarPosition(bar_readings[Bar_Number][0] - bar_readings[Bar_Number][1] + 1);
    tft.fillRect(20 + bar_width_start, bar_position[Bar_Number], bar_width_end, 30, Bar_Colour);
  }

  //Serial.println("Start: " + String(bar_width_start) + ", End: " + String(bar_width_end) + " , Last Per " + String(bar_readings[Bar_Number][1]) + " , New Per " + String(bar_readings[Bar_Number][0]) + " , ADC " + String(bar_readings[2][3]));
  bar_readings[Bar_Number][1] = bar_readings[Bar_Number][0];
}

void displayTwoColumns(const char displayLabel1[], const float displayValue1, const uint8_t dc_value1, const char displayLabel2[], const float displayValue2, const uint8_t dc_value2) {
  char floatbuf1[16] = { 0 };
  char floatbuf2[16] = { 0 };
  /*
  display.set1X();
  display.setFont(System5x7);
  display.setCursor(0, 0);
  display.print(displayLabel1);
  display.setCursor(64, 0);
  display.print(displayLabel2);
  display.set2X();

  display.setCursor(0, 3);
  dtostrf(displayValue1, 5, dc_value1, floatbuf1);
  display.print(floatbuf1);
  display.setCursor(64, 3);
  dtostrf(displayValue2, 5, dc_value2, floatbuf2);
  display.print(floatbuf2);
  */
}

void displaySingleRow(const char displayLabel1[], const float displayValue1, const uint8_t dc_value1, const uint8_t disp_row) {
  char stringBuffer1[10] = { 0 };
  char stringBuffer2[10] = { 0 };
  uint8_t temp_row = 0;
  temp_row = disp_row * 22;
  strncpy(stringBuffer1, displayLabel1, 6);
  dtostrf(displayValue1, 6, dc_value1, stringBuffer2);

  if (disp_row == 0) {
    tft.fillScreen(TFT_BLACK);
    counterChange = false;
  }

  tft.setTextSize(2);
  tft.setCursor(0, temp_row, 1);
  tft.print(displayLabel1);
  tft.setCursor(120, temp_row, 1);
  tft.print(stringBuffer2);
}


void displayTwoRows(char* displayLabel1, float displayValue1, uint8_t dc_value1, char* displayLabel2, float displayValue2, uint8_t dc_value2) {
  char stringBuffer1[10] = { 0 };
  char stringBuffer2[10] = { 0 };
  char stringBuffer3[10] = { 0 };
  char stringBuffer4[10] = { 0 };

  strncpy(stringBuffer1, displayLabel1, 6);
  strncpy(stringBuffer2, displayLabel2, 6);

  dtostrf(displayValue1, 6, dc_value1, stringBuffer3);
  dtostrf(displayValue2, 6, dc_value2, stringBuffer4);

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);

  /*
  if (counterChange == true) {
    tft.fillScreen(TFT_BLACK);
    counterChange = false;

    Serial.println("Clear");
*/
  tft.setCursor(0, 0, 4);
  tft.print(stringBuffer1);
  tft.setCursor(0, 65, 4);
  tft.print(stringBuffer2);

  tft.setCursor(80, 0, 7);
  tft.print(stringBuffer3);

  tft.setCursor(80, 65, 7);
  tft.print(stringBuffer4);
}


void displayThreeRows(char* displayLabel1, float displayValue1, uint8_t dc_value1, char* displayLabel2, float displayValue2, uint8_t dc_value2, char* displayLabel3, float displayValue3, uint8_t dc_value3) {
  char stringBuffer1[10] = { 0 };
  char stringBuffer2[10] = { 0 };
  char stringBuffer3[10] = { 0 };
  char stringBuffer4[10] = { 0 };
  char stringBuffer5[10] = { 0 };
  char stringBuffer6[10] = { 0 };

  strncpy(stringBuffer1, displayLabel1, 6);
  strncpy(stringBuffer2, displayLabel2, 6);
  strncpy(stringBuffer3, displayLabel3, 6);

  dtostrf(displayValue1, 6, dc_value1, stringBuffer4);
  dtostrf(displayValue2, 6, dc_value2, stringBuffer5);
  dtostrf(displayValue3, 6, dc_value2, stringBuffer6);

  tft.fillRect(120, 0, SCREEN_X, SCREEN_Y, TFT_BLACK);
  tft.setTextSize(1);

  tft.setCursor(0, 0, 4);
  tft.print(stringBuffer1);
  tft.setCursor(0, 90, 4);
  tft.print(stringBuffer2);
  tft.setCursor(0, 180, 4);
  tft.print(stringBuffer3);

  tft.setCursor(120, 0, 7);
  tft.print(stringBuffer4);
  tft.setCursor(120, 90, 7);
  tft.print(stringBuffer5);
  tft.setCursor(120, 180, 7);
  tft.print(stringBuffer6);
}


void displayFourRows(char* displayLabel1, float displayValue1, uint8_t dc_value1, char* displayLabel2, float displayValue2, uint8_t dc_value2, char* displayLabel3, float displayValue3, uint8_t dc_value3, char* displayLabel4, float displayValue4, uint8_t dc_value4) {
  char stringBuffer1[10] = { 0 };
  char stringBuffer2[10] = { 0 };
  char stringBuffer3[10] = { 0 };
  char stringBuffer4[10] = { 0 };
  char stringBuffer5[10] = { 0 };
  char stringBuffer6[10] = { 0 };
  char stringBuffer7[10] = { 0 };
  char stringBuffer8[10] = { 0 };

  strncpy(stringBuffer1, displayLabel1, 8);
  strncpy(stringBuffer2, displayLabel2, 8);
  strncpy(stringBuffer3, displayLabel3, 8);
  strncpy(stringBuffer4, displayLabel4, 8);

  dtostrf(displayValue1, 6, dc_value1, stringBuffer5);
  dtostrf(displayValue2, 6, dc_value2, stringBuffer6);
  dtostrf(displayValue3, 6, dc_value2, stringBuffer7);
  dtostrf(displayValue4, 6, dc_value2, stringBuffer8);

  tft.setTextSize(1);

  tft.setCursor(0, 0, 4);
  tft.print(stringBuffer1);
  tft.setCursor(SCREEN_X_HALF - 10, 0, 4);
  tft.print(stringBuffer2);
  tft.setCursor(0, 120, 4);
  tft.print(stringBuffer3);
  tft.setCursor(SCREEN_X_HALF - 10, 120, 4);
  tft.print(stringBuffer4);

  tft.fillRect(0, 40, SCREEN_X, 75, TFT_BLACK);
  tft.fillRect(0, 160, SCREEN_X, SCREEN_Y, TFT_BLACK);

  tft.setCursor(0, 40, 7);
  tft.print(stringBuffer5);
  tft.setCursor(SCREEN_X_HALF - 10, 40, 7);
  tft.print(stringBuffer6);
  tft.setCursor(0, 160, 7);
  tft.print(stringBuffer7);
  tft.setCursor(SCREEN_X_HALF - 10, 160, 7);
  tft.print(stringBuffer8);
}
