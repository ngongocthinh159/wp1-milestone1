#include <Arduino.h>
#include <Servo.h>
#include <HX711_ADC.h>
#include <ACS712.h>
#include <voltage_sensor.h>

// PIN definitions
#define PotPin 0
#define CURRENT_DT_PIN 2
#define VOLTAGE_DT_PIN 3
#define HX711_dout 7
#define HX711_sck 8
#define ESC_signal_pin 9

// Constructor
Servo ESC; // ESC object constructor
HX711_ADC LoadCell(HX711_dout, HX711_sck); // HX711 constructor
ACS712 sensor(ACS712_05B, CURRENT_DT_PIN); // ASC712 current sensor constructor

// Constant
const int MAX_P_WIDTH = 2000;
const int MIN_P_WIDTH = 1000;
const float CALIBRATION_FACTOR = 430.56;
const float MAX_MOTOR_POWER_CONSUMPTION = 150; // Corresponding to 150 (W), according to the data sheet of A2212/13T

// Global variables
const int serialPrintInterval = 40; // Increase value to slow down serial print activity
float motorUsage = 0; // Percentage of motor usage
unsigned long t = 0; // Time from start the microcontroller
int potValue = 0;  // Value from the analog pin of potentiometer
float currentValue = 0; // Value of current in amp (A)
float voltageValue = 0; // Value of voltage in vol (V)
float powerValue = 0;
float loadCellValue = 0; // Value of load cell (g)
static int tt = 0; // Time variable for serial update interval

// Function declaration
void initializeLoadcell();
float getCurrentValue();
void waitUntilReceive(char c);
char readSerial();

void setup() {
  Serial.begin(9600);
  analogReadResolution(12); // Only for microcontroller with 12bit ADC
  delay(2000);
  while(!Serial) {};

  Serial.println("");
  Serial.println("----- Welcome to the program -----");
  Serial.println("===> Disconnect power source, make source no current flow through the circuit");
  Serial.println("===> Send 's' to the serial...");
  Serial.println("");
  waitUntilReceive('s');

  // Calibrate voltage sensor
  Serial.println("Calibrate the voltage sensor...");
  calibrateVoltageSensor(); // Calibrate voltage sensor in this function
  Serial.println("DONE calibrate Voltage sensor!");
  Serial.println("");

  // Calibrate current sensor
  Serial.println("Calibrate the current sensor...");
  int zero = sensor.calibrate(); // Calibrate current sensor in this function
  Serial.println("DONE calibrate current sensor!");
  Serial.println("");

  // Loadcell tare and initialize
  Serial.println("Calibrate the loadcell...");
  Serial.println("Tare the loadcell...");
  initializeLoadcell(); // Tare loadcell and set known calibration factor
  Serial.printf("Tare done, the loadcell use the known calibration factor: %.2f\n", CALIBRATION_FACTOR);
  Serial.println("");
  
  // ESC calibration
  Serial.println("===> Rotate the potentiometer to zero position (rotate counter clockwise until can not)");
  Serial.println("===> Turn on power source, then send 's' to the serial");
  waitUntilReceive('s');
  Serial.println("Calibrate the ESC...");
  ESC.attach(ESC_signal_pin); // Attach the ESC on pin 6
  ESC.writeMicroseconds(MAX_PULSE_WIDTH); // Write new HIGH value for the motor
  delay(100);
  ESC.writeMicroseconds(MIN_PULSE_WIDTH); // Write LOW value for the motor
  Serial.println("Done calibrate the ESC!");
  Serial.println("");

  // Wait signal to start the motor
  Serial.println("===> Send 's' to the serial and now rotate the potentiometer to control the motor!");
  waitUntilReceive('s');
}

void loop() {
  char inByte = readSerial();
  static boolean newDataReady = 0;
  
  // Check for load cell data
  if (LoadCell.update()) newDataReady = true;
  
  // Change the speed of the motor with the potentiometer value
  potValue = analogRead(PotPin);   // reads the value of the potentiometer (value between 0 and 4095)
  potValue = map(potValue, 0, 4095, MIN_P_WIDTH, MAX_P_WIDTH);   // scale it to use it with the servo library (value between 0 and 180)
  ESC.writeMicroseconds(potValue);    // Send the signal to the ESC

  // Print values to Serial
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      // Get loadcell value from HX711 sensor
      loadCellValue = LoadCell.getData();
      newDataReady = 0;
      t = millis();
    }
  }

  if (millis() > (unsigned int)tt + serialPrintInterval) {
    // Get voltage value from voltage sensor
    voltageValue = getVoltageValue();

    // Get current value from current sensor
    currentValue = getCurrentValue();

    // Power value
    powerValue = voltageValue*currentValue;
    
    // Motor usage percentage
    motorUsage = (float) (powerValue/MAX_MOTOR_POWER_CONSUMPTION*100);

    // Print values
    Serial.printf("Loadcell: %.2f g\n", loadCellValue);
    Serial.printf("Motor usage: %.2f%%\n", motorUsage);
    Serial.printf("Current: %.2f A\n", currentValue);
    Serial.printf("Voltage: %.2f V\n", voltageValue);
    Serial.printf("Power: %.2f W\n", powerValue);
    Serial.printf("Time: %d\n", millis());
    Serial.printf("\r\n");

    tt = millis();
  }
}

// Function definitions
void initializeLoadcell() {
  LoadCell.begin();
  LoadCell.setReverseOutput(); // Comment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // Preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // Set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(CALIBRATION_FACTOR);
  }
}

float getCurrentValue() {
  float sum_I = 0, avg_I;
  for (int i = 0; i < 50; i++) {
    float I = sensor.getCurrentDC();
    sum_I += I;
  }
  avg_I = sum_I / 50;
  return avg_I;
}

void waitUntilReceive(char c) {
  bool _resume = false;
  while (!_resume) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
        if (inByte == c) {
          _resume = true;
        }
    }
  }
}

char readSerial() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
}