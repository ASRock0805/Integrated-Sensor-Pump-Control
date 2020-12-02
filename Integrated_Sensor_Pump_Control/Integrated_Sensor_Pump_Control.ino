/*
  Integrated Pressure/Flow Sensor and Pump Control Using Serial Port (USB CDC)

      The program integrates two sensors and pumps, namely a water pressure sensor and a flow sensor.
  The flow rate and pressure can be read through a personal computer, and the pump speed percentage (PWM)
  can be set through the serial port.

  - Water Pressure Sensor (DFRobot SEN0257)
      The output voltage of the sensor is converted by ADC to obtain the water pressure.

  - Compact Electromagnetic Flow Sensor (Aichi Tokei VN05)
      Use interrupts to calculate the number of unit pulses per interval to convert the water flow rate (mL/minute).

  - Kamoer KDS Peristaltic Pump (KDS-FE-2-S17B)
      12V DC brush motor

  Created 19 Oct. 2020
  by Yi-Xuan Wang

  References:
  https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257
  https://www.aichitokei.net/products/compact-electromagnetic-flow-sensor-vn/
  http://kamoer.com/Products/showproduct.php?id=234
*/

/*--- Preprocessor ---*/
#define pumpPin 5 // The pin location of the pump (~D5) w/ PWM
#define flowPin 3 // Pin location of the sensor (D3) w/ interrupt (INT.1)
#define sigPin A5 // Potentiometer signal pin w/ ADC

#define N 800     // Measurment sampling number for smoothing

/*--- Constants ---*/
const unsigned long baudSpeed = 115200;           // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000;                // The value is a number of milliseconds

const byte vIn = 5;                                   // Supply voltage from Arduino
const byte resBits = 10;                              // Resolution of ADC (10 bits)
const float vConv = vIn / (pow(2.0, resBits) - 1.0);  // Voltage of ADC level (2^bits)

// Spec. of water pressure sensor, Range: 0 - 16 MPa, Output: 0.5 - 4.5 V
const float pgMax = 16.0;             // Upper limit of pressure sensor
const float pgMin = 0.0;              // Lower limit of pressure sensor
const float pgVmax = 4.5;             // Maximum output voltage of pressure sensor
const float pgVmin = 0.5;             // Minimum output voltage of pressure sensor
const float offSet = 0.471772766113;  // Norminal value is 0.5 V


/*--- Global Variables ---*/
unsigned long startTime;            // Start time
unsigned long currentTime;          // Current time
unsigned long timer;                // Stopwatch

byte pumpPct;                       // Percentage of pump PWM

float vOut;                         // Output of the ADC
float waterPres;                    // Value of water pressure

volatile unsigned long pulseCount;  // Measuring the falling edges of the signal
static unsigned long cumCount;      // Cumulative count
float flowRate;                     // Value of water flow rate
float flowML;                       // Unit converter (milliliter, mL)
float totalML;                      // Volume of cumulative water

/*--- Function Prototype ---*/
void getCounter(void);
float getwaterPres(float );
void waterPressure(byte );
void setPump(byte , byte );
byte pumpOn(void);
byte pumpOff(void);
void serialEvent(void);
void setup(void);
void loop(void);

/*--- Functions Definition ---*/
void getCounter(void) {         // Interrupt Service Routine (ISR) for Flow Sensor
  pulseCount = pulseCount + 1;  // Every falling edge of the sensor signals to increment the pulse count
}

// Implementation of Water Pressure Calculation
float getwaterPres(float volt) {
  return ((volt - offSet) * ((pgMax - pgMin) / (pgVmax - pgVmin))) + pgMin;
}

// Water Pressure Sensor
void waterPressure(byte signalPin) {
  for (unsigned int i = 0; i < N; ++i) {    // Get samples for smooth the value
    vOut = vOut + analogRead(signalPin);
    delay(1);                               // delay in between reads for stability
  }
  vOut = (vOut * vConv) / N;                // ADC of voltage meter output voltage

  waterPres = getwaterPres(vOut);           // Calculate water pressure

  if (isinf(waterPres) || isnan(waterPres)) {
    waterPres = -1;
  }
}

// Set the pump speed percentage
void setPump(byte pwmPin, byte pwmPct) {
  analogWrite(pwmPin, (pwmPct * 2.55));
}

// Full speed
byte pumpOn(void) {
  pumpPct = 100;

  return pumpPct;
}

// Stop the pump
byte pumpOff(void) {
  pumpPct = 0;

  return pumpPct;
}

void serialEvent(void) {  // Set the pump speed percentage through serial input
  int percent;

  if (Serial.available()) {
    percent = Serial.parseInt();
    Serial.print("Pump PWM Set to: ");
    Serial.println(percent);

    percent = constrain(percent, 0, 100); // Limits the percentage of pump speed to between 0 - 100
    if ((percent > pumpPct) && (percent != 0)) {
      pumpPct = pumpPct + percent;
    } if ((percent < pumpPct) && (percent >= 50)) {
      pumpPct = pumpPct + (percent - pumpPct);
    } else if ((percent == 0) || (percent < 50)) {
      pumpOff();
    }
    // Flush the receive buffer
    Serial.flush(); 
    while (Serial.read() >= 0) { }
  } else {
    return;
  }
}

/*--- Initialization ---*/
void setup(void) {
  Serial.begin(baudSpeed);          // Initializes serial port
  pinMode(pumpPin, OUTPUT);         // Initializes pump pin
  pinMode(sigPin, INPUT);           // Initializes potentiometer pin
  pinMode(flowPin, INPUT_PULLUP);   // Initializes interrupt digital pin 2 declared as an input and pull-up resitor enabled
  startTime = millis();             // Initial start time

  // Pump Percentage Initialization
  pumpPct = 0;

  // Water Pressure Sensor Initialization
  vOut = 0.0;
  waterPres = 0.0;

  // Flow Sensor Initialization
  pulseCount = 0;
  cumCount = 0;
  flowRate = 0.0;
  flowML = 0.0;
  attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // The interrupt is attached
}

/*--- Measurement ---*/
void loop(void) {
  setPump(pumpPin, pumpPct);

  // Every second, calculate and print the measured value
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    timer = startTime / period;               // Calculate the period of time

    // Water Pressure Sensor
    waterPressure(sigPin);

    // Flow Sensor
    detachInterrupt(digitalPinToInterrupt(flowPin));  // Clears the function used to attend a specific interrupt
    cumCount = cumCount + pulseCount;                 // Count increment
    // Estimated Volume: 0.5004 ml/Pulse
    flowRate = abs(((-7.0 * pow(10.0, -18.0)) * sq(pulseCount)) + (0.5004 * pulseCount) - (8.0 * pow(10.0, -12.0)));
    flowML = flowRate * 60.0;                         // Milliliter per pulse converter to milliliter per minute

    if (isinf(flowML) || isnan(flowML) || (flowML <= 0.0)) {
      flowML = -1;
    }

    /*--- Sensor prompt ---*/
    Serial.print(pumpPct);

    Serial.print(", Voltage: ");
    Serial.print(vOut, 12);
    Serial.print(" V, ");
    // Unit converter for pressure, raw unit: MPa
    Serial.print("Pressure: ");
    Serial.print(waterPres * 1000, 1);    // 1 : 1000
    Serial.print(" kPa, ");
    Serial.print(waterPres, 2);           // Raw
    Serial.print(" MPa, ");
    Serial.print(waterPres * 10.1972, 1); // 1 : 10.1972
    Serial.print(" kg/cm^2, "); 

    Serial.print("Cumulative Count: ");
    Serial.print(cumCount);
    Serial.print(", Pulse Count: ");
    Serial.print(pulseCount);
    Serial.print(", Flow Rate: ");
    Serial.print(flowML);
    Serial.println(" mL/minute, ");

    Serial.print(timer);
    Serial.println(" sec.");

    /*--- System Return ---*/
    startTime = currentTime;                                              // Save the start time of the current state
    pulseCount = 0;                                                       // Set pulseCount to 0 ready for calculations
    attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // Reattach interrupt
  } else {
    return;
  }
}
