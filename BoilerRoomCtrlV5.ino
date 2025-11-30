#include <PID_v1.h>
#include <avr/wdt.h>

// Pin definitions
#define PUMP_RELAY 2
#define VALVE_CLOSE 3
#define VALVE_OPEN 4
#define ERROR_LED 12
#define THERMOSTAT_SWITCH 11
#define ValveRuntime 30000 //ms for full way

// Analog input pins for PT100 sensors
#define AMBIENT_TEMP A0
#define TANK_TEMP A4
#define FLOW_TEMP A3
#define RETURN_TEMP A2
#define ROOM_TEMP A1

#define Ta1 -20 //[°C]        Points of heating curve
#define Ta2 -5 //[°C]
#define Ta3 10 //[°C]
#define Ta4 25 //[°C]
#define Tflr1 42 //[°C]
#define Tflr2 37 //[°C]
#define Tflr3 35 //[°C]
#define Tflr4 33 //[°C]


                              // PID parameters
double Setpoint, Input, Output;
double Kp = 5.0, Ki = 0.5, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  //DIRECT is control Logic here can be changed to REVERSE for cooling mode.
float flowWeight = 1.0;
float roomWeight = 0.0;
float weightedInput=20.0;


// System parameters


const unsigned long SENSOR_CHECK_INTERVAL = 5000; // 5 seconds


// Variables
float tankTemp, flowTemp, returnTemp, roomTemp, ambientTemp, dT1, dT2;
float dTratio;
float avgFlowReturnTemp;
bool systemActive = false;
bool errorState = false;
unsigned long lastSensorCheck = 0;
bool isPumpWorking=LOW;
//===========================================================================================================================================================7
void setup() {
  // Initialize watchdog timer
    wdt_disable();
    wdt_enable(WDTO_8S);
 
    // Initialize serial for debugging
  Serial.begin(9600);

  // Set pin modes
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(VALVE_OPEN, OUTPUT);
  pinMode(VALVE_CLOSE, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(THERMOSTAT_SWITCH, INPUT_PULLUP);

  // set  outputs
  digitalWrite(13, LOW); //because it was always on even if I did not used
  digitalWrite(PUMP_RELAY, HIGH);
  digitalWrite(VALVE_OPEN, HIGH);
  digitalWrite(VALVE_CLOSE, HIGH);
  digitalWrite(ERROR_LED, LOW);

  //test outputs 
    digitalWrite(PUMP_RELAY, LOW);
    watchdogDelay(2000);
    digitalWrite(PUMP_RELAY, HIGH);
    watchdogDelay(2000);
    digitalWrite(VALVE_OPEN, LOW);
    watchdogDelay(2000);
    digitalWrite(VALVE_OPEN, HIGH);
    watchdogDelay(2000); 
    digitalWrite(VALVE_CLOSE, LOW);
    watchdogDelay(2000);   
    digitalWrite(VALVE_CLOSE, HIGH);
    watchdogDelay(2000); 
    digitalWrite(ERROR_LED, HIGH);
    watchdogDelay(2000);
    digitalWrite(ERROR_LED, LOW);

Serial.println("Output test done, have you checked them???");

  
  // Initialize PID
  Setpoint = 35.0; // Initial target for flow modyfied with room temp.
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-70, 70); // Full range for 3-point control
  Serial.println("PID intialization done");
 
  //setting Valve to zero
  digitalWrite(VALVE_CLOSE, LOW);
   Serial.println("Valve setting to minumum......pls Wait");
  watchdogDelay(ValveRuntime);
  digitalWrite(VALVE_CLOSE, HIGH);
  Serial.println("Valve set to minumum");

  // set analog reference input.
  analogReference(EXTERNAL);
  



  Serial.println("Heating system initialized");


}
//=============================================================================================================================================================7
void loop() {
  // Reset watchdog timer
  wdt_reset();  
checkMemory();

  static unsigned long loopCount = 0;
    loopCount++;
    
    Serial.print("Loop count: ");
    Serial.println(loopCount);

    if (loopCount % 10 == 0) { // Print every 10 loops
        Serial.println("System still running...");
    }



  // Read sensors periodically
  if (millis() - lastSensorCheck > SENSOR_CHECK_INTERVAL) {
      // Read thermostat state
      watchdogDelay(500);
      systemActive = digitalRead(THERMOSTAT_SWITCH) == LOW;  
    readAllSensors();
    lastSensorCheck = millis();
  }
  
  // Check for errors
  checkForErrors();
  
  if (errorState) {
    // Error state - turn off pump and valve, flash error LED
    Serial.println("Errorstate active!!");
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    resetPID();
    digitalWrite(ERROR_LED, (millis() % 1000) < 500); // Blink LED
    return;
  }
  
  if (!systemActive) {
    // System inactive - turn off pump and valve
    Serial.println("System inactive, no heat demand");
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    digitalWrite(ERROR_LED, LOW);
    resetPID();
    watchdogDelay(5000);
    return;
  }
  
// Control pump (on if we need heat and we have heat in the tank)
  if (tankTemp<=avgFlowReturnTemp+1) {
    digitalWrite(PUMP_RELAY, HIGH );
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    digitalWrite(ERROR_LED, HIGH);
    isPumpWorking=LOW;
    resetPID();
    Serial.println("Heat tank is discharged, ");
    watchdogDelay(5000);
    return;
  }
    if (tankTemp>=avgFlowReturnTemp+3) {
    digitalWrite(PUMP_RELAY, LOW );
    Serial.println("Heating is on ");
    digitalWrite(ERROR_LED, LOW);
    isPumpWorking=HIGH;
    watchdogDelay(5000);
    }
  else{
  Serial.println("No change in pump state");
  watchdogDelay(5000);  
  }
  
  if(isPumpWorking==HIGH){
  // Update PID
  Input = weightedInput;
  myPID.Compute();
 // Control valve based on PID output
  controlValve(Output);
  }   
}
//=============================================================================================================================================7
void readAllSensors() {
  tankTemp = readPT100(TANK_TEMP, 100.0);
  flowTemp = readPT100(FLOW_TEMP, 100.5);
  returnTemp = readPT100(RETURN_TEMP, 100.5);
  roomTemp = readPT100(ROOM_TEMP, 99.5);
  ambientTemp = readPT100(AMBIENT_TEMP, 100.0);
    avgFlowReturnTemp = (flowTemp + returnTemp) / 2.0;
  dT1=tankTemp-returnTemp;
  dT2=flowTemp-returnTemp;
  if (dT2==0){dT2=0.1;}
  dTratio=abs(dT1/dT2);
  dTratio= constrain(dTratio,0.1,20);
  
  // Update setpoint based on heating curve (adjust coefficients as needed)
  Setpoint = calculateHeatingCurve();
    weightedInput=calculateWeightedInput();

  
  Serial.print("Tank: "); Serial.print(tankTemp);
  Serial.print(" Flow: "); Serial.print(flowTemp);
  Serial.print(" Return: "); Serial.print(returnTemp);
  Serial.print(" Room: "); Serial.print(roomTemp);
  Serial.print(" Ambient: "); Serial.print(ambientTemp);
  Serial.print(" Setpoint: "); Serial.print(Setpoint);
  Serial.print(" Weightedinput: "); Serial.println(weightedInput);
}

float readPT100(int pin, float fixresistorvalue) {
  watchdogDelay(500);
  
  // Simple median filter instead of mode - uses less memory
  const int numReadings = 5;
  int readings[numReadings];
  
  // Take readings
  for (int i = 0; i < numReadings; i++) {
    readings[i] = analogRead(pin);
    delay(50);
  }
  
  // Sort and get median (middle value)
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = i + 1; j < numReadings; j++) {
      if (readings[i] > readings[j]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  int rawfiltered = readings[numReadings / 2]; // Median value
  
  wdt_reset();
  float voltage = rawfiltered * (5.0 / 1023.0);
  float resistance = (voltage * fixresistorvalue) / (5.0 - voltage);
  float temp = (resistance - 100.0) / 0.385;
  
  return temp;
}

float calculateHeatingCurve() {
  // Simple heating curve calculation
  float targetTemp = 0;
  
  if (ambientTemp < Ta1) {
    targetTemp = Tflr1;
  }
  else if (ambientTemp < Ta2) {
    targetTemp = (ambientTemp - Ta1) * (Tflr2 - Tflr1) / (Ta2 - Ta1) + Tflr1;
  }
  else if (ambientTemp < Ta3) {
    targetTemp = (ambientTemp - Ta2) * (Tflr3 - Tflr2) / (Ta3 - Ta2) + Tflr2;
  }
  else if (ambientTemp < Ta4) {
    targetTemp = (ambientTemp - Ta3) * (Tflr4 - Tflr3) / (Ta4 - Ta3) + Tflr3;
  }
  else {
    targetTemp = Tflr4;
  }
   
  return constrain(targetTemp, 20.0, 46.0);
}

    float calculateWeightedInput() {
      //This is the feedback we can mix the temperatures here as feedback. 
      //Weighted average of flow/return temp and room temp
      /* Adjust weights based on your preference (0.0-1.0) it 
      can be used as a feeled temp compensation regarding radiation of cold walls*/
            
      return (flowTemp * flowWeight) + ((roomTemp-22) * roomWeight);
    }

    void controlValve(double pidOutput) { 
      // 3-point control for the mixing valve
      Serial.print("PID output is:");
      Serial.println(pidOutput);
      float time = ValveRuntime/3/dTratio;
      unsigned long time2=(unsigned long)time;
      if (time2<250){time2=250;}
      checkMemory();

       if (pidOutput > (30.0)) {
        // Need more heat - open valve
        digitalWrite(VALVE_OPEN, LOW);
        watchdogDelay(time2*2);
        digitalWrite(VALVE_OPEN, HIGH);
        digitalWrite(VALVE_CLOSE, HIGH);
        watchdogDelay(10000);  }    
      
      
      else if (pidOutput > (10.0)) {
        // Need more heat - open valve
        digitalWrite(VALVE_OPEN, LOW);
        watchdogDelay(time2);
        digitalWrite(VALVE_OPEN, HIGH);
        digitalWrite(VALVE_CLOSE, HIGH);
        watchdogDelay(10000);
      } else if (pidOutput < -10.0) {
        // Need less heat - close valve
        digitalWrite(VALVE_OPEN, HIGH);       
        digitalWrite(VALVE_CLOSE, LOW);
        watchdogDelay(time2);
        digitalWrite(VALVE_CLOSE, HIGH);
        watchdogDelay(10000);}
else if (pidOutput < -30.0) {
        // Need less heat - close valve
        digitalWrite(VALVE_OPEN, HIGH);       
        digitalWrite(VALVE_CLOSE, LOW);
        watchdogDelay(time2*2);
        digitalWrite(VALVE_CLOSE, HIGH);
        watchdogDelay(10000);}

       else {
        // Within deadband - stop valve
        digitalWrite(VALVE_OPEN, HIGH);
        digitalWrite(VALVE_CLOSE, HIGH);
        watchdogDelay(10000);
      }
      }

void checkForErrors() {
  // Check for calculation failures
  if (isnan(tankTemp) || isnan(flowTemp) || isnan(returnTemp) || 
      isnan(roomTemp) || isnan(ambientTemp)) {
    errorState = true;
    return;
  }
  
  // Check for freeze or overheat or bad sensor circuit conditions 
  if (flowTemp < 2.0 || returnTemp < 2.0 || 
      tankTemp < 2.0 || roomTemp < 2.0 ||
      flowTemp > 50.0 || returnTemp > 50.0 || 
      tankTemp > 95.0 || roomTemp > 40.0 
      ) {
    errorState = true;
    return;
  }
  
  // Check for unreasonable temperature differences
  if ((returnTemp-flowTemp) > 4.0 ||(flowTemp-returnTemp)>15.0) {
    errorState = true;
    return;
  }
  
  // If we get here, no errors
  errorState = false;
}

void watchdogDelay(unsigned long ms) {
  // Delay function that resets the watchdog timer
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    wdt_reset();
    delay(100);
  }
}

void resetPID() {
  myPID.SetMode(MANUAL);  // Turn off PID
  Output = 0;             // Reset output
  myPID.SetMode(AUTOMATIC); // Turn back on - this resets internals
}

void checkMemory() {
    Serial.print("Free RAM: ");
    Serial.println(freeRam());
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

