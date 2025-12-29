#include <PID_v1.h>
#include <avr/wdt.h>

// ===== PIN CONFIGURATION =====
#define PUMP_RELAY 2
#define VALVE_CLOSE 3
#define VALVE_OPEN 4
#define ERROR_LED 12
#define THERMOSTAT_SWITCH 11

#define AMBIENT_TEMP A0
#define TANK_TEMP A4
#define FLOW_TEMP A3
#define RETURN_TEMP A2
#define ROOM_TEMP A1

// ===== SYSTEM CONSTANTS =====
#define VALVE_RUNTIME 30000
#define SENSOR_CHECK_INTERVAL 5000
#define PID_COMPUTE_INTERVAL 10000
#define SAFETY_DELAY 500

// Heating curve points
const float Ta1 = -5, Ta2 = -4, Ta3 = -3, Ta4 = 20;
const float Tflr1 = 40, Tflr2 = 39.5, Tflr3 = 39, Tflr4 = 20;
const float Troomgoal=19.0;

// ===== GLOBAL VARIABLES =====
double Setpoint = 35.0, Input = 20.0, Output;
PID myPID(&Input, &Output, &Setpoint, 5.0, 2, 0.0, DIRECT);
 const int PIDmin=-50,PIDmax=50;

float tankTemp, flowTemp, returnTemp, roomTemp, ambientTemp;
float avgFlowReturnTemp, dT1, dT2, dTratio;
bool systemActive = false, errorState = false, isPumpWorking = false;
unsigned long lastSensorCheck = 0,lastPIDCompute=0;
int hysteresis=0;


// ===== CORE FUNCTIONS =====
void watchdogDelay(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    wdt_reset();
    delay(100);
  }
}

void readAllSensors() {
  tankTemp = readPT100(TANK_TEMP, 100.0);
  flowTemp = readPT100(FLOW_TEMP, 100.5);
  returnTemp = readPT100(RETURN_TEMP, 100.5);
  roomTemp = readPT100(ROOM_TEMP, 99.5);
  ambientTemp = readPT100(AMBIENT_TEMP, 100.0);
  
  avgFlowReturnTemp = (flowTemp + returnTemp) / 2.0;
  dT1 = tankTemp - returnTemp;
  dT2 = abs(flowTemp - returnTemp);
  dTratio = (dT2 == 0) ? 100 : abs(dT1 / dT2);
  dTratio = constrain(dTratio, 1, 70);
  
  Setpoint = calculateHeatingCurve(); //goal temp in PID calculation
  Input = (flowTemp * 1.0) + (constrain((roomTemp - Troomgoal),0.0,10.0) * 0); // Weighted input in PID calculation
  
  Serial.print("Tank:"); Serial.print(tankTemp);
  Serial.print(" Flow:"); Serial.print(flowTemp);
  Serial.print(" Return:"); Serial.print(returnTemp);
  Serial.print(" Room:"); Serial.print(roomTemp);
  Serial.print(" Ambient:"); Serial.print(ambientTemp);
  Serial.print(" Setpoint:"); Serial.print(Setpoint);
  Serial.print(" weightedInput:"); Serial.println(Input);
}

float readPT100(int pin, float fixResistor) {
  const int numReadings = 5;
  int readings[numReadings];
  
  for (int i = 0; i < numReadings; i++) {
    readings[i] = analogRead(pin);
    delay(50);
  }
  
  // Simple sort for median
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = i + 1; j < numReadings; j++) {
      if (readings[i] > readings[j]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  int raw = readings[numReadings / 2];
  float voltage = raw * (5.0 / 1023.0);
  float resistance = (voltage * fixResistor) / (5.0 - voltage);
  return (resistance - 100.0) / 0.385;
}

float calculateHeatingCurve() {
  float targetTemp = Tflr4; // Default
  
  if (ambientTemp < Ta1) {
    targetTemp = Tflr1;
  } else if (ambientTemp < Ta2) {
    targetTemp = (ambientTemp - Ta1) * (Tflr2 - Tflr1) / (Ta2 - Ta1) + Tflr1;
  } else if (ambientTemp < Ta3) {
    targetTemp = (ambientTemp - Ta2) * (Tflr3 - Tflr2) / (Ta3 - Ta2) + Tflr2;
  } else if (ambientTemp < Ta4) {
    targetTemp = (ambientTemp - Ta3) * (Tflr4 - Tflr3) / (Ta4 - Ta3) + Tflr3;
  }
  
  return constrain(targetTemp, 20.0, 40.0);
}

void controlValve(double pidOutput) {
  Serial.print("PID output:"); Serial.println(pidOutput);
  
  unsigned long moveTime = VALVE_RUNTIME / 3 / dTratio;
  if (moveTime < 150) moveTime = 150;
  Serial.print("moveTime  ");Serial.print(moveTime);
  if (pidOutput > 30.0 && pidOutput<PIDmax) {
    digitalWrite(VALVE_OPEN, LOW);
    watchdogDelay(moveTime * 2);
    digitalWrite(VALVE_OPEN, HIGH);
  } else if (pidOutput > 13.0) {
    digitalWrite(VALVE_OPEN, LOW);
    watchdogDelay(moveTime);
    digitalWrite(VALVE_OPEN, HIGH);
  } else if (pidOutput < -30.0) {
    digitalWrite(VALVE_CLOSE, LOW);
    watchdogDelay(moveTime * 2);
    digitalWrite(VALVE_CLOSE, HIGH);
  } else if (pidOutput < -13.0) {
    digitalWrite(VALVE_CLOSE, LOW);
    watchdogDelay(moveTime);
    digitalWrite(VALVE_CLOSE, HIGH);
  } else {
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    watchdogDelay(SAFETY_DELAY);
    return;
  }
  
  watchdogDelay(SAFETY_DELAY);
}

bool checkForErrors() {
  // Check for invalid readings
  if (isnan(tankTemp) || isnan(flowTemp) || isnan(returnTemp) || 
      isnan(roomTemp) || isnan(ambientTemp)) {
    return true;
  }
  
  // Temperature limits
  if (flowTemp < 2.0 || returnTemp < 2.0 || tankTemp < 2.0 || roomTemp < 2.0 ||
       flowTemp > 50.0 || returnTemp > 50.0 ||tankTemp > 95.0 || roomTemp > 30.0) {
    return true;
  }
  
  // Temperature differences
  if ((returnTemp - flowTemp) > 4.0 || (flowTemp - returnTemp) > 20.0) {
    return true;
  }
  
  return false;
}

bool checkForOvertemp() {    
  // Temperature limits
  if (flowTemp > 50.0 || returnTemp > 50.0 ) {
    return true;
  } 
  return false;
}

bool checkForUndertemp() {    
  // Temperature limits
  if (flowTemp < 8.0 || returnTemp < 8.0 || tankTemp < 8.0 || roomTemp < 8.0 ) {
    return true;
  } 
  return false;
}


void resetPID() {
  myPID.SetMode(MANUAL);
  Output = 0;
  myPID.SetMode(AUTOMATIC);
}

// ===== SETUP =====
void setup() {
  wdt_disable();
  wdt_enable(WDTO_8S);
  Serial.begin(9600);
  
  // Pin setup
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(VALVE_OPEN, OUTPUT);
  pinMode(VALVE_CLOSE, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(THERMOSTAT_SWITCH, INPUT_PULLUP);
  
  // Initial states
  digitalWrite(13, LOW);
  digitalWrite(PUMP_RELAY, HIGH); // Pump off
  digitalWrite(VALVE_OPEN, HIGH); // Valve stopped
  digitalWrite(VALVE_CLOSE, HIGH);
  digitalWrite(ERROR_LED, LOW);
  
  // Output test
  digitalWrite(PUMP_RELAY, LOW); watchdogDelay(2000); digitalWrite(PUMP_RELAY, HIGH);
  digitalWrite(VALVE_OPEN, LOW); watchdogDelay(2000); digitalWrite(VALVE_OPEN, HIGH);
  digitalWrite(VALVE_CLOSE, LOW); watchdogDelay(2000); digitalWrite(VALVE_CLOSE, HIGH);
  digitalWrite(ERROR_LED, HIGH); watchdogDelay(2000); digitalWrite(ERROR_LED, LOW);
  Serial.println("Output test done");
  
  // PID setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(PIDmin, PIDmax);
  Serial.println("PID initialized");
  
  // Valve initialization
  digitalWrite(VALVE_CLOSE, LOW);
  Serial.println("Valve setting to minimum...");
  watchdogDelay(VALVE_RUNTIME);
  digitalWrite(VALVE_CLOSE, HIGH);
  Serial.println("Valve set to minimum");
  
  analogReference(EXTERNAL);
  Serial.println("System ready");
}

// ===== MAIN LOOP =====
void loop() {
  wdt_reset();
  /*
  static unsigned long loopCount = 0;
  loopCount++;
  if (loopCount % 10 == 0) {
    Serial.print("Loop: "); Serial.println(loopCount);
  }
  */
  
  // Read sensors
  if (millis() - lastSensorCheck > SENSOR_CHECK_INTERVAL) {
    watchdogDelay(250);
    systemActive = (digitalRead(THERMOSTAT_SWITCH) == LOW);
    readAllSensors();
    lastSensorCheck = millis();
  }
  

//overtemp
if( checkForOvertemp()){
  Serial.println("FLOW or RETURN is too warm!!");
digitalWrite(PUMP_RELAY, HIGH);
digitalWrite(VALVE_OPEN, HIGH);
digitalWrite(VALVE_CLOSE, LOW);
watchdogDelay(VALVE_RUNTIME);
digitalWrite(VALVE_CLOSE, HIGH);
watchdogDelay(500);
digitalWrite(PUMP_RELAY, LOW);
watchdogDelay(2000);
digitalWrite(PUMP_RELAY, HIGH);
resetPID();
}

  // Error handling
  errorState = checkForErrors();
  if (errorState) {
    Serial.println("ERROR STATE!");
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    resetPID();
    digitalWrite(ERROR_LED, (millis() % 1000) < 500);
    return;
  }


// check for freeze protection

if(checkForUndertemp()){
Serial.println("freeze protection is active");
digitalWrite(PUMP_RELAY, LOW);
digitalWrite(VALVE_CLOSE, HIGH);
digitalWrite(VALVE_OPEN, LOW);
return;
}

   
  // Tank discharged
  if (tankTemp < (avgFlowReturnTemp + 2 + hysteresis)) {
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    digitalWrite(ERROR_LED, HIGH);
    isPumpWorking = false;
    hysteresis=2;
    resetPID();
    Serial.println("Tank is discharged");
    watchdogDelay(10000);
    return;
  }

// System inactive
  if (!systemActive) {
    Serial.println("System inactive");
    digitalWrite(PUMP_RELAY, HIGH);
    isPumpWorking = false;
    digitalWrite(VALVE_OPEN, HIGH);
    digitalWrite(VALVE_CLOSE, HIGH);
    digitalWrite(ERROR_LED, LOW);
    resetPID();
    watchdogDelay(SAFETY_DELAY);
    return;
  }
else{
  digitalWrite(PUMP_RELAY, LOW);
  isPumpWorking = true;
    hysteresis=0;
    digitalWrite(ERROR_LED, LOW);
}


  //PID
if (isPumpWorking) {
  if(millis()-lastPIDCompute>PID_COMPUTE_INTERVAL){
    myPID.Compute();
    lastPIDCompute=millis();
    controlValve(Output);
  } else {
    watchdogDelay(SAFETY_DELAY);  // Wait if not time for PID compute
  }
}
 else {
  watchdogDelay(SAFETY_DELAY);    // Wait if pump not working
}
  
}
