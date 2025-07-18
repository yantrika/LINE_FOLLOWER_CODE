// --- L298N Motor Pins ---
#define IN1 11
#define IN2 10
#define IN3 3
#define IN4 9
#define ENA 6
#define ENB 5

// --- Multiplexer select lines (A0-A3 = D14–D17) ---
#define s0 14  // A0
#define s1 15  // A1
#define s2 16  // A2
#define s3 17  // A3

// --- Sensor Output Pin ---
#define SENSOR_PIN A4

// --- Line Settings ---
bool isBlackLine = 1;
unsigned int numSensors = 16;

// --- PID Variables ---
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 220;
int currentSpeed = 30;

int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };

int activeSensors;
float Kp = 0.07;
float Kd = 2;
float Ki = 0.001
int onLine = 1;

int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];
double startMillis, elapsedTime;

void setup() {
  Serial.begin(9600);

  // --- Motor Setup ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // --- Multiplexer Pins ---
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  // Stop motors initially
  motor1run(0);
  motor2run(0);

  // Calibrate sensors by spinning 360°
  calibrate();
}

void loop() {
  startMillis = millis();

  // Read line
  digitalWrite(s0, 0);
  digitalWrite(s1, 0);
  digitalWrite(s2, 0);
  digitalWrite(s3, 0);
  readLine();

  if (currentSpeed < lfSpeed) currentSpeed++;

  if (onLine == 1) {
    linefollow();
  } else {
    if (error > 1000) {
      motor1run(-100);
      motor2run(255);
    } else if (error < -1000) {
      motor1run(255);
      motor2run(-100);
    }
  }

  elapsedTime = millis() - startMillis;
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    }
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, -100, 255);
  rsp = constrain(rsp, -100, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 16; i++) {
    int val = sensorRead(i);
    minValues[i] = val;
    maxValues[i] = val;
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(70);
    motor2run(-70);

    for (int j = 0; j < 16; j++) {
      int val = sensorRead(j);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  for (int i = 0; i < 16; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;

  for (int i = 0; i < 16; i++) {
    int raw = sensorRead(i);
    sensorValue[i] = isBlackLine
                       ? map(raw, minValues[i], maxValues[i], 0, 1000)
                       : map(raw, minValues[i], maxValues[i], 1000, 0);

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    // Debug print: black or white block
    if (sensorArray[i]) {
      //Serial.print("⬛");  // Black block
      onLine = 1;
    } else {
      //Serial.print("⬜");  // White block
    }
  }

  //Serial.println();  // New line after all 16 sensors
}


void motor1run(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motor2run(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

int sensorRead(int sensor) {
  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);
  delayMicroseconds(5);  // small delay for mux to settle
  return analogRead(SENSOR_PIN);
}


// --- Multiplexer select lines (A0-A3 = D14–D17) ---
// #define s0 14
// #define s1 15
// #define s2 16
// #define s3 17

// // --- Sensor Output Pin ---
// #define SENSOR_PIN A4

// bool isBlackLine = 1;  // Set 1 for black line on white surface

// int minValues[16], maxValues[16], threshold[16];
// int sensorValue[16], sensorArray[16];

// void setup() {
//   Serial.begin(9600);

//   // Multiplexer pin setup
//   pinMode(s0, OUTPUT);
//   pinMode(s1, OUTPUT);
//   pinMode(s2, OUTPUT);
//   pinMode(s3, OUTPUT);

//   // Quick Calibration
//   calibrate();
// }

// void loop() {
//   readLine();       // Read sensors and print black/white blocks
//   delay(200);       // Adjust refresh rate
// }

// void calibrate() {
//   for (int i = 0; i < 16; i++) {
//     int val = sensorRead(i);
//     minValues[i] = val;
//     maxValues[i] = val;
//   }

//   // Simulated rotation (replace with real spin for accuracy)
//   for (int t = 0; t < 1000; t++) {
//     for (int i = 0; i < 16; i++) {
//       int val = sensorRead(i);
//       if (val < minValues[i]) minValues[i] = val;
//       if (val > maxValues[i]) maxValues[i] = val;
//     }
//   }

//   for (int i = 0; i < 16; i++) {
//     threshold[i] = (minValues[i] + maxValues[i]) / 2;
//     Serial.print("T"); Serial.print(i); Serial.print(": ");
//     Serial.println(threshold[i]);
//   }
//   Serial.println("Calibration Done.\n");
// }

// void readLine() {
//   for (int i = 0; i < 16; i++) {
//     int raw = sensorRead(i);

//     sensorValue[i] = isBlackLine
//                        ? map(raw, minValues[i], maxValues[i], 0, 1000)
//                        : map(raw, minValues[i], maxValues[i], 1000, 0);

//     sensorValue[i] = constrain(sensorValue[i], 0, 1000);
//     sensorArray[i] = sensorValue[i] > 500;

//     if (sensorArray[i]) {
//       Serial.print("⬛");  // Detected black
//     } else {
//       Serial.print("⬜");  // Detected white
//     }
//   }
//   Serial.println();
// }

// int sensorRead(int sensor) {
//   digitalWrite(s0, sensor & 0x01);
//   digitalWrite(s1, sensor & 0x02);
//   digitalWrite(s2, sensor & 0x04);
//   digitalWrite(s3, sensor & 0x08);
//   delayMicroseconds(5);
//   return analogRead(SENSOR_PIN);
// }

/* Last Updated- 7 March 2025
Sensor Test code for 16 Channel Line Following Sensor

Pin Connections
S0-A0
S1-A1
S2-A2
S3-A3
Signal-A4
Enable-Active Low

Please ensure the sensor has a separate 5V line which can provide around 300 mA. Do not use the Arduino 5V line since it may damage its on board regulator. 
*/


// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

// #define s0 14  // A0 defined as digital pin 14
// #define s1 15  // A1 defined as digital pin 15
// #define s2 16  // A2 defined as digital pin 16
// #define s3 17  // A3 defined as digital pin 17

// unsigned int numSensors = 16;
// int rawSensorValue[16];
// void setup() {

//   sbi(ADCSRA, ADPS2);
//   cbi(ADCSRA, ADPS1);
//   cbi(ADCSRA, ADPS0);

//   pinMode(s0, OUTPUT);
//   pinMode(s1, OUTPUT);
//   pinMode(s2, OUTPUT);
//   pinMode(s3, OUTPUT);


//   Serial.begin(9600);
// }

// void loop() {
//   read16Sens();

//   printRawSensorValues();
// }

// void read16Sens() {
//   for (int i = 0; i < 16; i++) {
//     digitalWrite(s0, i & 0x01);
//     digitalWrite(s1, i & 0x02);
//     digitalWrite(s2, i & 0x04);
//     digitalWrite(s3, i & 0x08);
//     rawSensorValue[i] = analogRead(4);
//   }
// }

// void printRawSensorValues() {
//   for (int i = 0; i < 16; i++) {
//     Serial.print(" S");
//     Serial.print(i);
//     Serial.print(" = ");
//     Serial.print(rawSensorValue[i]);
//   }
//   Serial.println();
// }
