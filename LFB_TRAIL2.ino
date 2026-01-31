//version 2
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ALL_BLACK 0b1111111111111111
#define ALL_WHITE 0b0000000000000000



// -------- TB6612FNG MOTOR DRIVER --------
#define IN1 5
#define IN2 4
#define IN3 7
#define IN4 8
#define enA 3
#define enB 9
#define LED 2
#define BUTTON 10 
#define STBY 6

// -------- MUX PINS --------
#define S0 A1
#define S1 A2
#define S2 A3
#define S3 A4

// -------- SENSOR --------
#define sensorPin A0

uint16_t pattern;

// -------- LINE FOLLOW CONFIG --------
bool isBlackLine = 1;
int sensorWeight[16] = { 7,6,5,4,3,2,1,0,0,-1,-2,-3,-4,-5,-6,-7 };

int minValues[16], maxValues[16];
int sensorValue[16], sensorArray[16];

int lfSpeed = 180;
int currentSpeed = 30;

float Kp = 0.12, Ki = 0.0002, Kd = 1;
int P, I, D, previousError, PIDvalue;
int lsp, rsp, activeSensors;
double error;
bool onLine;

// -------- SETUP --------
void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);

  // -------- 1. CALIBRATION --------
  calibrate();

  // // -------- 2. STOP MOTORS --------
  // motor1run(0);
  // motor2run(0);

  // // -------- 3. READY INDICATOR --------
  // digitalWrite(LED, HIGH);

  // // -------- 4. WAIT FOR BUTTON PRESS --------
  // while (digitalRead(BUTTON) == HIGH) {
  //   motor1run(0);
  //   motor2run(0);
  // }

  // delay(300);  // debounce

  // // -------- 5. START --------
  // digitalWrite(LED, LOW);
  // currentSpeed = 30;
  // previousError = 0;
}

void loop() {

  readLine();

  // Build 16-bit pattern
  pattern = 0;
  for (int i = 0; i < 16; i++) {
    pattern = (pattern << 1) | sensorArray[i];
  }

  // ===============================
  // 1️⃣ JUNCTION / ENDPOINT HANDLING
  // ===============================
  if (pattern == ALL_BLACK) {

    // move forward slightly
    motor1run(80);
    motor2run(80);
    delay(100);

    stopMotors();
    delay(80);

    // re-read sensors
    readLine();
    pattern = 0;
    for (int i = 0; i < 16; i++) {
      pattern = (pattern << 1) | sensorArray[i];
    }

    // 🔴 FINAL END POINT
    if (pattern == ALL_BLACK) {
        stopMotors();
  digitalWrite(LED, HIGH);   // Glow LED (final stop)

  // WAIT FOREVER until CENTER sensors see line
  while (true) {
    readLine();

    // Middle sensors active → back on track
    if (!sensorArray[0] && sensorArray[7] && sensorArray[8] || !sensorArray[15]) {
      digitalWrite(LED, LOW);   // Turn LED off
      previousError = 0;
      currentSpeed = 30;
      break;   // exit waiting loop
    }

    stopMotors();  // ensure motors stay stopped
  }

  return;
    }

    // 🟦 T-JUNCTION → LEFT TURN
    else if (pattern == ALL_WHITE) {

      stopMotors();
      delay(80);

      // rotate LEFT until line found
      motor1run(-100);
      motor2run(100);

      while (1) {
        readLine();
        if (sensorArray[7] || sensorArray[8] || sensorArray[9] || sensorArray[10]) break;
      }

      stopMotors();
      delay(50);

      previousError = 0;
      currentSpeed = 30;
      return;   // exit loop, resume PID
    }

    // 🟢 LINE CONTINUES
    else {
      previousError = 0;
      currentSpeed = 30;
      return;
    }
  }

  // ===============================
  // 2️⃣ SHARP TURN HANDLING
  // ===============================
  bool sharpTurn = false;

  if ((pattern & 0b1111110000000000) == 0b1111110000000000 ||
      (pattern & 0b1111111111100000) == 0b1111111111100000 ||
      (pattern & 0b1111111100000000) == 0b1111111100000000) {

    motor1run(-90);
    motor2run(200);
    sharpTurn = true;
  }

  else if ((pattern & 0b0000000000111111) == 0b0000000000111111 ||
           (pattern & 0b0000000011111111) == 0b0000000011111111||
          (pattern & 0b0000011111111111) == 0b0000011111111111 ){

    motor1run(200);
    motor2run(-90);
    sharpTurn = true;
  }

  // ===============================
  // 3️⃣ NORMAL PID FOLLOW
  // ===============================
  if (!sharpTurn) {

    if (currentSpeed < lfSpeed) currentSpeed++;

    if (onLine) {
      linefollow();
    } else {
      // lost line recovery
      if (previousError > 0) {
        motor1run(-200);
        motor2run(200);
      } else {
        motor1run(200);
        motor2run(-200);
      }
    }
  }
}


// -------- PID LINE FOLLOW --------
void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorValue[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0)
    error /= activeSensors;

  P = error;
  I += error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, -100, 200);
  rsp = constrain(rsp, -100, 200);

  motor1run(lsp);
  motor2run(rsp);
}

// -------- CALIBRATION --------
void calibrate() {
  for (int i = 0; i < 16; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    motor1run(70);
    motor2run(-70);

    for (int i = 0; i < 16; i++) {
      int val = sensorRead(i);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  motor1run(0);
  motor2run(0);
}

// -------- READ SENSORS --------
void readLine() {
  onLine = false;

  for (int i = 0; i < 16; i++) {
    if (isBlackLine)
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
    else
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) onLine = true;
  }
}


void stopMotors() {
  motor1run(0);
  motor2run(0);
}


// -------- MOTOR CONTROL --------
void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(enA, spd);
  } else if (spd < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(enA, -spd);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
    analogWrite(enA, 0);
  }
}

void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(enB, spd);
  } else if (spd < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(enB, -spd);
  } else {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
    analogWrite(enB, 0);
  }
}

// -------- MUX SENSOR READ --------
int sensorRead(int ch) {
  digitalWrite(S0, ch & 1);
  digitalWrite(S1, ch & 2);
  digitalWrite(S2, ch & 4);
  digitalWrite(S3, ch & 8);
  delayMicroseconds(5);
  return analogRead(sensorPin);
}