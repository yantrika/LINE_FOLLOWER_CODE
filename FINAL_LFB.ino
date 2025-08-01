
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//-------- Motor Driver Pins (TB6612FNG) --------
#define IN1 5
#define IN2 4
#define IN3 7
#define IN4 8
#define enA 3
#define enB 9
#define STBY 6

//-------- Multiplexer Select Pins --------------
#define S0 A1
#define S1 A2
#define S2 A3
#define S3 A4

//-------- IR Sensor Analog Pin -----------------
#define sensorPin A0

//-------- Line Follower Configuration ----------
bool isBlackLine = 1;  // 1 for black line, 0 for white
unsigned int numSensors = 16;
int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };

int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];
int lfSpeed = 220;
int currentSpeed = 30;
float Kp = 0.06, Kd = 1, Ki = 0;

int P, D, I, previousError, PIDvalue;
int lsp, rsp, activeSensors, onLine = 1;
double error;
double startMillis, elapsedTime;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Multiplexer pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Manual calibration
  calibrate();
}

void loop() {
  startMillis = millis();

  // Reset multiplexer select lines
  digitalWrite(S0, 0);
  digitalWrite(S1, 0);
  digitalWrite(S2, 0);
  digitalWrite(S3, 0);

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

// ---------------- LINE FOLLOWER LOGIC ----------------

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    }
    activeSensors += sensorArray[i];
  }

  error = error / activeSensors;

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

// ---------------- CALIBRATION ----------------

void calibrate() {
  for (int i = 0; i < 16; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  unsigned long startCalib = millis();
  while (millis() - startCalib < 4000) {
    motor1run(70);
    motor2run(-70);
    for (int i = 0; i < 16; i++) {
      int val = sensorRead(i);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
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

// ---------------- SENSOR READ ----------------

void readLine() {
  onLine = 0;
  for (int i = 0; i < 16; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    if (sensorArray[i]) onLine = 1;
  }
}

// ---------------- MOTOR CONTROL ----------------

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(enA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(enA, abs(motorSpeed));
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(enA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(enB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(enB, abs(motorSpeed));
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(enB, 0);
  }
}

// ---------------- SENSOR MUX READ ----------------

int sensorRead(int sensor) {
  digitalWrite(S0, sensor & 0x01);
  digitalWrite(S1, sensor & 0x02);
  digitalWrite(S2, sensor & 0x04);
  digitalWrite(S3, sensor & 0x08);
  delayMicroseconds(5); // small delay for settling
  return analogRead(sensorPin);
}
