 /*
yulissaa
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// TB6612FNG Motor Driver Pins
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// Multiplexer select pins
#define s0 14  // A0
#define s1 15  // A1
#define s2 16  // A2
#define s3 17  // A3

#define sensorAnalogPin A4

bool isBlackLine = 1;
unsigned int numSensors = 16;

// PID variables
int P, D, I, previousError, PIDvalue, previousPID;
double error;
int lsp, rsp;
int lfSpeed = 220;
int currentSpeed = 30;
int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };

int activeSensors;
float Kp = 0.06;
float Kd = 1.0;
float Ki = 0.0;
int onLine = 1;
int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];

double startMillis, prevMillis, elapsedTime;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(11, INPUT_PULLUP); // Pushbutton for start calibration
  pinMode(12, INPUT_PULLUP); // Pushbutton for start run
  pinMode(13, OUTPUT);       // LED

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    startMillis = millis();

    digitalWrite(s0, 0);
    digitalWrite(s1, 0);
    digitalWrite(s2, 0);
    digitalWrite(s3, 0);
    readLine();

    if (currentSpeed < lfSpeed) currentSpeed++;

    // Advanced turn detection using edge sensors
    bool leftSharp = sensorArray[0] || sensorArray[1];
    bool rightSharp = sensorArray[15] || sensorArray[14];

    if (onLine) {
      digitalWrite(13, HIGH);
      linefollow();
    } else {
      digitalWrite(13, LOW);
      if (leftSharp) {
        motor1run(-100);
        motor2run(200);
      } else if (rightSharp) {
        motor1run(200);
        motor2run(-100);
      } else {
        motor1run(-100);
        motor2run(100);
      }
    }

    elapsedTime = millis() - startMillis;
  }
}

void linefollow() {
  error = 0;
  int totalSensor = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorValue[i];
      totalSensor += sensorValue[i];
    }
  }
  if (totalSensor == 0) totalSensor = 1;
  error /= totalSensor;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  PIDvalue = constrain(PIDvalue, previousPID - 10, previousPID + 10);
  previousPID = PIDvalue;
  previousError = error;

  // Adaptive speed based on curve
  int spread = abs(error);
  if (spread < 500) currentSpeed = 220;
  else if (spread < 1000) currentSpeed = 180;
  else currentSpeed = 150;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  motor1run(constrain(lsp, -100, 255));
  motor2run(constrain(rsp, -100, 255));
}

void calibrate() {
  for (int i = 0; i < 16; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
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
    if (isBlackLine)
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
    else
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    if (sensorArray[i]) onLine = 1;
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1); digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0); digitalWrite(AIN2, 1);
    analogWrite(PWMA, -motorSpeed);
  } else {
    digitalWrite(AIN1, 1); digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1); digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0); digitalWrite(BIN2, 1);
    analogWrite(PWMB, -motorSpeed);
  } else {
    digitalWrite(BIN1, 1); digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

int sensorRead(int sensor) {
  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);
  return analogRead(sensorAnalogPin);
}

