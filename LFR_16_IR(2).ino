// --- Motor Pins ---
#define IN1 11
#define IN2 10
#define IN3 3
#define IN4 9
#define ENA 6
#define ENB 5

// --- Multiplexer Pins (A0–A3) ---
#define s0 14
#define s1 15
#define s2 16
#define s3 17

#define SENSOR_PIN A4

bool isBlackLine = 1;
unsigned int numSensors = 16;

// --- PID + Speed Variables ---
float Kp = 0.07;
float Kd = 2.0;
float Ki = 0.001;

int P, D, I = 0, PIDvalue, previousError = 0;
double error;

int lfSpeed = 220;
int baseSpeed = 0;
int lsp, rsp;
int onLine = 1;

int minValues[16], maxValues[16], threshold[16];
int sensorValue[16], sensorArray[16];
int sensorWeight[16] = {7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7};

void setup() {
  Serial.begin(9600);

  // Motor Pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // MUX Select Lines
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);

  motor1run(0); motor2run(0);
  calibrate();  // Auto calibration
}

void loop() {
  readLine();  // Update sensorArray + error

  if (onLine) {
    adjustSpeed();  // Auto speed adjustment logic
    lineFollow();
  } else {
    // Sharp turn recovery
    if (error > 1000) {
      motor1run(-100);
      motor2run(255);
    } else if (error < -1000) {
      motor1run(255);
      motor2run(-100);
    }
  }
}

void adjustSpeed() {
  int edgeSensors = sensorArray[0] + sensorArray[1] + sensorArray[14] + sensorArray[15];
  int centerSensors = sensorArray[7] + sensorArray[8];
  int curveError = abs(error);

  if (edgeSensors >= 2) {
    baseSpeed = lfSpeed - 100;  // Sharp turn
  } else if (curveError > 400) {
    baseSpeed = lfSpeed - 70;   // Medium curve
  } else if (centerSensors == 2) {
    baseSpeed = lfSpeed;        // Perfectly centered
  } else {
    baseSpeed = lfSpeed - 40;   // Small adjustment
  }

  baseSpeed = constrain(baseSpeed, 100, lfSpeed);
}

void lineFollow() {
  P = error;
  I += error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = baseSpeed - PIDvalue;
  rsp = baseSpeed + PIDvalue;

  lsp = constrain(lsp, -100, 255);
  rsp = constrain(rsp, -100, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void readLine() {
  error = 0;
  onLine = 0;
  int activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    int raw = sensorRead(i);
    sensorValue[i] = isBlackLine
                       ? map(raw, minValues[i], maxValues[i], 0, 1000)
                       : map(raw, minValues[i], maxValues[i], 1000, 0);

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) {
      onLine = 1;
      error += sensorWeight[i] * sensorValue[i];
      activeSensors++;
      //Serial.print("⬛");  // Detected black
    } else {
      //Serial.print("⬜");  // Detected white
    }
  }

  Serial.println();  // New line after 16 sensors

  if (activeSensors > 0) error = error / activeSensors;
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

  motor1run(0); motor2run(0);
}

int sensorRead(int sensor) {
  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);
  delayMicroseconds(5);
  return analogRead(SENSOR_PIN);
}

void motor1run(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motor2run(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

