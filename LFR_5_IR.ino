// --- Motor Pins (L298N) ---
#define IN1 11
#define IN2 10
#define IN3 3
#define IN4 9
#define enA 6
#define enB 5

// --- Sensor Pins ---
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4

int minValues[6], maxValues[6], threshold[6];

int lsp, rsp;
int lfspeed = 190;

int P, D, I = 0, previousError = 0, PIDvalue, error;
float Kp = 0.04;
float Kd = 0;
float Ki = 0;

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);

  delay(1000);
  calibrate();  // Auto calibration on startup
}

void loop() {
  printSensorStatus();  // <- Debug print for sensors

  if (analogRead(S1) > threshold[1] && analogRead(S5) < threshold[5]) {
    motor(0, lfspeed);  // Turn left
  }
  else if (analogRead(S5) > threshold[5] && analogRead(S1) < threshold[1]) {
    motor(lfspeed, 0);  // Turn right
  }
  else if (analogRead(S3) > threshold[3]) {
    Kp = 0.0006 * (1000 - analogRead(S3));
    Kd = 10 * Kp;
    linefollow();
  }

  delay(100);  // small delay for readability in Serial Monitor
}

void linefollow() {
  error = analogRead(S2) - analogRead(S4);  // Edge sensors

  P = error;
  I += error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // --- Speed reduction during curves ---
  int speedFactor = constrain(abs(PIDvalue), 0, 150);
  int speedReduction = map(speedFactor, 0, 150, 0, 100);
  int adjustedSpeed = lfspeed - speedReduction;
  adjustedSpeed = constrain(adjustedSpeed, 100, lfspeed); // lower bound

  lsp = adjustedSpeed - PIDvalue;
  rsp = adjustedSpeed + PIDvalue;

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor(lsp, rsp);
}

void calibrate() {
  // Init min/max
  for (int i = 1; i < 6; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  // 360-degree spin to capture all sensor ranges
  for (int i = 0; i < 3000; i++) {
    motor(70, -70);  // spin in place

    for (int j = 1; j < 6; j++) {
      int val = analogRead(j);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  for (int i = 1; i < 6; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print("Threshold S");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(threshold[i]);
  }

  motor(0, 0);  // stop after calibration
}

// --- Debug Print Function ---
void printSensorStatus() {
  int sensors[6] = {S1, S2, S3, S4, S5};
  Serial.print("Sensors: ");
  for (int i = 1; i < 6; i++) {
    int val = analogRead(sensors[i - 1]);
    if (val > threshold[i]) {
      Serial.print("⬛");  // black line detected
    } else {
      Serial.print("⬜");  // white surface
    }
  }
  Serial.println();
}

// --- Motor Control Function ---
void motor(int a, int b) {
  if (a > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    a = -a;
  }

  if (b > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    b = -b;
  }

  analogWrite(enA, constrain(a, 0, 255));
  analogWrite(enB, constrain(b, 0, 255));
}



// Sensor pins
// #define S1 A0
// #define S2 A1
// #define S3 A2
// #define S4 A3
// #define S5 A4

// int threshold[5] = {500, 500, 500, 500, 500}; // Replace with calibrated values if needed

// void setup() {
//   Serial.begin(9600);
// }

// void loop() {
//   int sensorValues[5];

//   sensorValues[0] = analogRead(S1);
//   sensorValues[1] = analogRead(S2);
//   sensorValues[2] = analogRead(S3);
//   sensorValues[3] = analogRead(S4);
//   sensorValues[4] = analogRead(S5);

//   for (int i = 0; i < 5; i++) {
//     Serial.print("S");
//     Serial.print(i + 1);
//     Serial.print(": ");
//     Serial.print(sensorValues[i]);
//     Serial.print(" ");
//     if (sensorValues[i] > threshold[i]) {
//       Serial.print("Black");
//     } else {
//       Serial.print("White");
//     }

//     if (i < 4) Serial.print(" | ");
//   }

//   Serial.println(); // New line after each loop
//   delay(300);       // Adjust as needed
// }
