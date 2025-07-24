// --- Motor Pins (TB6612FNG) ---
#define IN1 11
#define IN2 10
#define IN3 3
#define IN4 9
#define enA 6
#define enB 5
#define STBY 4  // STBY pin connected to digital pin 4

// --- Multiplexer Select Pins ---
#define S0 14
#define S1 15
#define S2 16
#define S3 17

// --- IR Sensor Analog Output Pin ---
#define sensorPin A4

// --- PID Constants ---
float Kp = 0.06;
float Ki = 0;
float Kd = 0;
int P = 0, I = 0, D = 0, error = 0, previousError = 0;
int PIDvalue;

// --- Line Follower Speeds ---
int lfspeed = 190;
int lsp = 0, rsp = 0;

// --- Sensor Arrays ---
int S[17]; // Sensor values
int minValues[17], maxValues[17], T[17]; // Thresholds
int positionWeights[17] = {0, -600, -525, -450, -375, -300, -225, -150, 0, 0, 150, 225, 300, 375, 450, 525, 600};

// --- Setup ---
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); // Enable TB6612FNG

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);

  delay(1000);
  calibrate();  // 360° auto calibration
}

// --- Main Loop ---
void loop() {
  readSensors();
  printSensorStatus();

  bool middleOnLine = isMiddleOnLine();
  bool LineDetected = isLineDetected();

  // --- Improved Side Decision Logic ---
  int leftBlack = 0;
  int rightBlack = 0;
  if(!middleOnLine){
    for (int i = 1; i <= 4; i++) {
      if (S[i] > T[i]) leftBlack++;
    }
    for (int i = 13; i <= 16; i++) {
      if (S[i] > T[i]) rightBlack++;
    }

    if (leftBlack > rightBlack) {
      motor(0, lfspeed);  // Turn Left
    }
    else if (rightBlack > leftBlack) {
      motor(lfspeed, 0);  // Turn Right
    }

  }
  else if( LineDetected ){
    float baseKp = 0.03;
    float maxKp = 0.08;
    float errorRatio = constrain(abs(error) / 600.0, 0, 1);  // Assuming max abs(error) = ~600
    Kp = baseKp + (maxKp - baseKp) * errorRatio;
    Kd = 10 * Kp;
    linefollow();
  }
  else {
    motor(lfspeed, lfspeed);  // fallback straight
  }

  delay(50);
}

// --- Line Following with PID ---
void linefollow() {
  long weightedSum = 0;
  long totalValue = 0;

  for (int i = 1; i <= 16; i++) {
    if (i == 8 || i == 9) continue;  // Skip diamond sensors
    if (S[i] > T[i]) {
      weightedSum += (long)positionWeights[i] * S[i];
      totalValue += S[i];
    }
  }

  if (totalValue == 0) {
    motor(0, 0); // Line lost
    return;
  }

  int position = weightedSum / totalValue;
  error = position;

  P = error;
  I += error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  int speedFactor = constrain(abs(PIDvalue), 0, 150);
  int speedReduction = map(speedFactor, 0, 150, 0, 100);
  int adjustedSpeed = lfspeed - speedReduction;
  adjustedSpeed = constrain(adjustedSpeed, 100, lfspeed);

  lsp = adjustedSpeed - PIDvalue;
  rsp = adjustedSpeed + PIDvalue;

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor(lsp, rsp);
}

// --- Read All 16 Sensors ---
void readSensors() {
  for (int i = 1; i <= 16; i++) {
    digitalWrite(S0, bitRead(i - 1, 0));
    digitalWrite(S1, bitRead(i - 1, 1));
    digitalWrite(S2, bitRead(i - 1, 2));
    digitalWrite(S3, bitRead(i - 1, 3));
    delayMicroseconds(5);
    S[i] = analogRead(sensorPin);
  }
}

// --- Check Middle Alignment ---
bool isMiddleOnLine() {
  int count = 0;
  if (S[7] > T[7]) count++;   // Left center
  if (S[8] > T[8]) count++;   // Diamond top
  if (S[9] > T[9]) count++;   // Diamond bottom
  if (S[10] > T[10]) count++; // Right center
  return (count >= 2);
}

bool isLineDetected() {
  for (int i = 1; i <= 16; i++) {
    if (S[i] > T[i]) {
      return true;  // Line seen by any sensor
    }
  }
  return false;  // Line completely lost
}

// --- Calibration by 360° Spin ---
void calibrate() {
  for (int i = 1; i <= 16; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  for (int i = 0; i < 3000; i++) {
    motor(70, -70);  // rotate
    readSensors();
    for (int j = 1; j <= 16; j++) {
      int val = S[j];
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  for (int i = 1; i <= 16; i++) {
    T[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print("S"); Serial.print(i); Serial.print(" Threshold: ");
    Serial.println(T[i]);
  }

  motor(0, 0);
}

// --- Print Sensor Status (⬛⬜) ---
void printSensorStatus() {
  Serial.print("Sensors: ");
  for (int i = 1; i <= 16; i++) {
    if (S[i] > T[i]) {
      // Serial.print("⬛");
    } else {
      // Serial.print("⬜");
    }
  }
  Serial.println();
}

// --- Motor Driver for TB6612FNG ---
void motor(int a, int b) {
  // Left Motor (A side)
  if (a > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (a < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    a = -a;
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); // Brake
  }

  // Right Motor (B side)
  if (b > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (b < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    b = -b;
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); // Brake
  }

  analogWrite(enA, constrain(a, 0, 255));
  analogWrite(enB, constrain(b, 0, 255));
}
