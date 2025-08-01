// HC-05 Bluetooth Test using line follower 2-byte protocol

int val, cnt = 0, v[3];   // For Bluetooth data parsing
float Kp = 0, Ki = 0, Kd = 0;
bool onoff = false;
uint8_t multiP = 1, multiI = 1, multiD = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("Bluetooth PID Test Ready...");
}

void loop() {
  if (Serial.available()) {
    while (Serial.available() == 0); // Wait for full byte
    valuesread();
    processing();
  }
}

// Reads one byte and stores in buffer
void valuesread() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

// Interprets the 2-byte command format
void processing() {
  int a = v[1];
  int b = v[2];
  switch (a) {
    case 1:
      Kp = b;
      Serial.print("Kp set to: "); Serial.println(Kp);
      break;
    case 2:
      multiP = b;
      Serial.print("multiP set to: "); Serial.println(multiP);
      break;
    case 3:
      Ki = b;
      Serial.print("Ki set to: "); Serial.println(Ki);
      break;
    case 4:
      multiI = b;
      Serial.print("multiI set to: "); Serial.println(multiI);
      break;
    case 5:
      Kd = b;
      Serial.print("Kd set to: "); Serial.println(Kd);
      break;
    case 6:
      multiD = b;
      Serial.print("multiD set to: "); Serial.println(multiD);
      break;
    case 7:
      onoff = b;
      Serial.print("onoff set to: "); Serial.println(onoff);
      break;
    default:
      Serial.print("Unknown command: "); Serial.println(a);
  }
}










// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

// //--------Pin definitions for the TB6612FNG Motor Driver----
// #define AIN1 4
// #define BIN1 6
// #define AIN2 3
// #define BIN2 7
// #define PWMA 9
// #define PWMB 10
// #define STBY 5
// //------------------------------------------------------------

// #define s0 14  // A0
// #define s1 15  // A1
// #define s2 16  // A2
// #define s3 17  // A3

// bool isBlackLine = 1;
// unsigned int numSensors = 16;

// int P, D, I, previousError, PIDvalue;
// double error;
// int lsp, rsp;
// int lfSpeed = 220;
// int currentSpeed = 30;
// int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };

// int activeSensors;
// float Kp = 0.06;
// float Kd = 1.0;
// float Ki = 0.0;
// int onLine = 1;
// int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];

// unsigned long startMillis;
// double elapsedTime;

// // Bluetooth PID control variables
// int val, cnt = 0, v[3];
// bool onoff = true;  // Default: allow line following
// uint8_t multiP = 1, multiI = 1, multiD = 1;

// void setup() {
//   // Speed up ADC
//   sbi(ADCSRA, ADPS2);
//   cbi(ADCSRA, ADPS1);
//   cbi(ADCSRA, ADPS0);

//   Serial.begin(9600);  // Bluetooth uses 9600
//   Serial.println("Bluetooth Line Follower Ready...");

//   // Motor pins
//   pinMode(AIN1, OUTPUT);
//   pinMode(AIN2, OUTPUT);
//   pinMode(BIN1, OUTPUT);
//   pinMode(BIN2, OUTPUT);
//   pinMode(PWMA, OUTPUT);
//   pinMode(PWMB, OUTPUT);
//   pinMode(STBY, OUTPUT);
//   digitalWrite(STBY, HIGH);  // Enable motor driver

//   // Multiplexer select pins
//   pinMode(s0, OUTPUT);
//   pinMode(s1, OUTPUT);
//   pinMode(s2, OUTPUT);
//   pinMode(s3, OUTPUT);
// }

// void loop() {
//   // Handle Bluetooth commands
//   if (Serial.available()) {
//     while (Serial.available() == 0);
//     valuesread();
//     processing();
//   }

//   // Run only if Bluetooth "onoff" flag is set to 1
//   if (onoff) {
//     calibrate();  // Auto-calibrate once

//     while (onoff) {
//       startMillis = millis();

//       digitalWrite(s0, 0);
//       digitalWrite(s1, 0);
//       digitalWrite(s2, 0);
//       digitalWrite(s3, 0);

//       readLine();

//       if (currentSpeed < lfSpeed) currentSpeed++;

//       if (onLine == 1) {
//         linefollow();
//       } else {
//         if (error > 1000) {
//           motor(-100, 255);
//         } else if (error < -1000) {
//           motor(255, -100);
//         }
//       }

//       elapsedTime = millis() - startMillis;

//       // Check if new Bluetooth command was received
//       if (Serial.available()) {
//         valuesread();
//         processing();
//       }
//     }

//     motor(0, 0);  // Stop when off
//   }
// }

// // ----------------------- Core Logic --------------------------

// void calibrate() {
//   Serial.println("Calibrating...");

//   for (int i = 0; i < 16; i++) {
//     minValues[i] = sensorRead(i);
//     maxValues[i] = sensorRead(i);
//   }

//   unsigned long start = millis();
//   while (millis() - start < 3000) {
//     motor(70, -70);

//     for (int i = 0; i < 16; i++) {
//       int value = sensorRead(i);
//       if (value < minValues[i]) minValues[i] = value;
//       if (value > maxValues[i]) maxValues[i] = value;
//     }
//   }

//   for (int i = 0; i < 16; i++) {
//     threshold[i] = (minValues[i] + maxValues[i]) / 2;
//     Serial.print(threshold[i]); Serial.print(" ");
//   }
//   Serial.println();

//   motor(0, 0);
//   Serial.println("Calibration Done.");
// }

// void linefollow() {
//   error = 0;
//   activeSensors = 0;

//   for (int i = 0; i < 16; i++) {
//     if (sensorArray[i]) {
//       error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
//     }
//     activeSensors += sensorArray[i];
//   }

//   error = error / activeSensors;

//   P = error;
//   I += error;
//   D = error - previousError;

//   PIDvalue = (Kp * multiP * P) + (Ki * multiI * I) + (Kd * multiD * D);
//   previousError = error;

//   lsp = currentSpeed - PIDvalue;
//   rsp = currentSpeed + PIDvalue;

//   lsp = constrain(lsp, -100, 255);
//   rsp = constrain(rsp, -100, 255);

//   motor(lsp, rsp);
// }

// void readLine() {
//   onLine = 0;
//   for (int i = 0; i < 16; i++) {
//     int raw = sensorRead(i);
//     sensorValue[i] = isBlackLine
//                        ? map(raw, minValues[i], maxValues[i], 0, 1000)
//                        : map(raw, minValues[i], maxValues[i], 1000, 0);
//     sensorValue[i] = constrain(sensorValue[i], 0, 1000);
//     sensorArray[i] = sensorValue[i] > 500;
//     if (sensorArray[i]) onLine = 1;
//   }
// }

// int sensorRead(int sensor) {
//   digitalWrite(s0, sensor & 0x01);
//   digitalWrite(s1, sensor & 0x02);
//   digitalWrite(s2, sensor & 0x04);
//   digitalWrite(s3, sensor & 0x08);
//   return analogRead(A4);
// }

// void motor(int leftSpeed, int rightSpeed) {
//   if (leftSpeed > 0) {
//     digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
//   } else if (leftSpeed < 0) {
//     digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
//     leftSpeed = -leftSpeed;
//   } else {
//     digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
//   }

//   if (rightSpeed > 0) {
//     digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
//   } else if (rightSpeed < 0) {
//     digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
//     rightSpeed = -rightSpeed;
//   } else {
//     digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
//   }

//   analogWrite(PWMA, constrain(leftSpeed, 0, 255));
//   analogWrite(PWMB, constrain(rightSpeed, 0, 255));
// }

// // ----------------------- Bluetooth Handling ------------------------

// void valuesread() {
//   val = Serial.read();
//   cnt++;
//   v[cnt] = val;
//   if (cnt == 2) cnt = 0;
// }

// void processing() {
//   int a = v[1];
//   int b = v[2];
//   switch (a) {
//     case 1:
//       Kp = b;
//       Serial.print("Kp set to: "); Serial.println(Kp);
//       break;
//     case 2:
//       multiP = b;
//       Serial.print("multiP set to: "); Serial.println(multiP);
//       break;
//     case 3:
//       Ki = b;
//       Serial.print("Ki set to: "); Serial.println(Ki);
//       break;
//     case 4:
//       multiI = b;
//       Serial.print("multiI set to: "); Serial.println(multiI);
//       break;
//     case 5:
//       Kd = b;
//       Serial.print("Kd set to: "); Serial.println(Kd);
//       break;
//     case 6:
//       multiD = b;
//       Serial.print("multiD set to: "); Serial.println(multiD);
//       break;
//     case 7:
//       onoff = b;
//       Serial.print("onoff set to: "); Serial.println(onoff);
//       break;
//     default:
//       Serial.print("Unknown command: "); Serial.println(a);
//   }
// }



