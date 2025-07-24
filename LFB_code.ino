#include <Arduino.h>

#define IN1 11
#define IN2 10
#define IN3 3
#define IN4 9
#define enA 6
#define enB 5

#define S0 14 // A0 
#define S1 15 // A1 
#define S2 16 // A2 
#define S3 17 // A3 
// IR Sensor Analog Pin
#define sensorPin A4

float Kp = 0.15;      
float Ki = 0.0001;     
float Kd = 0.3;       

int baseSpeed = 180;    
int maxSpeed = 220;     
int minSpeed = 80;

//S is for sensors and T is Threshold val
int S[17];       
byte T[17];          
const int positionWeights[17] PROGMEM = {0, -800, -700, -600, -500, -400, -300, -200, -100, 100, 200, 300, 400, 500, 600, 700, 800};

int error = 0;
int previousError = 0;
float integral = 0;

void setup() 
{
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);

  delay(1000);
  calibrate();
}

void loop() 
{
  unsigned long loopStart = micros();
  
  readSensors(); 
  int position = calculatePosition();
  error = position;
  
  if (isSharpTurn()) 
  {
    handleSharpTurn();
  } 
  else 
  {
    smoothLineFollowing();
  }
  
  // Maintain ~50Hz update rate (20ms) for Nano(dont change this)
  while (micros() - loopStart < 20000) { 
    delayMicroseconds(500); 
  }
}

void readSensors() 
{
  for (byte i = 1; i <= 16; i++) 
  {
    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, i & 0x02);
    digitalWrite(S2, i & 0x04);
    digitalWrite(S3, i & 0x08);
    
    delayMicroseconds(100);  // min stable reading time
    S[i] = analogRead(sensorPin) >> 2; // Convert 10-bit to 8-bit (0-255)
  }
}

int calculatePosition() 
{
  int weightedSum = 0;
  int sensorSum = 0;
  byte activeSensors = 0;
  
  for (byte i = 1; i <= 16; i++) 
  {
    if (S[i] > T[i]) 
    {
      weightedSum += pgm_read_word(&positionWeights[i]) * S[i];
      sensorSum += S[i];
      activeSensors++;
    }
  }
  
  if (activeSensors == 0) 
  {
    return previousError > 0 ? 800 : -800; 
  }
  
  return weightedSum / sensorSum;
}

void smoothLineFollowing() 
{
  // Calculate PID
  int P = error;
  integral += error;
  integral = constrain(integral, -500, 500);
  int D = error - previousError;
  
  int PIDvalue = (Kp * P) + (Ki * integral) + (Kd * D);
  
  int speedReduction = map(abs(error), 0, 800, 0, baseSpeed-minSpeed);
  int currentSpeed = baseSpeed - speedReduction;
  
  // Motor control with PWM constraints
  byte leftSpeed = constrain(currentSpeed - PIDvalue, minSpeed, maxSpeed);
  byte rightSpeed = constrain(currentSpeed + PIDvalue, minSpeed, maxSpeed);
  
  setMotors(leftSpeed, rightSpeed);
  previousError = error;
}

bool isSharpTurn() 
{
  byte leftCount = 0, rightCount = 0;
  
  // check outermost 2 sensors each side for speed(also if bot wobbles much increase sensor count from each end)
  if (S[1] > T[1] || S[2] > T[2]) leftCount++;
  if (S[15] > T[15] || S[16] > T[16]) rightCount++;
  
  return (leftCount >= 1 || rightCount >= 1);
}

void handleSharpTurn() 
{
  // 90° turn 
  const byte turnSpeed = baseSpeed * 0.6;
  
  if (S[1] > T[1] || S[2] > T[2]) 
  {
    // Sharp left turn
    setMotors(-turnSpeed, turnSpeed);
    delay(50); // min turn time
  } 
  else 
  {
    // Sharp right turn
    setMotors(turnSpeed, -turnSpeed);
    delay(50);
  }
}

void setMotors(byte leftSpeed, byte rightSpeed) 
{
  // Left motor 
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
  analogWrite(enA, abs(leftSpeed));
  
  // Right motor
  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
  analogWrite(enB, abs(rightSpeed));
}

void calibrate() 
{
  // Rotate to find thresholds
  setMotors(90, -90);
  
  for (byte n = 0; n < 100; n++) 
  { 
    readSensors();
    for (byte i = 1; i <= 16; i++) {
      // adjusting threshold with 20% margin
      T[i] = (S[i] * 0.8); 
    }
    delay(20);
  }
  setMotors(0, 0);
  
  for (byte i = 1; i <= 16; i++) 
  {
    Serial.print("S"); Serial.print(i); 
    Serial.print(": "); Serial.println(T[i]);
  }
}