#include <LiquidCrystal.h>

// Define pins for the line following sensors
const int NUM_SENSORS = 4; // Change this value to the number of sensors you are using
const int SENSOR_PINS[NUM_SENSORS] = {6, 5, 4, 7};
//const int SENSOR_PINS[NUM_SENSORS] = {6, 6, 6, 6};

// Define pins for the two motors
const int MOTOR1_IN1 = 10;
const int MOTOR1_IN2 = 11;
const int MOTOR2_IN1 = 9;
const int MOTOR2_IN2 = 5;

// Define base motor speed and PID constants
const int BASE_SPEED = 150;
const float Kp = 2;
const float Ki = 0.5;
const float Kd = 1;

// Variables for PID control
float lastError = 0;
float totalError = 0;

// Initialize the servo motor
const int SERVO_PIN = 6;
int servoPosition = 90;

void setup() {
  // Set up the line following sensors as inputs
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Set up the motor pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Set up the servo pin as an output
  pinMode(SERVO_PIN, OUTPUT);

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}

int customCeil(int num) {
   int den = 1;
   int inc = 0;

   while (num >= 10) {
     inc += num % 10;
     num /= 10;
     den *= 10;
   }

   return (num + (inc > 0)) * den;
}

void loop() {
  // Read the sensor values
  int sensorValues[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = customCeil(analogRead(SENSOR_PINS[i]));

    Serial.print(i);
    Serial.print("=");
    Serial.print(sensorValues[i]);
    Serial.print(",");
  }

  // Follow the line using PID control
  float error = calculateError(sensorValues);

  Serial.print("error=");
  Serial.println(error);

  totalError += error;
  float correction = Kp * error + Ki * totalError + Kd * (error - lastError);
  lastError = error;
  int motorSpeed1 = BASE_SPEED + correction;
  int motorSpeed2 = BASE_SPEED - correction;
  //setMotorSpeeds(motorSpeed1, motorSpeed2);
}

// Calculates the error between the current sensor values and the ideal values
float calculateError(int *sensorValues) {
  /*int idealValues[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    idealValues[i] = i * 1023 / (NUM_SENSORS - 1);
  }
  float weightedValues[NUM_SENSORS];
  float totalWeightedValue = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedValues[i] =
        (sensorValues[i] - idealValues[i]) * (i - (NUM_SENSORS - 1) / 2.0);
    totalWeightedValue += weightedValues[i];
  }
  return totalWeightedValue / 1000;*/
  float totalWeightedValue = 0;
  float totalValue = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    totalWeightedValue += sensorValues[i] * i * 1000;
    totalValue += sensorValues[i];
  }
  return totalWeightedValue / totalValue;
}

// Sets the speeds of the two motors
void setMotorSpeeds(int speed1, int speed2) {
  /*Serial.print("S1=");
  Serial.print(speed1);
  Serial.print(", ");
  Serial.print("S2=");
  Serial.println(speed2);*/
  // if (speed1 < 0) {
  //   digitalWrite(MOTOR1_IN1, LOW);
  //   digitalWrite(MOTOR1_IN2, HIGH);
  //   speed1 = -speed1;
  // } else {
  //   digitalWrite(MOTOR1_IN1, HIGH);
  //   digitalWrite(MOTOR1_IN2, LOW);
  // }
  // if (speed2 < 0) {
  //   digitalWrite(MOTOR2_IN1, LOW);
  //   digitalWrite(MOTOR2_IN2, HIGH);
  //   speed2 = -speed2;
  // } else {
  //   digitalWrite(MOTOR2_IN1, HIGH);
  //   digitalWrite(MOTOR2_IN2, LOW);
  // }
  // analogWrite(MOTOR1_IN1, speed1);
  // analogWrite(MOTOR2_IN1, speed2);
}

// Moves the servo motor to a specified position
void moveServo(int position) {
  if (position < 0) {
    position = 0;
  }
  if (position > 180) {
    position = 180;
  }
  servoPosition = position;
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(servoPosition * 11 +
                    500); // Convert position to pulse length
  digitalWrite(SERVO_PIN, LOW);
  delay(20);
}

// Finds the direction of the line using the line following sensors
int findLineDirection(int *sensorValues) {
  int leftSum = 0;
  int rightSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i < NUM_SENSORS / 2) {
      leftSum += sensorValues[i];
    } else {
      rightSum += sensorValues[i];
    }
  }
  if (leftSum > rightSum) {
    return -1;
  } else if (rightSum > leftSum) {
    return 1;
  } else {
    return 0;
  }
}
