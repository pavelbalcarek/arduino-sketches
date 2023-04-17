#include <Servo.h>

int sensor_count = 4;
long sensor[] = {A4, A3, A2, A1}; // leftmost - A4, rightmost - A0

// speeds
int speed = 200;

float p;
float i;
float d;
float lp;
float error;
float correction;
float sp;

float Kp = 5;  // dummy
float Ki = 0;  // dummy
float Kd = 40; //(Kp-1)*10

int pos;
long sensor_average;
int sensor_sum;

#define mRf 8
// pwm
#define mRb 6
#define mLf 5
// pwm
#define mLb 7

#define Servo_myservo 11;

Servo myservo;

void pid_calc();
void calc_turn();
void motor_drive(int, int);

void setup()
{
  // motory
  pinMode(mRf, OUTPUT);
  pinMode(mRb, OUTPUT);
  pinMode(mLf, OUTPUT);
  pinMode(mLb, OUTPUT);

  // čidla
  // sensors
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  Serial.begin(9600);
}


void loop()
{
  Serial.println(sensor_sum);
  pid_calc();
  calc_turn();
  // delay(500);
}

void pid_calc()
{
  sensor_average = 0;
  sensor_sum = 0;
  i = 0;

  for (int i = 0; i <= sensor_count; i++)
  {
    sensor[i] = analogRead(i);

    /*Serial.print(i);
    Serial.print("=");
    Serial.print(sensor[i]);
    Serial.print(",");*/
    sensor_average = sensor[i] * i * 1000; // weighted mean
    sensor_sum += sensor[i];
  }

  pos = int(sensor_average / sensor_sum);
  p = pos - 2000;
  i += p;
  d = p - lp;

  lp = p;

  correction = int(Kp * p + Ki * i + Kd * d);
  correction = p * Kp + i * Ki + d * Kd;
  const int max = speed / 2 + 30;
  if (correction > max)
    correction = max;
  if (correction < -max)
    correction = (-1 * max);
}

void calc_turn()
{
  if (correction < 0) // left
    set_motors(max + correction, max);
  else // right
    set_motors(max, max - correction);
}

void motor_drive(int right, int left)
{

  Serial.print("correction");
  Serial.print("=");
  Serial.print(correction);

  Serial.print(",pos");
  Serial.print("=");
  Serial.print(pos);

  Serial.print(",left");
  Serial.print("=");
  Serial.print(left);

  Serial.print(",right");
  Serial.print("=");
  Serial.println(right);

  if (right > 0)
  {
    analogWrite(mRf, right);
    analogWrite(mRb, 0);
  }
  else
  {
    analogWrite(mRf, 0);
    analogWrite(mRb, abs(right));
  }

  if (left > 0)
  {
    analogWrite(mLf, left);
    analogWrite(mLb, 0);
  }
  else
  {
    analogWrite(mLf, 0);
    analogWrite(mLb, abs(left));
  }
}
