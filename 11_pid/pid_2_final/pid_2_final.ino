#include <Servo.h>

int sensorCount = 4;
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
/*int pos = 0;
int pTrig = 2;
int pEcho = 3;
int errorOld;
int Isuma;
int vzdalenost;
int error;
int pid_value;

// koeficienty pro pid - otočení
float KpO = 10, KiO = 0, KdO = 20;

// koeficienty pro pid - jízda po čáře
float Kp = 4, Ki = 0.005, Kd = 20;

// proměnné pro normalizaci
int cl1_norma, cl2_norma, cl3_norma;
int cp1_norma, cp2_norma, cp3_norma;

// proměnné pro automatické načtení
int cl1_min = 1023, cl1_max = 0, cl2_min = 1023, cl2_max = 0, cl3_min = 1023, cl3_max = 0;
int cp1_min = 1023, cp1_max = 0, cp2_min = 1023, cp2_max = 0, cp3_min = 1023, cp3_max = 0;
int auto_p1, auto_p2, auto_p3, auto_l1, auto_l2, auto_l3;

// váhy čidel
int c1_vaha = 1, c2_vaha = 2, c3_vaha = 4;
*/
void pid_calc();
void calc_turn();
void motor_drive(int, int);

// automatické načtení max a min hodnot pro každé čidlo
/*void auto_nacteni() {

  digitalWrite(mRf, LOW);
  analogWrite(mRb, 100);
  digitalWrite(mLf, HIGH);
  analogWrite(mLb, 155);
  while (millis() < 1200 or analogRead(cidl1) < 200) {
    auto_l1 = analogRead(cidl1);
    auto_l2 = analogRead(cidl2);
    auto_l3 = analogRead(cidl3);
    auto_p1 = analogRead(cidp1);
    auto_p2 = analogRead(cidp2);
    auto_p3 = analogRead(cidp3);

    // Načtení hodnot max, min
    // levé čidlo
    if (auto_l1 > cl1_max) {
      cl1_max = auto_l1;
    }
    else if (auto_l1 < cl1_min) {
      cl1_min = auto_l1;
    }
    if (auto_l2 > cl2_max) {
      cl2_max = auto_l2;
    }
    else if (auto_l2 < cl2_min) {
      cl2_min = auto_l2;
    }
    if (auto_l3 > cl3_max) {
      cl3_max = auto_l3;
    }
    else if (auto_l3 < cl3_min) {
      cl3_min = auto_l3;
    }

    // pravé čidlo
    if (auto_p1 > cp1_max) {
      cp1_max = auto_p1;
    }
    else if (auto_p1 < cp1_min) {
      cp1_min = auto_p1;
    }
    if (auto_p2 > cp2_max) {
      cp2_max = auto_p2;
    }
    else if (auto_p2 < cp2_min) {
      cp2_min = auto_p2;
    }
    if (auto_p3 > cp3_max) {
      cp3_max = auto_p3;
    }
    else if (auto_p3 < cp3_min) {
      cp3_min = auto_p3;
    }
  }
  digitalWrite(mLf, LOW);
  analogWrite(mLb, LOW);
  digitalWrite(mRf, LOW);
  analogWrite(mRb, LOW);
}
*/
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
  /*
    // ultrazvuk
    pinMode(pTrig, OUTPUT);
    pinMode(pEcho, INPUT);

    // servo
    myservo.attach(11);

    // ostatní
    Serial.begin(9600);
    errorOld = 0;
    Isuma = 0;
    auto_nacteni();*/
}
/*
// ovládání ultrazvuku
int dalkomer() {
  digitalWrite(pTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(pTrig, LOW);
  long odezva = pulseIn(pEcho, HIGH, 5000);
  long vzdalenost = int(odezva / 58.31);
  if (vzdalenost == 0) {
    vzdalenost = 35;
  }
  Serial.println(vzdalenost);
  return vzdalenost;
}*/
/*
// výpočet pid hodnoty
int pid(int error, float Kp, float Ki, float Kd) {
  // P-složka
  int Pkorekce = int(Kp * error);

  // I-složka
  if (Isuma * error < 0) {
    Isuma = 0;
  } else {
    Isuma += error;
  }
  int Ikorekce = int(Isuma * Ki);

  // D-složka
  int Dkorekce = int(Kd * (error - errorOld));
  errorOld = error;

  //PID-regulace
  int pid = Pkorekce + Ikorekce + Dkorekce;

  return pid;
}

// normalizace čidel
int normalizace(int cidlo, int c_min, int c_max) {
  int cid = analogRead(cidlo);
  int c_norma = int(((cid - c_min) * 100.0) / (c_max - c_min));
  c_norma = constrain(c_norma, 0, 100);
  return c_norma;
}

// ovládání motorů pro pid
void motory(int smer, int rychlost = 100) {
  int ml = rychlost - smer;
  int mp = rychlost + smer;

  // Levý motor
  if (ml >= 0) {
    if (ml > rychlost) {
      ml = rychlost;
    }
    digitalWrite(mLf, LOW);
  } else {
    if (ml < -rychlost) {
      ml = -rychlost;
    }
    digitalWrite(mLf, HIGH);
    ml = abs(ml);
    ml = rychlost - ml;
  }

  // Pravý motor
  if (mp >= 0) {
    if (mp > rychlost) {
      mp = rychlost;
    }
    digitalWrite(mRf, LOW);
  } else {
    if (mp < -rychlost) {
      mp = -rychlost;
    }
    digitalWrite(mRf, HIGH);
    mp = abs(mp);
    mp = rychlost - mp;
  }

  analogWrite(mRb, mp);
  analogWrite(mLb, ml);
}*/
/*void loop() {

  // Normalizace čidel
  cl1_norma = normalizace(cidl1, cl1_min, cl1_max);
  cl2_norma = normalizace(cidl2, cl2_min, cl2_max);
  cl3_norma = normalizace(cidl3, cl3_min, cl3_max);
  cp1_norma = normalizace(cidp1, cp1_min, cp1_max);
  cp2_norma = normalizace(cidp2, cp2_min, cp2_max);
  cp3_norma = normalizace(cidp3, cp3_min, cp3_max);

  // Vážený průměr čidel
  int cl_prumer = int((cl1_norma * -c1_vaha + cl2_norma * -c2_vaha + cl3_norma * -c3_vaha) / (-c1_vaha + -c2_vaha + -c3_vaha));
  int cp_prumer = int((cp1_norma * c1_vaha + cp2_norma * c2_vaha + cp3_norma * c3_vaha) / (c1_vaha + c2_vaha + c3_vaha));

  // pid regulace jízdy po čáře
  error = int((cl_prumer - cp_prumer) / 2);
  pid_value = pid(error, Kp, Ki, Kd);
  motory(pid_value, 80);

  // pid regulace otočení
  vzdalenost = dalkomer();
  if (vzdalenost < 15 and vzdalenost != 0) {
    // vynuluj hodnoty pid
    Isuma = 0;
    errorOld = 0;
    motory(0, 0);
    // otoč ultrazvuk
    myservo.write(90);
    // otoč se doleva
    digitalWrite(mRf, LOW);
    analogWrite(mRb, 100);
    digitalWrite(mLf, HIGH);
    analogWrite(mLb, 155);
    delay(400);
    motory(0, 0);
    // pid regulace pro otočení
    while (analogRead(cidl1) < 200) {
      vzdalenost = dalkomer();
      error = 20 - vzdalenost;
      pid_value = pid(error, KpO, KiO, KdO);
      motory(pid_value, 200);
    }
    motory(0, 0);
    // otoč se doleva
    digitalWrite(mRf, LOW);
    analogWrite(mRb, 100);
    digitalWrite(mLf, HIGH);
    analogWrite(mLb, 155);
    delay(300);
    motory(0, 0);
    myservo.write(0);
  }


  for (pos = 0; pos <= 180; pos += 10) {
    myservo.write(pos);
    int cm = int(dalkomer());
    Serial.print ("Vzdalenost v cm: ");
    Serial.print (cm);
    Serial.print("\t");
    Serial.print ("Úhel natočení je: ");
    Serial.print (pos);
    Serial.print("\n");
    delay(1000);
  }
  for (pos = 180; pos >= 0; pos -= 10) {
    myservo.write(pos);
    int cm = int(dalkomer());
    Serial.print ("Vzdalenost v cm: ");
    Serial.print (cm);
    Serial.print("\t");
    Serial.print ("Úhel natočení je: ");
    Serial.print (pos);
    Serial.print("\n");
    delay(1000);
  }
}*/
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

  for (int i = 0; i <= 3; i++)
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
