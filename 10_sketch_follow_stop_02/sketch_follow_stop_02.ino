// motor
const int A1A_DIR = 10;      	//define pin 6 for A1A_DIR (direction)
const int A1B_SPEED = 11;      	//define pin 7 for A1B_SPEED (speed)

const int B1A_DIR = 9;      	//define pin 4 for B1A_DIR (direction)
const int B1B_SPEED = 5;      	//define pin 5 for B1B_SPEED (speed)

// #define SPEED 125
#define SPEED_FORWARD 125
#define SPEED_TURN 125
int SPEED = SPEED_FORWARD;

// HC-SR04 sensor
const int ECHO_PIN = 3; // attach pin D2 Arduino to pin Echo of HC-SR04
const int TRIG_PIN = 2; //attach pin D3 Arduino to pin Trig of HC-SR04

// IR sensors (analog)
// #define REMOVE_NOISE
const int IR_LEFT = 6;
const int IR_LEFT_POWER = 12;
const int IR_RIGHT = 7;
const int IR_RIGHT_POWER = 13;
const int IR_LIMIT = 512;

//#define BT_ENABLED
#define SERIAL_ENABLED

// Arduino Bluetooth modul HC-05
#ifdef BT_ENABLED
// nastavení propojovacích pinů Bluetooth
const int RX = 6;
const int TX = 7;
// připojení knihovny SoftwareSerial
#include <SoftwareSerial.h>
// inicializace Bluetooth modulu z knihovny SoftwareSerial
SoftwareSerial bluetooth(TX, RX);
#endif

int irLeftValue = 0;
int irRightValue = 0;

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int counter = 0;
char modeCurr = 's';
char modeLast = 's';

int active = 0;
int loopCounter = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(B1A_DIR, OUTPUT); // define pin as output
  pinMode(B1B_SPEED, OUTPUT);

  pinMode(A1A_DIR, OUTPUT);
  pinMode(A1B_SPEED, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an INPUT

  digitalWrite(B1A_DIR, LOW);
  digitalWrite(B1B_SPEED, LOW);
  digitalWrite(A1A_DIR, LOW);
  digitalWrite(A1B_SPEED, LOW);

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  Serial.print("IR_LIMIT = ");
  Serial.print(IR_LIMIT);
  Serial.print(", ");
  Serial.print("LOW = ");
  Serial.print(LOW);
  Serial.print(", ");
  Serial.print("HIGH = ");
  Serial.println(HIGH);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(IR_LEFT_POWER, OUTPUT);
  pinMode(IR_RIGHT_POWER, OUTPUT);

#ifdef BT_ENABLED
  // zahájení komunikace s Bluetooth modulem
  // skrze Softwarovou sériovou linku rychlostí 9600 baud
  bluetooth.begin(9600);
  bluetooth.println("Arduino zapnuto, test Bluetooth..");
#endif
  
  delay(5000);
}

unsigned long start = 0;
unsigned long currentModeStart = 0;
boolean printOnStop = true;
void loop() {

  start = millis();

  calculateDistance();

  /*irLeftValue = analogRead(IR_LEFT);
  irRightValue = analogRead(IR_RIGHT);*/
  irLeftValue = readIrSensor(IR_LEFT, IR_LEFT_POWER);
  irRightValue = readIrSensor(IR_RIGHT, IR_RIGHT_POWER);

  /*if (active == 0) {
    readActiveFromBluetooth();
    if (modeCurr != "s") {
      stop();    
    }
    return;    
  }*/

  if (counter > 0 ){
    delay(1000);
    return;
  }

  // if (distance < 5) {
  //   counter++;
  //   return;
  // }

  if (distance < 10) {
    printDebug();
    stop();
    counter++;
    delay(1000);
    return;
  }

  startModeChange();
  if (irLeftValue < IR_LIMIT && irRightValue < IR_LIMIT) {
    modeCurr = 'f';
    SPEED = SPEED_FORWARD;
    forward();
  } else if (irLeftValue >= IR_LIMIT && irRightValue < IR_LIMIT) {
    modeCurr = 'l';
    resetMotor();
    SPEED = SPEED_TURN;
    turnLeft();
  } else if (irLeftValue < IR_LIMIT && irRightValue >= IR_LIMIT) {
    modeCurr = 'r';
    resetMotor();
    SPEED = SPEED_TURN;
    turnRight();
  } else {
    modeCurr = 's';
    stop();
  }

  if (modeCurr != modeLast) {
    printDebug();
    modeLast = modeCurr;
    currentModeStart = 0;
  }

  // Serial.print("D: ");
  // Serial.print(" L = ");
  // Serial.print(irLeftValue);
  // Serial.print(" R = ");
  // Serial.print(irRightValue);
  // Serial.print(" loop = ");
  // Serial.println(millis() - start);
}

int readIrSensor(int analogPin, int powerPin) {
#ifdef REMOVE_NOISE
  digitalWrite(powerPin, HIGH);   // Turning ON LED
  delayMicroseconds(500);         //wait
  int a = analogRead(analogPin);  //take reading from photodiode(pin A3) :noise+signal
  digitalWrite(powerPin, LOW);    //turn Off LED
  delayMicroseconds(500);         //wait
  int b = analogRead(analogPin);  // again take reading from photodiode :noise

#ifdef SERIAL_ENABLED
  if (analogPin == 6) {
  Serial.print(analogPin);
  Serial.print(": ");
  Serial.print(a);
  Serial.print(" - ");
  Serial.print(b);
  Serial.print(" = ");
  Serial.println(a - b);
  }
#endif

  return a - b;                   //taking differnce:[ (noise+signal)-(noise)] just signal
#else

#ifdef SERIAL_ENABLED
  if (analogPin == 6) {
  Serial.print(analogPin);
  Serial.print(": ");
  Serial.println(analogRead(analogPin));
  }
#endif

  return analogRead(analogPin);
#endif
}

void startModeChange() {
  if (currentModeStart == 0) {
    currentModeStart = millis();
  }
}

void printDebug() {
#ifdef BT_ENABLED
    printDebugToBluetooth();
#endif
#ifdef SERIAL_ENABLED
    printDebugToSerial();
#endif
}

void printDebugToSerial() {
    Serial.print("Mode ");
    Serial.print(modeLast);
    Serial.print(" -> ");
    Serial.print(modeCurr);
    Serial.print(", duration? ");
    Serial.print(millis() - currentModeStart);
    Serial.print(", values - L: ");
    Serial.print(irLeftValue);
    Serial.print(", R: ");
    Serial.println(irRightValue);
}

#ifdef BT_ENABLED
void printDebugToBluetooth() {
    bluetooth.print("Mode ");
    bluetooth.print(modeLast);
    bluetooth.print(" -> ");
    bluetooth.print(modeCurr);
    bluetooth.print(", active? ");
    bluetooth.print(active);
    bluetooth.print(", values - L: ");
    bluetooth.print(irLeftValue);
    bluetooth.print(", R: ");
    bluetooth.println(irRightValue);
}
#endif

#ifdef BT_ENABLED
void readActiveFromBluetooth() {

  if (loopCounter <= 100) {
    loopCounter++;
    return;    
  }

  loopCounter = 0;
  byte BluetoothData;
  // kontrola Bluetooth komunikace, pokud je dostupná nová
  // zpráva, tak nám tato funkce vrátí počet jejích znaků
  if (bluetooth.available() > 0) {
    // načtení prvního znaku ve frontě do proměnné
    BluetoothData=bluetooth.read();
    // dekódování přijatého znaku
    switch (BluetoothData) {
      // každý case obsahuje dekódování jednoho znaku
      case '0':
        active = 0;
        break;
      case '1':
        active = 1;
        break;
      case '\r':
        // přesun na začátek řádku - znak CR
        break;
      case '\n':
        // odřádkování - znak LF
        break;
      default:
        // v případě přijetí ostatních znaků
        // vytiskneme informaci o neznámé zprávě
        //bluetooth.println("Neznamy prikaz.");
        break;
    }
  }
  delayMicroseconds(50);
}
#endif

void avoidObstacle() {
  // TODO Detection with servo help
  bool avoidLeft = true;
  turnLeft();
  delay(250);
  forward();
  delay(500);
  turnRight();
  delay(250);
  turnLeft();
  delay(250);
}

void forward(){  //forword  
	digitalWrite(A1A_DIR, LOW);  //Right Motor backword Pin 
  analogWrite(A1B_SPEED, SPEED); //Right Motor forword Pin 
	digitalWrite(B1A_DIR, LOW);  //Left Motor backword Pin 
	analogWrite(B1B_SPEED, SPEED); //Left Motor forword Pin 
}

void turnRight(){ //turnRight
  digitalWrite(A1A_DIR, LOW); //Right Motor backword Pin  
  analogWrite(A1B_SPEED, SPEED);  //Right Motor forword Pin 
  digitalWrite(B1A_DIR, LOW); //Left Motor forword Pin 
  analogWrite(B1B_SPEED, LOW);  //Left Motor backword Pin 
}

void turnLeft(){ //turnLeft
  digitalWrite(A1A_DIR, LOW); //Right Motor backword Pin  
  analogWrite(A1B_SPEED, LOW);  //Right Motor forword Pin 
  digitalWrite(B1A_DIR, LOW); //Left Motor forword Pin 
  analogWrite(B1B_SPEED, SPEED);  //Left Motor backword Pin 
}

void stop(){ //stop
  digitalWrite(A1A_DIR, LOW); //Right Motor backword Pin 
	analogWrite(A1B_SPEED, LOW); //Right Motor forword Pin 
	digitalWrite(B1A_DIR, LOW); //Left Motor forword Pin 
  analogWrite(B1B_SPEED, LOW); //Left Motor backword Pin 
}

void resetMotor() {
  stop();
  delayMicroseconds(10);
}

/*
   @motorA
   activation rotation of motor A
   d is the direction
   R = Right
   L = Left
*/
void motorA(char d)
{
  if (d == 'R') {
    digitalWrite(A1A_DIR, LOW);
    digitalWrite(A1B_SPEED, HIGH);
  } else if (d == 'L') {
    digitalWrite(A1A_DIR, HIGH);
    digitalWrite(A1B_SPEED, LOW);
  } else {
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF
    digitalWrite(A1A_DIR, LOW);
    digitalWrite(A1B_SPEED, LOW);
  }
}// motorA end


/*
   @motorB
   activation rotation of motor B
   d is the direction
   R = Right
   L = Left
*/
void motorB(char d)
{

  if (d == 'R') {
    digitalWrite(B1A_DIR, LOW);
    digitalWrite(B1B_SPEED, HIGH);
  } else if (d == 'L') {
    digitalWrite(B1A_DIR, HIGH);
    digitalWrite(B1B_SPEED, LOW);
  } else {
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF
    digitalWrite(B1A_DIR, LOW);
    digitalWrite(B1B_SPEED, LOW);
  }

}// motorB end


void calculateDistance() {
  // Clears the trigPin condition
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
#ifdef SERIAL_ENABLED
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");
#endif
}





void basic() {
  
  if (counter > 0) {
    return;
  }

  if (distance < 20) {
    counter++;
    motorA('O');// Turn motor A OFF
    motorB('O');// Turn motor B OFF
    delay(2000);
    return;
  }

  int valLeft = analogRead(IR_LEFT);
  int valRight = analogRead(IR_RIGHT);

  if (valLeft < IR_LIMIT && valRight < IR_LIMIT) { 
    forward();
  } else if (valLeft > IR_LIMIT && valRight < IR_LIMIT) {
    turnRight();
  } else if (valLeft < IR_LIMIT && valRight > IR_LIMIT) {
    turnLeft();
  } else {
    stop();
  }
  


  /*if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){forward();}   //if Right Sensor and Left Sensor are at White color then it will call forword function
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();} //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();}  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){Stop();} //if Right Sensor and Left Sensor are at Black color then it will call Stop function
  */
  

  /*digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  delay(500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
  delay(1500);                       // wait for half a second
  */


  /*
  if (distance >= 20) {
    motorA('R');// Turn motor A to RIGHT
    motorB('L');// Turn motor B to RIGHT
    delay(100);
    return;
  } else {
    counter++;
    motorA('O');// Turn motor A OFF
    motorB('O');// Turn motor B OFF
    delay(2000);
  }

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  delay(1500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off

  */
  /*motorA('R');// Turn motor A to RIGHT
    motorB('R'); // Turn motor A to RIGHT
    delay(2000);
    motorA('L');// Turn motor A to LEFT
    motorB('L');// Turn motor B to LEFT
    delay(3000);
    motorA('O');// Turn motor A OFF
    motorB('O');// Turn motor B OFF
    delay(5000);*/
}
