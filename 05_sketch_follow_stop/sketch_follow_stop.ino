// motor
const int A1A_DIR = 6;      	//define pin 6 for A1A_DIR (direction)
const int A1B_SPEED = 7;      	//define pin 7 for A1B_SPEED (speed)

const int B1A_DIR = 4;      	//define pin 4 for B1A_DIR (direction)
const int B1B_SPEED = 5;      	//define pin 5 for B1B_SPEED (speed)

#define SPEED 200

// HC-SR04 sensor
const int ECHO_PIN = 2; // attach pin D2 Arduino to pin Echo of HC-SR04
const int TRIG_PIN = 3; //attach pin D3 Arduino to pin Trig of HC-SR04

// IR sensors
const int IR_LEFT = 5;
const int IR_RIGHT = 7;
const int IR_LIMIT = 300;

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int counter = 0;

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
  
  delay(3000);
}

void loop() {
  if (counter > 0) {
    return;
  }

  forward();
  delay(1000);
  stop();
  delay(1000);

  counter++;  
}


void forward(){  //forword  
	digitalWrite(A1A_DIR, LOW);  //Right Motor backword Pin 
  digitalWrite(A1B_SPEED, SPEED); //Right Motor forword Pin 
	digitalWrite(B1A_DIR, LOW);  //Left Motor backword Pin 
	digitalWrite(B1B_SPEED, SPEED); //Left Motor forword Pin 
}

void turnRight(){ //turnRight
  digitalWrite(A1B_SPEED, LOW);  //Right Motor forword Pin 
  digitalWrite(A1A_DIR, HIGH); //Right Motor backword Pin  
  digitalWrite(B1B_SPEED, LOW);  //Left Motor backword Pin 
  digitalWrite(B1A_DIR, HIGH); //Left Motor forword Pin 
}

void turnLeft(){ //turnLeft
  digitalWrite(A1B_SPEED, HIGH); //Right Motor forword Pin 
  digitalWrite(A1A_DIR, LOW);  //Right Motor backword Pin 
  digitalWrite(B1B_SPEED, HIGH); //Left Motor backword Pin 
  digitalWrite(B1A_DIR, LOW);  //Left Motor forword Pin 
}

void stop(){ //stop
	digitalWrite(A1B_SPEED, LOW); //Right Motor forword Pin 
	digitalWrite(A1A_DIR, LOW); //Right Motor backword Pin 
	digitalWrite(B1B_SPEED, LOW); //Left Motor backword Pin 
	digitalWrite(B1A_DIR, LOW); //Left Motor forword Pin 
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


void basic() {
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
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
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
