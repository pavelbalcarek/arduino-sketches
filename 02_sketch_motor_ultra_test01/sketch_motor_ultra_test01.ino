const int A1A = 6;      //define pin 6 for A1A
const int A1B = 7;      //define pin 7 for A1B

const int B1A = 4;      //define pin 4 for B1A
const int B1B = 5;      //define pin 5 for B1B

const int ECHO_PIN = 2; // attach pin D2 Arduino to pin Echo of HC-SR04
const int TRIG_PIN = 3; //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int counter = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(B1A, OUTPUT); // define pin as output
  pinMode(B1B, OUTPUT);

  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an INPUT

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  
  delay(3000);
}

void loop() {
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

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  delay(500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
  delay(1500);                       // wait for half a second

  motorA('R');// Turn motor A to RIGHT
  delay(500);
  motorA('L');// Turn motor A to LEFT
  delay(500);
  motorA('O');// Turn motor A OFF
  delay(2000);

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on

  motorB('R');// Turn motor B to RIGHT
  delay(500);
  motorB('L');// Turn motor B to LEFT
  delay(500);
  motorB('O');// Turn motor B OFF
  delay(2000);

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
  delay(1500);                       // wait for half a second

  counter++;
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
    digitalWrite(A1A, LOW);
    digitalWrite(A1B, HIGH);
  } else if (d == 'L') {
    digitalWrite(A1A, HIGH);
    digitalWrite(A1B, LOW);
  } else {
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF
    digitalWrite(A1A, LOW);
    digitalWrite(A1B, LOW);
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
    digitalWrite(B1A, LOW);
    digitalWrite(B1B, HIGH);
  } else if (d == 'L') {
    digitalWrite(B1A, HIGH);
    digitalWrite(B1B, LOW);
  } else {
    //Robojax.com L9110 Motor Tutorial
    // Turn motor OFF
    digitalWrite(B1A, LOW);
    digitalWrite(B1B, LOW);
  }

}// motorB end
