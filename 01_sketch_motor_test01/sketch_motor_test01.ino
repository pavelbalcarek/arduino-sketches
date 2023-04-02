const int A1A = 6;//define pin 2 for A1A
const int A1B = 7;//define pin 3 for A1B

const int B1A = 4;//define pin 8 for B1A
const int B1B = 5;//define pin 9 for B1B


int counter = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(B1A, OUTPUT); // define pin as output
  pinMode(B1B, OUTPUT);

  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  delay(3000);
}

void loop() {
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
