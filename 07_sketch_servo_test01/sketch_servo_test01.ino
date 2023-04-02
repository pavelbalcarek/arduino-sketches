#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  // put your setup code here, to run once:
  myservo.attach(8);

}

void loop() {
  // put your main code here, to run repeatedly:
  myservo.write(90);
  delay(3000);
  myservo.write(180);
  delay(1000);
  myservo.write(0);
  delay(1000);
}
