//int sensorPin = 7;
int sensorPinLeft = 5;
int sensorPinRight = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(3000);

}

void loop() {
  // put your main code here, to run repeatedly:
  int valLeft = analogRead(sensorPinLeft);
  int valRight = analogRead(sensorPinRight);
  Serial.print("L:");
  Serial.print(valLeft);
  Serial.print("R:");
  Serial.println(valRight);


  delay(100);
}
