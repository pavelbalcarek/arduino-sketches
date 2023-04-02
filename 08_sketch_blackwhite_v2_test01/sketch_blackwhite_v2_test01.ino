#define IR_A 1

#define IR_A_OR 5

int analog;
int analogOr;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(IR_A, INPUT);
  pinMode(IR_A_OR, INPUT);
}

void loop() {
    // načtení hodnoty analogového vstupu
  analog = analogRead(IR_A);
  analogOr = analogRead(IR_A_OR);
  Serial.print("Analogova hodnota: ");
  Serial.print(analog);
  Serial.print(" / ");
  Serial.println(analogOr);
  // vyčkej 1 s kvůli zbytečnému množství tisknutých znaků
  delay(1000);
}
