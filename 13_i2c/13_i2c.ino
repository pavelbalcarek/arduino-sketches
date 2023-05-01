/*#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  
}

void scanI2CBus(byte from_addr, byte to_addr, void(*callback)(byte address, byte result) ) 
{
  byte rc;
  byte data = 0;
  for( byte addr = from_addr; addr <= to_addr; addr++ ) {
    rc = twi_writeTo(addr, &data, 0, 1, 0);
    callback( addr, rc );
  }
}

void scanFunc( byte addr, byte result ) {
  Serial.print("addr: ");
  Serial.print(addr,DEC);
  Serial.print( (result==0) ? " found!":"       ");
  Serial.print( (addr%4) ? "t":"n");
}

byte start_address = 8;       
byte end_address = 119;       

void setup()
{
    Wire.begin();
    Serial.begin(9600);                
    Serial.println("nI2CScanner ready!");

    Serial.print("starting scanning of I2C bus from ");
    Serial.print(start_address,DEC);
    Serial.print(" to ");
    Serial.print(end_address,DEC);
    Serial.println("...");

    scanI2CBus( start_address, end_address, scanFunc );

    Serial.println("ndone");
    pinMode(13, OUTPUT);
}

void loop() 
{
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(300);
}*/

 // --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//


/*
#include <Wire.h>


void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}*/

// 2x LCD displej
// navody.dratek.cz

// knihovna pro LCD
//#include <LiquidCrystal.h>
// knihovny pro LCD přes I2C
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// nastavení adresy I2C (0x27 v mém případě),
// a dále počtu znaků a řádků LCD, zde 20x4
//LiquidCrystal_I2C lcdI2C(0x27, 16, 2);
// inicializace pinů, lze vyměnit za jiné volné
// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
  Serial.begin(9600);
  for( uint8_t addr = 8; addr <= 128; addr++ ) {
    Serial.println(addr);
    LiquidCrystal_I2C lcdI2C(addr, 16, 2);
   // inicializace LCD
    Serial.println("begin...");
    lcdI2C.begin();
    // zapnutí podsvícení
    Serial.println("backlight...");
    lcdI2C.backlight();
    // vytisknutí hlášky na první řádek
    Serial.println("print...");
    lcdI2C.print(" ->dratek.cz<-");  // nastavení kurzoru na první znak, druhý řádek  // veškeré číslování je od nuly, poslední znak je tedy 19, 3  lcdI2C.setCursor ( 0, 1 );  lcdI2C.print("--------------------");  lcdI2C.setCursor ( 0, 2);  lcdI2C.print(" Test LCD pres I2C");  lcdI2C.setCursor ( 19, 3);  lcdI2C.print("!");  // nastavení počtu znaků a řádků LCD, zde 16x2  lcd.begin(16, 2);  // vytisknutí hlášky na první řádek  lcd.print("dratek.cz");
    delay(1000);
  }


  // inicializace LCD
  /*lcdI2C.begin();
  // zapnutí podsvícení
  lcdI2C.backlight();
  // vytisknutí hlášky na první řádek
  lcdI2C.print(" ->dratek.cz<-");  // nastavení kurzoru na první znak, druhý řádek  // veškeré číslování je od nuly, poslední znak je tedy 19, 3  lcdI2C.setCursor ( 0, 1 );  lcdI2C.print("--------------------");  lcdI2C.setCursor ( 0, 2);  lcdI2C.print(" Test LCD pres I2C");  lcdI2C.setCursor ( 19, 3);  lcdI2C.print("!");  // nastavení počtu znaků a řádků LCD, zde 16x2  lcd.begin(16, 2);  // vytisknutí hlášky na první řádek  lcd.print("dratek.cz");
  // nastavení kurzoru na první znak, druhý řádek
  // veškeré číslování je od nuly, poslední znak je tedy 15, 1
  //lcd.setCursor ( 0, 1 );
  //lcd.print("---------------!");*/
  delay(2000);
}

void loop()
{
  // nastavení kurzoru na devátý znak, druhý řádek
  // lcdI2C.setCursor(8, 1);
  // vytisknutí počtu sekund od začátku programu
  // lcdI2C.print(millis() / 1000);

  // nastavení kurzoru na osmý znak, druhý řádek
  //lcd.setCursor(7, 1);
  // vytisknutí počtu sekund od začátku programu
  //lcd.print(millis() / 1000);
}