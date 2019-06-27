/********************************/
// include the library code
#include <Wire.h> 
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#include "RTClib.h"
#if defined(ARDUINO_ARCH_SAMD)  // for Zero, output on USB Serial console
   #define Serial SerialUSB
#endif

#define DHTPIN 2
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;

/*********************************************************/

long tempo;
int buzzer = 23;
int powerLed = 22;
 
#define pinSensor 0;

long sensorValue;

void setup() {
   Serial.println(F("DHTxx test!"));
    dht.begin();
  pinMode(powerLed,OUTPUT);
  digitalWrite(powerLed,HIGH);
  pinMode(buzzer,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 


 while (!Serial);  // for Leonardo/Micro/Zero

  Serial.begin(57600);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

 tempo = rtc.now().unixtime();



  
  Serial.begin(9600);

}

void loop() {
 // sensorValue = analogRead(pinSensor);
  //sensorValue = map(sensorValue,0,1023,0,99);
 // lcd.clear();
  lcd.setCursor ( 0, 0 );            // go to the top left corner
  //lcd.print(" umidita terreno "); // write this string on the top ro
  //lcd.print(sensorValue);
   DateTime now = rtc.now();
    digitalWrite(LED_BUILTIN, HIGH);
    lcd.print("secondi ");
    
     lcd.print(now.unixtime() - tempo);
    if(now.unixtime() - tempo > 20) {
     // tone(buzzer,1000,1000);
      digitalWrite(LED_BUILTIN, LOW);
      tempo = now.unixtime();
      //delay(5000);
    }
      float h = dht.readHumidity();
      float t = dht.readTemperature();
     lcd.setCursor ( 0, 1 );   
     lcd.print("Temp = " );
     lcd.print(t);
     lcd.print( " C");
     lcd.setCursor ( 0, 2 );
     lcd.print("umidita = " );
     lcd.print(h);
     lcd.print( " %");
     
  //Serial.print("Valore: ");
//  Serial.println(sensorValue);
  
  delay(1000);      

}
