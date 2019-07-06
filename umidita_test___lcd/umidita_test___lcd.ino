/*********************************************************/
// include the library code
#include <Wire.h>    // library I2C
#include "DHT.h"    //libreria DHT11
#include "RTClib.h" //library modulo RTC
#include <LiquidCrystal_I2C.h>  //library display 
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#if defined(ARDUINO_ARCH_SAMD)  // for Zero, output on USB Serial console
   #define Serial SerialUSB
#endif

#define DHTPIN 2                 //define pin DHT
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE); //crezione oggetto di tipo dht
RTC_DS1307 rtc;           //crezione oggetto di tipo rtc

/**********************************************************/
int buttonDisplay = 53;
int lightSensor = A15;
int lightSensorThreshold = 500;
int moistureSensorThreshold = 70;
long timerDisplay = 0;
long irrigationTime =1800; //3600;
long timeOutIrrigation = 0;
int buzzer = 23;
int powerLed = 22;
int timeOutDisplay = 60;
bool turnOff = true;
bool buttonState = true;
int pinSensor = A0;
int elettrovalvola = 52;
bool irrigationState = false;

void setup() {
  Serial.println(F("DHTxx test!"));
  dht.begin();
  pinMode(elettrovalvola,OUTPUT);
  pinMode(lightSensor,INPUT);
  pinMode(pinSensor,INPUT);
  pinMode(powerLed,OUTPUT);
  pinMode(buttonDisplay,INPUT);
  digitalWrite(powerLed,HIGH);
  pinMode(buzzer,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 



 // Serial.begin(57600);
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

 timerDisplay = rtc.now().unixtime();
 timeOutIrrigation = rtc.now().unixtime();
  Serial.begin(9600);
}
/**
 * 
 */
void btnSound(int button){
  if(digitalRead(button) == HIGH && buttonState){
     buttonState = false;
     tone(buzzer,1000,200);
     buttonState = true;
  }
}
 /**
  * 
  */
void displayLayout(float temp, float humidity ,int soilMoisture,long atmPress,bool isNight,DateTime val) {
  printTemp(temp);
  printHumidity(humidity);
  printSoilMosture(soilMoisture);
  printAtmPress(atmPress,val); 
  printDay(isNight);
  //Serial.println(val.unixtime() - timerDisplay);
  if((val.unixtime() - timerDisplay > timeOutDisplay) && (turnOff) ){
     lcd.noDisplay();
     lcd.noBacklight();
     turnOff = false;
  }
  if(!turnOff && digitalRead(buttonDisplay) == HIGH ){
     tone(buzzer,1000,200);
     lcd.display();
     lcd.backlight();
     turnOff = true;
     timerDisplay = val.unixtime();
  }
}
/**
 * 
 */
void printTemp(float temp){
  lcd.setCursor (0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C");
}
/**
 * 
 */
void printHumidity(float hum) {
   lcd.setCursor (0, 1);
   lcd.print("Umd Att: ");
   lcd.print(hum);
   lcd.print(" %");
}
/**
 * 
 */
void printSoilMosture(int soilMoisture) {
   lcd.setCursor (0, 2);
   lcd.print("Umd Terr: ");
   lcd.print(soilMoisture);
   if(soilMoisture >= 10 && soilMoisture <= 99) {
       lcd.setCursor (12, 2);
       lcd.print(" ");
       lcd.setCursor(13, 2);
       lcd.print("%");
   }
   else if( soilMoisture <= 9) {
       lcd.setCursor(13, 2);
       lcd.print(" ");
       lcd.setCursor (11, 2);
       lcd.print(" %");
   }
}
/**
 * 
 */
void printAtmPress(long &atmPress,DateTime val) {
   lcd.setCursor (0, 3);
   if(!irrigationState){
     lcd.print("Press Atm: ");
     lcd.print(atmPress);
     lcd.print(" hpa");
   }
   else if(irrigationState){
     lcd.print("IRRIGO PER ");
     lcd.print((irrigationTime - (val.unixtime() -  timeOutIrrigation))/60);
     lcd.print(" MIN  ");
   }
}
/**
 * 
 */
void printDay(bool isNight) {
    lcd.setCursor (15, 2);
 if(isNight){
    lcd.print("NIGHT");
 }
 else if(!isNight) {
   lcd.print("DAY  ");
 } 
}
/**
 * 
 */
bool isNight(){
  int brightness = analogRead(lightSensor);
  if(brightness < lightSensorThreshold){
     return true;
  }
  else{
    return false;
  }
// Serial.println(brightness);
}
/**
 * 
 */
int soilMoistureControl(){
  int sensorValue = analogRead(pinSensor);
  sensorValue = map(sensorValue,0,1023,0,99);
  int soilMoisture = (99 - sensorValue);
  //Serial.println(soilMoisture);
  return soilMoisture;
}
/**
 * 
 */
struct DataSensor {
  int soilMoisture;
  bool timeOfDay;
  float humidity;
  float temperature;
  int atmPressure;
}dataValue;
/**
 * 
 */
void valueReader() {
  dataValue = {soilMoistureControl(),
               isNight(),dht.readHumidity(), 
               dht.readTemperature(),
               1023
               };
}
/*
 * 
 * 
 */
 void alarm() {
     int buzzerTime = 1000;
     int delayTime = 1000;
     tone(buzzer,1500,buzzerTime);
     delay(delayTime);
     noTone(buzzer);
     delay(delayTime);
     tone(buzzer,1500,buzzerTime);
     delay(delayTime);
     noTone(buzzer);
     delay(delayTime);
     tone(buzzer,1500,buzzerTime);
     delay(delayTime);
     noTone(buzzer);
 }
 /**
  * 
  */
  void message(String one, String two, String three) {
      lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(one);
        lcd.setCursor(0,1);
        lcd.print(two);
        lcd.setCursor(0,2);
        lcd.print(three);
     
  }
 void irrigationCycle(DateTime now) {
   if(!irrigationState) {
      if((dataValue.soilMoisture < moistureSensorThreshold) && (dataValue.timeOfDay == true )) {
        message("      SISTEMA   ","    IRRIGAZIONE ","      ATTIVO!  ");
        alarm();
        lcd.clear();
        digitalWrite(elettrovalvola,HIGH);
        irrigationState = true;
      }
    }
   Serial.println(now.unixtime() - timeOutIrrigation);
    if(now.unixtime() -  timeOutIrrigation > irrigationTime) {
       digitalWrite(elettrovalvola,LOW);
        timeOutIrrigation = now.unixtime(); 
        irrigationState = false;
     } 
 }



 
void loop() {
  DateTime now = rtc.now();
  btnSound(buttonDisplay);
  valueReader();
  displayLayout(dataValue.temperature,dataValue.humidity,dataValue.soilMoisture,1023,dataValue.timeOfDay,now);
  irrigationCycle(now);
  



}
