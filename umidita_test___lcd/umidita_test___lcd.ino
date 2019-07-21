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
int buttonManIrrig = 35;
int buttonDisplay = 37;
int lightSensor = A14;
int lightSensorThreshold = 500;
int moistureSensorThreshold = 45;
long timerDisplay = 0;
long irrigationTime = 1800; //120; //3600;
long timeOutIrrigation = 0;
int buzzer = 23;
int powerLed = 22;
int timeOutDisplay = 20;
bool turnOff = true;
int pinSensor = A0;
int elettrovalvola = 52;
bool irrigationState = false;
bool manualIrrigationState = false;


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
 * rileva se il pulsangte è premuto
 */
void btnSound(int button,int note){
  bool buttonState = true;
  if(digitalRead(button) == HIGH && buttonState){
     buttonState = false;
     tone(buzzer,note,200);
     buttonState = true;
  }
}
 /**
  * Funzione che visualizza i dati nel display
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
 * stampa layout temperatura
 */
void printTemp(float temp){
  lcd.setCursor (0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C");
}
/**
 * stampa layout umidità atmosferca
 */
void printHumidity(float hum) {
   lcd.setCursor (0, 1);
   lcd.print("Umd Att: ");
   lcd.print(hum);
   lcd.print(" %");
}
/**
 * stampa layout umidita terreno
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
 * stampa layout pressione atm e tempo di irrigazione se quest'utlima
 * è in corso
 */
void printAtmPress(long &atmPress,DateTime val) {
   lcd.setCursor (0, 3);
   if((!irrigationState) && (!manualIrrigationState)){
     lcd.print("Press Atm: ");
     lcd.print(atmPress);
     lcd.print(" hpa");
   }
  else  if((irrigationState) || (manualIrrigationState)){
     lcd.print("IRRIGO PER ");
     lcd.print(1 + ((irrigationTime - (val.unixtime() -  timeOutIrrigation))/60));
     lcd.print(" MIN  ");
   }
}
/**
 * stampa layout se è giorno o notte
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
 * Funzione lettura luminosita dal sensore
 */
bool isNight(){
  int brightness = analogRead(lightSensor);
  // Serial.println(brightness);
  if(brightness < lightSensorThreshold){
     return true;
  }
  else{
    return false;
  }

}
/**
 *  Funzione lettura umidita terreno dal sensore
 */
int soilMoistureControl(){
  int sensorValue = analogRead(pinSensor);
  //Serial.println(sensorValue);
  int soilMoisture = 1023 - sensorValue;
  soilMoisture = map(soilMoisture,0,1023,0,99);
 // Serial.println(soilMoisture);
  return soilMoisture;
}
/**
 * Struct che conterrà tutti i dati letti dai sensori
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
 * Funzione che legge tutti i dati dalle rispettive
 * funzioni di lettura e li salva nella struct
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
 * funzione che produce suono di inizio/fine irrigazione
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
  *  funzione che accende il display
  */
  void activeDisplay() {
      lcd.clear();
      lcd.display();
      lcd.backlight();
  }
  /**
   * funzione che spegne il display
   */
  void deactivatesDisplay() {
      lcd.noDisplay();
      lcd.noBacklight();
      lcd.clear();
  }
  /**
   * funzione ch assembla un msg da visualizzare
   */
  void message(String one, String two, String three) {
      lcd.setCursor(0,0);
      lcd.print(one);
      lcd.setCursor(0,1);
      lcd.print(two);
      lcd.setCursor(0,2);
      lcd.print(three);
  }
  /**
   * funzione di attivazione irrigazione
   */
   void startIrrigationProcess(DateTime now) {
        activeDisplay();
        message("      SISTEMA   ","    IRRIGAZIONE ","      ATTIVO!  ");
        alarm();
        digitalWrite(elettrovalvola,HIGH);
        deactivatesDisplay();
        timeOutIrrigation = now.unixtime(); 
   }
   /**
    * funzione di disattivazione irrigazione
    */
   void stopIrrigationProcess(DateTime now) {
      digitalWrite(elettrovalvola,LOW);
      timeOutIrrigation = now.unixtime(); 
      activeDisplay();
      message("      SISTEMA   ","    IRRIGAZIONE ","    DISATTIVATO!  ");
      alarm();
      deactivatesDisplay();
   }
   /**
    * funzione che si occupa di controllare se ci sono le condizioni per effettuare
    * l'irrigazione,se cosi è, essa avvia l'irrigazione è irriga fino allo scadere 
    * del tempo prestabilito
    */
 void irrigationCycle(DateTime now) {
   if(!irrigationState) {
      if((dataValue.soilMoisture <  moistureSensorThreshold) && (dataValue.timeOfDay == true ) && (!manualIrrigationState)) {
        startIrrigationProcess(now);
        irrigationState = true;
        
      }
    }
   // Serial.println(now.unixtime() - timeOutIrrigation);
   if((irrigationState)){
     if(now.unixtime() -  timeOutIrrigation > irrigationTime) {
        stopIrrigationProcess(now);
        irrigationState = false;
     } 
   } 
 }
 /**
  * funzione che attiva l'irrigazione da comando manuale bypassando le condizioni
  * di attivazione standard
  */
   void manualIrrigation(DateTime now) {
     if(!irrigationState) {
        if(digitalRead(buttonManIrrig) == HIGH){
          manualIrrigationState = true;
          startIrrigationProcess(now);
          turnOff = false;
        }
        if(manualIrrigationState) {
          if(now.unixtime() -  timeOutIrrigation > irrigationTime) {
            stopIrrigationProcess(now);
            manualIrrigationState = false;
            turnOff = false;
          } 
       }  
     }
   }
  /**
   * 
   */
void loop() {
  DateTime now = rtc.now();
  btnSound(buttonDisplay,1000);
  btnSound(buttonManIrrig,2000);
  valueReader();
  displayLayout(dataValue.temperature,dataValue.humidity,dataValue.soilMoisture,1023,dataValue.timeOfDay,now);
  irrigationCycle(now);
  manualIrrigation(now);
}
