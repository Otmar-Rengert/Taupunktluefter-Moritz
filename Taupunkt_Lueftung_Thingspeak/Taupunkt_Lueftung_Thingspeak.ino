// History
// 2024-12-20	Otmar	Clone from github and experiment
// 2024-12-31 	Otmar 	Based on Taupunkt_Lueftung.ino, but now for ESP8266 (WEMOS D1 mini)

// Dieser Code benötigt zwingend die folgenden Libraries:
#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <credentials.h>  // This private file is to hold all the credentials used by this code here

//////////////////////////////////////////////////////////////////////////
//                 Set WiFi                                             //
//////////////////////////////////////////////////////////////////////////
const char * stassid = WIFI_STASSID;                         // From credentials.h                     
const char * stapsk  = WIFI_STAPSK;                          // From credentials.h


//////////////////////////////////////////////////////////////////////////
//                  Set ThingsSpeak                                     //
//////////////////////////////////////////////////////////////////////////
#define THINGSPEAK
#ifdef THINGSPEAK
#include "ThingSpeak.h"
unsigned long myChannelNumber = THINGSPEAK_CHANNEL;         // From credentials.h
const char  * myWriteAPIKey   = THINGSPEAK_WRITE_API_KEY;   // From credentials.h
WiFiClient  client;
#endif


//////////////////////////////////////////////////////////////////////////
//                  Set NTP                                             //
//////////////////////////////////////////////////////////////////////////
#include <NTPClient.h>  // NTP time to dim display during night 
#include <WiFiUdp.h>

#include "TimeLib.h"
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 5*60*1000);  /* offset 3600secs from GMT (+1h), update every 5 minutes */





#define RELAIPIN 16 // Anschluss des Lüfter-Relais
#define DHTPIN_1  0 // Datenleitung für den DHT-Sensor 1 (innen)
#define DHTPIN_2  2 // Datenleitung für den DHT-Sensor 2 (außen)

#define RELAIS_EIN LOW
#define RELAIS_AUS HIGH
bool rel;

#define DHTTYPE_1 DHT22 // DHT 22 
#define DHTTYPE_2 DHT22 // DHT 22  

// *******  Korrekturwerte der einzelnen Sensorwerte  *******
#define TemperatureInAdjust  -0.7 // Korrekturwert Innensensor Temperatur
//#define TemperatureInAdjust  +20.7 // Korrekturwert Innensensor Temperatur - test only to get Ventilator on
#define TemperatureOutAdjust -0.4 // Korrekturwert Außensensor Temperatur
#define HumidityInAdjust     +1.9 // Korrekturwert Innensensor Luftfeuchtigkeit
#define HumidityOutAdjust    +1.3 // Korrekturwert Außensensor Luftfeuchtigkeit
//***********************************************************

#define DewpointMinDelta    5.0 // minimaler Taupunktunterschied, bei dem das Relais schaltet
#define DewpointHysteresis  1.0 // Abstand von Ein- und Ausschaltpunkt
#define TemperatureInMin   10.0 // Minimale Innentemperatur, bei der die Lüftung aktiviert wird
#define TemperatureOutMin -10.0 // Minimale Außentemperatur, bei der die Lüftung aktiviert wird

DHT dht1(DHTPIN_1, DHTTYPE_1); //Der Innensensor wird ab jetzt mit dht1 angesprochen
DHT dht2(DHTPIN_2, DHTTYPE_2); //Der Außensensor wird ab jetzt mit dht2 angesprochen

LiquidCrystal_I2C lcd(0x27,20,4); // LCD: I2C-Addresse und Displaygröße setzen

bool fehler = true;

void setup() {

  Serial.begin(115200);
  Serial.println("Taupunktluefter-Moritz"); // Serielle Ausgabe, falls noch kein LCD angeschlossen ist
  Serial.println(F("Teste Sensoren.."));

  pinMode(RELAIPIN, OUTPUT);          // Relaispin als Output definieren
  digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
  lcd.init();
  lcd.backlight();                      
  lcd.setCursor(2,0);
  lcd.print(F("Teste Sensoren.."));
  
  byte Grad[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // Sonderzeichen ° definieren
  lcd.createChar(0, Grad);
  byte Strich[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100};   // Sonderzeichen senkrechter Strich definieren
  lcd.createChar(1, Strich);
    
  dht1.begin(); // Sensoren starten
  dht2.begin();   
  
  // Start of WiFi module ////////////////////////////////////////////////////////
  WiFi.begin(stassid, stapsk);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(200);
    Serial.print ( "." );
  }
  Serial.println("\nWiFi connected");
  
  timeClient.begin();
  timeClient.update();
	
  Serial.println("Taupunktluefter booted at " + timeClient.getFormattedTime());

#ifdef THINGSPEAK
  ThingSpeak.begin(client);
#endif  
}

void loop() 
{
  float HumidityIn     = dht1.readHumidity()    + HumidityInAdjust;     // Innenluftfeuchtigkeit auslesen und unter „h1“ speichern
  float TemperatureIn  = dht1.readTemperature() + TemperatureInAdjust;  // Innentemperatur auslesen und unter „t1“ speichern
  float HumidityOut    = dht2.readHumidity()    + HumidityOutAdjust;    // Außenluftfeuchtigkeit auslesen und unter „h2“ speichern
  float TemperatureOut = dht2.readTemperature() + TemperatureOutAdjust; // Außentemperatur auslesen und unter „t2“ speichern
  
  if (fehler == true)  // Prüfen, ob gültige Werte von den Sensoren kommen
  {
    fehler = false; 
    if (isnan(HumidityIn) || isnan(TemperatureIn) || HumidityIn > 100 || HumidityIn < 1 || TemperatureIn < -40 || TemperatureIn > 80 )  {
      Serial.println(F("Fehler beim Auslesen vom Innensensor!"));
      lcd.setCursor(0,1);
      lcd.print(F("Fehler Innensensor"));
      fehler = true;
    }else {
     lcd.setCursor(0,1);
     lcd.print(F("Innensensor OK"));
   }
  
    delay(2000);  // Zeit um das Display zu lesen
  
      if (isnan(HumidityOut) || isnan(TemperatureOut) || HumidityOut > 100 || HumidityOut < 1 || TemperatureOut < -40 || TemperatureOut  > 80)  {
        Serial.println(F("Fehler beim Auslesen vom Aussensensor!"));
        lcd.setCursor(0,2);
        lcd.print(F("Fehler Aussensensor"));
        fehler = true;
      } else {
        lcd.setCursor(0,2);
        lcd.print(F("Aussensensor OK"));
     }

    delay(2000);  // Zeit um das Display zu lesen
  }
  
  if (isnan(HumidityIn) || isnan(TemperatureIn) || isnan(HumidityOut) || isnan(TemperatureOut)) fehler = true;
   
  if (fehler == true) {
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten 
    lcd.setCursor(0,3);
    lcd.print(F("CPU Neustart....."));
    while (1);  // Endlosschleife um das Display zu lesen und die CPU durch den Watchdog neu zu starten
  }

  //**** Taupunkte errechnen********
  float DewpointIn  = taupunkt(TemperatureIn,  HumidityIn);
  float DewpointOut = taupunkt(TemperatureOut, HumidityOut);

  // Werteausgabe auf Serial Monitor
  timeClient.update();
  Serial.println("Werte update at " + timeClient.getFormattedTime());  
  
  Serial.print(F("Sensor-1: " ));
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(HumidityIn);                     
  Serial.print(F("%  Temperatur: "));
  Serial.print(TemperatureIn);
  Serial.print(F("°C  "));
  Serial.print(F("  Taupunkt: "));
  Serial.print(DewpointIn);
  Serial.println(F("°C  "));

  Serial.print("Sensor-2: " );
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(HumidityOut);
  Serial.print(F("%  Temperatur: "));
  Serial.print(TemperatureOut);
  Serial.print(F("°C "));
  Serial.print(F("   Taupunkt: "));
  Serial.print(DewpointOut);
  Serial.println(F("°C  "));

  Serial.println();

  // Werteausgabe auf dem I2C-Display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("S1: "));
  lcd.print(TemperatureIn); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  // lcd.write((uint8_t)1); // Sonderzeichen |
  lcd.print(" ");
  lcd.print(HumidityIn);
  lcd.print(F("%"));

  lcd.setCursor(0,1);
  lcd.print(F("S2: "));
  lcd.print(TemperatureOut); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  //lcd.write((uint8_t)1); // Sonderzeichen |
  lcd.print(" ");  
  lcd.print(HumidityOut);
  lcd.print(F("%"));

  lcd.setCursor(0,2);
  lcd.print(F("Taupunkt 1: "));
  lcd.print(DewpointIn); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

  lcd.setCursor(0,3);
  lcd.print(F("Taupunkt 2: "));
  lcd.print(DewpointOut); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

  delay(5000); // Zeit um das Display zu lesen

  lcd.clear();
  lcd.setCursor(0,0);
  
  float DeltaDewpoint = DewpointIn - DewpointOut;

  if (DeltaDewpoint > (DewpointMinDelta + DewpointHysteresis))
	  rel = true;
  if (DeltaDewpoint < (DewpointMinDelta))
	  rel = false;
  if (TemperatureIn < TemperatureInMin )
	  rel = false;
  if (TemperatureOut < TemperatureOutMin )
	  rel = false;

  if (rel == true)
  {
    digitalWrite(RELAIPIN, RELAIS_EIN); // Relais einschalten
    lcd.print(F("Lueftung: AN"));  
  } 
  else 
  {                             
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
    lcd.print(F("Lueftung: AUS"));
  }

  lcd.setCursor(0,1);
  lcd.print("Delta TP: ");
  lcd.print(DeltaDewpoint);
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write('C');

  lcd.setCursor(0,3);
  lcd.print("Time " + timeClient.getFormattedTime() + " (MEZ)");  

#ifdef THINGSPEAK
  ThingSpeak.setStatus("Taupunktluefter");
  ThingSpeak.setField(1, TemperatureOut);
  ThingSpeak.setField(2, HumidityOut);  
  ThingSpeak.setField(3, DewpointOut);   
  ThingSpeak.setField(4, TemperatureIn);
  ThingSpeak.setField(5, HumidityIn);  
  ThingSpeak.setField(6, DewpointIn);
  ThingSpeak.setField(7, rel);   
  
  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (httpCode == 200) 
  {
	Serial.println("ThingSpeak Channel write successful.");
  }
  else 
  {
	Serial.println("Problem writing to ThingSpeak channel. HTTP error code " + String(httpCode));
  }
#endif
 delay(10000);   // Wartezeit zwischen zwei Messungen
}

float taupunkt(float t, float r) {
  
float a, b;
  
  if (t >= 0) {
    a = 7.5;
    b = 237.3;
  } else if (t < 0) {
    a = 7.6;
    b = 240.7;
  }
  
  // Sättigungsdampfdruck in hPa
  float sdd = 6.1078 * pow(10, (a*t)/(b+t));
  
  // Dampfdruck in hPa
  float dd = sdd * (r/100);
  
  // v-Parameter
  float v = log10(dd/6.1078);
  
  // Taupunkttemperatur (°C)
  float tt = (b*v) / (a-v);
  return { tt };  
}


 void software_Reset() // Startet das Programm neu, nicht aber die Sensoren oder das LCD 
  {

  }
