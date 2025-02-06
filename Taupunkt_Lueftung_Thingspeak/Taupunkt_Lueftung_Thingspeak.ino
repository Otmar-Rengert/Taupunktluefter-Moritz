// History
// 2024-12-20	Otmar	Clone from github and experiment
// 2024-12-31 	Otmar 	Based on Taupunkt_Lueftung.ino, but now for ESP8266 (WEMOS D1 mini)
// 2025-02-05   Otmar   Correct input for In/Out DHT22. Add WiFi RSSI. Nicen startup display
// 2025-02-06   Otmar   Commit final version for Moritz device

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

#define RELAY_PIN   14 // Anschluss des Lüfter-Relais (D5)
#define DHT_IN_PIN   2 // Datenleitung für den DHT-Sensor innen (D4)
#define DHT_OUT_PIN  0 // Datenleitung für den DHT-Sensor außen (D3)

#define RELAY_ON  LOW
#define RELAY_OFF HIGH
bool relay_on;

/* Both in and outside sensore use DHT22 type */
#define DHT_IN_TYPE  DHT22 
#define DHT_OUT_TYPE DHT22   

/*******  Korrekturwerte der einzelnen Sensorwerte measured vs. Testo hygrometer at 6.2.2025 *******/
#define TemperatureInAdjust  +0.1 // Korrekturwert Innensensor Temperatur
#define TemperatureOutAdjust +0.1 // Korrekturwert Außensensor Temperatur
#define HumidityInAdjust     +2.7 // Korrekturwert Innensensor Luftfeuchtigkeit 
#define HumidityOutAdjust    +1.9 // Korrekturwert Außensensor Luftfeuchtigkeit 

#define DewpointMinDelta    5.0 // minimaler Taupunktunterschied, bei dem das Relais schaltet
#define DewpointHysteresis  1.0 // Abstand von Ein- und Ausschaltpunkt
#define TemperatureInMin   10.0 // Minimale Innentemperatur, bei der die Lüftung aktiviert wird
#define TemperatureOutMin -10.0 // Minimale Außentemperatur, bei der die Lüftung aktiviert wird

DHT dht1(DHT_IN_PIN,  DHT_IN_TYPE);  //Der Innensensor wird ab jetzt mit DHT_IN angesprochen
DHT dht2(DHT_OUT_PIN, DHT_OUT_TYPE); //Der Außensensor wird ab jetzt mit DHT_OUT angesprochen

LiquidCrystal_I2C lcd(0x27,20,4); // LCD: I2C-Addresse und Displaygröße setzen

bool sensor_fault = true;

void setup() {

  Serial.begin(115200);
  Serial.println("Taupunktluefter-Moritz"); // Serielle Ausgabe, falls noch kein LCD angeschlossen ist
  Serial.println(F("Teste Sensoren.."));

  pinMode(RELAY_PIN, OUTPUT);          // Relaispin als Output definieren
  digitalWrite(RELAY_PIN, RELAY_OFF); // Relais ausschalten


  // 1. Initialise LCD and display startup message /////////////////////////////////////////////////
  lcd.init();
  
  byte Grad[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // Sonderzeichen ° definieren
  lcd.createChar(0, Grad);
  byte Strich[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100};   // Sonderzeichen senkrechter Strich definieren
  lcd.createChar(1, Strich);
  
  lcd.backlight();     
  
  lcd.setCursor(1,0);
  lcd.print(F("Taupunktluefter"));
  lcd.setCursor(0,1);
  lcd.print(F("(c) Marie&Papa, 2025"));
  delay(2000);  // Zeit um das Display zu lesen  

  // 2. Start up the sensors and test they are ok /////////////////////////////////////////////////
  dht1.begin();
  dht2.begin();   
  
  lcd.clear();  
  lcd.setCursor(2,0);
  lcd.print(F("Connect to WiFi:"));
  lcd.setCursor(2,1);
  lcd.print(stassid);  
 
  // 3. Start of WiFi module and connect to SSID from credentils.h /////////////////////////////////
  WiFi.begin(stassid, stapsk);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(200);
    Serial.print ( "." );
  }
  Serial.println("\nWiFi connected");

  lcd.setCursor(2,2);
  lcd.print(F("OK"));
  delay(2000);  // Zeit um das Display zu lesen  

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print(F("Teste Sensoren:"));
  
  // 4. Start NTP client so we can display the time of day /////////////////////////////////////////
  
  timeClient.begin();
  timeClient.update();
	
  Serial.println("Taupunktluefter booted at " + timeClient.getFormattedTime());

#ifdef THINGSPEAK
  // 5. Start Thingspeak client so we can push our results and the vent setting ////////////////////
  ThingSpeak.begin(client);
#endif  
}

void loop() 
{
  long  rssi;
  float HumidityIn     = dht1.readHumidity()    + HumidityInAdjust;     // Innenluftfeuchtigkeit auslesen und unter „h1“ speichern
  float TemperatureIn  = dht1.readTemperature() + TemperatureInAdjust;  // Innentemperatur auslesen und unter „t1“ speichern
  float HumidityOut    = dht2.readHumidity()    + HumidityOutAdjust;    // Außenluftfeuchtigkeit auslesen und unter „h2“ speichern
  float TemperatureOut = dht2.readTemperature() + TemperatureOutAdjust; // Außentemperatur auslesen und unter „t2“ speichern
  
  // 1. Check we get valid values from the sensors. ////////////////////////////////////////////////
  // Note we always enter this if and display after first startup, as sensor_fault defaults to 1 
  if (sensor_fault == true) 
  {
    sensor_fault = false; 
    if (isnan(HumidityIn) || isnan(TemperatureIn) || HumidityIn > 100 || HumidityIn < 1 || TemperatureIn < -40 || TemperatureIn > 80 )  {
      Serial.println(F("Fehler beim Auslesen vom Innensensor!"));
      lcd.setCursor(2,1);
      lcd.print(F("Innensensor  TOT"));
      sensor_fault = true;
    }else {
     lcd.setCursor(2,1);
     lcd.print(F("Innensensor  OK"));
    } 
    delay(2000);  // Time to read the display
  
    if (isnan(HumidityOut) || isnan(TemperatureOut) || HumidityOut > 100 || HumidityOut < 1 || TemperatureOut < -40 || TemperatureOut  > 80)  {
      Serial.println(F("Fehler beim Auslesen vom Aussensensor!"));
      lcd.setCursor(2,2);
      lcd.print(F("Aussensensor TOT"));
      sensor_fault = true;
    } else {
      lcd.setCursor(2,2);
      lcd.print(F("Aussensensor OK"));
    }
    delay(2000);  // Time to read the display
  }
  
  // 2. Make sure all values are numbers and not something else (like a string or so) //////////////
  if (isnan(HumidityIn) || isnan(TemperatureIn) || isnan(HumidityOut) || isnan(TemperatureOut)) sensor_fault = true;
   
  // 3. If something is wrong with the sensor status or the result, restart ////////////////////////
  if (sensor_fault == true) {
    digitalWrite(RELAY_PIN, RELAY_OFF); // Relay off for safety reasons 
    lcd.setCursor(2,3);
    lcd.print(F("Restart.."));
    while (1);  // Endless loop to wait for Watchdog restart
  }

  // 4. Calculate dewpoints //////////////////////////////////////////////////////////////////////// 
  float DewpointIn  = CalculateDewpoint(TemperatureIn,  HumidityIn);
  float DewpointOut = CalculateDewpoint(TemperatureOut, HumidityOut);

  // 5. Update the NTP time for display and get the WiFi RSSI (field strenght) /////////////////////
  timeClient.update();
  rssi = WiFi.RSSI();  // Get current WiFi signal strength
	
  // 6. Print debug information to the serial monitor //////////////////////////////////////////////
  Serial.println("Werte update at " + timeClient.getFormattedTime());  
  
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm"); 

  Serial.print(F("Sensor In : " ));
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(HumidityIn);                     
  Serial.print(F("%  Temperatur: "));
  Serial.print(TemperatureIn);
  Serial.print(F("°C  "));
  Serial.print(F("  Taupunkt: "));
  Serial.print(DewpointIn);
  Serial.println(F("°C  "));

  Serial.print("Sensor Out: " );
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(HumidityOut);
  Serial.print(F("%  Temperatur: "));
  Serial.print(TemperatureOut);
  Serial.print(F("°C "));
  Serial.print(F("   Taupunkt: "));
  Serial.print(DewpointOut);
  Serial.println(F("°C  "));

  Serial.println();

  // 7. Update the real HW I2C display with measurement results ////////////////////////////////////
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("In : "));
  lcd.print(TemperatureIn); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  lcd.print(" ");
  lcd.print(HumidityIn);
  lcd.print(F("%"));

  lcd.setCursor(0,1);
  lcd.print(F("Out: "));
  lcd.print(TemperatureOut); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  lcd.print(" ");  
  lcd.print(HumidityOut);
  lcd.print(F("%"));

  lcd.setCursor(0,2);
  lcd.print(F("TP In : "));
  lcd.print(DewpointIn); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

  lcd.setCursor(0,3);
  lcd.print(F("TP Out: "));
  lcd.print(DewpointOut); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

  delay(5000); // Zeit um das Display zu lesen

  lcd.clear();
  lcd.setCursor(0,0);
  
  
  // 8. Calculate the dewpoint difference in vs. out, and decide about the relay setting ///////////
  float DeltaDewpoint = DewpointIn - DewpointOut;

  if (DeltaDewpoint > (DewpointMinDelta + DewpointHysteresis))
	  relay_on = true;
  if (DeltaDewpoint < (DewpointMinDelta))
	  relay_on = false;
  if (TemperatureIn < TemperatureInMin )
	  relay_on = false;
  if (TemperatureOut < TemperatureOutMin )
	  relay_on = false;

  // 9. Control the relay, and display more information to the hardware I2C display ////////////////
  if (relay_on == true)
  {
    digitalWrite(RELAY_PIN, RELAY_ON); // Relais einschalten
    lcd.print(F("Lueftung: AN"));
	Serial.println("Lueftung: AN");	
  } 
  else 
  {                             
    digitalWrite(RELAY_PIN, RELAY_OFF); // Relais ausschalten
    lcd.print(F("Lueftung: AUS"));
	Serial.println("Lueftung: AUS");	
  }

  lcd.setCursor(0,1);
  lcd.print("Delta TP: ");
  lcd.print(DeltaDewpoint);
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write('C');

  lcd.setCursor(0,2);
  lcd.print("WiFiRSSI: ");
  lcd.print(rssi);
  lcd.print(" dBm");  

  lcd.setCursor(0,3);
  lcd.print("Time/MEZ: " + timeClient.getFormattedTime());  

#ifdef THINGSPEAK
  // 10. Push the results to the Thingspeak server onto the channel given in credentials.h /////////
  ThingSpeak.setStatus("Taupunktluefter");
  ThingSpeak.setField(1, TemperatureOut);
  ThingSpeak.setField(2, HumidityOut);  
  ThingSpeak.setField(3, DewpointOut);   
  ThingSpeak.setField(4, TemperatureIn);
  ThingSpeak.setField(5, HumidityIn);  
  ThingSpeak.setField(6, DewpointIn);
  ThingSpeak.setField(7, relay_on);
  ThingSpeak.setField(8, rssi);     
  
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



// Subroutine to calculate the dewpoint ////////////////////////////////////////////////////////////
float CalculateDewpoint(float t, float r) {
  
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
