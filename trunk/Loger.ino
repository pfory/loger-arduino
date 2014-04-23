/*PINOUT

D0 Rx
D1 
D2 
D3 GPS
D4 GPS
D5-D8
D9 
D10-D13 SD card 
A0 DALLAS temperature
A1 MPX4115A
A2 DHT11
A3 
A4 
A5 
*/


#include "TinyGPS++.h"
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//OLED
#include <IIC_without_ACK.h>
#include "oledfont.c"   //codetab

#define OLED_SDA 8
#define OLED_SCL 9

IIC_without_ACK oled(OLED_SDA, OLED_SCL);//8 -- sda,9 -- scl

//PRESS
int hPa; // pressure in Pa
#define pressurePin A1

//HUMIDITY
#include "DHT.h"
#define DHTTYPE DHT11   // DHT 11 
#define DHTPIN A2    // what pin we're connected to
DHT dht(DHTPIN, DHTTYPE);

//DALLAS
#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
#include <DallasTemperatureMinimal.h>
DallasTemperatureMinimal dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
DeviceAddress tempDeviceAddresses[1];
float temperature;

//SD
#include <SD.h>
const int chipSelect = 10;
File dFile;
#define delimiter ';'

//VAR
char sz[32];
bool dsMeasStarted=false;
unsigned long lastMeasTime=0;
unsigned long const dsMeassureInterval=750; //interval between measurements
unsigned long lastDsMeasStartTime;


unsigned long saveMeteoDataInterval = 1000;
unsigned long lastSaveMeteoData = 0;

void setup() {
	Serial.begin(115200);
  ss.begin(GPSBaud);
	//Serial.println(F("DataLoger"));
	
  oled.Initial();
  delay(10);
  oled.Fill_Screen(0x00);
  
  dht.begin();
  dht.startMeas();
  
  dsInit();
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //Serial.println(F("Card failed, or not present"));
    // don't do anything more:
  }
  //Serial.println("card initialized.");
	dFile = SD.open("datalog.txt", FILE_WRITE);
}

void loop() {
  if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
    dsMeasStarted=false;
    temperature = dsSensors.getTempC(tempDeviceAddresses[0]);
    Serial.println(temperature);
  }

  if (gps.time.isUpdated()) {
    printDateTime(gps.date, gps.time);
  }
  
  hPa = ((float)analogRead(pressurePin)/(float)1023+0.095)/0.0009;
  sprintf(sz, "%4dhPa", hPa);
  oled.Char_F6x8(0,1,sz);

  sprintf(sz, "%2d%%Rh", (int)dht.readHumidity());
  oled.Char_F6x8(50,1,sz);

  sprintf(sz, "%3d.%1dC", (int)temperature, (int)(temperature*10)%10);
  oled.Char_F6x8(80,1,sz);
  
  if (gps.location.isUpdated()) {
    sprintf(sz, "%3d.%3d", gps.location.rawLat().deg, gps.location.rawLat().billionths);
    oled.Char_F6x8(0,3,sz);

    sprintf(sz, "%3d.%3d", gps.location.rawLng().deg, gps.location.rawLng().billionths);
    oled.Char_F6x8(60,3,sz);
  }
  
  if (gps.altitude.isUpdated()) {
    sprintf(sz, "%4dm", (int)gps.altitude.meters());
    oled.Char_F6x8(0,4,sz);
  }
  
  
  if (gps.speed.isUpdated()) {
    sprintf(sz, "%3dkm/h", (int)gps.speed.kmph());
    oled.Char_F6x8(30,4,sz);
  }
  if (gps.course.isUpdated()) {
    if (gps.course.isValid()) {
      sprintf(sz, "%3d", (int)gps.course.deg());
      oled.Char_F6x8(80,4,sz);
      oled.Char_F6x8(100,4,TinyGPSPlus::cardinal(gps.course.value()));
    }
  }

  if (gps.satellites.isUpdated()) {
    sprintf(sz, "%2dsat", gps.satellites.value());
    oled.Char_F6x8(0,5,sz);
  }
  
  if (hdop.isUpdated()) {
    sprintf(sz, "%4dH", (int)hdop.value());
    oled.Char_F6x8(0,6,hdop.value());
  }
  if (vdop.isUpdated()) {
    sprintf(sz, "%4dV", (int)vdop.value());
    oled.Char_F6x8(40,6,vdop.value());
  }
  if (pdop.isUpdated()) {
    sprintf(sz, "%4dP", (int)pdop.value());
    oled.Char_F6x8(60,6,pdop.value());
  }

  //printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  //printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);


  
  smartDelay(1000);
}

void dsInit(void) {
  dsSensors.begin();
  //Serial.println(dsSensors.getDeviceCount());
  if (dsSensors.getAddress(tempDeviceAddress, 0)) {
    memcpy(tempDeviceAddresses[0],tempDeviceAddress,8);
  }
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (d.isValid())
  {
    sprintf(sz, "%02d.%02d.%02d ", d.day(), d.month(), d.year());
    //Serial.println(sz);
    oled.Char_F6x8(0,0,sz);
  }
  
  if (t.isValid())
  {
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    //Serial.println(sz);
    oled.Char_F6x8(66,0,sz);
  }

  //printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}