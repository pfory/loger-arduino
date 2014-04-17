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

 /*

 Example code for connecting a Parallax GPS module to the Arduino

 Igor Gonzalez Martin. 05-04-2007
 igor.gonzalez.martin@gmail.com

 English translation by djmatic 19-05-2007

 Listen for the $GPRMC string and extract the GPS location data from this.
 Display the result in the Arduino's serial monitor.

 */ 

//#include <string.h>
//#include <ctype.h>
#ifdef SD
#include <SD.h>
#endif

#include "TinyGPS++.h"
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

const int chipSelect = 10;

#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
#define dallasMinimal //-956 Bytes
#ifdef dallasMinimal
#include <DallasTemperatureMinimal.h>
DallasTemperatureMinimal dsSensors(&onewire);
#else
#include <DallasTemperature.h>
DallasTemperature dsSensors(&onewire);
#endif
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 1
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
//int  resolution = 12;
unsigned int numberOfDevices; // Number of temperature devices found
unsigned long lastDsMeasStartTime;
bool dsMeasStarted=false;
unsigned long const dsMeassureInterval=750; //inteval between meassurements
unsigned long lastMeasTime=0;
float sensor[NUMBER_OF_DEVICES];

float phPa; // pressure in kPa
#define pressurePin A1

#include "DHT.h"
#define DHTTYPE DHT11   // DHT 11 
#define DHTPIN A2    // what pin we're connected to

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDHTMeasTime;
unsigned long lastDisplayDHTTime;

unsigned long saveMeteoDataInterval = 1000;
unsigned long lastSaveMeteoData = 0;

#ifdef SD
File dFile;
#define delimiter ';'
#endif

#include <IIC_without_ACK.h>
#include "oledfont.c"   //codetab

#define OLED_SDA 8
#define OLED_SCL 9

IIC_without_ACK lucky(OLED_SDA, OLED_SCL);//8 -- sda,9 -- scl

char c[7];


void setup() {
	Serial.begin(115200);
  ss.begin(GPSBaud);
	Serial.println("DataLoger");
	Serial.print("Initializing SD card...");

#ifdef SD
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
	dFile = SD.open("datalog.txt", FILE_WRITE);
#endif
  
	dsInit();
	
  dhtInit();
  dht.startMeas();
  
  lucky.Initial();
  delay(10);
  lucky.Fill_Screen(0x00);
}

void loop() {
  if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    //digitalWrite(13,HIGH);
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
    dsMeasStarted=false;
    //digitalWrite(13,LOW);
    //saving temperatures into variables
    for (byte i=0;i<numberOfDevices; i++) {
      float tempTemp=-126;
      for (byte j=0;j<10;j++) { //try to read temperature ten times
        //tempTemp = dsSensors.getTempCByIndex(i);
        tempTemp = dsSensors.getTempC(tempDeviceAddresses[i]);
        if (tempTemp>=-55) {
          break;
        }
      }
      sensor[i] = tempTemp;
    } 
		Serial.print("Teplota DS18B20:");
    Serial.println(sensor[0]);
		
		Serial.print("Teplota DHT:");
		Serial.println(dht.readTemperature());
		Serial.print("Vlhkost:");
		Serial.println(dht.readHumidity());
		
		Serial.print("MPX4115A:");
		int val = analogRead(pressurePin);
		phPa = ((float)val/(float)1023+0.095)/0.0009;
		//pAtm = kpa2atm*pkPa;
		/* send pressure to serial port */
		Serial.print(phPa);
		Serial.println("hPa ");
		//Serial.print(pAtm);
		//Serial.println("Atm ");
    int t = (int)(sensor[0]*10);
    sprintf(c,"%d.%u",t/10,abs(t%10));
    lucky.Char_F6x8(0,0,c);
    t=dht.readTemperature();
    sprintf(c,"%d",t);
    lucky.Char_F6x8(40,0,c);
    t=dht.readHumidity();
    sprintf(c,"%d",t);
    lucky.Char_F6x8(80,0,c);
    
	}
  if (millis() - lastSaveMeteoData > saveMeteoDataInterval) {
    lastSaveMeteoData = millis();
#ifdef SD
    saveMeteoData();
#endif
  }
  
  /*printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);*/
  itoa(gps.location.lat(),c,10);
  lucky.Char_F6x8(0,4,c);
  
	//readGPSData();

  
}

#ifdef SD
void saveMeteoData() {
  dFile.close();
  //save meteo data
	File dMeteo = SD.open("meteo.csv", FILE_WRITE);
  if (dMeteo) {
    dMeteo.print(sensor[0]);
    dMeteo.print(delimiter);
    dMeteo.print(dht.readTemperature());
    dMeteo.print(delimiter);
    dMeteo.print(dht.readHumidity());
    dMeteo.print(delimiter);
    dMeteo.print(phPa);
    dMeteo.print(delimiter);
    dMeteo.println();
    dMeteo.close();
  }
	dFile = SD.open("datalog.txt", FILE_WRITE);
}
#endif

void readGPSData() {
}


#ifdef SD
void writeGPS() {
	// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  // if the file is available, write to it:
  if (dFile) {
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
#endif

void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();

  //lcd.setCursor (0, 0);
  //lcd.print(numberOfDevices);
  
	/*
  if (numberOfDevices==1)
    lcd.print(" sensor found");
  else
    lcd.print(" sensors found");
	*/	
		
  delay(1000);
  
  Serial.print("Sensor(s):");
  Serial.println(numberOfDevices);

  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
  }
#ifndef dallasMinimal
  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);
#endif

}

void dhtInit() {
  //Serial.println("\nDHT setup");
  dht.begin();
  //Serial.print("DHT software on PIN D");
  //Serial.print(DHTPIN);
  //Serial.println(" OK");
}

/*
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
*/