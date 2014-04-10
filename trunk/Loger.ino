/*PINOUT

D0 Rx
D1 GPS
D2 
D3 
D4 
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

#include <string.h>
#include <ctype.h>
#include <SD.h>

int ledPin = 13;                  // LED test pin
int rxPin = 0;                    // RX PIN 
int txPin = 1;                    // TX TX
int byteGPS=-1;
char linea[300] = "";
char comandoGPR[7] = "$GPRMC";
int cont=0;
int bien=0;
int conta=0;
int indices[13];
File dataFile;

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
#define NUMBER_OF_DEVICES 4
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

void setup() {
	pinMode(ledPin, OUTPUT);       // Initialize LED pin
	pinMode(rxPin, INPUT);
	pinMode(txPin, OUTPUT);
	Serial.begin(4800);
	for (int i=0;i<300;i++){       // Initialize a buffer for received data
	 linea[i]=' ';
	}  
	Serial.println("DataLoger");
	Serial.print("Initializing SD card...");

 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
	dataFile = SD.open("datalog.txt", FILE_WRITE);
	
	dsInit();
	
  dhtInit();
  dht.startMeas();
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
	}


 digitalWrite(ledPin, HIGH);
 byteGPS=Serial.read();         // Read a byte of the serial port
 if (byteGPS == -1) {           // See if the port is empty yet
	 delay(100); 
 } else {
	 // note: there is a potential buffer overflow here!
	 linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
	 conta++;                      
	 Serial.print((char)byteGPS); 
	 if (byteGPS==13){            // If the received byte is = to 13, end of transmission
		writeGPS();
		 // note: the actual end of transmission is <CR><LF> (i.e. 0x13 0x10)
		 digitalWrite(ledPin, LOW); 
		 cont=0;
		 bien=0;
		 // The following for loop starts at 1, because this code is clowny and the first byte is the <LF> (0x10) from the previous transmission.
		 for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPR
			 if (linea[i]==comandoGPR[i-1]){
				 bien++;
			 }
		 }
		 if(bien==6){               // If yes, continue and process the data
			 for (int i=0;i<300;i++){
				 if (linea[i]==','){    // check for the position of the  "," separator
					 // note: again, there is a potential buffer overflow here!
					 indices[cont]=i;
					 cont++;
				 }
				 if (linea[i]=='*'){    // ... and the "*"
					 indices[12]=i;
					 cont++;
				 }
			 }
			 Serial.println("");      // ... and write to the serial port
			 Serial.println("");
			 Serial.println("---------------");
			 for (int i=0;i<12;i++){
				 switch(i){
					 case 0 :Serial.print("Time in UTC (HhMmSs): ");break;
					 case 1 :Serial.print("Status (A=OK,V=KO): ");break;
					 case 2 :Serial.print("Latitude: ");break;
					 case 3 :Serial.print("Direction (N/S): ");break;
					 case 4 :Serial.print("Longitude: ");break;
					 case 5 :Serial.print("Direction (E/W): ");break;
					 case 6 :Serial.print("Velocity in knots: ");break;
					 case 7 :Serial.print("Heading in degrees: ");break;
					 case 8 :Serial.print("Date UTC (DdMmAa): ");break;
					 case 9 :Serial.print("Magnetic degrees: ");break;
					 case 10 :Serial.print("(E/W): ");break;
					 case 11 :Serial.print("Mode: ");break;
					 case 12 :Serial.print("Checksum: ");break;
				 }
				 for (int j=indices[i];j<(indices[i+1]-1);j++){
					 Serial.print(linea[j+1]); 
				 }
				 Serial.println("");
			 }
			 Serial.println("---------------");
		 }
		 conta=0;                    // Reset the buffer
		 for (int i=0;i<300;i++){    //  
			 linea[i]=' ';             
		 }                 
	 }
 }
}

void writeGPS() {
	// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  // if the file is available, write to it:
  if (dataFile) {
		for (int i=0;i<300;i++){
			dataFile.print(linea[i]);
			if (linea[i]=='*'){    // ... and the "*"
				dataFile.println("");
				break;
			}
		}
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}

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