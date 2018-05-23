/*
  AgroClimate  Datalogger by Thiago B. Onofre

  Pin:

   SD card attached to SPI bus as follows :
 ** MOSI - pin 51
 ** MISO - pin 50
 ** CLK  - pin 52
 ** CS   - pin 53

   Date and time functions using a DS1307 RTC connected via I2C and Wire lib
   SLC - A5 -
   SDA - A4

  Jan 12th, 2018 - Swtching from arduino Uno to Arduino Mega



  library declaration */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <XBee.h>


//configuring the xbee radio
// create the XBee object
XBee xbee = XBee();

//  // SH + SL Address of receiving XBee (gateway)
//  XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x406E61AC);
//   char payload[30];
//   ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
//  ZBTxStatusResponse txStatus = ZBTxStatusResponse();
//

XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
XBeeAddress64 remoteAdd64 = XBeeAddress64(0x0013a200, 0x00000000);
ModemStatusResponse msr = ModemStatusResponse();

int statusLed = 13;
int errorLed = 13;
int dataLed = 13;

/* I am using the am2315 temperature and relative hum. sensor.
  This sensor has a digital interface (I2C).
  This sensor goes in paralel with the RTC
  For hookup details using this sensor then visit
  5  http://cactus.io/hookups/sensors/temperature-humidity/am2315/hookup-arduino-to-am2315-temp-humidity-sensor
  Connect RED of the AM2315 sensor to 5.0V
  Connect BLACK to Ground
  Connect WHITE to i2c clock - on 'SCL' 21 Arduino MEGA
  Connect YELLOW to i2c data - on 'SDA' 20 Arduino MEGA
*/
//#include <Wire.h>
#include <Adafruit_AM2315.h>
Adafruit_AM2315 am2315;

/*As I am using the adafruit temp/rel sensor from Adafruit, it is required to
  add the following license: */
/***************************************************
  This is an example for the AM2315 Humidity + Temp sensor

  Designed specifically to work with the Adafruit BMP085 Breakout
  ----> https://www.adafruit.com/products/1293

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
// global variables
//sd card variables
const int chipSelect = 53;
File dataFile;

// rtc variables
RTC_DS1307 rtc;

//Configure timer and datalogging
// part of the code is from   http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time task was updated
// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

unsigned char previousHour , currentHour, previousMin , currentMin, seconds, rtc_running;   // will store last time minute

//debug variables
#define DEBUG_ON 1
#define DEBUG_OFF 0
char _DEBUG;


// constants won't change. Used here to set a pin number:
const int ledPin =  13;// the number of the LED pin
// Variables will change:
//int ledState = LOW;

#define SAMPLE_TIME 2
#define TXING_TIME 2

char transmittingData ; // software switch for transmitting data

char loggingData ; // software switch for logging data

//Function Prototypes
void SerialInit();
void SDCardInit();
void RTCInit();
void GPSInit();
void Temp_Rel_Sensor_Init();
void datalog_printHeader();
void datalog(String dataString);
void printRTC(char var);


//SENSORS
// TEMPERATURE VARIABLES
float tempPrevious;
float tempCurrent;
float tempAverage;

// RELATIVE HUMIDITY VARIABLES
float rhPrevious;
float rhCurrent;
float rhAverage;

// RAIN FALL SENSOR
const byte RAIN = 2;

float rain15Min; // [rain inches over the past 15min)] -- the accumulated rainfall in the past 15 min
volatile float dailyrainin; // [rain inches so far today in local time]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;


//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rain15Min   += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}



// want to build a state machine for the datalogger

int state1Led = 13;
int state2Led = 12;
int state3Led = 11;
int state4Led = 10;
int state5Led = 9;
int state6Led = 8;
int state7Led = 7;

int ledState = 0;

unsigned long timer;


struct flags {
  char logging;
};

struct requests {
  char sensor;
  char gps;
};

struct datalogCMD {
  char start;
  char pause;
  int loggingInterval;
  struct flags flag;
  struct requests request;
};

struct datalogCMD dataLogCMD;

/*
  States of the datalogger.
  states are mainly the smallest steps composed by functions the datalogger have to execute.
*/
enum state {
  idle = 0,
  sampling,
  processing,
  logging,
  transmitting,
  receiving,
  gpsReading
};

enum state currentState = 0;
enum state nextState;


/* sensor log data
  Global id, timeStamp, stationID, temperature, rh, rainfall
  example:
  34,12/32/2018, 13:00, 34,32,3,

  question: is the staionID part of the sensorLogField or this should be part of a configuration structure ?
*/
struct sensorLogFields {
  long int globalID;
  char timeStamp[10];
  char stationID[10];
  float temperature;
  float relativeHumdity;
  float rainFall;
};

struct sensorLogFields sensorLogData;
String sensorLog;




/*GPS CODE
// receives data from serial2
// finds the start delimiter
// receives and stores data until it finds any of the end delimiter: * 0x0D 0x0A 
// when receiving data, it checks for 0XFF, which is just trash... 
// also checks for possible new messages delimiter '$', we dont want to store them. 
// adds data into a buffer, simply an array.
// it checks for possible overflows. There is a variable counting the number of incoaming char
// if the number is greater than 99, it stops storing data. 
// finally, prints the string and the number of received characters in the serial. 
// while I am receiving the message, I dded a  comma counter. 
//  I am storing the position of the comma in a separated array named gpsCommaPosition. 
//   As NMEA as fixed positions for its API, knowing the comma position will make it easier for find the fields. 
// 
// next step is to save lat long in an array and add this to the datalogger code. 
// is to decode the received message. 
*/
 
   char gpsInString[300]; // I still need to check how many bytes the gps radio will dump into the serial port
   String gpsLatLongString; // 
   int  gpsCommaPosition[50];
   char t_out_gps = 30;
   int  GPSHwSwitch = 6; // connected to a npn transistor (base pin) (emissor = gnd arduino, coletor = GND radio)
 
   char gps_on = 0; 
   char latitude[10];
   char longitude[10];
   char lat_long = 0;

   char gpsReadFlag =0;
   char gpsStarting = 0;
   
   /*
  States of the gps radio.
  */
enum StateGPS {
  starting = 0,
  listening,
  turnOff,
};

enum StateGPS stateGPS = 0;

    
   char comandoGPR[7] = "GPRMC";
   int i;



void setup() {
  _DEBUG = DEBUG_ON;
  Serial.begin(9600);
  SDCardInit();
  SD_createFile("datalog.txt");
  SD_createFile("info.txt");
  SD_createFile("gps.txt");
  datalog_printHeader();
  RTCInit();
  GPS_Initialize();
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
  //    inputString.reserve(200);
  //
  pinMode(state1Led, OUTPUT);
  pinMode(state2Led, OUTPUT);
  pinMode(state3Led, OUTPUT);
  pinMode(state4Led, OUTPUT);
  pinMode(state5Led, OUTPUT);
  pinMode(state6Led, OUTPUT);
  pinMode(state7Led, OUTPUT);

  dataLogCMD.loggingInterval = 5;
  sensorLogData.globalID = 0; // reset the global identifier for the sensor data. (have to find a better way to keep the sample id - maybe using the eeprom -

  //   dataLogCMD.flag.logging = 0;

  // put your setup code here, to run once:

}

void loop() {

  // do something different depending on the range value:
  switch (currentState) {
    case idle:    // 

      //check if data loger is enabled, if it is, go to sampling, otherwise stays in idle
      // if the datalogger is working, the green led blinks, otherwise is solid


      /*This block of code, gets the time updates every second (1000 milis),
        if the rtc is working, it reads the date and time
        if the rtc is not working, it reads the internal calendar system.
      */
      unsigned long currentMillis;
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        // save the last time you executed your task
        previousMillis = currentMillis;
        DateTime now = rtc.now();
        currentMin = now.minute();

        printRTC('a'); // prints hh:mm:ss

        //if the rtc is not running, we have to mannually update the time
        if (!rtc.isrunning()) {
          printRTC('t'); // prints hh:mm:ss
          seconds++;
          if (seconds == 60) {
            seconds = 0;
            currentMin++;
            if (currentMin == 60) {
              currentMin = 0;
              currentHour++;
              if (currentHour == 24) {
                currentHour = 0;
                //here adds day, months and years
              }
            }
          }
        }
      }

      /*checks the datalogger logging flag.
        if the flag is 1, then the system is logging data
        if is zero, the datalogger is waiting in standby

      */
      if (dataLogCMD.flag.logging) {
        Serial.println("Idle - Logger On");
        /*we need to execute things @1min, for example
          read sensors, calculate stuff... so, we need to
          check if previous minute is  different from current minute
          if that is true, that means, we are in a "new" minute, therefore
          we have to execute the minute related tasks*/
        if (previousMin != currentMin) {

          previousMin = currentMin; // updates a new minute;
          nextState = sampling; // updates the state of the datalogger
          currentState = nextState; // goes to the next state.
        }
        // blinks the led to indicate the datalogger is working

        digitalWrite(state1Led, !digitalRead(state1Led)); // toogle led status
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // processing led
        digitalWrite(state4Led, LOW);  // logging data
        digitalWrite(state5Led, LOW);  // transmitting led
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS
      } else {

        Serial.println("Idle - Logger Off");

        nextState = idle;
        currentState = nextState;

        // turns only the idle led (green pin 2) on.
        digitalWrite(state1Led, HIGH); // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // processing led
        digitalWrite(state4Led, LOW);  // logging data
        digitalWrite(state5Led, LOW);  // transmitting led
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS

      }

      /*LOCAL COMMANDS FOR THE STATE MACHINE*/
      // read serial port, check for commands
      if (Serial.available() > 0) {
        int inByte = Serial.read();

        switch (inByte) {

          case 'a': //sample once
            nextState = sampling;
            currentState = nextState;
            //timer = millis();
            //digitalWrite(2, HIGH);
            break;

          case 'b': // status
            Serial.println("logging status: ");
            Serial.print("\t loggingInterval: ");
            Serial.print (dataLogCMD.loggingInterval);
            Serial.print("'\t' logging status: ");
            Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));

            //digitalWrite(3, HIGH);
            break;

          case 'c': // turn on datalogger

            dataLogCMD.flag.logging = 1;
            Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));


            //digitalWrite(4, HIGH);
            break;
          case 'd': // pause/stop data logging

            dataLogCMD.flag.logging = 0;
            Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));

            //digitalWrite(5, HIGH);
            break;
         case 'e': // pause/stop data logging

           nextState = gpsReading;
           currentState = nextState;
            //digitalWrite(5, HIGH);
            break;   
          case 'h':
            Serial.println("press: ");
            Serial.println("\t a -> Sample once (testing)");
            Serial.println("\t b -> Status of the Datalogger");
            Serial.println("\t c -> Start Logging Data - Green Led Blinking");
            Serial.println("\t d -> Stop Logging Data - Green Led Solid");
            Serial.println("\t e -> Read GPS RADIO");
            Serial.println("\t f -> System status");
            
            
          
            Serial.println("\t h -> Help");
            break;
            Serial.println("Help: ");
            //default:
            // turn all the LEDs off:
            //for (int thisPin = 2; thisPin < 7; thisPin++) {
            //digitalWrite(thisPin, LOW);
            //}
        }
      }

//      if(gpsReadFlag==1){
//
//        nextState = gpsReading;
//        currentState = nextState;
// 
//      }

            
      xbee.readPacket();
      if (xbee.getResponse().isAvailable()) {
                printRTC('a'); // prints hh:mm:ss
        Serial.println("Receiving");
        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // procesing led
        digitalWrite(state4Led, LOW);  // logging led
        digitalWrite(state5Led, LOW);  // transmitting data
        digitalWrite(state6Led, HIGH); //receiving data
        digitalWrite(state7Led, LOW);  //GPS
        
        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {

          // now fill our zb rx class
          xbee.getResponse().getZBRxResponse(rx);
          printRTC('a'); // prints hh:mm:ss

          /*here, the address of the sender can be filtered.
            it can be useful to learn if the data was sent by the gateway
          */
          if (rx.getRemoteAddress64().getLsb() == 0x406e61ac) {
            Serial.println("Data Received from Gateway: ");
            //            //print address
            //            Serial.print((rx.getRemoteAddress64().getMsb() >> 24) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print((rx.getRemoteAddress64().getMsb() >> 16) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print((rx.getRemoteAddress64().getMsb() >> 8) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print(rx.getRemoteAddress64().getMsb() & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print((rx.getRemoteAddress64().getLsb() >> 24) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print((rx.getRemoteAddress64().getLsb() >> 16) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print((rx.getRemoteAddress64().getLsb() >> 8) & 0xff, HEX);
            //            Serial.print(",");
            //            Serial.print(rx.getRemoteAddress64().getLsb() & 0xff, HEX);
            //            Serial.println("");

            Serial.println("data: ");
            /*
              this block of code helps with gettting the data from the xbee mesasge
              the data needs to be get using a char variable, and then converted
              into a string or int later on.
              now, I can analyze the xbee message
              for example: if a message is sent with the following character
              $message type 1
              or
              #message type 2

              *   */


            char dataRx;
            dataRx = rx.getData(0);
            
            switch (dataRx) {
              case '#': //Remote Control
                dataRx = rx.getData(1);
                switch (rx.getData(1)) {
                  case 'a': //sample once
                    Serial.print("Remote - Sample Request");
                    nextState = sampling;
                    currentState = nextState;
                    //timer = millis();
                    //digitalWrite(2, HIGH);
                    break;

                  case 'b': // status
                  
                    Serial.print("Remote - Status Request: ");
                    Serial.println("logging status: ");
                    Serial.print("\t loggingInterval: ");
                    Serial.print (dataLogCMD.loggingInterval);
                    Serial.print("'\t' logging status: ");
                    Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));

                    //digitalWrite(3, HIGH);
                    break;

                  case 'c': // turn on datalogger
                    Serial.print("Remote - Start: ");
                    dataLogCMD.flag.logging = 1;
                    Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));


                    //digitalWrite(4, HIGH);
                    break;
                  case 'd': // pause/stop data logging
                    Serial.print("Remote - Stop: ");
                    dataLogCMD.flag.logging = 0;
                    Serial.println((dataLogCMD.flag.logging) ? ("Datalogger On") : ("Datalogger Standby"));

                    //digitalWrite(5, HIGH);
                    break;
                  case 'h':
                    Serial.print("Remote - Help: ");
                    Serial.println("press: ");
                    Serial.println("\t a -> Sample once (testing)");
                    Serial.println("\t b -> Status of the Datalogger");
                    Serial.println("\t c -> Start Logging Data - Green Led Blinking");
                    Serial.println("\t d -> Stop Logging Data - Green Led Solid");
                    Serial.println("\t h -> Help");
                    break;
                    //default:
                    // turn all the LEDs off:
                    //for (int thisPin = 2; thisPin < 7; thisPin++) {
                    //digitalWrite(thisPin, LOW);
                    //}
                }
                break;
              default:
               // char dataRx;
                String strDataRx;
                dataRx = rx.getData(0);
                strDataRx += dataRx;

                Serial.print(dataRx);
                dataRx = rx.getData(1);
                strDataRx += dataRx;
                Serial.print(dataRx);
                dataRx = rx.getData(2);
                strDataRx += dataRx;
                Serial.print(dataRx);
                dataRx = rx.getData(3);
                strDataRx += dataRx;
                Serial.print(dataRx);
                dataRx = rx.getData(4);
                strDataRx += dataRx;
                Serial.print(dataRx);
                Serial.print(",");
                Serial.println(strDataRx);
                Serial.println("");
                
            }

            //            (0), DEC);
            //            Serial.print(rx.getData(1), DEC);
            //            Serial.print(rx.getData(2), DEC);
            //            Serial.print(rx.getData(3), DEC);
            //            Serial.print(rx.getData(4), DEC);
            //            Serial.print(rx.getData(5), DEC);
            //            Serial.print(rx.getData(6), DEC);
            //            Serial.print(rx.getData(7), DEC);
            //            Serial.print(rx.getData(8), DEC);
            //            Serial.print(rx.getData(9), DEC);
            


          }
//          nextState = receiving;
//          currentState = nextState;
        }

      }
        break;
      case sampling:    // your hand is close to the sensor
        printRTC('a'); // prints hh:mm:ss
        Serial.println("Sampling");

        sensorLogData.globalID++;
        sensorLogData.temperature = random(0, 5);
        sensorLogData.relativeHumdity = random(0, 100);
        sensorLogData.rainFall = random(0, 1);

        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, HIGH); // sampling led
        digitalWrite(state3Led, LOW);  // processing led
        digitalWrite(state4Led, LOW);  // logging data
        digitalWrite(state5Led, LOW);  // transmitting led
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS
        delay(1000);

        nextState = processing;
        currentState = nextState;

        break;
      case processing:    // your hand is a few inches from the sensor
        printRTC('a'); // prints hh:mm:ss
        Serial.println("Processing");

        //here it goes the calibration equations...
        //      sensorLogData.temperature *= 2;
        //      sensorLogData.relativeHumdity /=4);
        //      sensorLogData.rainFall *= 6;

        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, HIGH); // processing led
        digitalWrite(state4Led, LOW);  // logging data
        digitalWrite(state5Led, LOW);  // transmitting led
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS

        delay(1000);
        dataLogCMD.request.sensor = 1;
       
        nextState = logging;
        currentState = nextState;

        break;

      case logging:    // your hand is a few inches from the sensor
        printRTC('a'); // prints hh:mm:ss
        Serial.println("Logging");
        //led pattern (indicator)
        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // procesing led
        digitalWrite(state4Led, HIGH); // logging data
        digitalWrite(state5Led, LOW);  // transmitting led
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS
        delay(1000);

        
        if(dataLogCMD.request.sensor){
          // make a string for assembling the data to log:
          sensorLog = "";
          //
          sensorLog += String(sensorLogData.globalID);
          sensorLog += ",";
          sensorLog += String(sensorLogData.temperature);
          sensorLog += ",";
          sensorLog += String(sensorLogData.relativeHumdity);
          sensorLog += ",";
          sensorLog += String(sensorLogData.rainFall);
          sensorLog += ",";
  
          printRTC('a');
          Serial.print("Datalog String,");
  
          Serial.println(sensorLog);
          datalog("datalog.txt", sensorLog);
          dataLogCMD.request.sensor = 0;
          nextState = transmitting;
          currentState = nextState;
        }
        else if(dataLogCMD.request.gps){
           printRTC('a');
           Serial.println(gpsLatLongString);
           datalog("gps.txt", gpsLatLongString);
           dataLogCMD.request.gps = 0;
           nextState = idle;
           currentState = nextState;
        }
        break;
      case transmitting:    // your hand is a few inches from the sensor
        printRTC('a'); // prints hh:mm:ss
        Serial.println("Transmitting");
        //led pattern (indicator)
        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // procesing led
        digitalWrite(state4Led, LOW);  // logging led
        digitalWrite(state5Led, HIGH); // transmitting data
        digitalWrite(state6Led, LOW);  // receiving data
        digitalWrite(state7Led, LOW);  // GPS


        xbeetransmitData();
        delay(1000);

        nextState = idle;
        currentState = nextState;

        break;
      case receiving:    // your hand is nowhere near the sensor
        printRTC('a'); // prints hh:mm:ss
        Serial.println("Receiving");
        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // procesing led
        digitalWrite(state4Led, LOW);  // logging led
        digitalWrite(state5Led, LOW);  // transmitting data
        digitalWrite(state6Led, HIGH); //receiving data
        digitalWrite(state7Led, LOW);  //GPS



        delay(1000);
        nextState = idle;
        currentState = nextState;
        break;
      case gpsReading:    // your hand is nowhere near the sensor
        
        Serial.println("GPS - Reading");
        digitalWrite(state1Led, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(state2Led, LOW);  // sampling led
        digitalWrite(state3Led, LOW);  // procesing led
        digitalWrite(state4Led, LOW);  // logging led
        digitalWrite(state5Led, LOW);  // transmitting data
        digitalWrite(state6Led, LOW);  //receiving data
        digitalWrite(state7Led, HIGH); //GPS
//        printRTC('a'); // prints hh:mm:ss
//        Serial.println("GPS_Reading: ");
        GPS_read();
//        printRTC('a'); // prints hh:mm:ss

        if(dataLogCMD.request.gps){ // if this flag was set, means the radio found lat and long
          nextState = logging;   // goes to the logging state, stores gps data in the file
          currentState = nextState; // there, it reset the request flag and goes back to idle loop.
        }else{ // here I can give a second chance to the user/radio, as the coordinates where not found.
          nextState = idle;
          currentState = nextState;
        }
        
        
       
        break;
      }

      delay(1000);
  }


  /*
    digitalWrite(state1Led, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(state2Led, HIGH);
    digitalWrite(state3Led, HIGH);
    digitalWrite(state4Led, HIGH);
    digitalWrite(state5Led, HIGH);
    delay(1000);                       // wait for a second
    digitalWrite(state1Led, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(state2Led, LOW);
    digitalWrite(state3Led, LOW);
    digitalWrite(state4Led, LOW);
    digitalWrite(state5Led, LOW);
    delay(1000);
    // put your main code here, to run repeatedly:

  */




  /*
    input: the name of the file and the data
    possible names: datalog.txt and data
    info.txt data
    gps.txt gpsdata
  */
  void datalog(String FileNameString, String dataString) {

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    dataFile = SD.open(FileNameString, FILE_WRITE);

  
    // if the file is available, write to it:
    if (dataFile) {
 
        DateTime now = rtc.now();

        dataFile.print(now.year(), DEC);
        dataFile.print('/');
        dataFile.print(now.month(), DEC);
        dataFile.print('/');
        dataFile.print(now.day(), DEC);
        dataFile.print(',');
        dataFile.print(now.hour(), DEC);
        dataFile.print(':');
        dataFile.print(now.minute(), DEC);
        dataFile.print(',');
      
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      //Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
  }

  void datalog_printHeader(void) {

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    dataFile = SD.open("datalog.txt", FILE_WRITE);

    dataFile.println("YY/MM/DD,hh:mm,TEMP(C_deg),RH(%),");

    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);

  }


  /*print rtc uses a case to return different options of time/calendar:
     see menu:
    a -> all
    h-> hour
    m-> minute
    s-> second
    M-> month
    D-> day
    Y -> year
    C -> calendar YY MM DD
    t -> time hh mm ss
  */
  void printRTC(char var) {
    DateTime now = rtc.now();

    if (rtc.isrunning()) {

      switch (var) {
        case 'a':
          Serial.print(now.year(), DEC);
          Serial.print('/');
          Serial.print(now.month(), DEC);
          Serial.print('/');
          Serial.print(now.day(), DEC);
          Serial.print(',');

          /* Serial.print(" (");
            Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
            Serial.print(") ");
          */
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          Serial.print(now.second(), DEC);
          Serial.print(',');
          break;
        case 'D':
          Serial.println(now.day(), DEC);
          break;
        //      case 'W':
        //              Serial.print(" (");
        //              Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
        //              Serial.println(") ");
        //        break;
        case 'M':
          Serial.println(now.month(), DEC);
          break;
        case 'Y':
          Serial.println(now.year(), DEC);
          break;
        case 'c':

          Serial.print(now.year(), DEC);
          Serial.print('/');
          Serial.print(now.month(), DEC);
          Serial.print('/');
          Serial.print(now.day(), DEC);
          Serial.println(',');

          break;
        case 'h':
          Serial.println(now.hour(), DEC);
          break;
        case 'm':
          Serial.println(now.minute(), DEC);
          break;
        case 's':
          Serial.println(now.second(), DEC);
          break;
        case 't':
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          Serial.print(now.second(), DEC);
          Serial.println();
          break;
        default:
          Serial.print(now.year(), DEC);
          Serial.print('/');
          Serial.print(now.month(), DEC);
          Serial.print('/');
          Serial.print(now.day(), DEC);
          Serial.print(',');
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          Serial.print(now.second(), DEC);
          Serial.println();
      }
    } else {
      Serial.print(currentHour, DEC);
      Serial.print(':');
      Serial.print(currentMin, DEC);
      Serial.print(':');
      Serial.print(seconds, DEC);
      Serial.println();
    }
  }
  void SerialInit(void) {

    // Open serial with the computer

    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }

    if (_DEBUG) {
      Serial.println("Serial Initialized - 9600bps.");
    }
  }

  void SDCardInit() {

    // SD card initialization
    if (_DEBUG) {
      Serial.println("Initializing SD card.");
    }

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      if (_DEBUG) {
        Serial.println("Card failed, or not present");
      }

      // don't do anything more:
      return;
    }
    if (_DEBUG) {
      Serial.println("card initialized.");
    }
  }

  /* create file allows the user to create multiple files by calling:
     for example: SD_createFile("datalog.txt");
     Onofre,TB Jan 20th 2018
  */
  void SD_createFile(String FileName_txt) {

    // open a new file and immediately close it:
    // Check to see if the file exists:
    if (SD.exists(FileName_txt)) {
      // Serial.println("datalog.txt exists.");
    }
    else
    {
      //  Serial.println("datalog.txt doesn't exist.");
      if (_DEBUG) {
        Serial.print("Creating file: ");
        Serial.println(FileName_txt);
      }
      dataFile = SD.open(FileName_txt, FILE_WRITE);
      dataFile.close();
    }
  }

  void RTCInit() {

    Wire.begin();
    rtc.begin();
    // RTC initialization
    if (_DEBUG) {
      Serial.println("Initializing RTC.");
     // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      
    }

    if (! rtc.begin()) {
      if (_DEBUG) {
        Serial.println("Couldn't find RTC");
      }
    }

    if (rtc.isrunning()) {

      //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtc.adjust(DateTime(__DATE__, __TIME__));
      Serial.println("RTC initialized. Current time: ");
      printRTC('a');

      rtc_running = 1; // this flag tells main loop the rtc wasn't initialized.
      //manually initialize time variables.
      seconds = 0;
      currentMin = 0;
      currentHour = 0;
      previousMin  = 99;
      previousHour = 99;
    }
    //   else{ // if rtc is running, print and update the time.
    //    rtc_running = 1; // this variable tells main program the rtc is running.
    //
    //    // following line sets the RTC to the date & time this sketch was compiled
    //    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //
    //    // This line sets the RTC with an explicit date & time, for example to set
    //    // January 21, 2014 at 3am you would call:
    //    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    //      // calendar global variables
    //      previousMin  = 99;
    //      currentMin   = 99;
    //      previousHour = 99;
    //      currentHour  = 99;
    //    // print rtc current time
    //    if (_DEBUG) {
    //      Serial.println("RTC initialized. Current time: ");
    //      printRTC('a');
    //    }
    //
    //  }
  }

  void Temp_Rel_Sensor_Init() {
    if (! am2315.begin()) {
      if (_DEBUG) {
        Serial.println("Sensor not found, check wiring & pullups!");
      }
    }

    if (_DEBUG) {
      Serial.println("Sensor Initialized");
      Serial.print("Hum: "); Serial.println(am2315.readHumidity());
      Serial.print("Temp: "); Serial.println(am2315.readTemperature());
    }

  }


  void xbeetransmitData(void) {
    // SH + SL Address of receiving XBee (gateway)
    XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x406E61AC);
    char payload[30];
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
    ZBTxStatusResponse txStatus = ZBTxStatusResponse();
    sensorLog.toCharArray(payload, 20);
    xbee.send(zbTx);

    // flash TX indicator
    flashLed(statusLed, 1, 100);

    // after sending a tx request, we expect a status response
    // wait up to half second for the status response
    if (xbee.readPacket(500)) {
      // got a response!

      // should be a znet tx status
      if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(txStatus);

        // get the delivery status, the fifth byte
        if (txStatus.getDeliveryStatus() == SUCCESS) {
          // success.  time to celebrate
          flashLed(statusLed, 5, 50);
        } else {
          // the remote XBee did not receive our packet. is it powered on?
          flashLed(errorLed, 3, 500);
        }
      }
    } else if (xbee.getResponse().isError()) {
      //Serial.print("Error reading packet.  Error code: ");
      //Serial.println(xbee.getResponse().getErrorCode());
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen
      flashLed(errorLed, 2, 50);
    }
  }

  void flashLed(int pin, int times, int wait) {

    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);

      if (i + 1 < times) {
        delay(wait);
      }
    }
  }

  void blinkled() {

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);

  }


/*GPS functions, added on april 17th 2018
 * 
 */

 

void GPS_Initialize(void){
  
  Serial2.begin(9600);
  pinMode(GPSHwSwitch, OUTPUT);       // Initialize gps software switch
}

void GPS_PowerON(void){
  if(gps_on == 0){
    Serial.println("Starting GPS Radio ...");                                                                  

    digitalWrite(GPSHwSwitch, HIGH);
    int _gpsAux = 45;
    while(_gpsAux>=0){
      delay(1000);
      _gpsAux--;
      digitalWrite(state7Led, !digitalRead(state7Led)); // toogle led status

    }
//    delay(45000);
    Serial.println("radio On");                                                                  
    digitalWrite(state7Led, HIGH); // toogle led status


    gps_on = 1;
  }
}

void GPS_PowerOFF(void){
 if(gps_on == 1){
        Serial.println("radio Off");                                                                  

    digitalWrite(GPSHwSwitch, LOW);
    delay(100);
    gps_on = 0;
  }
}


void GPS_read(void){

  int inCharCount =0, inCharCountComma = 0;
  int gps_newMessage = 0;
  if( gps_on == 0){
    GPS_PowerON();
  }
  int _gpsAux = 0;
  while(lat_long==0){
        Serial.println("waiting for latlong");                                                                  
        if(_gpsAux>500){
                  break;
         }
         inCharCount =0;
         inCharCountComma = 0;
        while(Serial2.available()>0){
           _gpsAux++;
           //Serial.println(_gpsAux);
           
        unsigned char inchar; //needs to be unsiged to receive all 255 characters.
        inchar = Serial2.read();
        if(inchar=='$'){
          inchar = Serial2.read();
          if(inchar!= 255){
              while((inchar!='*')&&(inchar!=0x0D)&&(inchar!=0x0A)&&(inchar!='$')){
                
                 // store only if the data is not 0XFF, new message, and the buffer is not full (99 chars) 
                 if((inchar!= 255)&&(inchar!='$')&&(inCharCount<=99)){ //check if it is valid data
                    if(inchar == ','){
                      gpsCommaPosition[inCharCountComma] = inCharCount;
                      inCharCountComma++;
                    }
                    gpsInString[inCharCount] = inchar;
                    inCharCount++;
                    //Serial.write(inchar);
                 }
                 inchar = Serial2.read();
              }
             // Serial.println(gpsInString);
             gpsInString[inCharCount] = 0;
             gpsCommaPosition[inCharCountComma++] = 999;
             gps_newMessage += 1;// signs the new message
            }
 
         }
         
      }
    
    
//        Serial.println(gpsInString);                                                                  
      
         if(gps_newMessage >= 1){

          // this group of serial print help to debug the number of messages the gps radio received
          // uncoment then and also uncoment the serial print that prints (kind of redundant here) the lat and log in the ifs bellow. 
//              Serial.print("# of new messages" );
//           
//              Serial.println(gps_newMessage);
//                          
//              Serial.print("# of characters" );
//
//              Serial.println(inCharCount);
              inCharCountComma = 0;
              gps_newMessage = 0;
 
              inCharCount = 0;
              if(gpsInString[0] == 'G'){
                  if(gpsInString[1] == 'P'){
                     if(gpsInString[2] == 'R'){
                        if(gpsInString[3] == 'M'){
                           if(gpsInString[4] == 'C'){
    
    //                            Serial.println("GPRMC >>");
    //                            Serial.println( gpsInString);
    //                            Serial.print ("time utc: ");
    //                            for (i = (gpsCommaPosition[0]+1); i<= (gpsCommaPosition[1]-1); i++){
    //                                Serial.print(gpsInString[i]);
    //                                 
    //                            }
    //                           // Serial.println(" ");
                                
                             //   Serial.print ("status: ");
                                for (i = (gpsCommaPosition[1]+1); i<= (gpsCommaPosition[2]-1); i++){
                                  if(gpsInString[i]=='A'){
                                    // valid message arrived
                                    // 
                                    //  digitalWrite(GPSHwSwitch, LOW); // turn off the gps radio
                                   //   Serial.println(" ");
                                        GPS_PowerOFF();
                                        lat_long = 1;
                                       for (i = (gpsCommaPosition[2]+1); i<= (gpsCommaPosition[6]-1); i++){
                                            //Serial.print(gpsInString[i]); // prints: lat,N/S,long, W/E
                                            gpsLatLongString += gpsInString[i];
                                 
                                        }
                                        
                                        dataLogCMD.request.gps = 1;
                                        break;
                                        
    //                                     
    //                                    Serial.print ("Latitude: ");
    //                                    for (i = (gpsCommaPosition[2]+1); i<= (gpsCommaPosition[3]-1); i++){
    //                                        Serial.print(gpsInString[i]);
    //          
    //                                    }
    //                                    
    //                                    Serial.println(" ");
    //                                    Serial.print ("N/S: ");
    //                                    for (i = (gpsCommaPosition[3]+1); i<= (gpsCommaPosition[4]-1); i++){
    //                                        Serial.print(gpsInString[i]);
    //                                         
    //                                    }            
    //                                                   
    //                                    Serial.println(" ");
    //                                    Serial.print ("longitude: ");
    //                                    for (i = (gpsCommaPosition[4]+1); i<= (gpsCommaPosition[5]-1); i++){
    //                                        Serial.print(gpsInString[i]);
    //                                         
    //                                    }                           
    //        
    //                                    Serial.println(" ");
    //                                    Serial.print ("e/w: ");
    //                                    for (i = (gpsCommaPosition[5]+1); i<= (gpsCommaPosition[6]-1); i++){
    //                                        Serial.print(gpsInString[i]);
    //                                         
    //                                    }    
    //        //                           Serial.print ("COMMA >>");
    //        //
    //        //                            i=0;
    //        //                            while(gpsCommaPosition[i]!=999){
    //        //                               Serial.print(gpsCommaPosition[i++],DEC); 
    //        //                               Serial.print(";");
    //        //                            }
                                        Serial.println(" ");
                                            
                                  }
                               }
                           }
                        }
                      }else
                      if(gpsInString[2] == 'G'){ 
                          if(gpsInString[3] == 'G'){
                             if(gpsInString[4] == 'A'){
                                  //  digitalWrite(GPSHwSwitch, LOW); 
    
                                // Serial.println("GPGGA >>");
                                // Serial.println( gpsInString);
    //
    //                           
    //                             Serial.print ("COMMA >>");
    //                            i=0;
    //                            while(gpsCommaPosition[i]!=999){
    //                               Serial.print(gpsCommaPosition[i++],DEC); 
    //                               Serial.print(";");
    //                            }
    //                            Serial.println(" ");            
                              }
                           }else
                           if(gpsInString[3] == 'S'){
                             if(gpsInString[4] == 'A'){
                                // digitalWrite(GPSHwSwitch, LOW); 
    
    //                            Serial.println("GPGSA >>");
    //                            Serial.println( gpsInString);  
    //                             Serial.print ("COMMA >>");
    //                            i=0;
    //                            while(gpsCommaPosition[i]!=999){
    //                               Serial.print(gpsCommaPosition[i++],DEC); 
    //                               Serial.print(";");
    //                            }
    //                            Serial.println(" ");                 
                             }else
                             if(gpsInString[4] == 'V'){
                            //digitalWrite(GPSHwSwitch, LOW); 
    
    //                            Serial.println("GPGSV >>");
    //                            Serial.println( gpsInString);
    ////                             Serial.print ("COMMA >>");
    //                            i=0;
    //                            while(gpsCommaPosition[i]!=999){
    //                               Serial.print(gpsCommaPosition[i++],DEC); 
    //                               Serial.print(";");
    //                            }
    //                            Serial.println(" ");
                             }
                          }              
                       }  
                  }
              }
     // 
         }
             // Serial.println(" total chars received");
          //     inCharCount = 0;
     // Serial.println("***---***");
    //  digitalWrite(GPSHwSwitch, LOW); 
            delay(1000);

      }
      GPS_PowerOFF();
  
}

