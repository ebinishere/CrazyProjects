
#include <Wire.h>
#include <UnoWiFiDevEd.h>
#include "privatedata.h"  // Hiding my private data. Comment this to test.

#define MOIST_SENS_NUM        1
#define MOIST_PUMP_NUM        1

//#define DEBUG_PRINT_ENABLE  // Uncomment this to get logs in serial port
#ifdef DEBUG_PRINT_ENABLE
#define DBPRINTLN             Serial.println
#define DBPRINT               Serial.print
#else
#define DBPRINTLN             //
#define DBPRINT               //
#endif

uint8_t   moist_sens_pin[]    = { A0, A1, A2, A3 };
uint8_t   moist_sens_pwr[]    = { 3, 4, 5, 6 };

uint8_t   moist_pump_pin[]   = { 2, 7, 8, 10};

uint16_t  moist_pump_ontime   = 6;  // Pump On time ine each cycle - In Seconds
uint16_t  moist_cycle_time    = 30; // Check moisture cycle time - In Min
uint16_t  moist_thres_low     = 65; // Moisture lower threshold - In %
uint16_t  moist_samp_count    = 5;  // Number of samples - In Count

uint16_t  moist_sens_accum[MOIST_SENS_NUM]  = { 0 };
uint16_t  moist_sens_avj[MOIST_SENS_NUM]    = { 0 };

#define CONNECTOR             "rest" 
#define SERVER_ADDR           "api.thingspeak.com"

// Hiding my private data
#ifndef APIWKEY_THINGSPEAK
#define APIWKEY_THINGSPEAK    "M8HXOHBBHJ8FGEW0" //Insert your API Write Key
#endif
#ifndef APIRKEY_THINGSPEAK
#define APIRKEY_THINGSPEAK    "D6C2MNL7ORVK404H" //Insert your API Read Key
#endif

uint8_t   i,j,pump_on_req;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  delay(1000);
  DBPRINTLN("Starting Home Irrigation System V 3.0");
  
  // Configuring and Initializing pins
  for( i=0; i<MOIST_SENS_NUM; i++)
  {
    pinMode(moist_sens_pin[i], INPUT);
    pinMode(moist_sens_pwr[i], OUTPUT);
    digitalWrite(moist_sens_pwr[i], LOW);
  }
  for( i=0; i<MOIST_PUMP_NUM; i++)
  {
    pinMode(moist_pump_pin[i], OUTPUT); 
    digitalWrite(moist_pump_pin[i], LOW);
  }

  Ciao.begin(); // CIAO INIT
  
  DBPRINTLN("Initialized Hardware");
}

void loop() {
  uint16_t  sens_val;

  // Reset data
  for( i=0; i<MOIST_SENS_NUM; i++)
  {
    moist_sens_accum[i] = 0;
    moist_sens_avj[i] = 0;
  }
  pump_on_req = false;

  // Turn On sensor before reading
  for( i=0; i<MOIST_SENS_NUM; i++)
  {
    digitalWrite(moist_sens_pwr[i], HIGH);
  }

  // Wait for 2 second for sensor to start
  DBPRINTLN("Getting sensors ready");
  delay(2000);

  // Read sensor and take average
  for( j=0; j < moist_samp_count; j++)
  {
    for( i=0; i<MOIST_SENS_NUM; i++)
    {
      sens_val = analogRead(moist_sens_pin[i]);
      DBPRINT(" Moisture Raw : ");
      DBPRINTLN(sens_val);
      sens_val = constrain(sens_val, 130, 570);
      sens_val = map(sens_val, 130, 570, 100, 0);;
      DBPRINT(" Moisture Mapped : ");
      DBPRINTLN(sens_val);
      moist_sens_accum[i] += sens_val;
      delay(100);
    }  
  }
  
  // Turn Off sensor after reading
  DBPRINTLN("Turning off sensors");
  for( i=0; i<MOIST_SENS_NUM; i++)
  {    
    digitalWrite(moist_sens_pwr[i], LOW);
  }
  
  for( i=0; i<MOIST_SENS_NUM; i++)
  {
    moist_sens_avj[i] = moist_sens_accum[i] / moist_samp_count;

    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(" Moisture % : ");
    Serial.println(moist_sens_avj[i]);

    String uri = "/update?api_key=";
    uri += APIWKEY_THINGSPEAK;
    uri += "&field1=";
    uri += String(moist_sens_avj[i]);

    Ciao.println("Send data on ThingSpeak Channel"); 
    Serial.println("Send data on ThingSpeak Channel"); 
    CiaoData data = Ciao.write(CONNECTOR, SERVER_ADDR, uri);

    if (!data.isEmpty()){
      Ciao.println( "State: " + String (data.get(1)) );
      Ciao.println( "Response: " + String (data.get(2)) );
      
      Serial.println( "State: " + String (data.get(1)) );
      Serial.println( "Response: " + String (data.get(2)) );
    }
    else{ 
      Ciao.println("Write Error");
      Serial.println("ThingSpeak Write Error"); 
    }

    if(moist_sens_avj[i] < moist_thres_low)
      pump_on_req = true;
  }
  
  // Turn pumps on for configured time moisture is low
  if( pump_on_req )
  {
    DBPRINTLN("Turning on the Pumps");
    for( i=0; i<MOIST_PUMP_NUM; i++)
    {
      digitalWrite(moist_pump_pin[i], HIGH);
    }
    delay(moist_pump_ontime*1000);
    for( i=0; i<MOIST_PUMP_NUM; i++)
    {
      digitalWrite(moist_pump_pin[i], LOW);
    }
    DBPRINTLN("Pumps turned off");
  }

  // Force pumps off and wait for cycle time
  for( i=0; i<MOIST_PUMP_NUM; i++)
  {
    digitalWrite(moist_pump_pin[i], LOW);
  }
  Serial.println("Waiting for next cycle ...");
  delay(moist_cycle_time*60*1000UL);
}

