#include <Arduino.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "Adafruit_Sensor.h"
extern "C" {
#include <user_interface.h>
}

#define MQTT_MAX_PACKET_SIZE 200
#define DHTPIN D4
#define DHTTYPE DHT22
#define tempChangeThr 2.0f
#define humChangeThr 5.0f
#define voltChangeThr 0.01f
#define EEPROM_ADDR 0
//#define DEBUG 1


typedef struct {
	float temp;
	float hum;
	float volt;
	bool forceTrx;
} _Data;


_Data newData, oldData;

struct {
	uint32_t crc32;
	_Data data;
} memData;


struct {
	uint32_t crc32;
	uint32_t runCount;
} rtcData;

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid     = "*";
const char* password = "*";
const char* mqttServer = "192.168.1.110";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttTopicData = "parnik/SENSOR";
const char* mqttTopicStat = "parnik/STAT";
const char* mqttTopicIn = "parnik/cmd";


const uint sleepSeconds = 300;

int errStatus;
long RSSI;
bool dataOk;

void readSensorData();
void wifiinit(void);
void initmqtt(void);
bool isDataChanged();
void sendData();
void debugOutput(const char *msg);
void proceedRtcCounter();
void readEEPROM(),writeEEPROM();
uint32_t calculateCRC32(const uint8_t *data, size_t length);




//void callback(char* topic, byte* payload, unsigned int length);


void setup() {

#ifdef DEBUG
	Serial.begin(115200);
	delay(2000);
	Serial.setDebugOutput(true);
	Serial.println("WakeUp");
#endif
   	pinMode(D0, WAKEUP_PULLUP);
	pinMode(A0, INPUT);
	WiFi.mode( WIFI_OFF );
    delay(1);
	WiFi.forceSleepBegin();
    newData.hum = 0;
    newData.temp = 0;
    newData.volt = 0;
    oldData.forceTrx = false;
    proceedRtcCounter();




}


void loop() {

	  errStatus=0;

	  readSensorData();

	  readEEPROM();

	  String msg = "%12 : "+String(rtcData.runCount % 12);
	  debugOutput(msg.c_str());

	  if (rtcData.runCount % 12 == 10) {
		  oldData.forceTrx = true;
		}

	  if (isDataChanged() || oldData.forceTrx ) {
		  wifiinit();
		  if (errStatus == 0) {
			  initmqtt();
			  if (errStatus == 0) {
				  debugOutput("Sending data");
				  sendData();
			  }
		  }
		  if (errStatus == 1 || errStatus ==2){
		  		 newData.forceTrx = true;
		  		 String msg = "Transmission failed. error code "+String(errStatus);
		  		 debugOutput(msg.c_str());
		  	 }

		  writeEEPROM();


	 }

	  WiFi.disconnect( true );
	  delay( 1 );
      ESP.deepSleep(sleepSeconds * 1000000, RF_DISABLED);
}



void debugOutput(const char *msg){
#ifdef DEBUG
	Serial.println(msg);
#endif
	delay(1);
}

void wifiinit(){
  int i = 0;
  errStatus = 0;
  IPAddress ip( 192, 168, 1, 141 );
  IPAddress gateway( 192, 168, 1, 1 );
  IPAddress subnet( 255, 255, 255, 0 );
  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent( false );
  WiFi.mode( WIFI_STA );
  WiFi.config( ip, gateway, subnet );
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
	   i++;
	   if (i>5) {
		errStatus = 1;
		return;
	   };
	   delay(2000);
	  }
  espClient.setNoDelay(true);
  RSSI = WiFi.RSSI();
}

void initmqtt(){
  int i = 0;
  client.setServer(mqttServer, mqttPort);
  debugOutput("Trying to connect to MQTT");
  client.connect("ESP32Client", mqttUser, mqttPassword );
  while (!client.connected()) {
	   i++;
	   if (i>5) {
  		errStatus = 2;
        debugOutput("Failed to connect to MQTT");
  		return;
  	   };
  	   delay(2000);
  	  }
}



void readSensorData(){

      debugOutput("Read DHT sensor");
      int i = 0;
      float t,h;

      dht.begin();
      h = dht.readHumidity();
      t = dht.readTemperature();

      while(isnan(t) || isnan(h)) {
       	  i++;
    	  delay(3000);
    	  h = dht.readHumidity();
    	  t = dht.readTemperature();
    	  if ( i>3 ) t=h=0.0;
      }

      newData.hum = h;
	  newData.temp = t;


	  newData.volt = analogRead(A0)/1023.0*4.2;
}

bool isDataChanged(){


    if( abs(newData.temp-oldData.temp) > tempChangeThr || abs(newData.hum-oldData.hum) > humChangeThr || abs(newData.volt - oldData.volt) > voltChangeThr ) {

    	debugOutput("Sensor data changed");
       	return true; }
    	else {
   		debugOutput("Sensor data not changed");
    	return false;}

}


void sendData(){

	  String msg, msg_stat;
	  char str_humidity[10], str_temperature[10], str_voltage[10];

	  // Convert the floats to strings and round to 2 decimal places
	  dtostrf(newData.hum, 1, 2, str_humidity);
	  dtostrf(newData.temp, 1, 2, str_temperature);
	  dtostrf(newData.volt, 1, 2, str_voltage);

	  msg = "{\"Time\":\"2018-05-20T16:47:35\",\"DHT22\":{\"Temperature\":"+String(str_temperature)+",\"Humidity\":"+String(str_humidity)+"},\"TempUnit\":\"C\"}";
	  msg_stat = "{\"Time\":\"2018-05-20T20:35:29\",\"Uptime\":"+String(millis()/1000+rtcData.runCount*sleepSeconds)+",\"Vcc\":"+String(str_voltage)+",\"POWER\":\"ON\",\"Wifi\":{\"AP\":2,\"SSId\":\"iho\",\"RSSI\":"+String(RSSI)+",\"APMac\":\"64:66:B3:F8:70:50\"}}";

	  debugOutput(msg.c_str());
	  debugOutput(msg_stat.c_str());


	      if (client.connected()) {
	    	  client.publish(mqttTopicData, msg.c_str(),true);
	    	  client.publish(mqttTopicStat, msg_stat.c_str(),true);
	    	  client.loop();
	    	  delay(2000);
	    	  client.disconnect();
	      }

}

uint32_t calculateCRC32(const uint8_t *data, size_t length){

  uint32_t crc = 0xffffffff;

  while (length--) {

	   uint8_t c = *data++;
	   for (uint32_t i = 0x80; i > 0; i >>= 1) {
		   bool bit = crc & 0x80000000;
		   if (c & i) {
			   bit = !bit;}
		   crc <<= 1;
		   if (bit) {
			   crc ^= 0x04c11db7;}
	   }
  }

  debugOutput(String(crc).c_str());
  return crc;

}

void proceedRtcCounter(){
	 rst_info * ri;
	 ri = ESP.getResetInfoPtr();

	 ESP.rtcUserMemoryRead(0,(uint32_t *) &rtcData,sizeof(rtcData));

	    	if (calculateCRC32((uint8_t *)&rtcData.runCount, sizeof(uint32_t)) != rtcData.crc32){
	    		debugOutput("RTC data corrupted");
	    		dataOk = false;
	    	} else {
	    		debugOutput("RTC data CRC ok");
	    		dataOk = true;
	    	}

	    	if (ri->reason != 5 || !dataOk) {
	    			rtcData.runCount = 0;
	    			rtcData.crc32 = calculateCRC32((uint8_t *)&rtcData.runCount, sizeof(uint32_t));
	    			ESP.rtcUserMemoryWrite(0,(uint32_t *) &rtcData,sizeof(rtcData));
	    		} else {
	    			rtcData.runCount++;
	    			rtcData.crc32 = calculateCRC32((uint8_t *)&rtcData.runCount, sizeof(uint32_t));
	    			ESP.rtcUserMemoryWrite(0,(uint32_t *) &rtcData,sizeof(rtcData));

	    		}

	    String msg =    "Run count"+String(rtcData.runCount);
    	debugOutput(msg.c_str());


}


void readEEPROM(){

 uint32_t CRC;

 EEPROM.begin(sizeof(memData));
 EEPROM.get(EEPROM_ADDR,memData);

 debugOutput("Stored CRC:");
 debugOutput(String(memData.crc32).c_str());

 CRC = calculateCRC32((uint8_t *) &memData.data,sizeof(_Data));
 debugOutput("Calculated CRC:");
 debugOutput(String(CRC).c_str());


		  if ( CRC != memData.crc32 ){
			  debugOutput("CRC check failed!");
			  oldData.temp = newData.temp;
		  	  oldData.hum = newData.hum;
		  	  oldData.volt = newData.volt;
		  	  oldData.forceTrx = true;

		  } else {
		  debugOutput("CRC check sucsessfull");
		  oldData.temp = memData.data.temp;
		  oldData.hum = memData.data.hum;
		  oldData.volt = memData.data.volt;
		  oldData.forceTrx = memData.data.forceTrx;}

}

void writeEEPROM(){

	memData.data.hum = newData.hum;
	memData.data.temp = newData.temp;
	memData.data.volt = newData.volt;
	memData.data.forceTrx = newData.forceTrx;
	memData.crc32 = calculateCRC32((uint8_t *) &memData.data,sizeof(_Data));
	EEPROM.put(EEPROM_ADDR,memData);
	debugOutput("Storing data in EEPROM");
	EEPROM.commit();

}
