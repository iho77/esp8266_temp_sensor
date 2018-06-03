#include <Arduino.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "Adafruit_Sensor.h"

#define MQTT_MAX_PACKET_SIZE 200
#define DHTPIN D4
#define DHTTYPE DHT22
#define tempChangeThr 2.0
#define humChangeThr 5.0
#define voltChangeThr 0.01
//#define DEBUG 1


struct {
	float temp;
	float hum;
	float volt;
	const byte flag = 111;
} newData, oldData;


DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid     = "******"; 
const char* password = "*******";
const char* mqttServer = "192.168.1.110";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttTopicData = "parnik/SENSOR";
const char* mqttTopicStat = "parnik/STAT";
const char* mqttTopicIn = "parnik/cmd";


const int sleepSeconds = 300;

int errStatus;

void readSensorData();
void wifiinit(void);
void initmqtt(void);
bool isDataChanged();
void sendData();



//void callback(char* topic, byte* payload, unsigned int length);


void setup() {

#ifdef DEBUG
	Serial.begin(115200);
	delay(2000);
	Serial.println("WakeUp");
#endif

	pinMode(D0, WAKEUP_PULLUP);
	pinMode(A0, INPUT);
    newData.hum = 0;
    newData.temp = 0;
    newData.volt = 0;
   	errStatus=0;

}


void loop() {

	  readSensorData();

	  if (isDataChanged()) {

#ifdef DEBUG
	Serial.println("Data changed, sending");
#endif
		  wifiinit();
		  initmqtt();
		  sendData();
	  }

#ifdef DEBUG
	  delay(5000);

#else
      ESP.deepSleep(sleepSeconds * 1000000, RF_DEFAULT);
#endif

}





void wifiinit(){
  int i = 0;
  errStatus = 0;
  //Serial.println("Initializing WiFi");
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED) {
	   WiFi.begin(ssid, password);
	   i++;
	   if (i>5) {
		errStatus = 1;
		//Serial.println("WiFi init error");
		return;
	   };
	   delay(5000);
	  }
 // Serial.println("WiFi init successful");
}

void initmqtt(){
  int i = 0;
  errStatus = 0;
 // Serial.println("Initializing MQTT");
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
	  client.connect("ESP32Client", mqttUser, mqttPassword );
  	   i++;
  	   if (i>5) {
  		errStatus = 2;
  	//	Serial.println("MQTT init error");
  		return;
  	   };
  	   delay(5000);
  	  }
  //  Serial.println("MQTT init successful");

//  client.setCallback(callback);
//  client.subscribe(mqttTopicIn);

}



void readSensorData(){

#ifdef DEBUG
    Serial.println("Read DHT sensor");
#endif
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

	uint addr = 0;

	EEPROM.begin(25);

	//Ñ÷èòàëè ñòàðûå äàííûå
	EEPROM.get(addr,oldData);

	//Åñëè òàì ìóñîð ñ÷èòàåì ÷òî ñòàðûå ðàâíû íîâûì
	if (oldData.flag != 111) {
		oldData.hum = newData.hum;
		oldData.temp = newData.temp;
		EEPROM.put(addr,newData);
		EEPROM.commit();
	}



    if( abs(newData.temp-oldData.temp) > tempChangeThr || abs(newData.hum-oldData.hum) > humChangeThr || abs(newData.volt - oldData.volt) > voltChangeThr ) {

#ifdef DEBUG
    	Serial.println("Sensor data changed");
#endif
    	EEPROM.put(addr,newData);
    	EEPROM.commit();
    	return true; }
    	else {
#ifdef DEBUG
    	Serial.println("Sensor data not changed");
#endif
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
	  msg_stat = "{\"Time\":\"2018-05-20T20:35:29\",\"Uptime\":0,\"Vcc\":"+String(str_voltage)+",\"POWER\":\"ON\",\"Wifi\":{\"AP\":2,\"SSId\":\"iho\",\"RSSI\":0,\"APMac\":\"64:66:B3:F8:70:50\"}}";

#ifdef DEBUG
	  Serial.println(msg);
	  Serial.println(msg_stat);
#endif

	      if (client.connected()) {
	    	  client.publish(mqttTopicData, msg.c_str(),true);
	    	  client.publish(mqttTopicStat, msg_stat.c_str(),true);
	    	  client.loop();
	    	  delay(2000);
	    	  client.disconnect();
	      }






}


