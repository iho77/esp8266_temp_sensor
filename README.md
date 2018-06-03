# esp8266_temp_sensor
Wemos D1 mini based temperatures\humidity battery powered sensor 
Wemos D1 with DHT22 extension board and battery power board. About 3 week on 2000 mAh battery with readings every 3 min.
Readings send by MQTT only when changes more than threshold.
DHT22 has problems with deepsleep so additional delays and readings added.
MQTT client can not send last message if deepsleep is the nesxt command after client.publish, so delays added.
Normal code must be done with esp8266 no_os SDK without Arduino libs and stuff.

