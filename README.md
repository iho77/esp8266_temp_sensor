# esp8266_temp_sensor
Wemos D1 mini based temperature\humidity battery powered sensor.
Main features:
- DeepSleep and radio off power consumption optimization
- Measure every 5 min, send data only than readings exceed from thresholds + every one hour
- rtc mem used for uptime\wakeup counter 

About 2 weeks on 2700 mAh battery. Can not measure more precisely due to everyweek improvements :)
ToDo:
- make call back for WiFi - mqtt connections routine due to waste time on delay()
- make call back routine for mqtt.send (to delete delay() nedded for all packet to be send)
- chnage dht22 on something more fast (without 2 sec delay between measures)
