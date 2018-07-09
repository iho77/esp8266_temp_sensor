# esp8266_temp_sensor
Wemos D1 mini based temperature\humidity battery powered sensor. Coded in Eclipse, so .cpp not .ino

Main features:
- DeepSleep and radio off power consumption optimization
- Measure every 5 min, send data only than readings exceed from thresholds + every one hour
- rtc mem used for uptime\wakeup counter 
- OTA mode with internal web server
- sht30 instead DHT22 (much pricise and more simple communication)
 
 
 to switch to debug (OTA mode) send "1" to "parnik/cmd" (mosquitto_pub -h 127.0.0.1 -t "parnik/cmd" -r -m 1). Message MUST BE RETAINED to sensor read it after deep sleep wake up.
 to switch from debug mode to normal mode without update send "2"  (mosquitto_pub -h 127.0.0.1 -t "parnik/cmd" -r -m 1)
 to update over the air go to 192.168.1.141/update   files .bin from eclipse project dir/build works well
 
ToDo
looks like battery measurement not too precise, 2 weeks shows 4.2, after 0.01 drop in voltage and after can not wake up.
a lot of error checking procedures has to be written. But so boring :)
