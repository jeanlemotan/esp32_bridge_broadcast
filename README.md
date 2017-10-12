# esp32_bridge_broadcast
A SPI to ESP32 broadcast (rfmon) firmware

Dev blog: https://jeanleflambeur.wordpress.com/

This lib + firmware allows you to inject and receive packets using an esp32 module.
It's meant for streaming data - like video - similarly to the wifibroadcast project, but instead of using of the shelf wifi dongles with patched firmwares, it uses the esp32.

** WORK IN PROGRESS **  

I'm working on FEC encoding on the ESP now and the results are decent.  
The module has this fec bandwidth:  

FEC 2/4: 12 Mbyte/s (96Mbps)  
FEC 4/8: 8 Mbyte/s (64Mbps)  
FEC 6/12: 6 Mbyte/s (48Mbps)  
FEC 8/16: 4 Mbyte/s (32Mbps)  

This is more than enough for video.  
There will be overheads though, due to the extra bandwidth used by SPI & wifi transfers and scheduling overheads.  
I intend to use one core for FEC encoding and the other for fec & wifi.  


Doing the FEC on module will save both processing power on the PI and also SPI transfers - which is a bottleneck now.
It might open the posibility of connecting a JPEG camera module directly to the ESP itself.



References:

Raw Wifi hacking:  
https://github.com/ernacktob/esp8266_wifi_raw  

HSPI info:  
http://www.esp8266.com/viewtopic.php?f=13&t=7247#  
http://iot-bits.com/esp32/esp32-spi-register-description/  

PIGPIO:  
https://github.com/joan2937/pigpio  

FEC:  
https://github.com/tahoe-lafs/zfec  



