current stuff haven't integrate PID yet but wifi is able to transmit all values printed out/shown in Tera Term 

How to run the broker thingy

1) Download the mosquitto thingy and install 
2) Create C:\mosquitto and paste the dev.conf inside (or whereever you want)  
3) make sure your to build the file with your own mobile hotspot SID, password and broker IP addr (in which is your IP addr connected to hotspot)
4) build and paste into Maker Pi Pico when both broker and client are up and running


Broker Service: 
- on power shell run  
 & "C:\Program Files\mosquitto\mosquitto.exe" -v -c "C:\mosquitto\dev.conf"

edit the back half to locate the dev.conf file if not at C:\mosquitto

Client:
on another powershell, run the following:
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h 192.168.244.251 -t "#" -v