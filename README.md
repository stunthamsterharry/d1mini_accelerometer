# d1mini_accelerometer

NodeMCU with ESP8266 reading out ADXL345 accelerometer
  by stunthamsterHarry 
 
A D1-Mini or any ESP8266 NodeMCU (only tested with D1 Mini)
drives an ADXL345 via I2C (*) and reports measurements.

Intended to work battery-driven together with Sensorserver (see that repo) ESP8266  
 
Wiring:
D1 Mini         ADXL345
========================
D1 ------------ SCL    
GND ---10kOhm---SCL

D2 ------------ SDA   both pulled up with 10kOhm
GND ---10kOhm---SDA

3.3V ---------- CS 
3.3V ---------- VCC
GND ----------- GND

What it does:  
It tries to connect to a Sensorserver , if it fails it reports to Serial;
initializes ADXL345 and continually (400Hz at the moment) reads out accelleration data.
These are smoothed (or not) ant reported in programmable (CLI) time intervals.
It can do some WII-controler style calibration, but a better way is coming soon.
Theres tap detection, double tap detection activated. 
Reporting can be started and stopped from CLI

Purpose: Demonstrate acceleration measurements in classrooms with unreliable 
network access and old desktop PCs. 
d1mini_accellerometer wirelessly reports data to Sensorserver, which forwards it via Serial USD to ArduinoIDE.
Measured data are presented with ArduinoIDE's Serial Plotter.


I've learned how easy it ist to work with registers of I2C devices these days.
It's still required to dive deep into data sheets. For reference see:
https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf

Todo: lots of improvements...


(*) I know SPI bus can do faster than I2C but 250 data points per second is good enough for me... at the moment;)
