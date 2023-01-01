/*
 * Wire - I2C ACCELEROMETER ADXL345
 *
 * The WeMos D1 Mini I2C bus uses pins:
 * D1 = SCL    
 * D2 = SDA   both pulled up with 10kOhm
 * 3.3V CS + VCC
 * GND GND
 *
 * CAUTION: I tought myself programming with BASIC on 8-Bit Computers
 *          so don't expect any modern coding rules being applied 
 */


ADC_MODE(ADC_VCC);  // now system_get_vdd33() gives voltage in mV
                    // that also means, ADC not available for A0 input.... 
#include <Wire.h>
#include <Preferences.h>
#include <ESP8266WiFi.h>


// ***************** these credentials must be the same as defind the Sensorserver
#define SSID "SensorServer"   
#define PASSWORD "notMyPassword"
#define PORT ((uint16_t)160169)
// ***************** credentials end

WiFiClient client;
IPAddress server(192, 168, 4, 1);  // ip of the esp8266 WIFI-AP node
Preferences preferences;

Stream* console;
#define i2c_add (0x53)  // I2C Addresse des ADXL345

const int sclPin = D1;
const int sdaPin = D2;
bool running = true;
bool smoothing;
bool performancedata = false;
bool timeToPrint = false;
uint64_t data_count = 0;
uint64_t loop_count = 0;
uint64_t last_cycle = 0;
uint64_t cycle_time = 0;
uint32_t interrupttime = 5000000;
int16_t x, y, z;
float xs, ys, zs;          // for calibration
                           // Calibration Data
float gain[3], offset[3];  // a_x = gain*(x_value + offset)

// place to read data from ADXL345
const float naive_scale = 9.81 / 256.;
float aa[3];                                                    // raw, uncalibrated values by doing naive scaling
float scale[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };  // for better calibration
float bias[3] = { 0, 0, 0 };
byte value[1006];  // a bit larger, hunting for an stack overflow... yeah fixed it;)


#define SMOOTHING_INTERVAL 16                 // 16
int16_t smootharray[SMOOTHING_INTERVAL][3];  // for averaging out the small shorttimed variations and the jiggling
uint16_t smoothindex[3] = { 0, 0, 0 };

void preferences_init() {
  preferences.begin("ADXL345", false);
  // for old calibration - obsolete - remove when the new stuff is tested 
  gain[0] = preferences.getFloat("gain x", (9.81 / 256.));
  gain[1] = preferences.getFloat("gain y", (9.81 / 256.));
  gain[2] = preferences.getFloat("gain z", (9.81 / 256.));
  offset[0] = preferences.getFloat("offset x", 0.);
  offset[1] = preferences.getFloat("offset x", 0.);
  offset[2] = preferences.getFloat("offset z", 0.);

  // for calibration
  size_t schLen = preferences.getBytesLength("scale matrix");
  Serial.println(String(" Preferences: scale matrix bytes:") + schLen);
  if (schLen == sizeof(scale)) {
    char* buffer;
    buffer = (char*)&scale;
    preferences.getBytes("scale matrix", buffer, schLen);
  }
  schLen = preferences.getBytesLength("bias");
  Serial.println(String(" Preferences: bias bytes:") + schLen);
  if (schLen == sizeof(bias)) {
    char* buffer;
    buffer = (char*)&bias;
    preferences.getBytes("bias", buffer, schLen);
  }
  Serial.print("Calib result gain values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(scale[i][i], 6);
    Serial.print("  ");
  }
  Serial.print("\n");
  Serial.print("Calib result offset values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(bias[i], 6);
    Serial.print("  ");
  }
  // runtimeparameters
  smoothing = preferences.getBool("smoothing", true);
  interrupttime = preferences.getULong("Reporting interval", 5000000);

  preferences.end();
}

void preferences_store() {
  preferences.begin("ADXL345", false);
  preferences.putFloat("gain x", gain[0]);
  preferences.putFloat("gain y", gain[1]);
  preferences.putFloat("gain z", gain[2]);
  preferences.putFloat("offset x", offset[0]);
  preferences.putFloat("offset y", offset[1]);
  preferences.putFloat("offset z", offset[2]);
  preferences.putBytes("scale matrix", &scale, sizeof(scale));
  preferences.putBytes("bias", &bias, sizeof(bias));
Serial.print("Calib result gain values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(scale[i][i], 6);
    Serial.print("  ");
  }
  Serial.print("\n");
  Serial.print("Calib result offset values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(bias[i], 6);
    Serial.print("  ");
  }

  preferences.putBool("smoothing", smoothing);
  preferences.putULong("Reporting interval", interrupttime);
  preferences.end();
}

// some zeroing
void smooth_init() {
  for (int i = 0; i < SMOOTHING_INTERVAL; i++) {
    for (int j = 0; j < 3; j++) {
      smootharray[i][j] = 0;
    }
  }
}

float smooth(int16_t val, uint direction) {
  // direction = 0,1,2 for x,y,z
  if (direction > 2) return -99999999.; // should never happen
  smootharray[smoothindex[direction]][direction] = val;
  if (smoothindex[direction]++ >= SMOOTHING_INTERVAL) smoothindex[direction] = 0;
  //Serial.println(String("smoothing")+direction+" "+val);

  float res = 0.;
  int16_t max = -9999, min = 9999;
  // sum it all up
  for (int i = 0; i < SMOOTHING_INTERVAL; i++) {
    if (smootharray[i][direction] > max) max = smootharray[i][direction];
    if (smootharray[i][direction] < min) min = smootharray[i][direction];
    res += (float)(smootharray[i][direction]);
  }
  if (max == -9999 || min == 9999) {
    Serial.println(String("***ERROR:  something went terribly wrong in float smooth(int16_t val, uint direction) ***"));
    Serial.println(String("*** dir:") + direction + " val:" + val);
    Serial.println(String("*** res:") + res + " max:" + max + " max:" + max);
  }
  // remove higest and lowest...
  res = res - min - max;
  // ....and average over what remains  
  res = res / float(SMOOTHING_INTERVAL - 2.);

  return res;
}


IRAM_ATTR void my_Timerint() {  // Interrupt: it's...
  timeToPrint = true;
  timer1_write(interrupttime);  //120000 us
  return;
}

void i2c_write_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2c_add);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void init_tap_funktion() {
  i2c_write_register(0x1D, 60);    // TAP Threshhold > 48
  i2c_write_register(0x21, 24);    // DUR
  i2c_write_register(0x22, 24);    // Latent time in 0.125ms -
  i2c_write_register(0x23, 180);   // WINDOW in 0.125ms
  i2c_write_register(0x2A, 1);     // TAP enabled ->only z-Axis
  i2c_write_register(0x2C, 0x0C);  // Data Rate 0x0A=100HZ 0x0E=1600Hz

  i2c_write_register(0x2F, 0x80);  // Interrupt MAP: dataReady to Int2, all otherINT1
  i2c_write_register(0x2E, 0xE0);  // Interrupt enable:   dataReady + double tap
    // these two are bitmaps with the following interpretation
    // dataReady  SingleTap doubleTap, Activity, inactivity, Freefall, Watermark, Overrun
  i2c_clearinterrupts();

  timer1_attachInterrupt(my_Timerint);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(interrupttime);  //120000 us
}

void i2c_clearinterrupts() {
  Wire.beginTransmission(i2c_add);
  Wire.write(0x30);
  Wire.endTransmission();
  Wire.beginTransmission(i2c_add);
  Wire.requestFrom(i2c_add, 1);
  while (Wire.available()) {
    //    Serial.print(Wire.read(), HEX);
    Wire.read();
  }
  Wire.endTransmission();
}

bool i2c_data_available(int reg = 0x30) {
  uint8_t bitmap = 0;
  Wire.beginTransmission(i2c_add);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(i2c_add);
  Wire.requestFrom(i2c_add, 1);
  if (Wire.available()) bitmap = Wire.read();
  // if(bitmap&0x01) Serial.print("Overrun ");
  // if(bitmap&0x02) Serial.print("Watermark ");
  // if(bitmap&0x04) Serial.print("Free_Fall ");
  // if(bitmap&0x08) Serial.print("Inactivity ");
  // if(bitmap&0x10) Serial.print("Activity ");
  if (bitmap & 0x20) Serial.print("Double_Tap");
  if (bitmap & 0x40) Serial.print("Single_Tap\n");
  // if(bitmap&0x80) Serial.print("dataReady");
  //if (bitmap & 0x7D) Serial.println();
  if (bitmap & 0x80) return true;
  else return false;
}
// read adxl345 acceleration values
bool i2c_request_data(int reg = 0x32) {  // 0x32
  Wire.beginTransmission(i2c_add);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(i2c_add);
  Wire.requestFrom(i2c_add, 6);
  int i = 0;
  while (Wire.available()) {
    if (i < 6) {
      value[i] = Wire.read();
      //Serial.println(String(" *")+i+" "+value[i]);
    } else {
      Serial.println(String("***i2c_request_data * i:") + i);
    }

    i++;
  }
  Wire.endTransmission();
  x = (((int16_t)value[1]) << 8) | value[0];
  y = (((int16_t)value[3]) << 8) | value[2];
  z = (((int16_t)value[5]) << 8) | value[4];
  return (i == 6);
}


void setup() {
  Wire.begin(sdaPin, sclPin);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // set mode to wifi station
  WiFi.begin(SSID, PASSWORD); // connect to Wifi-AP
  delay(500);
  Serial.println(" Accelerometer - searching for Sensorserver AP");
  int tries = 0;
  console = &Serial;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {  // check status of connection
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("connecting to server ");
    if (client.connect(server, PORT)) {
      console = &client;
      Serial.println("connected to server ");
    } else Serial.print("connecting to server ***** FAILED *****"); // and console still points to Serial
  }
  preferences_init();

  console->println("\nAccellerometer");
  smooth_init();
  i2c_write_register(0x2D, 0);   // zero registers  0x2D  = Power_ctl register
  i2c_write_register(0x2D, 16);  // toggle auto sleep
  i2c_write_register(0x2D, 8);   // toggle measure

  init_tap_funktion();
  last_cycle = millis();
}



void calib_datasampling() {
  for (int i = 0; i < 16; i++) {
    i2c_request_data();
    xs = naive_scale *smooth(x, 0);
    ys = naive_scale *smooth(y, 1);
    zs = naive_scale *smooth(z, 2);
    delay(100);
    Serial.print(".");
  }
}

bool calibrate() {  // there's a better way, but still....
  Serial.println(" Lay device flat with zaxis upwards");
  delay(3000);
  float x0, xmax, y0, ymax, z0, zmax;
  calib_datasampling();
  zmax = zs;
  x0 = xs;
  y0 = ys;
  Serial.println(String("\n ") + x0 + " " + y0 + " " + z0);
  Serial.println(String(" ") + xmax + " " + ymax + " " + zmax);
  Serial.println("\n Lay device with yaxis upwards");
  delay(3000);
  calib_datasampling();
  z0 = zs;
  //x0 = xs; better measured in flat state
  ymax = ys;
  Serial.println(String("\n ") + x0 + " " + y0 + " " + z0);
  Serial.println(String(" ") + xmax + " " + ymax + " " + zmax);
  Serial.println("\n Lay device with xaxis upwards");
  delay(3000);
  calib_datasampling();
  z0 = zs;
  xmax = xs;
  //y0 = ys; better measured in flat state
  Serial.println(String("\n ") + x0 + " " + y0 + " " + z0);
  Serial.println(String(" ") + xmax + " " + ymax + " " + zmax);
  Serial.println("\n CHECKING result");
  if (abs(x0) < 2.) bias[0] = x0;
  if (abs(y0) < 2.) bias[1] = y0;
  if (abs(z0) < 2.) bias[2] = z0;
  if (xmax > 7) scale[0][0] = (9.81 / (xmax - x0));
  if (ymax > 7) scale[1][1] = (9.81 / (ymax - y0));
  if (zmax > 7) scale[2][2] = (9.81 / (zmax - z0));
  Serial.print("Calib result gain values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(scale[i][i], 6);
    Serial.print("  ");
  }
  Serial.print("\n");
  Serial.print("Calib result offset values:");
  for (int i = 0; i < 3; i++) {
    Serial.print(bias[i], 6);
    Serial.print("  ");
  }
  Serial.print("\n");
  return true;
}



void calc_and_print() {
  // calculate good values from actual raw values and print them wherever
  String ausgabe;
  if (performancedata) ausgabe += String(data_count) + " loops:" + loop_count + " t:" + cycle_time + " ";
  float a[3];
  float asquared=0.;
  for (int i = 0; i < 3; i++) {
    a[i] = 0;
    for (int j = 0; j < 3; j++) {
      a[i] +=  scale[i][j]* (aa[j]-bias[j]);
    }
//    a[i] -= bias[i];
    asquared+=  a[i]* a[i];
    ausgabe += String(a[i]) + " ";
  }
  ausgabe += String(sqrt(asquared)) + " ";
// add some other stuff later, eg calculated velocities ...
  console->println(ausgabe);
}


void parse_command(String com) {
  //split of the commant line string
  String c, rest, av[9];
  int pos = 0, pos2;
  int ac = 0;
  com += " "; // makes it easier to find the last arg
  
  pos2 = com.indexOf(" ");
  c = com.substring(pos, pos2);
  while (ac < 9 && pos2 < com.length()) {
    pos = pos2 + 1;
    pos2 = com.indexOf(" ", pos);
    av[ac] = com.substring(pos, pos2);
    //    Serial.println(String(" ") + pos + " " + pos2 + " " + ac + " " + av[ac]);
    ac++;
  }
  // and interpret it
  if (c.equalsIgnoreCase("help")) {
    // print help text
  } else if (c.equalsIgnoreCase("start")) {
    running = true;
  } else if (c.equalsIgnoreCase("m")) {
    calc_and_print();
  } else if (c.equalsIgnoreCase("stop")) {
    running = false;
  } else if (c.equalsIgnoreCase("calib1")) {
    calibrate();
  } else if (c.equalsIgnoreCase("power")) {
      console->println("voltage: "+String( system_get_vdd33())+"mV");
  } else if (c.equalsIgnoreCase("interval")) {
    uint32_t i = av[0].toInt();
    if (i > 3) {
      interrupttime = i * 5000;
    } else {
      console->println("Error with arg 1 of" + c + ":" + av[0]);
    }
  } else if (c.equalsIgnoreCase("smoothing")) {
    if (av[0].equalsIgnoreCase("on")) smoothing = true;
    else if (av[0].equalsIgnoreCase("off")) smoothing = false;
    else console->println(" unknown argument \"" + av[0] + "\" for command" + c);
  } else if (c.equalsIgnoreCase("store")) {
    preferences_store();
  } else {
    console->println(" unknown command:" + c);
  }
}


void loop() {
  if (i2c_data_available()) {
    i2c_request_data();
    if (smoothing) {
      aa[0] = naive_scale * smooth(x, 0);  // raw, uncalibrated values by doing naive scaling
      aa[1] = naive_scale * smooth(y, 1);  // also does the smoothing
      aa[2] = naive_scale * smooth(z, 2);
    } else {

      aa[0] = naive_scale * x;  // raw, uncalibrated values by doing naive scaling
      aa[1] = naive_scale * y;  // also does the smoothing
      aa[2] = naive_scale * z;
    }

    data_count++;
  }
  loop_count++;

  while (console->available()) {
    // EINGABE VORHANDEN
    String line = console->readStringUntil('\n');
    parse_command(line);
  }

  if (timeToPrint) {
    cycle_time = millis() - last_cycle;
    last_cycle = millis();
    if (running) calc_and_print();
    timeToPrint = false;
    loop_count = 0;
    data_count = 0;
  }
}
