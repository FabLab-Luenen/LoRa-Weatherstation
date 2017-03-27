#include <math.h>
#include <TheThingsNetwork.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <AS_BH1750.h>
#include <ams_as5048b.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <Bounce.h>
#include <Snooze.h>

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";

#define debugSerial Serial
#define loraSerial  Serial1
#define sdsSerial   Serial2

elapsedMillis MeasureTimer_Wind;
elapsedMillis MeasureTimer2;
elapsedMillis MeasureTimer_SDS;
elapsedMillis ReadingTimer_SDS;
elapsedMillis SendTimer;

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

//#define RX1          0  // PIN 0
//#define TX1          1  // PIN 1
#define RN2483RESETPIN 2  // PIN 2
#define DHTPIN         3  // PIN 3
#define RAINCNTR       4  // PIN 4
#define WINDSPD        5  // PIN 5
//#define RX2          9  // PIN 9
//#define TX2         10  // PIN 10

#define CHARGE_PIN    12  // PIN 12
//#define INTERNAL_LED 13 // Pin 13
#define VOLTAGE_BAT   A0  // PIN 14 ADC_0
#define RAINING       A1  // PIN 15 ADC_0
#define VOLTAGE_SOL   A2  // PIN 16 ADC_1

//#define SDA0        A4  // PIN 18
//#define SCL0        A5  // PIN 19

//deep sleep parameters                    // take CARE! only modules for Teensy3.2 & timer wake up  are included 
SnoozeTimer timer;                         // with Win10 (probably others) the USB disconnect/connect while sleeping blings
SnoozeBlock config_teensy32(timer);        // also the Serial Monitor will close  :-(
int who;                                   //
int mSecondsTimer = 10000;                 // sleeping time
int longsleep     = 12;                    // 12 x timer (10Seconds) see setup      ********** sleep between transmition **********

#define ANALOG_RESOLUTION_VOLTAGE  12
#define ANALOG_REFERENCE_VOLTAGE   1.2
#define ANALOG_RESOLUTION_RAIN  8
#define ANALOG_REFERENCE_RAIN   3.3


// This factor (float) is used to calculate the battery voltage.
// If the external voltage divider is 220 kOhm / 15 kOhm the factor is theoretically 15.66666 == (220 + 15) / 15.
// You must fine tune this value until the battery voltage is displayed correctly when you hit Enter in the Terminal.
// Therefor you must unplug the 220V power suppply and measure the real voltage at the battery.

#define VOLTAGE_FACTOR_BAT   3.951
#define VOLTAGE_FACTOR_SOL   18.820 
//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------
#define S_SDS011       1                         // SDS011
#define S_DHT          1                         // DHT22
#define S_AMS5048B     1                         // hall effect wind angle
#define S_AH3390_WIND  1                         // hall effect windspeed
#define S_AH3390_RAIN  1                         // hall effect raincounter
#define S_BH1750_SUN   1                         // luxmeter sun
#define S_BH1750_SHD   1                         // luxmeter shadow
#define S_BMP280       1                         // pressure  (pressure & temp in sun)
#define S_LM393        1                         // rainsensor
#define Solar          1                         // solar panel

uint16_t  north = 0;

#if S_SDS011 == 1
  // timings for SDS011 in ms
  #define SDS_WARMUP_TIME 10000
  #define SDS_SAMPLE_TIME  1000 
  #define SDS_READ_TIME    5000
  bool is_SDS_running = true;
  float pm10          = 0;
  float pm2_5         = 0;
  int sds_pm10_sum    = 0;          
  int sds_pm25_sum    = 0;
  int sds_val_count   = 0;
#endif

#if S_DHT == 1
  float avg_humidity = 0;
  float act_humidity = 0;
  float avg_temperature = 0;
  float act_temperature = 0;
  #define DHTTYPE DHT22
  DHT dht(DHTPIN, DHTTYPE, 25);
#endif

#if S_AMS5048B == 1
  float avg_wind_direction = 0;
  float act_wind_direction = 0;
  float min_wind_dir_ne = -1;
  float max_wind_dir_ne = -1;
  float min_wind_dir_es = -1;
  float max_wind_dir_es = -1;
  float min_wind_dir_sw = -1;
  float max_wind_dir_sw = -1;
  float min_wind_dir_wn = -1;
  float max_wind_dir_wn = -1;
  float wind_direction_flm;
  float u_wind_avg = 0;
  float v_wind_avg = 0;
  bool ww_ne = false;
  bool ww_es = false;
  bool ww_sw = false;
  bool ww_wn = false;
  //unit consts
  #define U_RAW 1
  #define U_TRN 2
  #define U_DEG 3
  #define U_RAD 4
  #define U_GRAD 5
  #define U_MOA 6
  #define U_SOA 7
  #define U_MILNATO 8
  #define U_MILSE 9
  #define U_MILRU 10
  AMS_AS5048B as5048;
#endif

#if S_AH3390_WIND == 1
  // datasheet parameters is 2.4 km/h at 30 RPM
  volatile float act_rpm_wind;
  const int halfshelldiameter = 40;                                                  // in mm
  const int outeradius        = 90;                                                  // in mm
  const int halfshellcount    =  3;
  const float circumference   = 2 * PI * (outeradius - (halfshelldiameter / 2));     // round to cm per round
  const float correction      =  3.0244;  // datasheet:  2.4 km/h at 30 RPM                                           
  const float rpm2kmph        = ((circumference * 60) / 1000000) * correction ;      // rounds per minute to kilometer per hour
  unsigned long timelast_wind;
  volatile float avg_rpm_wind = 0;
  volatile float max_rpm_wind = 0; 
#endif

#if S_AH3390_RAIN == 1
  const int rainsqmm = 5000;
  const float mlpp   = 2;
  volatile int rain_counter = 0;
#endif

#if S_BH1750_SUN == 1
  float avg_lux_sun = 0;
  float act_lux_sun = 0;
  AS_BH1750 luxmeter_sun(0x23);
#endif

#if S_BH1750_SHD == 1
  float avg_lux_shd = 0;
  float act_lux_shd = 0;
  AS_BH1750 luxmeter_shd(0x5c);
#endif

#if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
  int   sunshine_duration;
  int   sunshine_threshold;
#endif

#if S_BMP280 == 1
  float avg_bme_pressure    = 0;
  float act_bme_pressure    = 0;
  float avg_bme_temperature = 0;
  float act_bme_temperature = 0;
  Adafruit_BMP280 bme; 
#endif

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

volatile int   Counter_Wind_Speed = 0;
int   Counter_Wind_Direction = 0;
int   counter2 = 0;
int   Counter_SDS = 0;

volatile int   raining = 0; 
int   ledState = LOW;
uint8_t errorstate = 0;
int   MeasureTimerDelay_Wind;
int   MeasureTimerDelay2;
int   MeasureTimerDelay_SDS;
int   SendInterval;
volatile int   VoltageBat;
volatile int   VoltageSol;
bool  DEBUG = true;
int   rain_threshold = 0;
int   rtc_compensation = 0;
int   reset_counter_odc = 0;
int   lux_threshold = 500; // todo: to settings
   
// Print Serial Debug Messages
void MyDebug(bool prtln = false, String outputstr = "")
{
  if (DEBUG == true) {
    if (prtln == true) {
      debugSerial.println(outputstr);
    } else {
      debugSerial.print(outputstr);
    }
  }
}

void setup()
{
  sdsSerial.begin(9600);
  delay(100);
  start_SDS();
  debugSerial.begin(115200);
  delay(100);
  loraSerial.begin(57600);
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Reset RN2483
  pinMode(RN2483RESETPIN,OUTPUT);
  digitalWrite(RN2483RESETPIN,LOW);
  delay(500);
  digitalWrite(RN2483RESETPIN,HIGH);
  delay(2000);
  
  
  pinMode(VOLTAGE_BAT, INPUT);

  
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;
  
  ReadConfig();
  delay(100);  
  MyDebug(true,"-- STATUS");
  ttn.showStatus();
 
  // Set callback for incoming messages
  ttn.onMessage(Recieve);
  delay(100); 
  
  MyDebug(true,"-- JOIN");
  //ttn.join(appEui, appKey, 2,1000);
  ttn.join(appEui, appKey);
  delay(100); 
   
  #if S_DHT == 1
    dht.begin();
  #endif
  
  #if S_AMS5048B == 1
  as5048.begin();
  as5048.setClockWise(true);
  as5048.zeroRegW(0x00);
  as5048.zeroRegW(north);
  #endif

  #if S_AH3390_WIND == 1
  pinMode(WINDSPD, INPUT);
  attachInterrupt(WINDSPD, MeasureRPM, FALLING);
  #endif

  #if S_AH3390_RAIN == 1
  pinMode(RAINCNTR, INPUT);
  attachInterrupt(RAINCNTR, MeasureRain, FALLING);
  #endif

  #if S_BH1750_SUN == 1 
  if (!luxmeter_sun.begin()) { 
    MyDebug(true,"Could not find a valid BH1750 Sun sensor, check wiring!");
    errorstate = errorstate + 64;
  }
  #endif

  #if S_BH1750_SHD == 1
  if (!luxmeter_shd.begin()) { 
    MyDebug(true,"Could not find a valid BH1750 Shadow sensor, check wiring!");
    errorstate = errorstate + 32;
  }
  #endif
  
  #if S_BMP280 == 1   
  if (!bme.begin(0x76)) {  
    MyDebug(true,"Could not find a valid BMP280 sensor, check wiring!");
    errorstate = errorstate + 128;
  }
  #endif
  
  #if S_LM393 == 1
  pinMode(RAINING, INPUT);
  #endif
  
  #if Solar == 1
  // Disable Charging
  pinMode(VOLTAGE_SOL, INPUT);
  pinMode(CHARGE_PIN, INPUT);
  digitalWrite(CHARGE_PIN, LOW);
  #endif

  ReadStatus(); 
  Measure_Wind();
  Measure2();
  Read_SDS(true);
  SendTTNData();
  InitTimer();
  setSyncProvider(getTeensy3Time);
}

void loop()
{
  #if S_AH3390_WIND == 1
  if ((millis() - timelast_wind) >= 5000) {
    act_rpm_wind = 0;
    timelast_wind = 0;
  }
  #endif
  if (MeasureTimer_Wind >= MeasureTimerDelay_Wind * 100) {
    MeasureTimer_Wind -= MeasureTimerDelay_Wind * 100;
    digitalClockDisplay();
    if (ledState == LOW) {
       ledState = HIGH;
    } else {
       ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState); 
    Measure_Wind();
  }
  
  if (MeasureTimer2 >= MeasureTimerDelay2 * 100) {
    MeasureTimer2 -= MeasureTimerDelay2 * 100 ;
    Measure2();
  }
  
  if (MeasureTimer_SDS >= ((MeasureTimerDelay_SDS * 100) - SDS_WARMUP_TIME)) {
    if (ReadingTimer_SDS > SDS_SAMPLE_TIME) {
      ReadingTimer_SDS = 0;
      Measure_SDS();
    }
  }
  
  if (SendTimer >= SendInterval * 100) {
    SendTimer -= SendInterval * 100;
    SendTTNData();
  }
  
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
}

//Measure wind direction 
#if S_AMS5048B == 1
void MeasureWindDirection(void)
{
  double w_wind_avg;
  Counter_Wind_Direction ++;
  act_wind_direction = as5048.angleR(U_DEG);
  //if (avg_wind_direction == 0) {
  //  avg_wind_direction = act_wind_direction;
  //}
  //as5048.updateMovingAvgExp();
  //avg_wind_direction = as5048.getMovingAvgExp(U_DEG);
  float u = sin(act_wind_direction * PI / 180);
  float v = cos(act_wind_direction * PI / 180);

  u_wind_avg = ((u_wind_avg * (Counter_Wind_Direction - 1))  +  u)/Counter_Wind_Direction;
  v_wind_avg = ((v_wind_avg * (Counter_Wind_Direction - 1))  +  v)/Counter_Wind_Direction;

  w_wind_avg = atan2(abs(u_wind_avg), abs(v_wind_avg)) * 180 / PI;
  if ((u_wind_avg >= 0) && (v_wind_avg >= 0)){
    avg_wind_direction = w_wind_avg;
  }
  if ((u_wind_avg >= 0) && (v_wind_avg < 0)){
    avg_wind_direction = 180 - w_wind_avg;
  }
  if ((u_wind_avg < 0) && (v_wind_avg >= 0)){
    avg_wind_direction = 360 - w_wind_avg;
  }
  if ((u_wind_avg < 0) && (v_wind_avg < 0)){
    avg_wind_direction = 180 + w_wind_avg;
  }
  
   
  if ((act_wind_direction >= 0) && (act_wind_direction < 90)) {
    ww_ne = true;
    if (min_wind_dir_ne == -1) {
        min_wind_dir_ne = act_wind_direction;
    }
    if (min_wind_dir_ne > act_wind_direction) {
        min_wind_dir_ne = act_wind_direction;
    }
    if (max_wind_dir_ne == -1) {
        max_wind_dir_ne = act_wind_direction;
    }
    if (max_wind_dir_ne < act_wind_direction) {
        max_wind_dir_ne = act_wind_direction;
    }
  }
  if ((act_wind_direction >= 90) && (act_wind_direction < 180)) {
    ww_es = true;
    if (min_wind_dir_es == -1) {
        min_wind_dir_es = act_wind_direction;
    }
    if (min_wind_dir_es > act_wind_direction) {
        min_wind_dir_es = act_wind_direction;
    }
    if (max_wind_dir_es == -1) {
        max_wind_dir_es = act_wind_direction;
    }
    if (max_wind_dir_es < act_wind_direction) {
        max_wind_dir_es = act_wind_direction;
    }
  }
  if ((act_wind_direction >= 180) && (act_wind_direction < 270)) {
    ww_sw = true;
    if (min_wind_dir_sw == -1) {
        min_wind_dir_sw = act_wind_direction;
    }
    if (min_wind_dir_sw > act_wind_direction) {
        min_wind_dir_sw = act_wind_direction;
    }
    if (max_wind_dir_sw == -1) {
        max_wind_dir_sw = act_wind_direction;
    }
    if (max_wind_dir_sw < act_wind_direction) {
        max_wind_dir_sw = act_wind_direction;
    }
  }
  if ((act_wind_direction >= 270) && (act_wind_direction < 360)) {
    ww_wn = true;
    if (min_wind_dir_wn == -1) {
        min_wind_dir_wn = act_wind_direction;
    }
    if (min_wind_dir_wn > act_wind_direction) {
        min_wind_dir_wn = act_wind_direction;
    }
    if (max_wind_dir_wn == -1) {
        max_wind_dir_wn = act_wind_direction;
    }
    if (max_wind_dir_wn < act_wind_direction) {
        max_wind_dir_wn = act_wind_direction;
    }
  }
 
  if (ww_ne && ww_es && ww_sw && ww_wn) {  // at all sectors
      wind_direction_flm = max_wind_dir_wn - min_wind_dir_ne;
  }
  
  if (!ww_ne && ww_es && ww_sw && ww_wn) { // not in ne
      wind_direction_flm = max_wind_dir_wn - min_wind_dir_es;
  }

  if (ww_ne && !ww_es && ww_sw && ww_wn) { // not in es
      wind_direction_flm = 360 - (min_wind_dir_sw - max_wind_dir_ne);
  }

  if (ww_ne && ww_es && !ww_sw && ww_wn) { // not in sw
      wind_direction_flm = 360 - (min_wind_dir_wn - max_wind_dir_es); 
  }
  
  if (ww_ne && ww_es && ww_sw && !ww_wn) { // not in wn
      wind_direction_flm = max_wind_dir_sw - min_wind_dir_ne;
  }
   
  if (!ww_ne && !ww_es && ww_sw && ww_wn) { // not in ne + es
      wind_direction_flm = max_wind_dir_wn - min_wind_dir_sw;
  }

  if (ww_ne && !ww_es && !ww_sw && ww_wn) { // not in es + sw
      wind_direction_flm = 360 - (min_wind_dir_wn - max_wind_dir_ne);
  }
 
  if (ww_ne && ww_es  && !ww_sw && !ww_wn) { // not in sw + wn
      wind_direction_flm = max_wind_dir_es - min_wind_dir_ne;
  }

  if (!ww_ne && ww_es && ww_sw && !ww_wn) { // not in wn + ne
      wind_direction_flm = max_wind_dir_sw - min_wind_dir_es;
  }
 
  if (ww_ne && !ww_es && !ww_sw && !ww_wn) { // only in ne
      wind_direction_flm = max_wind_dir_ne - min_wind_dir_ne;
  }
     
  if (!ww_ne && ww_es && !ww_sw && !ww_wn) { // only in es
      wind_direction_flm = max_wind_dir_es - min_wind_dir_es;
  }
 
  if (!ww_ne && !ww_es && ww_sw && !ww_wn) { // only in sw
      wind_direction_flm = max_wind_dir_sw - min_wind_dir_sw;
  }
 
  if (!ww_ne && !ww_es && !ww_sw && ww_wn) { // only in wn
      wind_direction_flm = max_wind_dir_wn - min_wind_dir_wn;
  }

    
  MyDebug(false,"Wind Direction: ");
  MyDebug(false,avg_wind_direction);
  MyDebug(false," Grad (Avg.) ");
  MyDebug(false, act_wind_direction);
  MyDebug(true," Grad (Act.)"); 
  MyDebug(false,"Wind Direction Fluctuation Margin: ");
  MyDebug(false,wind_direction_flm);
  MyDebug(true," Grad");
}
#endif


#if S_AH3390_WIND == 1
//This function is called whenever a magnet/interrupt is detected from the anemometer
void MeasureRPM(void) // one pulse per revolution
{
  int timediff = millis() - timelast_wind;
  if ((timelast_wind > 0) && (timediff <= 5000)) {
    act_rpm_wind = 60000 / timediff;
  } else {
    act_rpm_wind = 0;
  }
  timelast_wind = millis();
  if (act_rpm_wind > max_rpm_wind) {
    max_rpm_wind = act_rpm_wind;
  }
}
#endif

#if S_AH3390_RAIN == 1
//This function is called whenever a magnet/interrupt is detected from the pluviometer
void MeasureRain(void)
{
  rain_counter ++;
  if (rain_counter > 65535) {
      rain_counter = rain_counter - 65535;
  }
  EEPROM.write(22,highByte(rain_counter));
  EEPROM.write(23,lowByte(rain_counter));
}
#endif

uint8_t CheckRain(void)
{
    analogReadResolution(ANALOG_RESOLUTION_RAIN);
    analogReference(DEFAULT);
    delay(25);
    const uint32_t maxValue = (1 << ANALOG_RESOLUTION_RAIN) -1;
    //int value = analogRead(RAINING);
    int value =  maxValue - analogRead(RAINING);
    delay(25);
    if (value > rain_threshold) {
      raining = 1;
    } else {
      raining = 0;
    }
    return (uint8_t)(value);
}

uint16_t Measure_BAT_Voltage(void)
{
    analogReadResolution(ANALOG_RESOLUTION_VOLTAGE);
    analogReference(ANALOG_REFERENCE_VOLTAGE);
    delay(25);
    const uint32_t maxValue = (1 << ANALOG_RESOLUTION_VOLTAGE) -1;
    int value =  analogRead(VOLTAGE_BAT);
    delay(25);
    return (uint16_t)((100* value * ANALOG_REFERENCE_VOLTAGE * VOLTAGE_FACTOR_BAT) / maxValue);
}

#if Solar == 1
uint16_t Measure_SOL_Voltage(void)
{
    analogReadResolution(ANALOG_RESOLUTION_VOLTAGE);
    analogReference(ANALOG_REFERENCE_VOLTAGE);
    delay(25); 
    const uint32_t maxValue = (1 << ANALOG_RESOLUTION_VOLTAGE) -1;
    int value = analogRead(VOLTAGE_SOL);
    delay(25);
    return (uint16_t)((100 * value * ANALOG_REFERENCE_VOLTAGE * VOLTAGE_FACTOR_SOL) / maxValue);
}

uint8_t CheckCharging(void)
{
    if (digitalRead(CHARGE_PIN) == LOW)
    {
      return 0;
    } else {
      return 1;
    }
    return 0;
}
#endif

void SendTTNData()
{
    // Split all variables into bytes of 8 for payload transmission
    byte payload[40];
    for (int i = 0; i < sizeof(payload); i++) { payload[i] = 0; }
    payload[0]  = 0xA0;
    payload[1]  = errorstate;
    #if S_DHT == 1
    uint16_t snd_temperature  = avg_temperature  * 100;
    avg_temperature  = 0;
    uint16_t snd_humidity     = avg_humidity * 100;
    avg_humidity     = 0;
    payload[2]  = highByte(snd_temperature);
    payload[3]  = lowByte(snd_temperature);
    payload[4]  = highByte(snd_humidity);
    payload[5]  = lowByte(snd_humidity);
    #endif
    #if S_BMP280 == 1
    uint32_t snd_bme_pressure = avg_bme_pressure * 100;  
    avg_bme_pressure = 0;
    uint16_t snd_bme_temperature = avg_bme_temperature * 100;
    avg_bme_temperature = 0;
    payload[6]  = (snd_bme_pressure >> 16) & 0xFF;
    payload[7]  = (snd_bme_pressure >> 8) & 0xFF;
    payload[8]  = (snd_bme_pressure) & 0xFF;;
    payload[9]  = highByte(snd_bme_temperature);
    payload[10] = lowByte(snd_bme_temperature); 
    #endif
    #if S_BH1750_SUN == 1
    uint32_t snd_lux_sun      = avg_lux_sun * 100;
    avg_lux_sun      = 0;
    payload[11] = (snd_lux_sun >> 16) & 0xFF;
    payload[12] = (snd_lux_sun >> 8) & 0xFF;
    payload[13] = (snd_lux_sun) & 0xFF;
    #endif
    #if S_BH1750_SHD == 1
    uint32_t snd_lux_shd      = avg_lux_shd * 100;
    avg_lux_shd      = 0;
    payload[14] = (snd_lux_shd >> 16) & 0xFF;
    payload[15] = (snd_lux_shd  >> 8) & 0xFF;
    payload[16] = (snd_lux_shd)  & 0xFF;
    #endif
    #if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
    payload[17] = highByte((uint16_t)sunshine_duration);
    payload[18] = lowByte((uint16_t)sunshine_duration);
    #endif
    #if S_AMS5048B == 1
    uint16_t snd_wind_direction = avg_wind_direction * 100;
    uint16_t snd_wind_direction_flm = wind_direction_flm * 100;
    wind_direction_flm = 0;
    min_wind_dir_ne = -1;
    max_wind_dir_ne = -1;
    min_wind_dir_es = -1;
    max_wind_dir_es = -1;
    min_wind_dir_sw = -1;
    max_wind_dir_sw = -1;
    min_wind_dir_wn = -1;
    max_wind_dir_wn = -1;
    ww_ne = false;
    ww_es = false;
    ww_sw = false;
    ww_wn = false;
    payload[19] = highByte((uint16_t)snd_wind_direction);
    payload[20] = lowByte((uint16_t)snd_wind_direction);
    payload[21] = lowByte((uint16_t)snd_wind_direction_flm);
    payload[22] = highByte((uint16_t)snd_wind_direction_flm);
    #endif
    #if S_AH3390_WIND == 1
    uint16_t snd_max_wind_speed = max_rpm_wind * rpm2kmph * 100;
    uint16_t snd_avg_wind_speed = avg_rpm_wind * rpm2kmph * 100;
    avg_rpm_wind = 0;
    max_rpm_wind = 0;
    payload[23] = highByte(snd_avg_wind_speed);
    payload[24] = lowByte(snd_avg_wind_speed);
    payload[25] = highByte(snd_max_wind_speed);
    payload[26] = lowByte(snd_max_wind_speed);
    #endif
    payload[28] = CheckRain();
    payload[27] = raining;
    payload[29] = highByte((uint16_t)rain_counter);
    payload[30] = lowByte((uint16_t)rain_counter);
    #if S_SDS011 == 1
    payload[31] = highByte((uint16_t)pm10);
    payload[32] = lowByte((uint16_t)pm10);
    payload[33] = highByte((uint16_t)pm2_5);
    payload[34] = lowByte((uint16_t)pm2_5);
    #endif
    payload[35] = highByte((uint16_t)VoltageBat);
    payload[36] = lowByte((uint16_t)VoltageBat);
    #if Solar == 1
    payload[37] = highByte((uint16_t)VoltageSol);
    payload[38] = lowByte((uint16_t)VoltageSol);
    payload[39] = CheckCharging();
    #endif
    Counter_Wind_Speed = 0;
    Counter_Wind_Direction = 0;
    counter2 = 0;
    Counter_SDS = 0;
    MyDebug(true,"-- SENDING");
    ttn.sendBytes(payload, sizeof(payload));
}

void Measure_Wind(void)
{
  Counter_Wind_Speed ++;
  avg_rpm_wind = ((avg_rpm_wind  * (Counter_Wind_Speed - 1)) + act_rpm_wind)/Counter_Wind_Speed;
  MyDebug(false,"-- Measure 1 - ");
  MyDebug(true,Counter_Wind_Speed);
  //Measure wind direction 
  #if S_AMS5048B == 1
  if (act_rpm_wind >= 10) {
    MeasureWindDirection();
  } else {
    act_wind_direction = 0;
  }
  #endif
  MyDebug(false, "Wind RPM: ");
  MyDebug(false, avg_rpm_wind);
  MyDebug(false, " U/min (Avg.) ");
  MyDebug(false, act_rpm_wind);
  MyDebug(false, " U/min (Act.) ");
  MyDebug(false, max_rpm_wind);
  MyDebug(true, " U/min (Max.)");
  MyDebug(false, "Wind Speed: ");
  MyDebug(false, avg_rpm_wind*rpm2kmph);
  MyDebug(false, " km/h (Avg.) ");
  MyDebug(false, act_rpm_wind*rpm2kmph);
  MyDebug(false," km/h (Act.) ");  
  MyDebug(false, max_rpm_wind*rpm2kmph);
  MyDebug(true, " km/h (Max.)");
  MyDebug(false, "Rain Itensity ");
  MyDebug(true, CheckRain());
  MyDebug(false, "Rain Counter: ");
  MyDebug(true, rain_counter);
}

void Measure2(void) // temperature, humidity, pressure, voltage, lux_sun, lux_shd, temperature_sun
{
  counter2 ++;
  MyDebug(false,"-- Measure 2 - ");
  MyDebug(true,counter2);
  #if S_DHT == 1
  act_humidity        = dht.readHumidity(false);
  avg_humidity        = ((avg_humidity * (counter2 - 1)) + act_humidity)/counter2;
  act_temperature     = dht.readTemperature(false);   // false: Celsius (default); true: Farenheit
  avg_temperature     = ((avg_temperature  * (counter2 - 1)) + act_temperature)/counter2;
  MyDebug(false,"DHT22 Temperature: ");
  MyDebug(false,avg_temperature);
  MyDebug(false," *C (Avg.) ");
  MyDebug(false,act_temperature);
  MyDebug(true," *C (Act.)");
  MyDebug(false,"DHT22 Humidity: ");
  MyDebug(false,avg_humidity);
  MyDebug(false," % (Avg.) ");
  MyDebug(false,act_humidity);
  MyDebug(true," % (Act.)");
  #endif
  #if S_BMP280 == 1
  act_bme_pressure    = bme.readPressure();
  avg_bme_pressure    = ((avg_bme_pressure  * (counter2 - 1)) + act_bme_pressure)/counter2;
  act_bme_temperature = bme.readTemperature();
  avg_bme_temperature = ((avg_bme_temperature * (counter2 - 1)) + act_bme_temperature)/counter2;
  MyDebug(false,"BMP280 Pressure: ");
  MyDebug(false,avg_bme_pressure);
  MyDebug(false," ha (Avg.) ");
  MyDebug(false,act_bme_pressure);
  MyDebug(true," ha (Act.)");
  MyDebug(false,"BMP280 Temperature: ");
  MyDebug(false,avg_bme_temperature);
  MyDebug(false," *C (Avg.) ");
  MyDebug(false,act_bme_temperature);
  MyDebug(true," *C (Act.)");
  #endif
  #if S_BH1750_SUN == 1
  act_lux_sun         = luxmeter_sun.readLightLevel();
  avg_lux_sun         = ((avg_lux_sun  * (counter2 - 1)) + act_lux_sun)/counter2;
  MyDebug(false,"BH1750 Lux Sun: ");
  MyDebug(false,avg_lux_sun);
  MyDebug(false," lux (Avg.) ");
  MyDebug(false,act_lux_sun);
  MyDebug(true," lux (Act.)");
  #endif
  #if S_BH1750_SHD == 1
  act_lux_shd         = luxmeter_shd.readLightLevel();
  avg_lux_shd         = ((avg_lux_shd  * (counter2 - 1)) + act_lux_shd)/counter2;
  MyDebug(false,"BH1750 Lux Shadow: ");
  MyDebug(false,avg_lux_shd);
  MyDebug(false," lux (Avg.) ");
  MyDebug(false,act_lux_shd);
  MyDebug(true," lux (Act.)");
  #endif
  MyDebug(false,"BAT_Voltage: ");
  VoltageBat = Measure_BAT_Voltage();
  MyDebug(false,(float)VoltageBat/100);
  MyDebug(true," V");
  #if Solar == 1
  VoltageSol = Measure_SOL_Voltage();
  MyDebug(false,"SOL_Voltage: ");
  MyDebug(false,(float)VoltageSol/100);
  MyDebug(true," V");
  MyDebug(false,"Charging: ");
  MyDebug(true,CheckCharging());
  #endif

  #if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
  if (act_lux_sun-act_lux_shd > lux_threshold) {
    sunshine_duration = sunshine_duration + (MeasureTimerDelay_Wind/10);
    if (sunshine_duration > 65535) {
      sunshine_duration = sunshine_duration - 65535;
    }
    EEPROM.write(20,highByte(sunshine_duration));
    EEPROM.write(21,lowByte(sunshine_duration));
  }
  MyDebug(false,"Sunshine Duration: ");
  MyDebug(false,sunshine_duration);
  MyDebug(true," sec");
  #endif   
}

void Measure_SDS(void) // pm10 pm2_5
{
  MyDebug(false, "-- Measure SDS - ");
  Counter_SDS ++;
  MyDebug(true, Counter_SDS);
  Read_SDS(false);
}

void InitTimer(void){
  MeasureTimer_Wind = 0;
  MeasureTimer2 = 0;
  MeasureTimer_SDS = 0;
  SendTimer = 0;
  Teensy3Clock.compensate(rtc_compensation);
  return;       
}

void ReadConfig(void){  
  uint8_t MeasureTimerDelay_Wind_High  = EEPROM.read( 1);
  uint8_t MeasureTimerDelay_Wind_Low   = EEPROM.read( 2);
  uint8_t MeasureTimerDelay2_High      = EEPROM.read( 3);
  uint8_t MeasureTimerDelay2_Low       = EEPROM.read( 4);
  uint8_t MeasureTimerDelay_SDS_High   = EEPROM.read( 5);
  uint8_t MeasureTimerDelay_SDS_Low    = EEPROM.read( 6);
  uint8_t SendInterval_High            = EEPROM.read( 7);
  uint8_t SendInterval_Low             = EEPROM.read( 8);
  rain_threshold                       = EEPROM.read( 9);
  uint8_t north_High                   = EEPROM.read(10);
  uint8_t north_Low                    = EEPROM.read(11);
  rtc_compensation                     = EEPROM.read(12);
  reset_counter_odc                    = EEPROM.read(13);
  uint8_t sunshine_threshold_High      = EEPROM.read(14);
  uint8_t sunshine_threshold_Low       = EEPROM.read(15);
  north                                = (north_High * 256) + north_Low;
  MeasureTimerDelay_Wind               = (MeasureTimerDelay_Wind_High * 256) + MeasureTimerDelay_Wind_Low;
  MeasureTimerDelay2                   = (MeasureTimerDelay2_High * 256) + MeasureTimerDelay2_Low;
  MeasureTimerDelay_SDS                = (MeasureTimerDelay_SDS_High * 256) + MeasureTimerDelay_SDS_Low;
  SendInterval                         = (SendInterval_High * 256) + SendInterval_Low;
  sunshine_threshold                   = (sunshine_threshold_High * 256) + sunshine_threshold_Low;
  MyDebug(true,"-- READING SETTINGS FROM EEPROM");
  MyDebug(false,"MeasureTimerDelay_Wind: ");
  MyDebug(false, (float)MeasureTimerDelay_Wind / 10);
  MyDebug(true, " sec.");
  MyDebug(false,"MeasureTimerDelay2: ");
  MyDebug(false, (float)MeasureTimerDelay2 / 10);
  MyDebug(true, " sec.");
  MyDebug(false,"MeasureTimerDelay_SDS: ");
  MyDebug(false, (float)MeasureTimerDelay_SDS / 10);
  MyDebug(true, " sec.");
  MyDebug(false,"SendInterval: ");
  MyDebug(false, (float)SendInterval / 10);
  MyDebug(true, " sec.");
  MyDebug(false,"Rain Threshold: ");
  MyDebug(true, rain_threshold);
  #if S_AMS5048B == 1
  MyDebug(false,"North: ");
  MyDebug(true, north);
  #endif
  MyDebug(false,"RTC Compensation: ");
  MyDebug(true, rtc_compensation);
  MyDebug(false,"Reset Counters on day change: ");
  MyDebug(true, reset_counter_odc);
  MyDebug(false,"Sunshine Threshold: ");
  MyDebug(true, sunshine_threshold);
  return;
}

void ReadStatus(void)
{
  MyDebug(true,"-- READING STATUS FROM EEPROM");
  #if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
  uint8_t sunshine_duration_High    = EEPROM.read(20);
  uint8_t sunshine_duration_Low     = EEPROM.read(21);
  sunshine_duration = (sunshine_duration_High * 256) + sunshine_duration_Low;  
  MyDebug(false,"Sunshine Duration: ");
  MyDebug(true,sunshine_duration);
  #endif  
  #if S_AH3390_RAIN == 1
  uint8_t rain_counter_High         = EEPROM.read(22);
  uint8_t rain_counter__Low         = EEPROM.read(23);
  rain_counter      = (rain_counter_High * 256) + rain_counter__Low;
  MyDebug(false,"Rain Counter: ");
  MyDebug(true,rain_counter);
  #endif  
  return;
}

byte send_payload[17];

void SendSettings(void) {
  send_payload[0]  = 0xA1;
  send_payload[1]  = errorstate;      
  send_payload[2]  = highByte(MeasureTimerDelay_Wind);
  send_payload[3]  = lowByte(MeasureTimerDelay_Wind);
  send_payload[4]  = highByte(MeasureTimerDelay2);
  send_payload[5]  = lowByte(MeasureTimerDelay2);
  send_payload[6]  = highByte(MeasureTimerDelay_SDS);
  send_payload[7]  = lowByte(MeasureTimerDelay_SDS);
  send_payload[8]  = highByte(SendInterval);
  send_payload[9]  = lowByte(SendInterval);
  send_payload[10] = rain_threshold;
  send_payload[11] = highByte(north);
  send_payload[12] = lowByte(north);
  send_payload[13] = rtc_compensation;
  send_payload[14] = reset_counter_odc; 
  send_payload[15]  = highByte(sunshine_threshold);
  send_payload[16]  = lowByte(sunshine_threshold);   
  ttn.sendBytes(send_payload, sizeof(send_payload)); 
}

void Recieve(const uint8_t *payload, size_t size, port_t port)
{
  MyDebug(true, "-- MESSAGE");
  MyDebug(false, "Received Payload " + String(size) + " bytes on port " + String(port) + ":");

  for (int i = 0; i < size; i++)
  {
    MyDebug(false, " " + String(payload[i])); 
  }
  int switch_case = payload[0];
  uint32_t tm;
  int secs;
  
  switch (switch_case) {
    case 0xF0: //Read Settings
      MyDebug(true, "-- SENDING Settings");
      SendSettings();
    break;

    case 0xF1: //write MeasureTimerDelay_Wind settings
      EEPROM.write(1,payload[1]);
      EEPROM.write(2,payload[2]);
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
      tm = payload[1] + payload[2] + (payload[3]*256) + payload[4];
    break;

    case 0xF2: //write MeasureTimerDelay2 settings
      EEPROM.write(3,payload[1]);
      EEPROM.write(4,payload[2]);
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF3: //write MeasureTimerDelay_SDS settings
      EEPROM.write(5,payload[1]);
      EEPROM.write(6,payload[2]);
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF4: //write SendInterval settings
      EEPROM.write(7,payload[1]);
      EEPROM.write(8,payload[2]);
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF5: //write rain_threshold settings
      EEPROM.write(9,payload[1]);
      ReadConfig();
      InitTimer();      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF6: //write north settings
      EEPROM.write(10,payload[1]);
      EEPROM.write(11,payload[2]);
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF7: //write rtc_compensation settings
      EEPROM.write(12,payload[1]);
      ReadConfig();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF8: //write reset_counter_odc settings
      EEPROM.write(13,payload[1]);
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xF9: //write sunshine_threshold settings
      EEPROM.write(14,payload[1]);
      EEPROM.write(15,payload[2]);
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;
        
    case 0xFF: //write all settings
      MyDebug(true,"-- WRITING Settings");
      for (int i = 1; i < size; i++)
      {
        MyDebug(false," " + String(payload[i]));
        EEPROM.write(i,payload[i]);
      }
      ReadConfig();
      InitTimer();
      MyDebug(true, "-- SENDING Changed Settings");
      SendSettings();
    break;

    case 0xC0: // get time & send   
       MyDebug(true,"-- SENDING RTC Time");
       tm = Teensy3Clock.get();
       send_payload[0]  = 0xA2;
       send_payload[1]  = errorstate;
       send_payload[2]  = (tm >> 24) & 0xFF;
       send_payload[3]  = (tm >> 16) & 0xFF;
       send_payload[4]  = (tm >> 8) & 0xFF;
       send_payload[5]  = (tm) & 0xFF;
       ttn.sendBytes(send_payload, sizeof(send_payload)); 
    break;
    
    case 0xC1: //setclock   
       MyDebug(false,"-- ADDING TO RTC TIME : ");
       tm = payload[1] + payload[2] + (payload[3]*256) + payload[4];
       Teensy3Clock.set(tm); // set the RTC
       setTime(tm);
       MyDebug(false, tm);
       MyDebug(true," Seconds");
    break;

    case 0xC2: //add seconds to clock
       MyDebug(false,"-- SUBTRACTING FROM RTC TIME : ");
       secs = (payload[1]*256) + payload[2];
       tm = Teensy3Clock.get();
       tm = tm + secs;
       Teensy3Clock.set(tm); // set the RTC
       setTime(tm);
       MyDebug(false, tm);
       MyDebug(true," Seconds");
    break;

    case 0xC3: //subtract seconds from clock
       secs = (payload[1]*256) + payload[2];
       tm = Teensy3Clock.get();
       tm = tm - secs;
       Teensy3Clock.set(tm); // set the RTC
       setTime(tm);
    break;

    #if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
    case 0xD0: //Reset Sunshine Duration
       MyDebug(true,"");
       MyDebug(true,"-- RESET Sunshine Duration");
       sunshine_duration  = 0;
       EEPROM.write(20,0x00);
       EEPROM.write(21,0x00);
    break;
    #endif
    
    #if S_AH3390_RAIN == 1
    case 0xD1: //Reset Rain Counter
       MyDebug(true,"");
       MyDebug(true,"-- RESET Raincounter");
       rain_counter       = 0;
       EEPROM.write(22,highByte(rain_counter));
       EEPROM.write(23,lowByte(rain_counter));
    break;
    #endif 

    case 0xD2: //zero wind_direction
       MyDebug(true,"");
       MyDebug(true,"-- SET ACTUAL WIND DIRECTION AS ZERO");
       north = as5048.angleRegR();
       EEPROM.write(10, highByte(north));
       EEPROM.write(11, lowByte(north));
       as5048.zeroRegW(0x00);
       as5048.zeroRegW(north);
       MyDebug(false,"North: ");
       MyDebug(true, north);
    break;

    case 0xDF: //Reset All
       MyDebug(true,"");
       MyDebug(true,"-- RESET All");
       MeasureTimerDelay_Wind     = 10;
       MeasureTimerDelay2         = 100;
       MeasureTimerDelay_SDS      = 9000;
       north                      = 0;
       rain_threshold             = 128;
       rtc_compensation           = 93;
       reset_counter_odc          = 0;
       sunshine_threshold         = 2500;
       EEPROM.write( 1, highByte(MeasureTimerDelay_Wind));
       EEPROM.write( 2, lowByte(MeasureTimerDelay_Wind));
       EEPROM.write( 3, highByte(MeasureTimerDelay2));
       EEPROM.write( 4, lowByte(MeasureTimerDelay2));
       EEPROM.write( 5, highByte(MeasureTimerDelay_SDS));
       EEPROM.write( 6, lowByte(MeasureTimerDelay_SDS));
       EEPROM.write( 7, highByte(SendInterval));
       EEPROM.write( 8, lowByte(SendInterval));
       EEPROM.write( 9, rain_threshold);
       EEPROM.write(10, highByte(north));
       EEPROM.write(11, lowByte(north));
       EEPROM.write(12, rtc_compensation);
       EEPROM.write(13, reset_counter_odc);        
       #if S_BH1750_SUN == 1 && S_BH1750_SHD == 1
       sunshine_duration  = 0;
       sunshine_threshold = 2500;
       EEPROM.write(20, highByte(sunshine_duration));
       EEPROM.write(21, lowByte(sunshine_duration)); 
       EEPROM.write(14, highByte(sunshine_threshold));
       EEPROM.write(15, lowByte(sunshine_threshold));
       send_payload[15]  = highByte(sunshine_threshold);
       send_payload[16]  = lowByte(sunshine_threshold);     
       #endif
       #if S_AH3390_RAIN == 1
       rain_counter       = 0;
       EEPROM.write(22, highByte(rain_counter));
       EEPROM.write(23, lowByte(rain_counter)); 
       #endif
       #if S_AMS5048B == 1
       as5048.zeroRegW(0x00);
       as5048.zeroRegW(north);
       #endif
       ReadConfig();
       InitTimer();
       MyDebug(true, "-- SENDING Settings");
       send_payload[0]  = 0xA1;
       send_payload[1]  = errorstate;      
       send_payload[2]  = highByte(MeasureTimerDelay_Wind);
       send_payload[3]  = lowByte(MeasureTimerDelay_Wind);
       send_payload[4]  = highByte(MeasureTimerDelay2);
       send_payload[5]  = lowByte(MeasureTimerDelay2);
       send_payload[6]  = highByte(MeasureTimerDelay_SDS);
       send_payload[7]  = lowByte(MeasureTimerDelay_SDS);
       send_payload[8]  = highByte(SendInterval);
       send_payload[9]  = lowByte(SendInterval);
       send_payload[10] = rain_threshold;
       send_payload[11] = highByte(north);
       send_payload[12] = lowByte(north);
       send_payload[13] = rtc_compensation;
       send_payload[14] = reset_counter_odc;      
       ttn.sendBytes(send_payload, sizeof(send_payload));   
    break;
    
    default:
       MyDebug(true,"-- UNKNOWN REQUEST");
    break;
  }
  

  
  MyDebug(true,"");
}
      

#if S_SDS011 == 1
//****************************************************************
// convert float to string with a                                *               result_SDS (routine): 18645.0;24.4;12.0
// precision of 1 decimal place                                  *
//****************************************************************
String Float2String(const float value) {
  // Convert a float to String with two decimals.
  char temp[15];
  String s;

  dtostrf(value,13, 1, temp);
  s = String(temp);
  s.trim();
  return s;
}

// Kommands to start and stop SDS011
void stop_SDS(void) {
   const byte stop_SDS_cmd[] = {0xFF, 0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
   sdsSerial.write(stop_SDS_cmd,sizeof(stop_SDS_cmd)); is_SDS_running = false;
   delay(10);
   is_SDS_running = false;
}

void start_SDS(void) {
   const byte start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
   sdsSerial.write(start_SDS_cmd,sizeof(start_SDS_cmd)); is_SDS_running = true;
   delay(10);
   is_SDS_running = true;
}

// read SDS011 sensor values
void Read_SDS(bool ignoretimer) {
  char buffer;
  int value;
  int  len           = 0;
  int  pm10_serial   = 0;
  int  pm25_serial   = 0;
  long SDS_ID        = 0;
  int  checksum_is   = 0;
  bool  checksum_ok  = false;

  if (!is_SDS_running) {
    MyDebug(true, "delayed -- SDS Offline! Starting SDS ...");
    start_SDS();
    return;
  }
  
  // SDS runs: read serial buffer
  while (sdsSerial.available() > 0) {
    buffer = sdsSerial.read();
    value = int(buffer);
    switch (len) {
      case (0): if (value != 0xAA) { len = -1; }; break;
      case (1): if (value != 0xC0) { len = -1; }; break;
      case (2): pm25_serial = value;         checksum_is  = value; break;
      case (3): pm25_serial += (value << 8); checksum_is += value; break;
      case (4): pm10_serial = value;         checksum_is += value; break;
      case (5): pm10_serial += (value << 8); checksum_is += value; break;
      case (6): SDS_ID = value;              checksum_is += value; break;
      case (7): SDS_ID += (value << 8);      checksum_is += value; break;
      case (8):
      if (value == (checksum_is % 256)) { checksum_ok = true; } else { len = -1; checksum_ok = false;}; break;
      case (9): if (value != 0xAB) { len = -1; }; break;
    }
    len++;
    if ((len == 10) && (checksum_ok == true) && ((ignoretimer == true) || (MeasureTimer_SDS > MeasureTimerDelay_SDS * 100))) {
      if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
        sds_pm10_sum += pm10_serial;
        sds_pm25_sum += pm25_serial;
        sds_val_count++;
      }
      len = 0; checksum_ok = false; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
    }
  }
  if ((ignoretimer == true) || (MeasureTimer_SDS > ((MeasureTimerDelay_SDS * 100) + SDS_READ_TIME))) {
    // Calculate average
    MyDebug(true,"Sum pm10: " + String(sds_pm10_sum) + "  Cnt: " + String(sds_val_count));
    MyDebug(true,"Sum pm2.5: " + String(sds_pm25_sum) + "  Cnt: " + String(sds_val_count));
    pm10  = (sds_pm10_sum)/sds_val_count;
    pm2_5 = (sds_pm25_sum)/sds_val_count;
    String SDS_ID_s = Float2String(SDS_ID);          // DEBUG_PLN(SDS_ID_s);
    int dezPoint = SDS_ID_s.indexOf('.');
    SDS_ID_s = SDS_ID_s.substring(0, dezPoint);
    MyDebug(false,"Dev-ID: "+SDS_ID_s+"  "); MyDebug(true,SDS_ID);
    MyDebug(false,"PM10: ");
    MyDebug(true, pm10/10);
    MyDebug(false,"PM2.5: ");
    MyDebug(true,pm2_5/10);
    // clear sums and count
    sds_pm10_sum = 0;
    sds_pm25_sum = 0;
    sds_val_count = 0;
    stop_SDS();
    MeasureTimer_SDS -= (MeasureTimerDelay_SDS * 100) - SDS_WARMUP_TIME;
    Counter_SDS = 0;
    MyDebug(true,"SDS stopped");  
  }
}
#endif


//Teensy Time
void digitalClockDisplay() {
  // digital clock display of the time
  printDigits(hour());
  MyDebug(false,":");
  printDigits(minute());
  MyDebug(false,":");
  printDigits(second());
  MyDebug(false," ");
  printDigits(day());
  MyDebug(false,".");
  printDigits(month());
  MyDebug(false,".");
  MyDebug(true,year());  
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    MyDebug(false,'0');
  MyDebug(false,digits);
}
