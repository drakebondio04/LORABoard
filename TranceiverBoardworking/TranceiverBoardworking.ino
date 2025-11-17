// ===== TX: ESP32 + MPU9250 + GPS -> LoRa (adds heading; newline-terminated) =====
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <RadioLib.h>

// ---- LoRa SX1262 pins ----
#define LORA_NSS   8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13
SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

// ---- GPS (TinyGPS++) ----
TinyGPSPlus gps;
#define GPS_BAUD 9600
#define GPS_RX   46   // ESP32 RX (connect to GPS TX)
#define GPS_TX   45   // ESP32 TX (optional)

// ---- MPU9250 (Accel only) ----
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV      0x19
#define CONFIG_REG      0x1A
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D
#define ACCEL_XOUT_H    0x3B
static const float ACC_LSB_PER_G = 16384.0f;

// Axis mapping (keep A unless your frame requires B/C)
#define AXIS_PRESET_A
#ifdef AXIS_PRESET_A
  #define MAP_AX(ax,ay,az) ( ax)
  #define MAP_AY(ax,ay,az) ( ay)
  #define MAP_AZ(ax,ay,az) ( az)
#endif
#ifdef AXIS_PRESET_B
  #define MAP_AX(ax,ay,az) (  az)
  #define MAP_AY(ax,ay,az) (  ay)
  #define MAP_AZ(ax,ay,az) (-(ax))
#endif
#ifdef AXIS_PRESET_C
  #define MAP_AX(ax,ay,az) (-(az))
  #define MAP_AY(ax,ay,az) (  ay)
  #define MAP_AZ(ax,ay,az) (  ax)
#endif

float ax_bias=0, ay_bias=0, az_bias=0;

// ---- I2C helpers ----
void wr(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
int16_t rd16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available()<2) return 0;
  uint8_t hi=Wire.read(), lo=Wire.read(); return (int16_t)((hi<<8)|lo);
}
void readAccelG_raw(float &ax,float &ay,float &az){
  ax = (float)rd16(ACCEL_XOUT_H)     / ACC_LSB_PER_G;
  ay = (float)rd16(ACCEL_XOUT_H + 2) / ACC_LSB_PER_G;
  az = (float)rd16(ACCEL_XOUT_H + 4) / ACC_LSB_PER_G;
}

// ---- MPU init + quick accel cal ----
void mpuInit(){
  wr(PWR_MGMT_1, 0x80); delay(100);
  wr(PWR_MGMT_1, 0x01); delay(10);
  wr(SMPLRT_DIV,    0x09);
  wr(CONFIG_REG,    0x03);
  wr(ACCEL_CONFIG2, 0x03);
  wr(ACCEL_CONFIG,  0x00);   // ±2 g
}
void calibrateAccel(size_t N=300){
  float sx=0, sy=0, sz=0;
  for(size_t i=0;i<N;i++){
    float ax,ay,az; readAccelG_raw(ax,ay,az);
    sx+=ax; sy+=ay; sz+=az; delay(2);
  }
  ax_bias = (sx/N);
  ay_bias = (sy/N);
  az_bias = (sz/N) - 1.0f;
}

// ---- roll/pitch from accel ----
void accelToRollPitch(float ax,float ay,float az, float &roll_deg, float &pitch_deg){
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  roll_deg  = roll  * 180.0f / PI;
  pitch_deg = pitch * 180.0f / PI;
}

// (optional) set GPS to 5 Hz; safe to omit if you already did this elsewhere
void gpsSetRate5Hz(){
  uint8_t setRate[] = {0xB5,0x62,0x06,0x08,0x06,0x00, 0xC8,0x00, 0x01,0x00, 0x01,0x00, 0xDF,0x3E};
  Serial2.write(setRate, sizeof(setRate));
}

uint32_t lastSend = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(4,5); Wire.setClock(400000);
  mpuInit(); calibrateAccel();

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(300);
  gpsSetRate5Hz();

  int st = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8);
  radio.setSyncWord(0x12); radio.setCRC(2);
  Serial.println(st == RADIOLIB_ERR_NONE ? "[LoRa] ready" : "[LoRa] init failed");
}

void loop() {
  while (Serial2.available()) gps.encode(Serial2.read());
  if (millis() - lastSend < 200) return;  // ~5 Hz
  lastSend = millis();

  // IMU
  float axr,ayr,azr; readAccelG_raw(axr,ayr,azr);
  float ax = MAP_AX(axr-ax_bias, ayr-ay_bias, azr-az_bias);
  float ay = MAP_AY(axr-ax_bias, ayr-ay_bias, azr-az_bias);
  float az = MAP_AZ(axr-ax_bias, ayr-ay_bias, azr-az_bias);

  float roll, pitch; accelToRollPitch(ax, ay, az, roll, pitch);

  // GPS heading (course over ground, deg), lat, lon
  double heading = gps.course.isValid() ? gps.course.deg() : NAN;
  double lat     = gps.location.isValid() ? gps.location.lat() : NAN;
  double lon     = gps.location.isValid() ? gps.location.lng() : NAN;

  // CSV + newline (helps RX trim cleanly)
  char pkt[120];
  int n = snprintf(pkt, sizeof(pkt),
                   "%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.6f,%.6f\n",
                   ax, ay, az, roll, pitch, heading, lat, lon);

  radio.transmit((uint8_t*)pkt, (size_t)n);
  Serial.print("TX: "); Serial.print(pkt); // already ends with \n
}
