// ===== Transmitter: ESP32 + MPU9250 + GPS -> LoRa (SX1262 / RadioLib) =====
// Sends: ax,ay,az,roll,pitch,lat,lon @ ~5 Hz and forces GPS to 5 Hz update rate.

#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <RadioLib.h>

// -------- LoRa pins (SX1262) --------
#define LORA_NSS   8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13
SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

// -------- GPS --------
TinyGPSPlus gps;
#define GPS_BAUD 9600
#define GPS_RX   46
#define GPS_TX   45

// ⭐ NEW ⭐ — Set GPS update rate to 5 Hz
void gpsSetRate5Hz() {
  uint8_t setRate[] = {
    0xB5,0x62,  // UBX header
    0x06,0x08,  // CFG-RATE
    0x06,0x00,  // payload length
    0xC8,0x00,  // measRate = 200 ms (1000ms / 5 Hz = 200ms)
    0x01,0x00,  // navRate = 1
    0x01,0x00,  // timeRef = GPS time
    0xDF,0x3E   // checksum
  };
  Serial2.write(setRate, sizeof(setRate));
}

// ⭐ NEW ⭐ — Save GPS config to flash (optional but recommended)
void gpsSaveConfig() {
  uint8_t saveCfg[] = {
    0xB5,0x62,0x06,0x09,0x0D,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,
    0x17,0x31
  };
  Serial2.write(saveCfg, sizeof(saveCfg));
}

// -------- MPU9250 -------- (unchanged setup & mapping)
#define MPU_ADDR        0x68
#define ACCEL_XOUT_H    0x3B
static const float ACC_LSB_PER_G = 16384.0f;

#define MAP_AX(ax,ay,az) ( ax)
#define MAP_AY(ax,ay,az) ( ay)
#define MAP_AZ(ax,ay,az) ( az)

float ax_bias=0, ay_bias=0, az_bias=0;

void wr(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
int16_t rd16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  uint8_t hi=Wire.read(), lo=Wire.read();
  return (int16_t)((hi<<8)|lo);
}
void readAccelG_raw(float &ax,float &ay,float &az){
  ax = (float)rd16(ACCEL_XOUT_H)     / ACC_LSB_PER_G;
  ay = (float)rd16(ACCEL_XOUT_H + 2) / ACC_LSB_PER_G;
  az = (float)rd16(ACCEL_XOUT_H + 4) / ACC_LSB_PER_G;
}
void mpuInit(){
  wr(0x6B, 0x80); delay(100);
  wr(0x6B, 0x01); delay(10);
  wr(0x19, 0x09);
  wr(0x1A, 0x03);
  wr(0x1D, 0x03);
  wr(0x1C, 0x00);
}
void calibrateAccel(){
  float sx=0, sy=0, sz=0;
  for(int i=0;i<300;i++){
    float ax,ay,az; readAccelG_raw(ax,ay,az);
    sx+=ax; sy+=ay; sz+=az;
    delay(2);
  }
  ax_bias = (sx/300);
  ay_bias = (sy/300);
  az_bias = (sz/300) - 1.0f;
}
void accelToRollPitch(float ax,float ay,float az, float &roll, float &pitch){
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
}

uint32_t lastSend = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(4,5);
  Wire.setClock(400000);
  mpuInit();
  calibrateAccel();

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(400);

  // ⭐ NEW ⭐ Apply 5 Hz update rate + persist
  gpsSetRate5Hz();
  gpsSaveConfig();
  Serial.println("[gps] Set to 5 Hz ✅");

  radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8);
  radio.setSyncWord(0x12);
  radio.setCRC(2);
  Serial.println("[lora] ready ✅");
}

void loop() {
  while (Serial2.available()) gps.encode(Serial2.read());
  if (millis() - lastSend < 200) return;
  lastSend = millis();

  float axr,ayr,azr; readAccelG_raw(axr,ayr,azr);
  float ax = MAP_AX(axr-ax_bias, ayr-ay_bias, azr-az_bias);
  float ay = MAP_AY(axr-ax_bias, ayr-ay_bias, azr-az_bias);
  float az = MAP_AZ(axr-ax_bias, ayr-ay_bias, azr-az_bias);

  float roll, pitch;
  accelToRollPitch(ax, ay, az, roll, pitch);

  double lat = gps.location.isValid() ? gps.location.lat() : NAN;
  double lon = gps.location.isValid() ? gps.location.lng() : NAN;

  char pkt[100];
  //snprintf(pkt, sizeof(pkt), "%.3f,%.3f,%.3f,%.1f,%.1f,%.6f,%.6f", ax,ay,az,roll,pitch,lat,lon);
    snprintf(pkt, sizeof(pkt), "Hello")
  radio.transmit((uint8_t*)pkt, strlen(pkt));
  Serial.println(pkt);
}
