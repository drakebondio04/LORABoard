// ===== Receiver: ESP32 + SX1262 (RadioLib) -> print parsed CSV =====
// Expects CSV: ax,ay,az,roll,pitch,lat,lon

#include <SPI.h>
#include <RadioLib.h>

// LoRa pins (match transmitter)
#define LORA_NSS   8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13
SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== RX: ESP32 + SX1262 ===");

  // LoRa init (must match TX)
  int st = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setSyncWord(0x12);
    radio.setCRC(2);
    Serial.println("[lora] OK");
  } else {
    Serial.print("[lora] init failed code "); Serial.println(st);
  }
}

void loop() {
  // Receive into a buffer
  uint8_t buf[160];
  int16_t len = sizeof(buf);
  int st = radio.receive(buf, len);

  if (st == RADIOLIB_ERR_NONE) {
    // Ensure termination
    if (len <= 0) return;
    if (len >= (int)sizeof(buf)) len = sizeof(buf)-1;
    buf[len] = 0;

    // Raw line
    Serial.print("RX: ");
    Serial.println((char*)buf);

    // Parse CSV
    float ax=0, ay=0, az=0, roll=0, pitch=0;
    double lat=NAN, lon=NAN;

    // Using sscanf for brevity
    // Note: %lf for double (lat/lon)
    int parsed = sscanf((char*)buf, "%f,%f,%f,%f,%f,%lf,%lf",
                        &ax, &ay, &az, &roll, &pitch, &lat, &lon);

    if (parsed == 7) {
      Serial.print("  Accel[g]: ");
      Serial.print(ax,3); Serial.print(", ");
      Serial.print(ay,3); Serial.print(", ");
      Serial.println(az,3);

      Serial.print("  R/P[deg]: ");
      Serial.print(roll,1); Serial.print(", ");
      Serial.println(pitch,1);

      Serial.print("  Lat/Lon: ");
      if (isnan(lat) || isnan(lon)) Serial.println("NO_FIX");
      else { Serial.print(lat,6); Serial.print(", "); Serial.println(lon,6); }

      Serial.print("  RSSI: "); Serial.print(radio.getRSSI()); Serial.print(" dBm");
      Serial.print(" | SNR: "); Serial.print(radio.getSNR()); Serial.println(" dB");
    } else {
      Serial.println("  [warn] CSV parse failed.");
    }
  }
  // If no packet: RadioLib returns error codes; we just keep looping.
}
