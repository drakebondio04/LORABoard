// ===== RX: ESP32 + SX1262 — clean parse, no trailing gibberish =====
#include <SPI.h>
#include <RadioLib.h>

#define LORA_NSS   8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13
SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

static inline bool isPrintableCSV(uint8_t c) {
  // allow digits, signs, decimal, comma, space, CR/LF
  return (c >= '0' && c <= '9') || c=='-' || c=='+' || c=='.' || c==',' ||
         c==' ' || c=='\r' || c=='\n';
}

void setup(){
  Serial.begin(115200); delay(200);
  int st = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setSyncWord(0x12);
    radio.setCRC(2);
    Serial.println("[LoRa] ready");
  } else {
    Serial.print("[LoRa] init fail "); Serial.println(st);
  }
}

void loop(){
  uint8_t buf[200];
  int16_t len = sizeof(buf);
  int st = radio.receive(buf, len);

  if (st == RADIOLIB_ERR_NONE && len > 0) {
    // 1) Bound to 'len'
    size_t n = (size_t)len;
    if (n > sizeof(buf)) n = sizeof(buf);

    // 2) Copy only printable/CSV chars up to first newline
    static char line[200];
    size_t w = 0;
    for (size_t i = 0; i < n && w < sizeof(line)-1; i++) {
      uint8_t c = buf[i];
      if (!isPrintableCSV(c)) continue;             // drop garbage
      if (c == '\n' || c == '\r') break;            // stop at end-of-line
      line[w++] = (char)c;
    }
    line[w] = '\0';

    // 3) Parse: ax,ay,az,roll,pitch,heading,lat,lon  (heading may be NA/NAN)
    float ax=0, ay=0, az=0, roll=0, pitch=0, heading=0;
    double lat=NAN, lon=NAN;

    // Try full 8-field parse (with heading)
    int parsed = sscanf(line, "%f,%f,%f,%f,%f,%f,%lf,%lf",
                        &ax,&ay,&az,&roll,&pitch,&heading,&lat,&lon);

    if (parsed < 8) {
      // Fallback to legacy 7-field packet (no heading)
      heading = NAN;
      parsed = sscanf(line, "%f,%f,%f,%f,%f,%lf,%lf",
                      &ax,&ay,&az,&roll,&pitch,&lat,&lon);
      if (parsed != 7) {
        Serial.print("RX(raw): "); Serial.println(line);
        Serial.println("  [warn] CSV parse failed.");
        return;
      }
    }

    // Local timestamp
    uint32_t t_ms = millis();

    // Clean print
    Serial.print("t[ms]="); Serial.print(t_ms);
    Serial.print(" | A[g]: ");
    Serial.print(ax,3); Serial.print(", ");
    Serial.print(ay,3); Serial.print(", ");
    Serial.print(az,3);
    Serial.print(" | R/P[deg]: ");
    Serial.print(roll,1); Serial.print(", ");
    Serial.print(pitch,1);
    Serial.print(" | Hdg[deg]: ");
    if (isnan(heading)) Serial.print("NA"); else Serial.print(heading,1);
    Serial.print(" | Lat/Lon: ");
    if (isnan(lat) || isnan(lon)) Serial.print("NO_FIX");
    else { Serial.print(lat,6); Serial.print(", "); Serial.print(lon,6); }
    Serial.print(" | RSSI "); Serial.print(radio.getRSSI());
    Serial.print(" dBm | SNR "); Serial.print(radio.getSNR()); Serial.println(" dB");
  }
}
