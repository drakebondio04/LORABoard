// ===== RX: ESP32 + SX1262 — sticky GPS & robust CSV parsing =====
// Accepts 5 fields (ax,ay,az,roll,pitch) or 7/8 fields (… + lat/lon and optional heading)
// Keeps last valid GPS/heading while continuing IMU updates every packet.

#include <SPI.h>
#include <RadioLib.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#define LORA_NSS   8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13
SX1262 radio = new Module(LORA_NSS, LORA_IRQ, LORA_RST, LORA_BUSY);

// ---- Sticky GPS/heading ----
static double last_lat = NAN, last_lon = NAN;
static float  last_heading = NAN;
static uint32_t last_gps_ms = 0, last_hdg_ms = 0;

static inline bool isPrintableCSV(uint8_t c) {
  // allow digits, signs, decimal, comma, space, CR/LF, 'n' 'a' for "nan"/"NA"
  return (c >= '0' && c <= '9') || c=='-' || c=='+' || c=='.' || c==',' ||
         c==' ' || c=='\r' || c=='\n' || c=='n' || c=='N' || c=='a' || c=='A';
}

static bool parseFloatMaybe(const char* s, float& out) {
  if (!s || !*s) return false;
  while (*s==' ') s++;
  if (!*s) return false;
  if (!strncasecmp(s, "nan", 3) || !strncasecmp(s, "NA", 2)) return false;
  char* endp = nullptr;
  float v = strtof(s, &endp);
  if (endp == s) return false;
  out = v; return true;
}
static bool parseDoubleMaybe(const char* s, double& out) {
  if (!s || !*s) return false;
  while (*s==' ') s++;
  if (!*s) return false;
  if (!strncasecmp(s, "nan", 3) || !strncasecmp(s, "NA", 2)) return false;
  char* endp = nullptr;
  double v = strtod(s, &endp);
  if (endp == s) return false;
  out = v; return true;
}

// Tokenize up to 8 fields (copy into local buffer to use strtok_r)
static uint8_t splitCSV(const char* line, const char* out[8]) {
  static char buf[200];
  size_t L = strnlen(line, sizeof(buf)-1);
  memcpy(buf, line, L);
  buf[L] = '\0';
  uint8_t count = 0;
  char* save = nullptr;
  char* tok = strtok_r(buf, ",", &save);
  while (tok && count < 8) {
    while (*tok==' ') tok++;
    char* e = tok + strlen(tok);
    while (e>tok && e[-1]==' ') { e--; }
    *e = '\0';
    out[count++] = tok;
    tok = strtok_r(nullptr, ",", &save);
  }
  return count;
}

void setup(){
  Serial.begin(115200); delay(200);

  // Match TX fast PHY:
  int st = radio.begin(915.0, 250.0, 7, 5,
                       RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 6);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setSyncWord(0x12);
    radio.setCRC(2);
    Serial.println("[LoRa] ready (BW 250k, SF7, CR 4/5)");
  } else {
    Serial.print("[LoRa] init fail "); Serial.println(st);
  }
}

void loop(){
  uint8_t buf[200];
  int16_t len = sizeof(buf);
  int st = radio.receive(buf, len);

  if (st == RADIOLIB_ERR_NONE && len > 0) {
    // Bound copy and filter to printable CSV chars
    static char line[200];
    size_t n = (size_t)len; if (n > sizeof(buf)) n = sizeof(buf);
    size_t w = 0;
    for (size_t i = 0; i < n && w < sizeof(line)-1; i++) {
      uint8_t c = buf[i];
      if (!isPrintableCSV(c)) continue;
      if (c == '\n' || c == '\r') break;
      line[w++] = (char)c;
    }
    line[w] = '\0';
    if (!w) return;

    // Split
    const char* fld[8] = {0};
    uint8_t fc = splitCSV(line, fld);

    // Accept 5, 7, or 8 fields
    if (fc != 5 && fc != 7 && fc != 8) {
      Serial.print("RX(raw): "); Serial.println(line);
      Serial.println("  [warn] CSV parse failed (need 5, 7, or 8 fields).");
      return;
    }

    // Required IMU
    float ax=0, ay=0, az=0, roll=0, pitch=0;
    bool ok =
      parseFloatMaybe(fld[0], ax) &&
      parseFloatMaybe(fld[1], ay) &&
      parseFloatMaybe(fld[2], az) &&
      parseFloatMaybe(fld[3], roll) &&
      parseFloatMaybe(fld[4], pitch);
    if (!ok) {
      Serial.print("RX(raw): "); Serial.println(line);
      Serial.println("  [warn] CSV parse failed (bad required fields).");
      return;
    }

    // Optional heading/lat/lon
    if (fc == 8) {
      float hdg_tmp;
      if (parseFloatMaybe(fld[5], hdg_tmp)) {
        last_heading = hdg_tmp;
        last_hdg_ms  = millis();
      }
      double lat_tmp, lon_tmp;
      bool lat_ok = parseDoubleMaybe(fld[6], lat_tmp);
      bool lon_ok = parseDoubleMaybe(fld[7], lon_tmp);
      if (lat_ok && lon_ok) { last_lat = lat_tmp; last_lon = lon_tmp; last_gps_ms = millis(); }
    } else if (fc == 7) {
      double lat_tmp, lon_tmp;
      bool lat_ok = parseDoubleMaybe(fld[5], lat_tmp);
      bool lon_ok = parseDoubleMaybe(fld[6], lon_tmp);
      if (lat_ok && lon_ok) { last_lat = lat_tmp; last_lon = lon_tmp; last_gps_ms = millis(); }
      // heading sticky
    } // fc==5 → IMU only; keep sticky values

    uint32_t t_ms = millis();

    // Pretty print
    Serial.print("t[ms]="); Serial.print(t_ms);
    Serial.print(" | A[g]: ");
    Serial.print(ax,3); Serial.print(", ");
    Serial.print(ay,3); Serial.print(", ");
    Serial.print(az,3);
    Serial.print(" | R/P[deg]: ");
    Serial.print(roll,1); Serial.print(", ");
    Serial.print(pitch,1);
    Serial.print(" | Hdg[deg]: ");
    if (isnan(last_heading)) Serial.print("NA"); else Serial.print(last_heading,1);
    Serial.print(" | Lat/Lon: ");
    if (isnan(last_lat) || isnan(last_lon)) Serial.print("NO_FIX");
    else { Serial.print(last_lat,6); Serial.print(", "); Serial.print(last_lon,6); }
    Serial.print(" | RSSI "); Serial.print(radio.getRSSI());
    Serial.print(" dBm | SNR "); Serial.print(radio.getSNR());
    Serial.print(" dB | age_gps ");
    if (last_gps_ms) Serial.print(t_ms - last_gps_ms); else Serial.print("NA");
    Serial.print(" ms | age_hdg ");
    if (last_hdg_ms) Serial.print(t_ms - last_hdg_ms); else Serial.print("NA");
    Serial.println(" ms");
  }
}
