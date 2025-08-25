// ===== ESP32-C3 NTRIP + BLE(NUS) (WiFi manual profiles 1..5 + Autoload) =====
// - BLE NUS: RX(Write) 명령, TX(Notify) 상태/로그
// - 부팅: prefs의 w_auto=1 && w_cur(1..5) 있으면 로드+WiFi 연결 시도
// - 명령:
//   WIFI SET ssid="..." pass="..."
//   WIFI CONNECT
//   WIFI SAVE idx=1..5
//   WIFI LOAD idx=1..5
//   WIFI AUTO on|off
//   WIFI STATUS
//   LOGHEX on|off
//   PING / CONNECT / DISCONNECT (NTRIP)
// - 상태 노티: "STATE wifi=connected ip=..." / "STATE wifi=disconnected"
//              "STATE ntrip=connected" / "STATE ntrip=disconnected"

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

// ---------- NTRIP 기본값 ----------
struct NtripConfig {
  String host = "rts2.ngii.go.kr";
  uint16_t port = 2101;
  String mount = "VRS-RTCM31";
  String user  = "kongseal03";
  String pass  = "ngii";
  double lat   = 36.3520;   // Uiseong fake
  double lon   = 128.6970;
  double altM  = 100.0;
  int    sats  = 12;
  float  hdop  = 1.0;
} cfg;

String authB64_default = "a29uZ3NlYWwwMzpuZ2lp"; // "kongseal03:ngii" base64

// ---------- Wi‑Fi 수동 입력 / 프로필 ----------
String wifi_ssid = "";
String wifi_pass = "";
Preferences prefs;

static const char* PREF_NS = "ntripble"; // preferences namespace
// 저장 키: w_ssid1..5, w_pass1..5, w_cur, w_auto

// ---------- Globals ----------
WiFiClient client;
uint32_t lastAlive = 0;
uint32_t lastGGA   = 0;
const uint32_t ALIVE_TIMEOUT_MS = 15000;
const uint32_t GGA_PERIOD_MS    = 5000;

bool bleConnected = false;
bool logHexToBle  = true;
uint32_t bleRateLimiterMs = 0;

// ---------- BLE (NUS) UUID ----------
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write
#define NUS_CHARACTERISTIC_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTx     = nullptr; // notify
NimBLECharacteristic* pRx     = nullptr; // write

// ---------- Utils ----------
String degToNmea(double deg, bool isLat) {
  double a = fabs(deg);
  int d = (int)a;
  double m = (a - d) * 60.0;
  char buf[20];
  if (isLat) sprintf(buf, "%02d%06.3f", d, m);
  else       sprintf(buf, "%03d%06.3f", d, m);
  return String(buf);
}
uint8_t nmeaChecksum(const String& s) {
  uint8_t cs = 0;
  for (size_t i = 0; i < s.length(); ++i) cs ^= (uint8_t)s[i];
  return cs;
}
String makeGGA() {
  time_t now = time(nullptr);
  struct tm* utc = gmtime(&now);
  char tbuf[16];
  if (utc) sprintf(tbuf, "%02d%02d%02d", utc->tm_hour, utc->tm_min, utc->tm_sec);
  else strcpy(tbuf, "000000");

  String lat = degToNmea(cfg.lat, true);
  String lon = degToNmea(cfg.lon, false);
  char ns = (cfg.lat >= 0) ? 'N' : 'S';
  char ew = (cfg.lon >= 0) ? 'E' : 'W';

  char core[160];
  sprintf(core, "GPGGA,%s,%s,%c,%s,%c,1,%02d,%.1f,%.1f,M,20.0,M,,",
          tbuf, lat.c_str(), ns, lon.c_str(), ew, cfg.sats, cfg.hdop, cfg.altM);

  String sentence = "$";
  sentence += core;
  char tail[8];
  sprintf(tail, "*%02X\r\n", nmeaChecksum(core));
  sentence += tail;
  return sentence;
}
void bleNotify(const String& s) {
  if (!bleConnected || pTx == nullptr) return;
  pTx->setValue((uint8_t*)s.c_str(), s.length());
  pTx->notify();
}
void bleNotifyHexSample(const uint8_t* data, int n) {
  if (!bleConnected || !logHexToBle) return;
  if (millis() - bleRateLimiterMs < 50) return;
  bleRateLimiterMs = millis();

  String line = "RTCM ";
  int maxBytes = min(n, 64);
  for (int i = 0; i < maxBytes; ++i) {
    char hex[4]; sprintf(hex, "%02X", data[i]);
    line += hex;
    if (i != maxBytes - 1) line += " ";
  }
  line += "\n";
  bleNotify(line);
}

// ---------- Wi‑Fi ----------
bool waitWiFi(uint32_t ms = 15000) {
  // 수동 입력 값을 사용 (wifi_ssid / wifi_pass)
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

  Serial.printf("[WiFi] Connecting to SSID=\"%s\"", wifi_ssid.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < ms) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
    bleNotify("STATE wifi=connected ip=" + WiFi.localIP().toString() + "\n");
    return true;
  }
  Serial.println("[WiFi] Failed.");
  bleNotify("STATE wifi=disconnected\n");
  return false;
}

String b64(const String& plain) {
  const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const uint8_t* p = (const uint8_t*)plain.c_str();
  size_t len = plain.length();
  String out;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = (uint32_t)p[i] << 16;
    if (i + 1 < len) v |= (uint32_t)p[i + 1] << 8;
    if (i + 2 < len) v |= (uint32_t)p[i + 2];
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += (i + 1 < len) ? tbl[(v >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? tbl[v & 0x3F] : '=';
  }
  return out;
}
String makeAuthB64() {
  if (cfg.user == "kongseal03" && cfg.pass == "ngii") return authB64_default;
  return b64(cfg.user + ":" + cfg.pass);
}

// ---------- NTRIP ----------
bool sendNtripRequest() {
  WiFiClient& s = client;
  Serial.printf("[NTRIP] Connecting to %s:%u\n", cfg.host.c_str(), cfg.port);
  if (!s.connect(cfg.host.c_str(), cfg.port)) {
    Serial.println("[NTRIP] TCP connect failed.");
    return false;
  }

  String req = "GET /" + cfg.mount + " HTTP/1.0\r\n";
  req += "Host: " + cfg.host + "\r\n";
  req += "User-Agent: NTRIP\r\n";
  req += "Accept: */*\r\n";
  req += "Authorization: Basic " + makeAuthB64() + "\r\n\r\n";

  Serial.println("[NTRIP] Request:");
  Serial.println(req);
  s.print(req);

  // 헤더 읽기
  String header;
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    while (s.available()) {
      char c = s.read();
      header += c;
      if (header.endsWith("\r\n\r\n")) {
        Serial.println("[NTRIP] Header:");
        Serial.println(header);
        if (header.startsWith("HTTP/1.0 200") ||
            header.startsWith("HTTP/1.1 200") ||
            header.startsWith("ICY 200")) {
          Serial.println("[NTRIP] OK, start streaming.");
          return true;
        } else {
          Serial.println("[NTRIP] Bad response.");
          return false;
        }
      }
    }
    delay(10);
  }

  Serial.println("[NTRIP] Header timeout.");
  return false;
}


void pumpRtcmToSerial() {
  static uint8_t buf[512];
  int n = client.read(buf, sizeof(buf));
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      if ((i % 32) == 0) Serial.println();
      char hex[4]; sprintf(hex, "%02X ", buf[i]); Serial.print(hex);
    }
    bleNotifyHexSample(buf, n);
    lastAlive = millis();
  }
}

// ---------- BLE Callbacks ----------
class ServerCB : public NimBLEServerCallbacks {
  void onConnectedCommon(const char* tag) {
    bleConnected = true;
    Serial.printf("[BLE] onConnect(%s)\n", tag);
    bleNotify("STATE ble=connected\n");
    bleNotify("HELLO from ESP32-C3\n");
  }
  void onDisconnectedCommon(const char* tag) {
    bleConnected = false;
    Serial.printf("[BLE] onDisconnect(%s)\n", tag);
    bleNotify("STATE ble=disconnected\n");
    NimBLEDevice::startAdvertising();
  }

  void onConnect(NimBLEServer* s) { onConnectedCommon("no-info"); }
  void onDisconnect(NimBLEServer* s) { onDisconnectedCommon("no-info"); }

  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) { onConnectedCommon("ConnInfo"); }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info) { onDisconnectedCommon("ConnInfo"); }
};

// ---------- Wi‑Fi Profiles helpers ----------
void wifiSaveSlot(int slot) {
  prefs.begin(PREF_NS, false);
  char key[12];
  sprintf(key, "w_ssid%d", slot); prefs.putString(key, wifi_ssid);
  sprintf(key, "w_pass%d", slot); prefs.putString(key, wifi_pass);
  prefs.putInt("w_cur", slot);
  prefs.end();
}

bool wifiLoadSlot(int slot) {
  prefs.begin(PREF_NS, false);
  char keySsid[12], keyPass[12];
  sprintf(keySsid, "w_ssid%d", slot);
  sprintf(keyPass, "w_pass%d", slot);
  String s = prefs.getString(keySsid, "");
  String p = prefs.getString(keyPass, "");
  prefs.end();
  if (s.length() == 0) return false;
  wifi_ssid = s;
  wifi_pass = p;
  return true;
}

void wifiSetAuto(bool on) {
  prefs.begin(PREF_NS, false);
  prefs.putBool("w_auto", on);
  prefs.end();
}

bool wifiGetAuto() {
  prefs.begin(PREF_NS, true);
  bool on = prefs.getBool("w_auto", false);
  prefs.end();
  return on;
}

int wifiGetCur() {
  prefs.begin(PREF_NS, true);
  int cur = prefs.getInt("w_cur", 0);
  prefs.end();
  return cur;
}

// ---------- BLE Command parsing ----------
void handleCommand(const String& cmdLine) {
  String s = cmdLine; String orig = s; s.trim();
  if (s.length() == 0) return;

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String rest = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") { bleNotify("PONG\n"); Serial.println("[BLE RX] PING"); return; }

  if (cmd == "CONNECT") {
    Serial.println("[CMD] CONNECT (NTRIP)");
    if (WiFi.status() != WL_CONNECTED) {
      if (!waitWiFi()) return;
    }
    if (!client.connected()) sendNtripRequest();
    return;
  }
  if (cmd == "DISCONNECT") {
    Serial.println("[CMD] DISCONNECT (NTRIP)");
    if (client.connected()) client.stop();
    bleNotify("STATE ntrip=disconnected\n");
    return;
  }
  if (cmd == "LOGHEX") {
    String v = rest; v.trim(); v.toLowerCase();
    if (v == "on")  { logHexToBle = true;  bleNotify("INFO loghex=on\n"); }
    if (v == "off") { logHexToBle = false; bleNotify("INFO loghex=off\n"); }
    return;
  }

  if (cmd == "SET") {
    // NTRIP 필드 설정
    Serial.print("[CMD] "); Serial.println(orig);
    int idx = 0;
    while (idx < rest.length()) {
      while (idx < rest.length() && rest[idx] == ' ') idx++;
      int next = rest.indexOf(' ', idx);
      String kv = (next < 0) ? rest.substring(idx) : rest.substring(idx, next);
      if (kv.length() == 0) break;
      int eq = kv.indexOf('=');
      if (eq > 0) {
        String k = kv.substring(0, eq); String v = kv.substring(eq + 1);
        k.toLowerCase();
        if (k == "host")  cfg.host = v;
        else if (k == "port") cfg.port = (uint16_t)v.toInt();
        else if (k == "mount") cfg.mount = v;
        else if (k == "user")  cfg.user  = v;
        else if (k == "pass")  cfg.pass  = v;
        else if (k == "lat")   cfg.lat   = v.toFloat();
        else if (k == "lon")   cfg.lon   = v.toFloat();
        else if (k == "alt")   cfg.altM  = v.toFloat();
        else if (k == "sats")  cfg.sats  = v.toInt();
        else if (k == "hdop")  cfg.hdop  = v.toFloat();
      }
      if (next < 0) break;
      idx = next + 1;
    }
    bleNotify("INFO set=ok\n");
    return;
  }

  // ===== Wi‑Fi manual provisioning =====
  if (cmd == "WIFI") {
    String sub; int sp2 = rest.indexOf(' ');
    sub = (sp2 < 0) ? rest : rest.substring(0, sp2);
    String args = (sp2 < 0) ? "" : rest.substring(sp2 + 1);
    sub.toUpperCase();

    if (sub == "SET") {
      // WIFI SET ssid="..." pass="..."
      // 단순 파서 (따옴표 지원)
      String ssid = wifi_ssid, pass = wifi_pass;
      int i;

      i = args.indexOf("ssid=");
      if (i >= 0) {
        int j = i + 5;
        if (j < (int)args.length() && args[j] == '\"') {
          int k = args.indexOf('\"', j + 1);
          if (k > j) ssid = args.substring(j + 1, k);
        } else {
          int k = args.indexOf(' ', j);
          if (k < 0) k = args.length();
          ssid = args.substring(j, k);
        }
      }
      i = args.indexOf("pass=");
      if (i >= 0) {
        int j = i + 5;
        if (j < (int)args.length() && args[j] == '\"') {
          int k = args.indexOf('\"', j + 1);
          if (k > j) pass = args.substring(j + 1, k);
        } else {
          int k = args.indexOf(' ', j);
          if (k < 0) k = args.length();
          pass = args.substring(j, k);
        }
      }

      wifi_ssid = ssid;
      wifi_pass = pass;
      Serial.printf("[WIFI] SET ssid=\"%s\" pass_len=%d\n", wifi_ssid.c_str(), wifi_pass.length());
      bleNotify("INFO wifi_set ok\n");
      return;
    }

    if (sub == "CONNECT") {
      Serial.println("[WIFI] CONNECT");
      waitWiFi();
      return;
    }

    if (sub == "SAVE") {
      int i = args.indexOf("idx=");
      if (i < 0) { bleNotify("ERROR need idx=1..5\n"); return; }
      int j = i + 4;
      int k = args.indexOf(' ', j);
      int slot = args.substring(j, (k < 0 ? args.length() : k)).toInt();
      if (slot < 1 || slot > 5) { bleNotify("ERROR idx_range_1_5\n"); return; }
      wifiSaveSlot(slot);
      Serial.printf("[WIFI] SAVE idx=%d\n", slot);
      bleNotify("INFO wifi_save ok idx=" + String(slot) + "\n");
      return;
    }

    if (sub == "LOAD") {
      int i = args.indexOf("idx=");
      if (i < 0) { bleNotify("ERROR need idx=1..5\n"); return; }
      int j = i + 4;
      int k = args.indexOf(' ', j);
      int slot = args.substring(j, (k < 0 ? args.length() : k)).toInt();
      if (slot < 1 || slot > 5) { bleNotify("ERROR idx_range_1_5\n"); return; }
      bool ok = wifiLoadSlot(slot);
      Serial.printf("[WIFI] LOAD idx=%d -> %s\n", slot, ok ? "ok" : "empty");
      bleNotify(ok ? "INFO wifi_load ok idx=" + String(slot) + "\n"
                   : "ERROR wifi_load empty\n");
      return;
    }

    if (sub == "AUTO") {
      String v = args; v.trim(); v.toLowerCase();
      if (v == "on")  { wifiSetAuto(true);  bleNotify("INFO wifi_auto=on\n"); }
      else if (v == "off") { wifiSetAuto(false); bleNotify("INFO wifi_auto=off\n"); }
      else { bleNotify("ERROR wifi_auto need on|off\n"); }
      return;
    }

    if (sub == "STATUS") {
      String st = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
      String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
      int cur = wifiGetCur();
      bool au = wifiGetAuto();
      bleNotify("STATE wifi=" + st + " ip=" + ip + " cur=" + String(cur) + " auto=" + String(au ? 1:0) + "\n");
      return;
    }

    bleNotify("ERROR wifi_unknown\n");
    return;
  }

  bleNotify("ERROR unknown_cmd\n");
}

class RxCB : public NimBLECharacteristicCallbacks {
  void handle(const char* tag, NimBLECharacteristic* ch) {
    std::string v = ch->getValue();
    String s = String(v.c_str());
    Serial.printf("[BLE RX] (%s) %s\n", tag, s.c_str());
    handleCommand(s);
  }
  void onWrite(NimBLECharacteristic* ch) { handle("no-info", ch); }
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo& /*info*/) { handle("ConnInfo", ch); }
};

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== ESP32-C3 NTRIP + BLE(NUS) (manual WiFi profiles) =====");

  // 시간 동기 (GGA 시각 출력용)
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  // BLE
  NimBLEDevice::init("NTRIPBLE-C3");
  NimBLEDevice::setMTU(185);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  NimBLEService* svc = pServer->createService(NUS_SERVICE_UUID);
  pTx = svc->createCharacteristic(NUS_CHARACTERISTIC_TX, NIMBLE_PROPERTY::NOTIFY);
  pRx = svc->createCharacteristic(NUS_CHARACTERISTIC_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRx->setCallbacks(new RxCB());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  NimBLEAdvertisementData ad;  ad.setName("NTRIPBLE-C3"); adv->setAdvertisementData(ad);
  NimBLEAdvertisementData srd; srd.setName("NTRIPBLE-C3"); adv->setScanResponseData(srd);
  adv->start();

  Serial.println("[BLE] Advertising as NTRIPBLE-C3");

  // ===== 부팅 자동 연결 (auto on && cur slot 존재 시) =====
  int cur = wifiGetCur();
  bool au = wifiGetAuto();
  Serial.printf("[BOOT] auto=%d cur=%d\n", au ? 1:0, cur);
  if (au && cur >= 1 && cur <= 5) {
    if (wifiLoadSlot(cur)) {
      Serial.printf("[BOOT] Load slot %d ssid=\"%s\"\n", cur, wifi_ssid.c_str());
      waitWiFi(); // 결과는 STATE wifi=... 으로 BLE에 알림
    } else {
      Serial.println("[BOOT] Current slot empty, skip connect");
    }
  }
}

// ---------- Loop ----------
void loop() {
  // BLE Heartbeat (2s)
  static uint32_t lastBeat = 0;
  if (bleConnected && millis() - lastBeat > 2000) {
    bleNotify("HEARTBEAT\n");
    lastBeat = millis();
  }

  // NTRIP
  if (client.connected()) {
    if (millis() - lastGGA >= GGA_PERIOD_MS) {
      String gga = makeGGA();
      client.print(gga);
      Serial.print("\n[TX GGA] "); Serial.print(gga);
      bleNotify("TX GGA " + gga);
      lastGGA = millis();
    }
    if (client.available()) {
      pumpRtcmToSerial();
    } else {
      if (millis() - lastAlive > ALIVE_TIMEOUT_MS) {
        Serial.println("\n[NTRIP] No data. Reconnect.");
        bleNotify("STATE ntrip=disconnected\n");
        client.stop();
      }
    }
  }
  delay(5);
}