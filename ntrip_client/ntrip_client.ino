// ===== ESP32-C3 NTRIP + BLE(NUS) =====
// - BLE NUS: RX(Write) 명령, TX(Notify) 로그/상태
// - 부팅시 WiFi 자동 연결(옵션), 프로필 1..5 저장/로드
// - NTRIP: 표준 v2 요청(HTTP/1.1 + Ntrip-Version:2.0 + UA 고정)로 복원
// - 5초마다 GGA 전송, RTCM 수신을 시리얼 HEX + BLE 샘플로 노티

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <NimBLEDevice.h>

// ---------- BLE UUID (Nordic NUS) ----------
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write
#define NUS_CHARACTERISTIC_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify

// ---------- NTRIP 설정 ----------
struct NtripConfig {
  String host  = "rts2.ngii.go.kr";
  uint16_t port = 2101;
  String mount = "VRS-RTCM31";
  String user  = "kongseal03";
  String pass  = "ngii";
  double lat   = 36.3520;
  double lon   = 128.6970;
  double altM  = 100.0;
  int    sats  = 12;
  float  hdop  = 1.0f;
} cfg;

// "kongseal03:ngii" base64 (속도 최적화용 고정값)
String authB64_default = "a29uZ3NlYWwwMzpuZ2lp";

// ---------- Wi-Fi 프로필 ----------
struct WifiProfile {
  String ssid;
  String pass;
} wcur;

Preferences prefs;
bool wifiAuto = false;   // 부팅 자동 연결
uint8_t wifiLast = 0;    // 마지막 사용 슬롯(1..5)

// ---------- 글로벌 ----------
WiFiClient client;

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTx     = nullptr;
NimBLECharacteristic* pRx     = nullptr;

bool bleConnected = false;
bool logHexToBle  = true;

uint32_t lastAlive = 0;
uint32_t lastGGA   = 0;
const uint32_t ALIVE_TIMEOUT_MS = 15000;
const uint32_t GGA_PERIOD_MS    = 5000;
uint32_t bleRateLimiterMs = 0;

// ---------- 유틸 ----------
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
  if (millis() - bleRateLimiterMs < 60) return;
  bleRateLimiterMs = millis();

  String line = "RTCM ";
  const int maxBytes = min(n, 64);
  for (int i = 0; i < maxBytes; ++i) {
    char hex[4]; sprintf(hex, "%02X", data[i]);
    line += hex;
    if (i != maxBytes - 1) line += " ";
  }
  line += "\n";
  bleNotify(line);
}

// 간단 Base64 (권한 헤더용)
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

// ---------- Wi-Fi ----------
bool wifiConnectNow(uint32_t ms = 20000) {
  if (wcur.ssid.isEmpty()) {
    Serial.println("[WiFi] ssid empty");
    bleNotify("STATE wifi=disconnected\n");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  // erase=false 로 가볍게 끊었다가 다시 붙기
  WiFi.disconnect();   // 기본값: erase=false
  delay(150);

  Serial.printf("[WiFi] Connecting to '%s'...\n", wcur.ssid.c_str());
  WiFi.begin(wcur.ssid.c_str(), wcur.pass.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < ms) {
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

  // ✔ 여기 타입을 uint8_t(또는 int)로 받기
  uint8_t r = WiFi.waitForConnectResult(2000);
  Serial.printf("[WiFi] Failed. status=%d waitRes=%d\n", (int)WiFi.status(), (int)r);
  bleNotify("STATE wifi=disconnected\n");
  return false;
}



void wifiSaveSlot(uint8_t slot) {
  if (slot < 1 || slot > 5) return;
  prefs.begin("ntripble", false);
  char k[16];
  sprintf(k, "wf%d_ssid", slot); prefs.putString(k, wcur.ssid);
  sprintf(k, "wf%d_pass", slot); prefs.putString(k, wcur.pass);
  prefs.putUChar("wifi_last", slot);
  prefs.end();
  wifiLast = slot;
}
bool wifiLoadSlot(uint8_t slot) {
  if (slot < 1 || slot > 5) return false;
  prefs.begin("ntripble", true);
  char k[16];
  sprintf(k, "wf%d_ssid", slot); wcur.ssid = prefs.getString(k, "");
  sprintf(k, "wf%d_pass", slot); wcur.pass = prefs.getString(k, "");
  prefs.end();
  return !wcur.ssid.isEmpty();
}
void wifiLoadSettings() {
  prefs.begin("ntripble", true);
  wifiAuto = prefs.getBool("wifi_auto", false);
  wifiLast = prefs.getUChar("wifi_last", 0);
  prefs.end();
  if (wifiAuto && wifiLast >= 1 && wifiLast <= 5) {
    if (wifiLoadSlot(wifiLast)) {
      Serial.printf("[WiFi] Auto connect slot=%u (%s)\n", wifiLast, wcur.ssid.c_str());
      wifiConnectNow();
    }
  }
}

// ---------- NTRIP ----------
bool sendNtripRequest() {
  String req;
  req  = "GET /" + cfg.mount + " HTTP/1.1\r\n";
  req += "Host: " + cfg.host + "\r\n";
  req += "Ntrip-Version: Ntrip/2.0\r\n";
  req += "User-Agent: NTRIP ntripble/0.4\r\n";
  req += "Authorization: Basic " + makeAuthB64() + "\r\n";
  req += "Connection: keep-alive\r\n\r\n";

  Serial.printf("[NTRIP] Connecting to %s:%u\n", cfg.host.c_str(), cfg.port);
  if (!client.connect(cfg.host.c_str(), cfg.port)) {
    Serial.println("[NTRIP] TCP connect failed.");
    bleNotify("STATE ntrip=disconnected\n");
    return false;
  }

  client.print(req);
  Serial.println("[NTRIP] Request sent.");

  String header;
  uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    while (client.available()) {
      char c = client.read();
      header += c;
      if (header.endsWith("\r\n\r\n")) {
        Serial.println("[NTRIP] Header:");
        Serial.println(header);
        if (header.startsWith("HTTP/1.1 200") ||
            header.startsWith("HTTP/1.0 200") ||
            header.startsWith("ICY 200")) {
          Serial.println("[NTRIP] 200 OK. Start streaming");
          bleNotify("STATE ntrip=connected\n");
          lastAlive = millis();
          lastGGA   = 0;
          return true;
        } else {
          Serial.println("[NTRIP] Non-200. Close.");
          bleNotify("STATE ntrip=disconnected\n");
          client.stop();
          return false;
        }
      }
    }
    delay(1);
  }
  Serial.println("[NTRIP] Header timeout.");
  bleNotify("STATE ntrip=disconnected\n");
  client.stop();
  return false;
}

void pumpRtcmToSerial() {
  static uint8_t buf[512];
  int n = client.read(buf, sizeof(buf));
  if (n > 0) {
    // Serial HEX
    for (int i = 0; i < n; ++i) {
      if ((i % 32) == 0) Serial.println();
      char hex[4]; sprintf(hex, "%02X ", buf[i]); Serial.print(hex);
    }
    // BLE 샘플
    bleNotifyHexSample(buf, n);
    lastAlive = millis();
  }
}

// ---------- BLE 콜백 ----------
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
  // 구버전/신버전 모두 지원
  void onConnect(NimBLEServer* s) { onConnectedCommon("no-info"); }
  void onDisconnect(NimBLEServer* s) { onDisconnectedCommon("no-info"); }
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) { onConnectedCommon("ConnInfo"); }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info) { onDisconnectedCommon("ConnInfo"); }
};

void handleCommand(const String& cmdLine) {
  String s = cmdLine; String orig = s; s.trim();
  if (s.isEmpty()) return;

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String rest = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") { bleNotify("PONG\n"); return; }

  if (cmd == "LOGHEX") {
    String v = rest; v.trim(); v.toLowerCase();
    if (v == "on")  { logHexToBle = true;  bleNotify("INFO loghex=on\n"); }
    if (v == "off") { logHexToBle = false; bleNotify("INFO loghex=off\n"); }
    return;
  }

  if (cmd == "SET") {
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

  // ---- WiFi 명령 ----
  if (cmd == "WIFI") {
    rest.trim();
    int sp2 = rest.indexOf(' ');
    String sub = (sp2 < 0) ? rest : rest.substring(0, sp2);
    String arg = (sp2 < 0) ? "" : rest.substring(sp2 + 1);
    sub.toUpperCase();

    if (sub == "SET") {
      // WIFI SET ssid="..." pass="..."
      auto getQuoted = [](const String& src, const char* key)->String {
        String pat = String(key) + "=\"";
        int i = src.indexOf(pat);
        if (i < 0) return "";
        i += pat.length();
        int j = src.indexOf('"', i);
        if (j < 0) return "";
        return src.substring(i, j);
      };
      String ss = getQuoted(arg, "ssid");
      String pw = getQuoted(arg, "pass");
      wcur.ssid = ss;
      wcur.pass = pw;
      Serial.printf("[WiFi] SET ssid='%s'\n", wcur.ssid.c_str());
      bleNotify("INFO wifi_set=ok\n");
      return;
    }
    if (sub == "CONNECT") {
      if (wifiConnectNow()) {
        // ok
      }
      return;
    }
    if (sub == "SAVE") {
      int i = arg.indexOf("idx=");
      if (i < 0) { bleNotify("ERROR need idx=1..5\n"); return; }
      int slot = arg.substring(i + 4).toInt();
      if (slot < 1 || slot > 5) { bleNotify("ERROR idx_range_1_5\n"); return; }
      wifiSaveSlot(slot);
      bleNotify("INFO wifi_save=ok idx=" + String(slot) + "\n");
      return;
    }
    if (sub == "LOAD") {
      int i = arg.indexOf("idx=");
      if (i < 0) { bleNotify("ERROR need idx=1..5\n"); return; }
      int slot = arg.substring(i + 4).toInt();
      if (!wifiLoadSlot(slot)) { bleNotify("ERROR wifi_load_fail\n"); return; }
      bleNotify("INFO wifi_load=ok idx=" + String(slot) + "\n");
      return;
    }
    if (sub == "AUTO") {
      String v = arg; v.trim(); v.toLowerCase();
      wifiAuto = (v == "on");
      prefs.begin("ntripble", false);
      prefs.putBool("wifi_auto", wifiAuto);
      prefs.end();
      bleNotify(String("INFO wifi_auto=") + (wifiAuto ? "on\n" : "off\n"));
      return;
    }
    if (sub == "STATUS") {
      if (WiFi.status() == WL_CONNECTED) {
        bleNotify("STATE wifi=connected ip=" + WiFi.localIP().toString() + "\n");
      } else {
        bleNotify("STATE wifi=disconnected\n");
      }
      return;
    }
    bleNotify("ERROR wifi_unknown\n");
    return;
  }

  // ---- NTRIP 연결 제어 ----
  if (cmd == "CONNECT") {
    Serial.println("[CMD] CONNECT (NTRIP)");
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnectNow(); // 현재 프로필로 시도
      if (WiFi.status() != WL_CONNECTED) return;
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

  bleNotify("ERROR unknown_cmd\n");
}

class RxCB : public NimBLECharacteristicCallbacks {
  void handle(const char* tag, NimBLECharacteristic* ch) {
    std::string v = ch->getValue();
    String s = String(v.c_str());
    Serial.printf("[BLE RX] (%s) %s\n", tag, s.c_str());
    handleCommand(s);
  }
  // 두 시그니처 모두 지원
  void onWrite(NimBLECharacteristic* ch) { handle("no-info", ch); }
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo& /*info*/) { handle("ConnInfo", ch); }
};

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== ESP32-C3 NTRIP + BLE(NUS) =====");

  // 시간 동기 (GGA hhmmss 출력용)
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  // 저장된 WiFi 자동연결 시도
  wifiLoadSettings();
  WiFi.onEvent([](WiFiEvent_t e){
    Serial.printf("[WiFiEv] %d\n", e);
  });
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
}

// ---------- Loop ----------
void loop() {
  // BLE heartbeat (2s)
  static uint32_t lastBeat = 0;
  if (bleConnected && millis() - lastBeat > 2000) {
    bleNotify("HEARTBEAT\n");
    lastBeat = millis();
  }

  // NTRIP 스트림 유지
  if (client.connected()) {
    // 5s 마다 GGA 업링크
    if (millis() - lastGGA >= GGA_PERIOD_MS) {
      String gga = makeGGA();
      client.print(gga);
      Serial.print("\n[TX GGA] "); Serial.print(gga);
      bleNotify("TX GGA " + gga);
      lastGGA = millis();
    }
    // RTCM 수신
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