// ===== ESP32-C3 NTRIP + BLE(NUS) =====
// - BLE로 명령 수신/로그 알림
// - WiFi: 수동 입력/프로필 SAVE/LOAD(1..5), AUTO on/off, 부팅 시 자동 연결
// - NTRIP: 200 OK 받은 뒤부터만 GGA 주기 송신, RTCM 수신시 시리얼 HEX + BLE 샘플
// - 헤더 타임아웃 12초, WiFi 절전 off, TCP noDelay

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

// ---------- 기본값(초기 부팅/리셋용) ----------
const char* WIFI_SSID_DEFAULT = "";
const char* WIFI_PASS_DEFAULT = "";

struct NtripConfig {
  String host = "rts2.ngii.go.kr";
  uint16_t port = 2101;
  String mount = "VRS-RTCM31";
  String user  = "kongseal03";
  String pass  = "ngii";
  double lat   = 36.3520;
  double lon   = 128.6970;
  double altM  = 100.0;
  int    sats  = 12;
  float  hdop  = 1.0;
} cfg;

String authB64_default = "a29uZ3NlYWwwMzpuZ2lp"; // "kongseal03:ngii" base64

// ---------- 전역 ----------
WiFiClient client;
Preferences prefs;

bool bleConnected = false;
bool logHexToBle  = true;

bool wifiAuto = true;      // 부팅 시 자동 연결
String wifiSsid = WIFI_SSID_DEFAULT;
String wifiPass = WIFI_PASS_DEFAULT;

bool ntripOk = false;      // 200 OK 이후에만 GGA/RTCM 루프
uint32_t lastAlive = 0;
uint32_t lastGGA   = 0;

const uint32_t ALIVE_TIMEOUT_MS = 15000;
const uint32_t GGA_PERIOD_MS    = 5000;

// ---------- BLE NUS ----------
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write
#define NUS_CHARACTERISTIC_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTx     = nullptr;
NimBLECharacteristic* pRx     = nullptr;
NimBLEAdvertising*    pAdv    = nullptr;

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

// ---- Base64 (간단) ----
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
void wifiLoadBoot() {
  prefs.begin("ntripble", true);
  wifiAuto = prefs.getBool("auto", true);
  wifiSsid = prefs.getString("ssid", WIFI_SSID_DEFAULT);
  wifiPass = prefs.getString("pass", WIFI_PASS_DEFAULT);
  prefs.end();
}
void wifiSaveBoot() {
  prefs.begin("ntripble", false);
  prefs.putBool("auto", wifiAuto);
  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);
  prefs.end();
}
bool wifiConnectNow(uint32_t ms = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (wifiSsid.length() == 0) {
    Serial.println("[WiFi] SSID empty, skip connect.");
    bleNotify("STATE wifi=disconnected\n");
    return false;
  }
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  Serial.printf("[WiFi] Connecting to '%s'...", wifiSsid.c_str());
  bleNotify("STATE wifi=connecting ssid=\"" + wifiSsid + "\"\n");
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

// ---------- NTRIP ----------
bool sendNtripRequest() {
  ntripOk = false;
  if (client.connected()) client.stop();

  client.setNoDelay(true);
  client.setTimeout(12000);

  IPAddress ip;
  if (!WiFi.hostByName(cfg.host.c_str(), ip)) {
    Serial.println("[NTRIP] DNS resolve failed");
    return false;
  }

  Serial.printf("[NTRIP] Connecting to %s:%u\n", cfg.host.c_str(), cfg.port);
  if (!client.connect(ip, cfg.port)) {
    Serial.println("[NTRIP] TCP connect failed.");
    bleNotify("ERROR tcp_connect_failed\n");
    return false;
  }

  // 고전 포맷(상대 서버에 가장 호환 잘됨)
  String req;
  req  = "GET /" + cfg.mount + " HTTP/1.0\r\n";
  req += "User-Agent: NTRIP ntripble\r\n";
  req += "Accept: */*\r\n";
  req += "Authorization: Basic " + makeAuthB64() + "\r\n";
  req += "\r\n";

  client.print(req);
  Serial.println("[NTRIP] Request sent.");

  String header; header.reserve(512);
  uint32_t t0 = millis();
  while (millis() - t0 < 12000) {
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
          ntripOk = true;
          lastAlive = millis();
          lastGGA   = 0;     // 즉시 첫 GGA
          return true;
        } else {
          Serial.println("[NTRIP] Non-200, closing.");
          bleNotify("ERROR ntrip_non_200\n");
          client.stop();
          return false;
        }
      }
    }
    delay(1);
  }
  Serial.println("[NTRIP] Header timeout.");
  bleNotify("ERROR header_timeout\n");
  client.stop();
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

// ---------- 광고 재시작 도우미 ----------
void restartAdvertising() {
  if (!pAdv) return;
  // 안전하게 stop → 잠깐 대기 → start
  if (pAdv->isAdvertising()) {
    pAdv->stop();
    delay(120);
  }
  // (광고 데이터는 setup에서 이미 채워둠)
  pAdv->start();
  Serial.println("[BLE] Advertising restarted");
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
    // 연결이 끊기면 광고를 확실하게 재개
    restartAdvertising();
    // 상태 통지(참고: notify는 연결 상태에서만 유효하므로 로그만)
    // bleNotify("STATE ble=disconnected\n"); // 연결 없음 → notify 불가
  }
  // 두 시그니처 모두 지원(버전 호환)
  void onConnect(NimBLEServer* s) { onConnectedCommon("no-info"); }
  void onDisconnect(NimBLEServer* s) { onDisconnectedCommon("no-info"); }
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) { onConnectedCommon("ConnInfo"); }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info) { onDisconnectedCommon("ConnInfo"); }
};

void handleCommand(const String& cmdLine) {
  String s = cmdLine; String orig = s; s.trim();
  if (s.length() == 0) return;

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String rest = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") { bleNotify("PONG\n"); Serial.println("[BLE RX] PING"); return; }

  // ---- WIFI ----
  if (cmd == "WIFI") {
    int sp2 = rest.indexOf(' ');
    String sub = (sp2 < 0) ? rest : rest.substring(0, sp2);
    String args = (sp2 < 0) ? ""   : rest.substring(sp2 + 1);
    sub.toUpperCase();

    if (sub == "SET") {
      // WIFI SET ssid="..." pass="..."
      String sssid, spass;
      int a = args.indexOf("ssid=\"");
      if (a >= 0) { int b = args.indexOf("\"", a+6); if (b > a) sssid = args.substring(a+6, b); }
      a = args.indexOf("pass=\"");
      if (a >= 0) { int b = args.indexOf("\"", a+6); if (b > a) spass = args.substring(a+6, b); }
      if (sssid.length() > 0) wifiSsid = sssid;
      wifiPass = spass; // 빈문자열 허용(오픈 AP)
      wifiSaveBoot();
      bleNotify("INFO wifi_set ok\n");
      Serial.printf("[WIFI] SET ssid=\"%s\" pass=\"%s\"\n", wifiSsid.c_str(), wifiPass.c_str());
      return;
    }
    if (sub == "CONNECT") {
      wifiConnectNow();
      return;
    }

    // === 추가: 실제 Wi-Fi 끊기 처리 ===
    if (sub == "DISCONNECT") {
      Serial.println("[WIFI] DISCONNECT requested");
      // NTRIP 정리
      if (client.connected()) client.stop();
      ntripOk = false;

      // 실제 Wi-Fi 연결 끊기(무선 끄고 → STA로 복귀)
      WiFi.disconnect(true /*wifioff*/, false /*eraseAP*/);
      delay(120);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);

      bleNotify("STATE wifi=disconnected\n");
      return;
    }

    if (sub == "AUTO") {
      args.toLowerCase();
      wifiAuto = (args.indexOf("on") >= 0);
      wifiSaveBoot();
      bleNotify(String("INFO wifi_auto=") + (wifiAuto ? "on\n" : "off\n"));
      return;
    }
    if (sub == "SAVE") {
      // WIFI SAVE idx=1
      int i = args.indexOf("idx=");
      int slot = (i>=0)? args.substring(i+4).toInt(): 0;
      if (slot<1 || slot>5){ bleNotify("ERROR idx_range_1_5\n"); return; }
      prefs.begin("ntripble", false);
      char key[8];
      sprintf(key,"ws%d",slot); prefs.putString(key, wifiSsid);
      sprintf(key,"wp%d",slot); prefs.putString(key, wifiPass);
      prefs.end();
      bleNotify("INFO wifi_save ok\n");
      return;
    }
    if (sub == "LOAD") {
      int i = args.indexOf("idx=");
      int slot = (i>=0)? args.substring(i+4).toInt(): 0;
      if (slot<1 || slot>5){ bleNotify("ERROR idx_range_1_5\n"); return; }
      prefs.begin("ntripble", true);
      char key[8];
      sprintf(key,"ws%d",slot); wifiSsid = prefs.getString(key, wifiSsid);
      sprintf(key,"wp%d",slot); wifiPass = prefs.getString(key, wifiPass);
      prefs.end();
      bleNotify("INFO wifi_load ok\n");
      Serial.printf("[WIFI] LOAD -> ssid=\"%s\"\n", wifiSsid.c_str());
      return;
    }
    if (sub == "STATUS") {
      bleNotify(String("STATE wifi=") + (WiFi.status()==WL_CONNECTED?"connected ip=" + WiFi.localIP().toString():"disconnected") + "\n");
      return;
    }
    bleNotify("ERROR wifi_unknown\n");
    return;
  }

  // ---- NTRIP ----
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
  if (cmd == "CONNECT") {
    Serial.println("[CMD] CONNECT (NTRIP)");
    if (WiFi.status() != WL_CONNECTED) {
      if (!wifiConnectNow()) return;
    }
    sendNtripRequest();
    return;
  }
  if (cmd == "DISCONNECT") {
    Serial.println("[CMD] DISCONNECT (NTRIP)");
    if (client.connected()) client.stop();
    ntripOk = false;
    bleNotify("STATE ntrip=disconnected\n");
    return;
  }
  if (cmd == "LOGHEX") {
    String v = rest; v.trim(); v.toLowerCase();
    if (v == "on")  { logHexToBle = true;  bleNotify("INFO loghex=on\n"); }
    if (v == "off") { logHexToBle = false; bleNotify("INFO loghex=off\n"); }
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

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== ESP32-C3 NTRIP + BLE(NUS) =====");

  wifiLoadBoot();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // NTP (GGA 시각)
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

  pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  NimBLEAdvertisementData ad;  ad.setName("NTRIPBLE-C3"); pAdv->setAdvertisementData(ad);
  NimBLEAdvertisementData srd; srd.setName("NTRIPBLE-C3"); pAdv->setScanResponseData(srd);
  pAdv->start();

  Serial.println("[BLE] Advertising as NTRIPBLE-C3");

  if (wifiAuto) {
    wifiConnectNow();
  }
}

void loop() {
  // BLE heartbeat
  static uint32_t lastBeat = 0;
  if (bleConnected && millis() - lastBeat > 2000) {
    bleNotify("HEARTBEAT\n");
    lastBeat = millis();
  }

  // 광고 보조 워치독: 연결 안된 상태에서 광고가 꺼져있으면 재시작
  if (!bleConnected && pAdv && !pAdv->isAdvertising()) {
    restartAdvertising();
  }

  // WiFi 끊겼으면 정리
  if (WiFi.status() != WL_CONNECTED) {
    if (client.connected()) { client.stop(); ntripOk = false; }
  }

  // NTRIP 스트림
  if (client.connected() && ntripOk) {
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
        ntripOk = false;
      }
    }
  }

  delay(5);
}