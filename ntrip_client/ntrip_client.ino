// ===== ESP32-C3 NTRIP + BLE(NUS) =====
// - BLE로 명령 수신/로그 알림
// - WiFi: 수동 입력/프로필 SAVE/LOAD(1..5)
// - NTRIP: 200 OK 받은 뒤부터만 GGA 주기 송신, RTCM 수신시 시리얼 HEX + BLE 샘플
// - 헤더 타임아웃 12초, WiFi 절전 off, TCP noDelay
//
// [ADD] LoRa UART, UBX NAV-PVT 파서, BLE RTK 상태 전송
// [MOD] RTCM→LoRa 중계, 수동 연결만 유지(자동 연결 제거)
// [FIX] 앱 표시 동기화를 위해 연결/끊김 시 INFO wifi_ssid / INFO ntrip_mount 송신

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

// ---------- 기본값 ----------
const char* WIFI_SSID_DEFAULT = "";
const char* WIFI_PASS_DEFAULT = "";

struct NtripConfig {
  String host = "rts2.ngii.go.kr";
  uint16_t port = 2101;
  String mount = "VRS-RTCM31";
  String user  = "";
  String pass  = "";
  double lat   = 36.3520;
  double lon   = 128.6970;
  double altM  = 100.0;
  int    sats  = 12;
  float  hdop  = 1.0;
} cfg;

// ---------- 전역 ----------
WiFiClient client;
Preferences prefs;

bool bleConnected = false;
bool logHexToBle  = true;

bool      suppressAdv = false;
uint32_t  suppressUntilMs = 0;
uint16_t  lastConnHandle = 0;

String wifiSsid = WIFI_SSID_DEFAULT;
String wifiPass = WIFI_PASS_DEFAULT;

bool ntripOk = false;
uint32_t lastAlive = 0;
uint32_t lastGGA   = 0;

const uint32_t ALIVE_TIMEOUT_MS = 15000;
const uint32_t GGA_PERIOD_MS    = 5000;

// ---------- BLE NUS ----------
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTx     = nullptr;
NimBLECharacteristic* pRx     = nullptr;
NimBLEAdvertising*    pAdv    = nullptr;

// ---------- LoRa UART ----------
#define LORA_RX_PIN 4
#define LORA_TX_PIN 5
#define LORA_BAUD   57600
HardwareSerial LoRaUart(1);

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

// ---------- UBX NAV-PVT 파서 ----------
enum { U_S1, U_S2, U_CL, U_ID, U_L1, U_L2, U_PAY, U_CKA, U_CKB };
uint8_t  u_st = U_S1, u_cls=0, u_id=0, u_cka=0, u_ckb=0;
uint16_t u_len=0, u_idx=0;
uint8_t  u_buf[128];
uint8_t  u_fixType=0, u_flags=0, u_numSV=0;

void ubxReset(){ u_st=U_S1; }
void ubxHandleNavPvt() {
  u_fixType = u_buf[20];
  u_flags   = u_buf[21];
  u_numSV   = u_buf[23];

  int32_t lon = *(int32_t*)&u_buf[24];
  int32_t lat = *(int32_t*)&u_buf[28];
  int32_t hmsl= *(int32_t*)&u_buf[36];

  cfg.lat  = lat * 1e-7;
  cfg.lon  = lon * 1e-7;
  cfg.altM = hmsl * 0.001;
  cfg.sats = u_numSV;
}
void ubxFeed(uint8_t b) {
  switch (u_st) {
    case U_S1: if (b==0xB5) u_st=U_S2; break;
    case U_S2: if (b==0x62) u_st=U_CL; else u_st=U_S1; break;
    case U_CL: u_cls=b; u_cka=0; u_ckb=0; u_cka+=b; u_ckb+=u_cka; u_st=U_ID; break;
    case U_ID: u_id=b;  u_cka+=b; u_ckb+=u_cka; u_st=U_L1; break;
    case U_L1: u_len=b; u_cka+=b; u_ckb+=u_cka; u_st=U_L2; break;
    case U_L2: u_len|=(uint16_t)b<<8; u_cka+=b; u_ckb+=u_cka;
               if (u_len>sizeof(u_buf)) { ubxReset(); break; }
               u_idx=0; u_st=U_PAY; break;
    case U_PAY:
      u_buf[u_idx++]=b; u_cka+=b; u_ckb+=u_cka;
      if (u_idx>=u_len) u_st=U_CKA;
      break;
    case U_CKA: if (u_cka==b) u_st=U_CKB; else ubxReset(); break;
    case U_CKB:
      if (u_ckb==b) {
        if (u_cls==0x01 && u_id==0x07 && u_len>=92) ubxHandleNavPvt();
      }
      ubxReset();
      break;
  }
}
const char* rtkStateText() {
  uint8_t carrSoln = (u_flags >> 6) & 0x03; // 2=FIXED,1=FLOAT,0=NONE
  if (carrSoln==2) return "FIXED";
  if (carrSoln==1) return "FLOAT";
  return "SINGLE";
}

// ---------- Wi-Fi ----------
void wifiLoadBoot() {
  prefs.begin("ntripble", true);
  wifiSsid = prefs.getString("ssid", WIFI_SSID_DEFAULT);
  wifiPass = prefs.getString("pass", WIFI_PASS_DEFAULT);
  prefs.end();
}
void wifiSaveBoot() {
  prefs.begin("ntripble", false);
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
    bleNotify("INFO wifi_ssid=\"-\"\n");
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
    String ss = WiFi.SSID();
    if (ss.length() == 0) ss = wifiSsid;
    bleNotify("INFO wifi_ssid=\"" + ss + "\"\n");
    return true;
  }
  Serial.println("[WiFi] Failed.");
  bleNotify("STATE wifi=disconnected\n");
  bleNotify("INFO wifi_ssid=\"-\"\n");
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
    bleNotify("STATE ntrip=disconnected\n");
    bleNotify("INFO ntrip_mount=\"-\"\n");
    return false;
  }

  Serial.printf("[NTRIP] Connecting to %s:%u\n", cfg.host.c_str(), cfg.port);
  if (!client.connect(ip, cfg.port)) {
    Serial.println("[NTRIP] TCP connect failed.");
    bleNotify("STATE ntrip=disconnected\n");
    bleNotify("INFO ntrip_mount=\"-\"\n");
    return false;
  }

  // Basic Auth b64
  String authB64;
  {
    String up = cfg.user + String(":") + cfg.pass;
    const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint8_t* b = (const uint8_t*)up.c_str(); size_t L = up.length();
    for (size_t i = 0; i < L; i += 3) {
      uint32_t v = (uint32_t)b[i] << 16;
      if (i + 1 < L) v |= (uint32_t)b[i + 1] << 8;
      if (i + 2 < L) v |= (uint32_t)b[i + 2];
      authB64 += tbl[(v >> 18) & 0x3F];
      authB64 += tbl[(v >> 12) & 0x3F];
      authB64 += (i + 1 < L) ? tbl[(v >> 6) & 0x3F] : '=';
      authB64 += (i + 2 < L) ? tbl[v & 0x3F] : '=';
    }
  }

  String req;
  req  = "GET /" + cfg.mount + " HTTP/1.0\r\n";
  req += "User-Agent: NTRIP ntripble\r\n";
  req += "Accept: */*\r\n";
  req += "Authorization: Basic " + authB64 + "\r\n";
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
          bleNotify("INFO ntrip_mount=\"" + cfg.mount + "\"\n");
          ntripOk = true;
          lastAlive = millis();
          lastGGA   = 0;
          return true;
        } else {
          Serial.println("[NTRIP] Non-200, closing.");
          client.stop();
          bleNotify("STATE ntrip=disconnected\n");
          bleNotify("INFO ntrip_mount=\"-\"\n");
          return false;
        }
      }
    }
    delay(1);
  }
  Serial.println("[NTRIP] Header timeout.");
  client.stop();
  bleNotify("STATE ntrip=disconnected\n");
  bleNotify("INFO ntrip_mount=\"-\"\n");
  return false;
}

// ---------- RTCM → HEX 로그 + BLE 샘플 + LoRa 송신 ----------
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
    LoRaUart.write(buf, n);
  }
}

void restartAdvertising() {
  if (!pAdv) return;
  if (suppressAdv && millis() < suppressUntilMs) return;
  if (suppressAdv && millis() >= suppressUntilMs) suppressAdv = false;
  if (pAdv->isAdvertising()) { pAdv->stop(); delay(120); }
  pAdv->start();
  Serial.println("[BLE] Advertising restarted");
}

void broadcastCurrentState() {
  String w = (WiFi.status()==WL_CONNECTED)
             ? String("STATE wifi=connected ip=") + WiFi.localIP().toString() + "\n"
             : String("STATE wifi=disconnected\n");
  bleNotify(w);
  String n = String("STATE ntrip=") + ( (client.connected() && ntripOk) ? "connected\n" : "disconnected\n");
  bleNotify(n);
}

// ---------- BLE Callbacks ----------
class ServerCB : public NimBLEServerCallbacks {
  void onConnectedCommon(const char* tag) {
    bleConnected = true;
    Serial.printf("[BLE] onConnect(%s)\n", tag);
    bleNotify("STATE ble=connected\n");
    bleNotify("HELLO from ESP32-C3\n");
    broadcastCurrentState(); // 자동 연결 없음
  }
  void onDisconnectedCommon(const char* tag) {
    bleConnected = false;
    Serial.printf("[BLE] onDisconnect(%s)\n", tag);
    restartAdvertising();
  }
  void onConnect(NimBLEServer* s) { onConnectedCommon("no-info"); }
  void onDisconnect(NimBLEServer* s) { onDisconnectedCommon("no-info"); }
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) { onConnectedCommon("ConnInfo"); }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info) { onDisconnectedCommon("ConnInfo"); }
};

void handleCommand(const String& cmdLine) {
  String s = cmdLine; s.trim();
  if (s.length() == 0) return;

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String rest = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") { bleNotify("PONG\n"); Serial.println("[BLE RX] PING"); return; }

  if (cmd == "BLE") {
    String sub = rest; sub.trim(); sub.toUpperCase();
    if (sub == "BYE") {
      bleNotify("INFO bye\n");
      suppressAdv = true;
      suppressUntilMs = millis() + 12000;
      if (lastConnHandle != 0 && pServer) {
        Serial.println("[BLE] BYE: disconnect current central");
        pServer->disconnect(lastConnHandle);
      }
      return;
    }
    bleNotify("ERROR ble_unknown\n");
    return;
  }

  if (cmd == "STATUS") { broadcastCurrentState(); return; }

  // ---- WIFI ----
  if (cmd == "WIFI") {
    int sp2 = rest.indexOf(' ');
    String sub = (sp2 < 0) ? rest : rest.substring(0, sp2);
    String args = (sp2 < 0) ? ""   : rest.substring(sp2 + 1);
    sub.toUpperCase();

    if (sub == "SET") {
      String sssid, spass;
      int a = args.indexOf("ssid=\"");
      if (a >= 0) { int b = args.indexOf("\"", a+6); if (b > a) sssid = args.substring(a+6, b); }
      a = args.indexOf("pass=\"");
      if (a >= 0) { int b = args.indexOf("\"", a+6); if (b > a) spass = args.substring(a+6, b); }
      if (sssid.length() > 0) wifiSsid = sssid;
      wifiPass = spass;
      wifiSaveBoot();
      bleNotify("INFO wifi_set ok\n");
      Serial.printf("[WIFI] SET ssid=\"%s\"\n", wifiSsid.c_str());
      return;
    }
    if (sub == "CONNECT") { wifiConnectNow(); return; }
    if (sub == "DISCONNECT") {
      Serial.println("[WIFI] DISCONNECT requested");
      if (client.connected()) client.stop();
      ntripOk = false;
      WiFi.disconnect(true, false);
      delay(120);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      bleNotify("STATE wifi=disconnected\n");
      bleNotify("INFO wifi_ssid=\"-\"\n");
      return;
    }
    if (sub == "SAVE") {
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
    if (WiFi.status() != WL_CONNECTED) { bleNotify("STATE ntrip=disconnected\n"); bleNotify("INFO ntrip_mount=\"-\"\n"); return; }
    sendNtripRequest();
    return;
  }
  if (cmd == "DISCONNECT") {
    if (client.connected()) client.stop();
    ntripOk = false;
    bleNotify("STATE ntrip=disconnected\n");
    bleNotify("INFO ntrip_mount=\"-\"\n");
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

  configTime(0, 0, "pool.ntp.org", "time.google.com");

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

  LoRaUart.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
}

void loop() {
  static uint32_t lastBeat = 0;
  if (bleConnected && millis() - lastBeat > 2000) {
    bleNotify("HEARTBEAT\n");
    lastBeat = millis();
  }

  if (!bleConnected && pAdv) {
    if (suppressAdv && millis() >= suppressUntilMs) suppressAdv = false;
    if (!suppressAdv && !pAdv->isAdvertising()) {
      restartAdvertising();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (client.connected()) { client.stop(); ntripOk = false; }
  }

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
    } else if (millis() - lastAlive > ALIVE_TIMEOUT_MS) {
      Serial.println("\n[NTRIP] No data. Reconnect.");
      bleNotify("STATE ntrip=disconnected\n");
      bleNotify("INFO ntrip_mount=\"-\"\n");
      client.stop();
      ntripOk = false;
    }
  }

  while (LoRaUart.available()) {
    uint8_t b = (uint8_t)LoRaUart.read();
    ubxFeed(b);
  }

  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 1000) {
    lastStat = millis();
    char line[64];
    snprintf(line, sizeof(line), "STAT:{\"rtk\":\"%s\",\"sv\":%u}\n", rtkStateText(), (unsigned)u_numSV);
    bleNotify(String(line));
  }

  delay(5);
}