// ESP32 (M5Stack StamPLC / ESP32-S3) ANCS test: pair an iPhone like a watch and print every iOS notification
// (app, title, subtitle, message) as one JSON line on USB serial. Step 1 of "iPhone navigation -> comma" (11/05):
// see what Apple Maps / Google Maps actually send while navigating. Nothing is sent anywhere else.
//
// Pair: iPhone Settings > Bluetooth > "comma-nav" (Just Works). Then allow "Share System Notifications" when asked.
// Build:  arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc esp32_ancs
#include <NimBLEDevice.h>

static const char* ANCS_SVC = "7905F431-B5CE-4E99-A40F-4B1E122D00D0";
static const char* NS_CHR   = "9FBF120D-6301-42D9-8C58-25E699A21DBD";   // Notification Source (notify)
static const char* CP_CHR   = "69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9";   // Control Point (write)
static const char* DS_CHR   = "22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB";   // Data Source (notify)

static NimBLEServer* server = nullptr;
static volatile uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool authed = false, ready = false;
static NimBLERemoteCharacteristic* cp = nullptr;

// pending UIDs to fetch (ring)
static uint32_t q[32]; static volatile uint8_t qh = 0, qt = 0;
static uint8_t evcat[256];  // category of recent UIDs (by low byte), for printing
static std::vector<uint8_t> ds;  // Data Source reassembly

static void jsonStr(const char* k, const uint8_t* p, uint16_t n, bool comma) {
  Serial.printf("\"%s\":\"", k);
  for (uint16_t i = 0; i < n; i++) {
    uint8_t c = p[i];
    if (c == '"' || c == '\\') { Serial.write('\\'); Serial.write(c); }
    else if (c == '\n') Serial.print("\\n");
    else if (c >= 0x20 || c >= 0x80) Serial.write(c);
  }
  Serial.print(comma ? "\"," : "\"");
}

// Data Source: CommandID(0) UID(4) then (attrId 1, len 2 LE, data len) x 4 (AppId, Title, Subtitle, Message)
static void parseDs() {
  while (ds.size() >= 5) {
    size_t p = 5; int got = 0;
    while (got < 4) {
      if (p + 3 > ds.size()) return;            // wait for more fragments
      uint16_t len = ds[p + 1] | (ds[p + 2] << 8);
      if (p + 3 + len > ds.size()) return;
      p += 3 + len; got++;
    }
    uint32_t uid = ds[1] | (ds[2] << 8) | (ds[3] << 16) | ((uint32_t)ds[4] << 24);
    Serial.printf("{\"t\":%lu,\"uid\":%lu,\"cat\":%u,", (unsigned long)millis(), (unsigned long)uid, evcat[uid & 0xFF]);
    size_t r = 5; const char* names[4] = {"app", "title", "subtitle", "msg"};
    for (int i = 0; i < 4; i++) {
      uint16_t len = ds[r + 1] | (ds[r + 2] << 8);
      jsonStr(names[i], &ds[r + 3], len, i < 3);
      r += 3 + len;
    }
    Serial.println("}");
    ds.erase(ds.begin(), ds.begin() + p);
  }
}

static void onNs(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 8) return;
  uint8_t ev = d[0], cat = d[2];
  uint32_t uid = d[4] | (d[5] << 8) | (d[6] << 16) | ((uint32_t)d[7] << 24);
  evcat[uid & 0xFF] = cat;
  if (ev == 2) { Serial.printf("{\"t\":%lu,\"uid\":%lu,\"removed\":1}\n", (unsigned long)millis(), (unsigned long)uid); return; }
  if (d[1] & 0x04) return;                       // pre-existing notification at connect: skip the backlog
  uint8_t nt = (qt + 1) & 31;
  if (nt != qh) { q[qt] = uid; qt = nt; }
}

static void onDs(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  ds.insert(ds.end(), d, d + n);
  parseDs();
}

class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    conn = ci.getConnHandle(); authed = false; ready = false;
    Serial.println("{\"status\":\"connected\"}");
    NimBLEDevice::startSecurity(conn);
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) override {
    conn = BLE_HS_CONN_HANDLE_NONE; authed = false; ready = false; cp = nullptr; ds.clear();
    Serial.printf("{\"status\":\"disconnected\",\"reason\":%d}\n", reason);
    NimBLEDevice::startAdvertising();
  }
  void onAuthenticationComplete(NimBLEConnInfo& ci) override {
    authed = ci.isEncrypted();
    Serial.printf("{\"status\":\"auth\",\"encrypted\":%d,\"bonded\":%d}\n", ci.isEncrypted(), ci.isBonded());
  }
};

void setup() {
  Serial.begin(115200);
  delay(1500);
  NimBLEDevice::init("comma-nav");
  NimBLEDevice::setSecurityAuth(true, false, true);              // bonding, no MITM, secure connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  server = NimBLEDevice::createServer();
  server->setCallbacks(new SrvCb());
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  // AD type 0x15: 128-bit service solicitation = ANCS (little-endian), so iOS offers notification sharing
  NimBLEUUID u(ANCS_SVC);
  const uint8_t* b = u.getValue();
  std::vector<uint8_t> sol = {17, 0x15};
  for (int i = 0; i < 16; i++) sol.push_back(b[i]);
  ad.addData(sol);
  adv->setAdvertisementData(ad);
  NimBLEAdvertisementData sr; sr.setName("comma-nav");
  adv->setScanResponseData(sr);
  adv->start();
  Serial.println("{\"status\":\"advertising\",\"name\":\"comma-nav\"}");
}

static bool setupAncs() {
  NimBLEClient* c = server->getClient(conn);
  if (!c) return false;
  NimBLERemoteService* svc = c->getService(ANCS_SVC);
  if (!svc) { Serial.println("{\"status\":\"no ANCS service yet\"}"); return false; }
  NimBLERemoteCharacteristic* ns = svc->getCharacteristic(NS_CHR);
  NimBLERemoteCharacteristic* dsc = svc->getCharacteristic(DS_CHR);
  cp = svc->getCharacteristic(CP_CHR);
  if (!ns || !dsc || !cp) { Serial.println("{\"status\":\"ANCS chars missing\"}"); return false; }
  bool a = dsc->subscribe(true, onDs);
  bool bb = ns->subscribe(true, onNs);
  Serial.printf("{\"status\":\"ancs\",\"ds\":%d,\"ns\":%d}\n", a, bb);
  return a && bb;
}

void loop() {
  static uint32_t lastTry = 0;
  if (conn != BLE_HS_CONN_HANDLE_NONE && authed && !ready && millis() - lastTry > 2000) {
    lastTry = millis();
    ready = setupAncs();
  }
  if (ready && cp && qh != qt) {
    uint32_t uid = q[qh]; qh = (qh + 1) & 31;
    uint8_t cmd[] = {0, (uint8_t)uid, (uint8_t)(uid >> 8), (uint8_t)(uid >> 16), (uint8_t)(uid >> 24),
                     0,                      // AppIdentifier
                     1, 0x80, 0x00,          // Title, max 128
                     2, 0x80, 0x00,          // Subtitle, max 128
                     3, 0x00, 0x01};         // Message, max 256
    cp->writeValue(cmd, sizeof(cmd), true);
  }
  delay(20);
}
