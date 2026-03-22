/*
 * WT32-ETH01 — CI-V Bus → Antenna Genius Bridge
 * ================================================
 * Hardware : WT32-ETH01 (ESP32 + LAN8720)
 * CI-V Bus : UART2  (GPIO 17 = TX2, GPIO 16 = RX2)
 *            Hinweis: am CI-V-Bus nur RX2 wird benötigt;
 *            TX2 wird für optionale Polling-Anfragen verwendet.
 *            CI-V ist Open-Collector → 1kΩ Pull-up auf 3.3 V empfohlen.
 *
 * Ethernet : DHCP, Discovery via UDP Broadcast Port 9007
 * AG API   : TCP Port 9007 (ASCII-Protokoll)
 *
 * Ablauf:
 *  1. Ethernet per DHCP initialisieren
 *  2. UDP-Broadcast auf Port 9007 lauschen → Antenna Genius finden
 *  3. TCP-Verbindung zum AG aufbauen
 *  4. Konfiguration auslesen: band list, antenna list
 *  5. CI-V-Frequenz empfangen → Band bestimmen → port set senden
 *
 * Bibliotheken (Board: esp32 by Espressif ≥ 2.x):
 *  - ETH.h       (im ESP32-Arduino-Core enthalten)
 *  - WiFiUdp.h   (im ESP32-Arduino-Core enthalten)
 *  - WiFiClient.h (im ESP32-Arduino-Core enthalten)
 *
 * Board-Einstellungen in der Arduino IDE:
 *  Board      : "ESP32 Dev Module"  (oder "WT32-ETH01" falls installiert)
 *  Flash Mode : QIO
 *  Flash Size : 4MB
 */

#include <ETH.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebServer.h>   // im ESP32-Arduino-Core enthalten

// ─── WT32-ETH01 Ethernet-Pins ───────────────────────────────────────────────
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN   // GPIO0  = RMII CLK Eingang
#define ETH_POWER_PIN   16                   // GPIO16 = LAN8720 Power (auf manchen Boards)
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1                    // PHY-Adresse
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

// ─── CI-V UART ──────────────────────────────────────────────────────────────
#define CIV_BAUD        19200    // Typisch 9600 oder 19200 – ggf. anpassen
#define CIV_RX_PIN      5       // GPIO5 = UART2 RX  (am WT32-ETH01 frei verfügbar)
#define CIV_TX_PIN      17       // GPIO17 = UART2 TX
// ACHTUNG: GPIO16 wird auch als ETH_POWER_PIN genutzt. Falls der LAN8720 keinen
// separaten Power-Pin braucht, ETH_POWER_PIN auf -1 setzen.
// Auf dem WT32-ETH01 v1.4 ist GPIO16 = UART2 RX und NICHT am LAN8720 angeschlossen.
// → ETH_POWER_PIN unten auf -1 setzen wenn nötig.

// ─── Antenna Genius ─────────────────────────────────────────────────────────
#define AG_PORT         9007
#define AG_KEEPALIVE_MS 5000     // alle 5 s einen Ping senden
#define MAX_BANDS       16
#define MAX_ANTENNAS    16
#define AG_RADIO_PORT   1        // Radio-Port A = 1

// ─── Konflikt-Ausgang ────────────────────────────────────────────────────────
// GPIO4 wird HIGH gesetzt wenn das neu gewählte Band einen Port ansteuern würde,
// der bereits von einem anderen Radio-Port belegt ist (txant != 0 am Ziel-Port).
// Quellen: AG-Status-Nachrichten (S0|port ...) + aktives port get Polling.
// Der Pin bleibt HIGH bis der Konflikt aufgelöst ist (Port wieder frei).
#define CONFLICT_PIN    4        // freier GPIO am WT32-ETH01

// ─── Web-Server ──────────────────────────────────────────────────────────────
#define WEB_PORT        80
#define LOG_BUF_LINES   80       // Anzahl gespeicherter Log-Zeilen für die Webseite

// ─── CI-V Konstanten ────────────────────────────────────────────────────────
#define CIV_PREAMBLE    0xFE
#define CIV_EOM         0xFD
#define CIV_BCAST       0x00     // Broadcast-Adresse
#define CIV_CONTROLLER  0xE0     // Unsere Controller-Adresse
#define CIV_CMD_FREQ    0x03     // Befehl: Frequenz lesen
#define CIV_BUF_SIZE    32

// ═══════════════════════════════════════════════════════════════════════════
//  Datenstrukturen
// ═══════════════════════════════════════════════════════════════════════════

struct BandEntry {
  uint8_t  id;
  char     name[16];
  double   freqStart;   // MHz
  double   freqStop;    // MHz
  bool     valid;
};

struct AntennaEntry {
  uint8_t  id;
  char     name[32];
  uint16_t txMask;
  uint16_t rxMask;
  bool     valid;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Globale Variablen
// ═══════════════════════════════════════════════════════════════════════════

// Ethernet
static bool ethConnected = false;

// UDP für Discovery
WiFiUDP udp;

// TCP zu Antenna Genius
WiFiClient agClient;
static IPAddress agIP;
static bool      agFound      = false;
static bool      agConnected  = false;
static bool      agConfigDone = false;

// AG Konfiguration
BandEntry    bands[MAX_BANDS];
uint8_t      bandCount = 0;
AntennaEntry antennas[MAX_ANTENNAS];
uint8_t      antennaCount = 0;

// Sequenznummer für AG-Kommandos (1–255, rollt über)
static uint8_t  agSeq = 1;

// Keepalive
static uint32_t lastPing = 0;

// CI-V Parser
static uint8_t  civBuf[CIV_BUF_SIZE];
static uint8_t  civLen = 0;
static uint8_t  civAddr = 0x00;   // wird beim ersten gültigen Frame gelernt

// Letztes aktives Band (AG-Band-ID)
static int8_t   lastBandId = -1;

// AG Antwort-Puffer
static String   agRxBuf;

// ─── Port-Status Tracking ────────────────────────────────────────────────────
// portTxAnt[i] = aktuell an Port (i+1) geschaltete TX-Antenne (0 = kein)
// Wird durch AG-Status-Nachrichten und port-get-Polling aktuell gehalten.
static uint8_t  portTxAnt[2]    = {0, 0};  // Index 0 = Port A, 1 = Port B
static bool     conflictActive  = false;
static uint32_t lastPortPoll    = 0;
#define PORT_POLL_MS  2000   // alle 2 s Port-Status aktiv abfragen

// ─── Web-Server & SSE ────────────────────────────────────────────────────────
WebServer webServer(WEB_PORT);

// Log-Ringpuffer (für Webseite)
static String   logLines[LOG_BUF_LINES];
static uint8_t  logHead  = 0;   // nächste Schreibposition
static uint16_t logTotal = 0;   // Gesamtanzahl geschriebener Zeilen

// SSE-Client (einfaches Single-Client-SSE; reicht für Monitoring)
static WiFiClient sseClient;
static bool       sseConnected = false;

// Live-Status-Felder (für SSE-Push und JSON-Endpunkt)
static char     civAddrStr[8]   = "---";
static char     civFreqStr[20]  = "---";
static char     agNameStr[48]   = "---";
static char     agIPStr[20]     = "---";
static char     agStatusStr[16] = "Suche...";
static double   civFreqMHz      = 0.0;

// ═══════════════════════════════════════════════════════════════════════════
//  Logging: Serial + Ringpuffer + SSE-Push
// ═══════════════════════════════════════════════════════════════════════════

// Interne Funktion: fertige Zeile einreihen und per SSE senden
static void _logAppend(const String &line) {
  logLines[logHead] = line;
  logHead  = (logHead + 1) % LOG_BUF_LINES;
  logTotal++;

  // SSE-Push falls Client verbunden
  if (sseConnected && sseClient.connected()) {
    sseClient.print("event: log\ndata: ");
    sseClient.print(line);
    sseClient.print("\n\n");
  }
}

// Öffentliche printf-kompatible Log-Funktion
void webLog(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  // Zeilenumbruch am Ende entfernen (SSE-Protokoll)
  int len = strlen(buf);
  while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) buf[--len] = '\0';

  Serial.println(buf);
  _logAppend(String(buf));
}

// SSE-Status-Push (schickt alle Live-Felder als JSON-Event)
void ssePushStatus() {
  if (!sseConnected || !sseClient.connected()) return;

  // Antennen-Status Port A und B als JSON zusammenbauen
  String antA = "---", antB = "---";
  uint8_t idxA = portTxAnt[0]; // 1-basiert, 0 = kein
  uint8_t idxB = portTxAnt[1];
  if (idxA > 0 && idxA <= MAX_ANTENNAS && antennas[idxA-1].valid)
    antA = String(antennas[idxA-1].name);
  else if (idxA > 0)
    antA = "Ant " + String(idxA);
  if (idxB > 0 && idxB <= MAX_ANTENNAS && antennas[idxB-1].valid)
    antB = String(antennas[idxB-1].name);
  else if (idxB > 0)
    antB = "Ant " + String(idxB);

  String activeBand = "---";
  if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid)
    activeBand = String(bands[lastBandId].name);

  String json = "{";
  json += "\"civAddr\":\"" + String(civAddrStr) + "\",";
  json += "\"civFreq\":\"" + String(civFreqStr) + "\",";
  json += "\"activeBand\":\"" + activeBand + "\",";
  json += "\"agName\":\"" + String(agNameStr) + "\",";
  json += "\"agIP\":\"" + String(agIPStr) + "\",";
  json += "\"agStatus\":\"" + String(agStatusStr) + "\",";
  json += "\"portA\":\"" + antA + "\",";
  json += "\"portB\":\"" + antB + "\",";
  json += "\"conflict\":" + String(conflictActive ? "true" : "false");
  json += "}";

  sseClient.print("event: status\ndata: ");
  sseClient.print(json);
  sseClient.print("\n\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Ethernet-Ereignis-Handler
// ═══════════════════════════════════════════════════════════════════════════

void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      webLog("[ETH] Gestartet");
      ETH.setHostname("wt32-civ-bridge");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      webLog("[ETH] Link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      webLog("[ETH] IP: %s", ETH.localIP().toString().c_str());
      ethConnected = true;
      webServer.begin();
      webLog("[WEB] Server gestartet auf http://%s", ETH.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      webLog("[ETH] Link down");
      ethConnected  = false;
      agConnected   = false;
      agConfigDone  = false;
      agFound       = false;
      agClient.stop();
      snprintf(agStatusStr, sizeof(agStatusStr), "Getrennt");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      webLog("[ETH] Gestoppt");
      ethConnected = false;
      break;
    default:
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Forward Declarations (Web-Handler werden erst spaeter im Code definiert)
// ═══════════════════════════════════════════════════════════════════════════

void handleWebRoot();
void handleApiJson();
void handleSSE();
void handleSSEClient();
void ssePushStatus();
void pollPortStatus();
void sendKeepalive();
void handleAGReceive();
void handleCIV();
void discoverAntennaGenius();
void connectToAG();
void readAGConfig();
void parsePortStatus(const String &line);
void parsePortGetResponse(const String &line);
void checkAndSignalConflict(uint8_t wantedAntId);
void setAGPort(uint8_t portId, uint8_t bandId, uint8_t antId);
void sendAGCommand(const char *cmd);
bool readAGResponse(uint8_t seq, uint32_t timeoutMs);
void parseBandList();
void parseAntennaList();
void processCIVFrame();
uint64_t decodeCIVFreq(const uint8_t *data);
int8_t freqToBandId(double freqMHz);
uint8_t selectAntenna(uint8_t bandId);

// ═══════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  webLog("\n=== WT32 CI-V → Antenna Genius Bridge ===");

  // CI-V UART initialisieren
  Serial2.begin(CIV_BAUD, SERIAL_8N1, CIV_RX_PIN, CIV_TX_PIN);
  webLog("[CIV] UART2 gestartet: %d Baud, RX=GPIO%d", CIV_BAUD, CIV_RX_PIN);

  // Ethernet initialisieren
  WiFi.onEvent(onEthEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN,
            ETH_TYPE, ETH_CLK_MODE);

  // Bänder initialisieren
  memset(bands,    0, sizeof(bands));
  memset(antennas, 0, sizeof(antennas));

  // Konflikt-Ausgang initialisieren
  pinMode(CONFLICT_PIN, OUTPUT);
  digitalWrite(CONFLICT_PIN, LOW);
  webLog("[CONF] Konflikt-Ausgang: GPIO%d", CONFLICT_PIN);

  // Web-Server Routen registrieren
  webServer.on("/",        HTTP_GET, handleWebRoot);
  webServer.on("/api",     HTTP_GET, handleApiJson);
  webServer.on("/events",  HTTP_GET, handleSSE);
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "Not found");
  });
  // Hinweis: webServer.begin() wird erst in onEthEvent(GOT_IP) aufgerufen
}

// ═══════════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Web-Server und SSE immer bedienen – unabhaengig vom AG/CIV-Status
  if (ethConnected) {
    webServer.handleClient();
    handleSSEClient();
  }

  if (!ethConnected) return;

  // 1. Antenna Genius suchen
  if (!agFound) {
    discoverAntennaGenius();
    return;
  }

  // 2. TCP-Verbindung aufbauen
  if (!agConnected) {
    connectToAG();
    return;
  }

  // 3. Konfiguration auslesen
  if (!agConfigDone) {
    readAGConfig();
    return;
  }

  // 4. Laufender Betrieb
  handleAGReceive();
  handleCIV();
  sendKeepalive();
  pollPortStatus();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Antenna Genius Discovery (UDP Broadcast, Port 9007)
// ═══════════════════════════════════════════════════════════════════════════

void discoverAntennaGenius() {
  static uint32_t lastTry = 0;
  static bool     udpOpen = false;

  if (!udpOpen) {
    udp.begin(AG_PORT);
    udpOpen = true;
    webLog("[DISC] Warte auf Antenna Genius Broadcast...");
    snprintf(agStatusStr, sizeof(agStatusStr), "Suche...");
  }

  int pktSize = udp.parsePacket();
  if (pktSize <= 0) return;

  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';

  // Prüfen ob das Paket mit "AG " beginnt
  if (strncmp(buf, "AG ", 3) != 0) return;

  webLog("[DISC] Broadcast empfangen: %s", buf);

  // IP aus dem Paket parsen (ip=x.x.x.x)
  char *ipStr = strstr(buf, "ip=");
  if (!ipStr) {
    agIP = udp.remoteIP();
  } else {
    ipStr += 3;
    uint8_t a, b, c, d;
    if (sscanf(ipStr, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d) == 4) {
      agIP = IPAddress(a, b, c, d);
    } else {
      agIP = udp.remoteIP();
    }
  }

  // Infos ausgeben + Live-Felder befüllen
  char *nameStr  = strstr(buf, "name=");
  char *portsStr = strstr(buf, "ports=");
  char *antStr   = strstr(buf, "antennas=");
  if (nameStr) {
    // name endet beim nächsten Leerzeichen
    char tmpName[48] = {0};
    sscanf(nameStr + 5, "%47s", tmpName);
    snprintf(agNameStr, sizeof(agNameStr), "%s", tmpName);
    webLog("[DISC] Name     : %s", agNameStr);
  }
  if (portsStr)  webLog("[DISC] Ports    : %c", portsStr[6]);
  if (antStr)    webLog("[DISC] Antennas : %c", antStr[9]);

  snprintf(agIPStr, sizeof(agIPStr), "%s", agIP.toString().c_str());
  snprintf(agStatusStr, sizeof(agStatusStr), "Verbinde...");
  webLog("[DISC] AG gefunden: %s", agIPStr);
  agFound = true;
  udp.stop();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TCP-Verbindung zum Antenna Genius aufbauen
// ═══════════════════════════════════════════════════════════════════════════

void connectToAG() {
  webLog("[AG] Verbinde mit %s:%d ...", agIP.toString().c_str(), AG_PORT);
  if (!agClient.connect(agIP, AG_PORT)) {
    webLog("[AG] Verbindung fehlgeschlagen, neuer Versuch in 3 s");
    snprintf(agStatusStr, sizeof(agStatusStr), "Fehler");
    // Non-blocking warten: handleClient() laeuft weiter
    uint32_t t = millis();
    while (millis() - t < 3000) {
      webServer.handleClient();
      handleSSEClient();
      delay(10);
    }
    return;
  }
  agClient.setNoDelay(true);
  agConnected = true;
  agRxBuf     = "";
  agSeq       = 1;
  lastPing    = millis();
  snprintf(agStatusStr, sizeof(agStatusStr), "Verbunden");
  webLog("[AG] Verbunden!");

  // Prologue lesen (V<a.b.c> AG[ AUTH])
  uint32_t t = millis();
  while (millis() - t < 2000) {
    while (agClient.available()) {
      char c = agClient.read();
      if (c == '\r' || c == '\n') {
        if (agRxBuf.length() > 0) {
          webLog("[AG] Prologue: %s", agRxBuf.c_str());
          agRxBuf = "";
        }
      } else {
        agRxBuf += c;
      }
    }
  }
  agRxBuf = "";
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG Konfiguration auslesen
// ═══════════════════════════════════════════════════════════════════════════

void readAGConfig() {
  webLog("[AG] Lese Konfiguration...");

  // --- band list ---
  sendAGCommand("band list");
  bandCount = 0;
  if (readAGResponse(agSeq - 1, 3000)) {
    parseBandList();
  }

  // --- antenna list ---
  sendAGCommand("antenna list");
  antennaCount = 0;
  if (readAGResponse(agSeq - 1, 3000)) {
    parseAntennaList();
  }

  // Ergebnis ausgeben
  webLog("[AG] %d Bänder geladen:", bandCount);
  for (uint8_t i = 0; i < MAX_BANDS; i++) {
    if (bands[i].valid && bands[i].freqStop > 0) {
      webLog("     Band %2d: %-12s %.3f - %.3f MHz",
             bands[i].id, bands[i].name,
             bands[i].freqStart, bands[i].freqStop);
    }
  }
  webLog("[AG] %d Antennen geladen:", antennaCount);
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (antennas[i].valid) {
      webLog("     Ant  %2d: %s (TX-Mask: %04X)",
             antennas[i].id, antennas[i].name, antennas[i].txMask);
    }
  }

  agConfigDone = true;
  snprintf(agStatusStr, sizeof(agStatusStr), "OK");
  webLog("[AG] Konfiguration abgeschlossen. Warte auf CI-V...");
  ssePushStatus();
}

// ─── AG-Kommando senden ────────────────────────────────────────────────────

void sendAGCommand(const char *cmd) {
  char buf[128];
  snprintf(buf, sizeof(buf), "C%d|%s\r", agSeq, cmd);
  agClient.print(buf);
  webLog("[AG] >> C%d|%s", agSeq, cmd);
  agSeq++;
  if (agSeq > 255) agSeq = 1;
}

// ─── AG-Antwort blockierend lesen (bis leere Zeile = Ende) ─────────────────

bool readAGResponse(uint8_t seq, uint32_t timeoutMs) {
  agRxBuf = "";
  uint32_t t = millis();
  String collected = "";

  while (millis() - t < timeoutMs) {
    while (agClient.available()) {
      char c = agClient.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (agRxBuf.length() == 0) {
          agRxBuf = collected;
          return true;
        }
        collected += agRxBuf + "\n";
        webLog("[AG] << %s", agRxBuf.c_str());
        agRxBuf = "";
      } else {
        agRxBuf += c;
      }
    }
    delay(1);
  }
  agRxBuf = collected;
  return false;
}

// ─── Band-Liste parsen ─────────────────────────────────────────────────────
// Format: R9|0|band 1 name=160m freq_start=1.600000 freq_stop=2.200000

void parseBandList() {
  String s = agRxBuf;
  int pos = 0;
  bandCount = 0;

  while (pos < (int)s.length()) {
    int nl = s.indexOf('\n', pos);
    String line = (nl < 0) ? s.substring(pos) : s.substring(pos, nl);
    pos = (nl < 0) ? s.length() : nl + 1;

    // Zeile enthält "band <id> name=..."
    int bandPos = line.indexOf("|band ");
    if (bandPos < 0) continue;
    String rest = line.substring(bandPos + 1); // "band 1 name=..."

    uint8_t id = 0;
    char    name[16] = {0};
    double  fs = 0, fe = 0;

    // ID
    sscanf(rest.c_str(), "band %hhu", &id);
    if (id >= MAX_BANDS) continue;

    // name=
    char *np = strstr(rest.c_str(), "name=");
    if (np) sscanf(np, "name=%15s", name);

    // freq_start=
    char *fsp = strstr(rest.c_str(), "freq_start=");
    if (fsp) sscanf(fsp, "freq_start=%lf", &fs);

    // freq_stop=
    char *fep = strstr(rest.c_str(), "freq_stop=");
    if (fep) sscanf(fep, "freq_stop=%lf", &fe);

    bands[id].id        = id;
    strncpy(bands[id].name, name, 15);
    bands[id].freqStart = fs;
    bands[id].freqStop  = fe;
    bands[id].valid     = true;
    if (fe > 0) bandCount++;
  }
}

// ─── Antennen-Liste parsen ─────────────────────────────────────────────────
// Format: R5|0|antenna 1 name=Antenna_1 tx=0000 rx=0001 inband=0000

void parseAntennaList() {
  String s = agRxBuf;
  int pos = 0;
  antennaCount = 0;

  while (pos < (int)s.length()) {
    int nl = s.indexOf('\n', pos);
    String line = (nl < 0) ? s.substring(pos) : s.substring(pos, nl);
    pos = (nl < 0) ? s.length() : nl + 1;

    int antPos = line.indexOf("|antenna ");
    if (antPos < 0) continue;
    String rest = line.substring(antPos + 1);

    uint8_t id = 0;
    char    name[32] = {0};
    uint16_t tx = 0, rx = 0;

    sscanf(rest.c_str(), "antenna %hhu", &id);
    if (id == 0 || id > MAX_ANTENNAS) continue;

    char *np = strstr(rest.c_str(), "name=");
    if (np) sscanf(np, "name=%31s", name);

    char *txp = strstr(rest.c_str(), "tx=");
    if (txp) sscanf(txp, "tx=%hx", &tx);

    char *rxp = strstr(rest.c_str(), "rx=");
    if (rxp) sscanf(rxp, "rx=%hx", &rx);

    uint8_t idx = id - 1;
    if (idx < MAX_ANTENNAS) {
      antennas[idx].id     = id;
      strncpy(antennas[idx].name, name, 31);
      antennas[idx].txMask = tx;
      antennas[idx].rxMask = rx;
      antennas[idx].valid  = true;
      antennaCount++;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG laufende Kommunikation (Status-Nachrichten empfangen)
// ═══════════════════════════════════════════════════════════════════════════

void handleAGReceive() {
  if (!agClient.connected()) {
    webLog("[AG] Verbindung getrennt - reconnect...");
    agClient.stop();
    agConnected  = false;
    agConfigDone = false;
    snprintf(agStatusStr, sizeof(agStatusStr), "Getrennt");
    ssePushStatus();
    return;
  }

  while (agClient.available()) {
    char c = agClient.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (agRxBuf.length() > 0) {
        if (agRxBuf.startsWith("S")) {
          webLog("[AG] Status: %s", agRxBuf.c_str());
          parsePortStatus(agRxBuf);
          ssePushStatus();
        } else if (agRxBuf.startsWith("R")) {
          parsePortGetResponse(agRxBuf);
        }
        agRxBuf = "";
      }
    } else {
      agRxBuf += c;
    }
  }
}

// ─── Keepalive (ping) ──────────────────────────────────────────────────────

void sendKeepalive() {
  if (millis() - lastPing < AG_KEEPALIVE_MS) return;
  lastPing = millis();
  sendAGCommand("ping");
  // Antwort nicht blockierend abwarten – wird in handleAGReceive() verarbeitet
}

// ═══════════════════════════════════════════════════════════════════════════
//  CI-V Empfang und Verarbeitung
// ═══════════════════════════════════════════════════════════════════════════

void handleCIV() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (civLen == 0 && b != CIV_PREAMBLE) return; // auf Preamble warten
    if (civLen < CIV_BUF_SIZE) {
      civBuf[civLen++] = b;
    }

    // Auf EOM warten
    if (b == CIV_EOM) {
      processCIVFrame();
      civLen = 0;
    }
  }
}

// ─── CI-V Frame verarbeiten ────────────────────────────────────────────────
// Typisches Frequenz-Broadcast-Frame vom Transceiver:
// FE FE E0 <src> 00 <f4> <f3> <f2> <f1> <f0> FD
// oder ohne Controller-Adresse:
// FE FE 00 <src> 03 <f4> <f3> <f2> <f1> <f0> FD  (Polling-Antwort)

void processCIVFrame() {
  if (civLen < 7) return;
  if (civBuf[0] != CIV_PREAMBLE || civBuf[1] != CIV_PREAMBLE) return;
  if (civBuf[civLen - 1] != CIV_EOM) return;

  uint8_t dst = civBuf[2];
  uint8_t src = civBuf[3];
  uint8_t cmd = civBuf[4];

  if (civAddr == 0x00 && src != CIV_CONTROLLER && src != CIV_BCAST) {
    civAddr = src;
    snprintf(civAddrStr, sizeof(civAddrStr), "0x%02X", civAddr);
    webLog("[CIV] Transceiver-Adresse gelernt: %s", civAddrStr);
    ssePushStatus();
  }

  if (cmd != 0x00 && cmd != 0x03) return;
  if (dst != CIV_CONTROLLER && dst != CIV_BCAST && dst != 0x00) return;
  if (civLen < 11) return;

  uint64_t freqHz = decodeCIVFreq(&civBuf[5]);
  double   freqMHz = freqHz / 1e6;
  if (freqMHz < 0.1) return;

  // Frequenz-String aktualisieren
  snprintf(civFreqStr, sizeof(civFreqStr), "%.6f MHz", freqMHz);
  civFreqMHz = freqMHz;

  int8_t bandId = freqToBandId(freqMHz);

  if (bandId < 0) {
    if (lastBandId != bandId) {
      webLog("[CIV] %.6f MHz - kein passendes Band", freqMHz);
      lastBandId = bandId;
      ssePushStatus();
    }
    return;
  }

  // Frequenz-SSE immer pushen (auch ohne Bandwechsel, damit Freq live bleibt)
  ssePushStatus();

  if (bandId == lastBandId) return;

  lastBandId = bandId;
  webLog("[CIV] %.6f MHz -> Band %d (%s)", freqMHz, bandId, bands[bandId].name);

  uint8_t antId = selectAntenna(bandId);
  if (antId == 0) {
    webLog("[CIV] Keine Antenne fuer Band %d konfiguriert", bandId);
    return;
  }

  webLog("[AG] Schalte Port %d: Band=%d Ant=%d (%s)",
         AG_RADIO_PORT, bandId, antId, antennas[antId - 1].name);

  checkAndSignalConflict(antId);
  setAGPort(AG_RADIO_PORT, bandId, antId);
  ssePushStatus();
}

// ─── 5 BCD-Bytes → Hz dekodieren ──────────────────────────────────────────

uint64_t decodeCIVFreq(const uint8_t *data) {
  // data[0] = 10Hz / 1Hz Stelle (BCD)
  // data[4] = GHz-Stelle (BCD)
  uint64_t freq = 0;
  uint64_t mult = 1;
  for (int i = 0; i < 5; i++) {
    freq += (data[i] & 0x0F) * mult;        mult *= 10;
    freq += ((data[i] >> 4) & 0x0F) * mult; mult *= 10;
  }
  return freq; // in Hz
}

// ─── Frequenz → AG-Band-ID ────────────────────────────────────────────────

int8_t freqToBandId(double freqMHz) {
  for (uint8_t i = 1; i < MAX_BANDS; i++) {
    if (!bands[i].valid) continue;
    if (bands[i].freqStop <= 0) continue;
    if (freqMHz >= bands[i].freqStart && freqMHz <= bands[i].freqStop) {
      return (int8_t)i;
    }
  }
  return -1;
}

// ─── Beste Antenne für Band wählen ────────────────────────────────────────
// Wählt die erste Antenne, deren TX-Maske das Band einschließt

uint8_t selectAntenna(uint8_t bandId) {
  uint16_t bandBit = (1u << bandId);
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (!antennas[i].valid) continue;
    if (antennas[i].txMask & bandBit) {
      return antennas[i].id;
    }
  }
  // Fallback: erste gültige Antenne
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (antennas[i].valid) return antennas[i].id;
  }
  return 0;
}

// ─── AG Port setzen ────────────────────────────────────────────────────────
// C22|port set 1 auto=0 source=MANUAL band=3 rxant=1 txant=1

void setAGPort(uint8_t portId, uint8_t bandId, uint8_t antId) {
  if (!agConnected) return;
  char cmd[64];
  snprintf(cmd, sizeof(cmd),
           "port set %d auto=0 source=MANUAL band=%d rxant=%d txant=%d",
           portId, bandId, antId, antId);
  sendAGCommand(cmd);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Web-Handler
// ═══════════════════════════════════════════════════════════════════════════

// ─── GET /events  — SSE-Verbindung aufbauen ────────────────────────────────

void handleSSE() {
  // Alten Client trennen falls noch verbunden
  if (sseConnected) sseClient.stop();

  sseClient    = webServer.client();
  sseConnected = true;

  // HTTP-Header für SSE
  sseClient.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/event-stream\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: keep-alive\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n"
  );

  // Sofort aktuellen Status + kompletten Log senden
  uint16_t count = (logTotal < LOG_BUF_LINES) ? logTotal : LOG_BUF_LINES;
  uint8_t  start = (logTotal < LOG_BUF_LINES) ? 0 : logHead;
  for (uint16_t i = 0; i < count; i++) {
    uint8_t idx = (start + i) % LOG_BUF_LINES;
    sseClient.print("event: log\ndata: ");
    sseClient.print(logLines[idx]);
    sseClient.print("\n\n");
  }
  ssePushStatus();
  webLog("[WEB] SSE-Client verbunden");
}

// ─── SSE-Client keepalive (Kommentar-Ping alle 15 s) ─────────────────────

void handleSSEClient() {
  static uint32_t lastSsePing = 0;
  if (!sseConnected) return;
  if (!sseClient.connected()) {
    sseConnected = false;
    return;
  }
  if (millis() - lastSsePing > 15000) {
    lastSsePing = millis();
    sseClient.print(": ping\n\n");
  }
}

// ─── GET /api  — JSON-Snapshot (kein SSE, einmaliger Abruf) ───────────────

void handleApiJson() {
  String antA = "---", antB = "---";
  uint8_t idxA = portTxAnt[0];
  uint8_t idxB = portTxAnt[1];
  if (idxA > 0 && idxA <= MAX_ANTENNAS && antennas[idxA-1].valid)
    antA = String(antennas[idxA-1].name);
  else if (idxA > 0) antA = "Ant " + String(idxA);
  if (idxB > 0 && idxB <= MAX_ANTENNAS && antennas[idxB-1].valid)
    antB = String(antennas[idxB-1].name);
  else if (idxB > 0) antB = "Ant " + String(idxB);

  String activeBand = "---";
  if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid)
    activeBand = String(bands[lastBandId].name);

  String json = "{";
  json += "\"civAddr\":\"" + String(civAddrStr) + "\",";
  json += "\"civFreq\":\"" + String(civFreqStr) + "\",";
  json += "\"activeBand\":\"" + activeBand + "\",";
  json += "\"agName\":\"" + String(agNameStr) + "\",";
  json += "\"agIP\":\"" + String(agIPStr) + "\",";
  json += "\"agStatus\":\"" + String(agStatusStr) + "\",";
  json += "\"portA\":\"" + antA + "\",";
  json += "\"portB\":\"" + antB + "\",";
  json += "\"conflict\":" + String(conflictActive ? "true" : "false") + ",";

  // Bänder-Array
  json += "\"bands\":[";
  bool first = true;
  for (uint8_t i = 0; i < MAX_BANDS; i++) {
    if (!bands[i].valid || bands[i].freqStop <= 0) continue;
    if (!first) json += ",";
    first = false;
    json += "{\"id\":" + String(bands[i].id) + ",";
    json += "\"name\":\"" + String(bands[i].name) + "\",";
    json += "\"start\":" + String(bands[i].freqStart, 3) + ",";
    json += "\"stop\":"  + String(bands[i].freqStop,  3) + "}";
  }
  json += "],";

  // Antennen-Array
  json += "\"antennas\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (!antennas[i].valid) continue;
    if (!first) json += ",";
    first = false;
    json += "{\"id\":" + String(antennas[i].id) + ",";
    json += "\"name\":\"" + String(antennas[i].name) + "\",";
    json += "\"txMask\":" + String(antennas[i].txMask) + "}";
  }
  json += "]}";

  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", json);
}

// ─── GET /  — Haupt-HTML-Seite ─────────────────────────────────────────────

void handleWebRoot() {
  // Die Seite nutzt SSE (/events) für automatische Aktualisierung.
  // Status-Event: JSON mit Live-Daten
  // Log-Event   : einzelne Log-Zeile (wird in Textarea geschrieben)

  String html = R"rawhtml(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WT32 CI-V Bridge</title>
<style>
  :root {
    --bg: #0f1117; --card: #1a1d27; --border: #2a2d3e;
    --accent: #4f8ef7; --green: #3ecf8e; --red: #f87171;
    --yellow: #fbbf24; --text: #e2e8f0; --muted: #94a3b8;
    --mono: 'Courier New', monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: system-ui, sans-serif;
         font-size: 14px; padding: 16px; }
  h1 { font-size: 1.2rem; color: var(--accent); margin-bottom: 16px;
       border-bottom: 1px solid var(--border); padding-bottom: 8px; }
  h2 { font-size: 0.85rem; text-transform: uppercase; letter-spacing: .08em;
       color: var(--muted); margin-bottom: 10px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
          gap: 12px; margin-bottom: 12px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 8px;
          padding: 14px; }
  .kv { display: flex; justify-content: space-between; align-items: center;
        padding: 5px 0; border-bottom: 1px solid var(--border); }
  .kv:last-child { border-bottom: none; }
  .kv .label { color: var(--muted); }
  .kv .val   { font-weight: 600; font-family: var(--mono); font-size: 0.9rem; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 99px;
           font-size: 0.75rem; font-weight: 700; }
  .badge-green  { background: #14532d; color: var(--green); }
  .badge-red    { background: #450a0a; color: var(--red); }
  .badge-yellow { background: #451a03; color: var(--yellow); }
  .badge-blue   { background: #1e3a5f; color: var(--accent); }
  .badge-muted  { background: #1e2235; color: var(--muted); }
  #conflict-banner { display: none; background: #450a0a; border: 1px solid var(--red);
                     color: var(--red); border-radius: 8px; padding: 10px 14px;
                     margin-bottom: 12px; font-weight: 700; font-size: 0.95rem; }
  table { width: 100%; border-collapse: collapse; font-size: 0.82rem; }
  th { color: var(--muted); text-align: left; padding: 4px 6px;
       border-bottom: 1px solid var(--border); }
  td { padding: 4px 6px; border-bottom: 1px solid var(--border); }
  tr:last-child td { border-bottom: none; }
  td.active { color: var(--green); font-weight: 700; }
  #log { width: 100%; height: 260px; background: #0a0c13; color: #7dd3fc;
         border: 1px solid var(--border); border-radius: 6px; padding: 8px;
         font-family: var(--mono); font-size: 0.75rem; resize: vertical;
         overflow-y: scroll; white-space: pre-wrap; word-break: break-all; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
         margin-right: 6px; }
  .dot-green  { background: var(--green); box-shadow: 0 0 6px var(--green); }
  .dot-red    { background: var(--red);   box-shadow: 0 0 6px var(--red); }
  .dot-muted  { background: var(--muted); }
  footer { margin-top: 12px; color: var(--muted); font-size: 0.75rem; text-align: center; }
</style>
</head>
<body>
<h1>&#128225; WT32-ETH01 &mdash; CI-V &rarr; Antenna Genius Bridge</h1>

<div id="conflict-banner">&#9888; ANTENNEN-KONFLIKT: Ziel-Antenne bereits von anderem Port belegt! GPIO4 = HIGH</div>

<div class="grid">
  <!-- Funkgerät -->
  <div class="card">
    <h2>Funkger&auml;t (CI-V)</h2>
    <div class="kv"><span class="label">CI-V Adresse</span>
      <span class="val" id="civAddr">---</span></div>
    <div class="kv"><span class="label">Frequenz</span>
      <span class="val" id="civFreq">---</span></div>
    <div class="kv"><span class="label">Aktives Band</span>
      <span class="val" id="activeBand"><span class="badge badge-muted">---</span></span></div>
  </div>

  <!-- Antenna Genius -->
  <div class="card">
    <h2>Antenna Genius</h2>
    <div class="kv"><span class="label">Name</span>
      <span class="val" id="agName">---</span></div>
    <div class="kv"><span class="label">IP-Adresse</span>
      <span class="val" id="agIP">---</span></div>
    <div class="kv"><span class="label">Status</span>
      <span id="agStatusWrap"><span class="dot dot-muted"></span>
      <span id="agStatus">---</span></span></div>
  </div>

  <!-- Port-Belegung -->
  <div class="card">
    <h2>Port-Belegung</h2>
    <div class="kv"><span class="label">Port A (Radio 1)</span>
      <span class="val" id="portA">---</span></div>
    <div class="kv"><span class="label">Port B (Radio 2)</span>
      <span class="val" id="portB">---</span></div>
    <div class="kv"><span class="label">Konflikt</span>
      <span id="conflictBadge"><span class="badge badge-muted">---</span></span></div>
  </div>
</div>

<!-- Bänder & Antennen -->
<div class="grid">
  <div class="card">
    <h2>Band-Konfiguration (AG)</h2>
    <table><thead><tr><th>#</th><th>Name</th><th>Start</th><th>Stop</th></tr></thead>
    <tbody id="bandTable"><tr><td colspan="4" style="color:var(--muted)">Lade...</td></tr></tbody>
    </table>
  </div>
  <div class="card">
    <h2>Antennen-Konfiguration (AG)</h2>
    <table><thead><tr><th>#</th><th>Name</th><th>TX-Maske</th></tr></thead>
    <tbody id="antTable"><tr><td colspan="3" style="color:var(--muted)">Lade...</td></tr></tbody>
    </table>
  </div>
</div>

<!-- Debug-Log -->
<div class="card">
  <h2>Debug-Log <span id="sseState" style="font-size:0.7rem;margin-left:8px"></span></h2>
  <div id="log"></div>
</div>

<footer>WT32-ETH01 &bull; Live via SSE &bull; Aktualisierung automatisch</footer>

<script>
const MAX_LOG = 300;
let logLines = [];
let activeBandName = '';

function esc(s){
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

function appendLog(line){
  logLines.push(line);
  if(logLines.length > MAX_LOG) logLines.shift();
  const el = document.getElementById('log');
  el.textContent = logLines.join('\n');
  el.scrollTop = el.scrollHeight;
}

function badgeForStatus(s){
  if(s==='OK'||s==='Verbunden') return `<span class="badge badge-green">${esc(s)}</span>`;
  if(s==='Getrennt'||s==='Fehler') return `<span class="badge badge-red">${esc(s)}</span>`;
  if(s==='Suche...'||s==='Verbinde...') return `<span class="badge badge-yellow">${esc(s)}</span>`;
  return `<span class="badge badge-muted">${esc(s)}</span>`;
}

function dotForStatus(s){
  if(s==='OK'||s==='Verbunden') return 'dot-green';
  if(s==='Getrennt'||s==='Fehler') return 'dot-red';
  return 'dot-muted';
}

function applyStatus(d){
  document.getElementById('civAddr').textContent    = d.civAddr;
  document.getElementById('civFreq').textContent    = d.civFreq;

  activeBandName = d.activeBand;
  const ab = document.getElementById('activeBand');
  if(d.activeBand !== '---'){
    ab.innerHTML = `<span class="badge badge-blue">${esc(d.activeBand)}</span>`;
  } else {
    ab.innerHTML = `<span class="badge badge-muted">---</span>`;
  }

  document.getElementById('agName').textContent = d.agName;
  document.getElementById('agIP').textContent   = d.agIP;

  const sw = document.getElementById('agStatusWrap');
  sw.innerHTML = `<span class="dot ${dotForStatus(d.agStatus)}"></span>`
               + `<span id="agStatus">${esc(d.agStatus)}</span>`;

  document.getElementById('portA').innerHTML =
    d.portA==='---' ? `<span class="badge badge-muted">---</span>`
                    : `<span class="badge badge-blue">${esc(d.portA)}</span>`;
  document.getElementById('portB').innerHTML =
    d.portB==='---' ? `<span class="badge badge-muted">---</span>`
                    : `<span class="badge badge-blue">${esc(d.portB)}</span>`;

  const cb = document.getElementById('conflictBadge');
  const banner = document.getElementById('conflict-banner');
  if(d.conflict){
    cb.innerHTML = `<span class="badge badge-red">&#9888; KONFLIKT</span>`;
    banner.style.display = 'block';
  } else {
    cb.innerHTML = `<span class="badge badge-green">OK</span>`;
    banner.style.display = 'none';
  }
}

function loadConfig(){
  fetch('/api').then(r=>r.json()).then(d=>{
    // Bänder-Tabelle
    const bt = document.getElementById('bandTable');
    if(d.bands && d.bands.length){
      bt.innerHTML = d.bands.map(b=>{
        const active = (b.name === activeBandName);
        return `<tr><td class="${active?'active':''}">${b.id}</td>`
             + `<td class="${active?'active':''}">${esc(b.name)}</td>`
             + `<td>${b.start.toFixed(3)}</td><td>${b.stop.toFixed(3)}</td></tr>`;
      }).join('');
    } else {
      bt.innerHTML = '<tr><td colspan="4" style="color:var(--muted)">Keine Daten</td></tr>';
    }
    // Antennen-Tabelle
    const at = document.getElementById('antTable');
    if(d.antennas && d.antennas.length){
      at.innerHTML = d.antennas.map(a=>{
        return `<tr><td>${a.id}</td><td>${esc(a.name)}</td>`
             + `<td style="font-family:var(--mono)">${a.txMask.toString(16).padStart(4,'0').toUpperCase()}</td></tr>`;
      }).join('');
    } else {
      at.innerHTML = '<tr><td colspan="3" style="color:var(--muted)">Keine Daten</td></tr>';
    }
    applyStatus(d);
  }).catch(()=>{});
}

// SSE verbinden
function connectSSE(){
  const state = document.getElementById('sseState');
  const es = new EventSource('/events');

  es.addEventListener('status', e=>{
    try {
      const d = JSON.parse(e.data);
      applyStatus(d);
      // Band-Tabelle Highlighting aktualisieren
      document.querySelectorAll('#bandTable tr').forEach(tr=>{
        const cell = tr.cells[1];
        if(!cell) return;
        const active = (cell.textContent === activeBandName);
        tr.cells[0].className = active ? 'active' : '';
        tr.cells[1].className = active ? 'active' : '';
      });
    } catch(ex){}
  });

  es.addEventListener('log', e=>{
    appendLog(e.data);
  });

  es.onopen = ()=>{
    state.innerHTML = '<span class="badge badge-green">Live</span>';
  };

  es.onerror = ()=>{
    state.innerHTML = '<span class="badge badge-red">Unterbrochen &ndash; reconnect...</span>';
    // Browser reconnected automatisch
  };
}

// Initiales Laden + SSE starten
loadConfig();
connectSSE();
// Konfigurationstabellen alle 30 s neu laden (AG-Konfig kann sich ändern)
setInterval(loadConfig, 30000);
</script>
</body>
</html>)rawhtml";

  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "text/html; charset=utf-8", html);
}
// ═══════════════════════════════════════════════════════════════════════════
// Format: S0|port 1 auto=1 source=AUTO band=3 rxant=1 txant=1 tx=0 inhibit=0

void parsePortStatus(const String &line) {
  int pp = line.indexOf("|port ");
  if (pp < 0) return;
  String rest = line.substring(pp + 1);

  uint8_t portId = 0;
  sscanf(rest.c_str(), "port %hhu", &portId);
  if (portId < 1 || portId > 2) return;

  uint8_t txant = 0;
  char *tp = strstr(rest.c_str(), "txant=");
  if (tp) sscanf(tp, "txant=%hhu", &txant);

  uint8_t idx = portId - 1;
  if (portTxAnt[idx] != txant) {
    webLog("[CONF] Port %d txant geaendert: %d -> %d", portId, portTxAnt[idx], txant);
    portTxAnt[idx] = txant;
  }
}

// ─── port get Antwort parsen ──────────────────────────────────────────────
// Format: R15|0|port 1 auto=1 source=AUTO band=0 rxant=0 txant=0 tx=0 inhibit=0

void parsePortGetResponse(const String &line) {
  // Nur Antworten die "port <id>" im Message-Teil enthalten
  int pp = line.indexOf("|port ");
  if (pp < 0) return;

  // Sicherstellen dass es eine gültige R-Antwort ist (R<seq>|0|port ...)
  // Fehler-Antworten (|<non-zero>|) ignorieren
  int p1 = line.indexOf('|');
  int p2 = (p1 >= 0) ? line.indexOf('|', p1 + 1) : -1;
  if (p1 < 0 || p2 < 0) return;
  String hexResp = line.substring(p1 + 1, p2);
  hexResp.trim();
  if (hexResp != "0") return;

  parsePortStatus("S0" + line.substring(p2)); // gleiche Logik wiederverwenden
}

// ═══════════════════════════════════════════════════════════════════════════
//  Aktives Port-Status-Polling (Fallback / Ergänzung zu Status-Nachrichten)
// ═══════════════════════════════════════════════════════════════════════════

void pollPortStatus() {
  if (!agConnected || !agConfigDone) return;
  if (millis() - lastPortPoll < PORT_POLL_MS) return;
  lastPortPoll = millis();

  // Port A und Port B abfragen
  sendAGCommand("port get 1");
  sendAGCommand("port get 2");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Konflikt erkennen und GPIO setzen
// ═══════════════════════════════════════════════════════════════════════════
// Logik:
//  - Wir sind Radio-Port AG_RADIO_PORT (= Port A).
//  - Der andere Radio-Port ist Port B (Index 1).
//  - Ein Konflikt liegt vor, wenn die Antenne, die wir schalten wollen,
//    bereits als txant am anderen Port eingetragen ist.

void checkAndSignalConflict(uint8_t wantedAntId) {
  uint8_t otherIdx   = (AG_RADIO_PORT == 1) ? 1 : 0;
  uint8_t otherTxAnt = portTxAnt[otherIdx];

  bool conflict = (wantedAntId != 0) && (otherTxAnt == wantedAntId);

  if (conflict != conflictActive) {
    conflictActive = conflict;
    digitalWrite(CONFLICT_PIN, conflict ? HIGH : LOW);

    if (conflict) {
      webLog("[CONF] *** KONFLIKT: Antenne %d wird von Port %d belegt! GPIO%d = HIGH ***",
             wantedAntId, (AG_RADIO_PORT == 1) ? 2 : 1, CONFLICT_PIN);
    } else {
      webLog("[CONF] Konflikt aufgeloest. GPIO%d = LOW", CONFLICT_PIN);
    }
    ssePushStatus();
  }
}
