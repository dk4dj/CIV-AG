/*
 * WT32-ETH01 — CI-V Bus → Antenna Genius Bridge
 * ================================================
 * Hardware : WT32-ETH01 (ESP32 + LAN8720)
 * CI-V Bus : UART2  (GPIO 17 = TX2, GPIO 5 = RX2)
 *            CI-V ist Open-Collector → 1kΩ Pull-up auf 3.3 V empfohlen.
 *
 * Ethernet : DHCP, Discovery via UDP Broadcast Port 9007
 * AG API   : TCP Port 9007 (ASCII-Protokoll)
 *
 * Ablauf:
 *  1. Ethernet per DHCP initialisieren
 *  2. UDP-Broadcast auf Port 9007 lauschen → Antenna Genius finden
 *  3. TCP-Verbindung zum AG aufbauen, Prologue lesen
 *  4. sub port + sub antenna → Status-Nachrichten aktivieren
 *  5. band list + antenna list → Konfiguration einlesen
 *  6. port get 1 + port get 2 → initialen Port-Status abfragen
 *  7. CI-V-Frequenz empfangen (Broadcast) oder aktiv pollen (Cmd 0x03)
 *     → Band bestimmen → port set senden
 *
 * AG-Protokoll-Hinweise:
 *  - Antwortblock endet mit einer Zeile "R<seq>|0|" (leere Message)
 *  - Ohne "sub port" / "sub antenna" sendet AG KEINE Status-Nachrichten
 *  - Status-Format: S0|<message>
 *  - Frequenzformat im CI-V: 5 BCD-Bytes, little-endian (1 Hz Auflösung)
 *
 * Bibliotheken (Board: esp32 by Espressif ≥ 2.x):
 *  - ETH.h, WiFiUdp.h, WiFiClient.h, WebServer.h (alle im ESP32-Core)
 *
 * Board-Einstellungen in der Arduino IDE:
 *  Board      : "ESP32 Dev Module"
 *  Flash Mode : QIO / Flash Size : 4MB
 */

#include <ETH.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebServer.h>

// ─── WT32-ETH01 Ethernet-Pins ───────────────────────────────────────────────
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN   16
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

// ─── CI-V UART ──────────────────────────────────────────────────────────────
#define CIV_BAUD        19200
#define CIV_RX_PIN      5
#define CIV_TX_PIN      17
#define CIV_POLL_MS     500     // Frequenz aktiv abfragen wenn kein Broadcast
#define CIV_BROADCAST_TIMEOUT_MS 3000  // nach 3 s ohne Broadcast → aktiv pollen

// ─── Antenna Genius ─────────────────────────────────────────────────────────
#define AG_PORT         9007
#define AG_KEEPALIVE_MS 5000
#define AG_RECONNECT_MS 5000
#define AG_CFG_TIMEOUT_MS 5000  // max. Wartezeit pro Config-Schritt
#define MAX_BANDS       16
#define MAX_ANTENNAS    16
#define AG_RADIO_PORT   1       // Radio-Port A = 1

// ─── Konflikt-Ausgang ────────────────────────────────────────────────────────
#define CONFLICT_PIN    4

// ─── Web-Server ──────────────────────────────────────────────────────────────
#define WEB_PORT        80
#define LOG_BUF_LINES   100

// ─── CI-V Konstanten ────────────────────────────────────────────────────────
#define CIV_PREAMBLE    0xFE
#define CIV_EOM         0xFD
#define CIV_CONTROLLER  0xE0
#define CIV_BUF_SIZE    32

// ═══════════════════════════════════════════════════════════════════════════
//  Datenstrukturen
// ═══════════════════════════════════════════════════════════════════════════

struct BandEntry {
  uint8_t  id;
  char     name[16];
  double   freqStart;
  double   freqStop;
  bool     valid;
};

struct AntennaEntry {
  uint8_t  id;
  char     name[32];
  uint16_t txMask;
  uint16_t rxMask;
  bool     valid;
};

// AG-Konfigurations-Einlese-Zustand (nicht-blockierend)
enum AgConfigState {
  AGCFG_IDLE,
  AGCFG_WAIT_SUB_PORT,      // warte auf Antwort von "sub port all"
  AGCFG_WAIT_BAND_LIST,     // warte auf Antwort von "band list"
  AGCFG_WAIT_ANTENNA_LIST,  // warte auf Antwort von "antenna list"
  AGCFG_WAIT_PORT1,         // warte auf Antwort von "port get 1"
  AGCFG_WAIT_PORT2,         // warte auf Antwort von "port get 2"
  AGCFG_DONE
};

// ═══════════════════════════════════════════════════════════════════════════
//  Globale Variablen
// ═══════════════════════════════════════════════════════════════════════════

static bool ethConnected = false;

WiFiUDP    udp;
WiFiClient agClient;

static IPAddress agIP;
static bool      agFound      = false;
static bool      agConnected  = false;
static bool      agConfigDone = false;
static uint32_t  agLastConnect = 0;

// AG Konfiguration
BandEntry    bands[MAX_BANDS];
uint8_t      bandCount    = 0;
AntennaEntry antennas[MAX_ANTENNAS];
uint8_t      antennaCount = 0;

// AG nicht-blockierender Config-State
static AgConfigState agCfgState   = AGCFG_IDLE;
static uint8_t       agCfgSeq     = 0;
static uint32_t      agCfgTimeout = 0;  // Timeout-Zeitstempel pro Config-Schritt

// Sequenznummer für AG-Kommandos (1–255)
static uint8_t  agSeq    = 1;
static uint32_t lastPing = 0;

// AG Zeilen-Empfangspuffer (nicht-blockierend)
static String   agLineBuf;

// Gesammelte Antwortzeilen für laufendes Konfigurations-Kommando
static String   agRespLines;

// Port-Status
static uint8_t  portTxAnt[2]   = {0, 0};
static uint8_t  portRxAnt[2]   = {0, 0};
static uint8_t  portBand[2]    = {0, 0};
static bool     portTx[2]      = {false, false};
static bool     conflictActive = false;
static uint32_t lastPortPoll   = 0;
#define PORT_POLL_MS 3000

// CI-V
static uint8_t  civBuf[CIV_BUF_SIZE];
static uint8_t  civLen        = 0;
static uint8_t  civAddr       = 0x00;
static int8_t   lastBandId    = -1;
static uint32_t lastCivRx     = 0;   // Zeitstempel letzter CI-V Frequenzempfang
static uint32_t lastCivPoll   = 0;   // Zeitstempel letztes aktives Polling
static double   civFreqMHz    = 0.0;

// Web-Server & SSE
WebServer webServer(WEB_PORT);
static String   logLines[LOG_BUF_LINES];
static uint8_t  logHead  = 0;
static uint16_t logTotal = 0;
static WiFiClient sseClient;
static bool       sseConnected = false;

// Live-Status-Felder
static char civAddrStr[8]    = "---";
static char civFreqStr[20]   = "---";
static char agNameStr[48]    = "---";
static char agSerialStr[24]  = "---";
static char agFwStr[16]      = "---";
static char agIPStr[20]      = "---";
static char agStatusStr[20]  = "Suche...";
// Discovery-Felder
static uint16_t agTcpPort    = 0;
static uint8_t  agPorts      = 0;    // Anzahl Radio-Ports
static uint8_t  agAntennas   = 0;    // Anzahl Antennen-Ports
static char     agModeStr[12]= "---";
static uint32_t agUptime     = 0;    // Sekunden seit letztem Boot

// ═══════════════════════════════════════════════════════════════════════════
//  Forward Declarations
// ═══════════════════════════════════════════════════════════════════════════

void webLog(const char *fmt, ...);
void ssePushStatus();
void handleWebRoot();
void handleApiJson();
void handleSSE();
void handleSSEClient();
void discoverAntennaGenius();
void connectToAG();
void agStartConfig();
void agProcessLine(const String &line);
void agHandleConfigResponse(const String &line);
void agHandleStatusMessage(const String &line);
void sendAGCommand(const char *cmd);
void pollPortStatus();
void handleAGReceive();
void sendKeepalive();
void handleCIV();
void processCIVFrame();
void requestCIVFreq();
uint64_t decodeCIVFreq(const uint8_t *data);
int8_t freqToBandId(double freqMHz);
uint8_t selectAntenna(uint8_t bandId);
void setAGPort(uint8_t portId, uint8_t bandId, uint8_t antId);
void checkAndSignalConflict(uint8_t wantedAntId);
void parseBandListLine(const String &line);
void parseAntennaListLine(const String &line);
void parsePortLine(const String &line);

// ═══════════════════════════════════════════════════════════════════════════
//  Logging
// ═══════════════════════════════════════════════════════════════════════════

void webLog(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  int len = strlen(buf);
  while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) buf[--len] = '\0';
  Serial.println(buf);
  logLines[logHead] = String(buf);
  logHead = (logHead + 1) % LOG_BUF_LINES;
  logTotal++;
  if (sseConnected && sseClient.connected()) {
    sseClient.print("event: log\ndata: ");
    sseClient.print(buf);
    sseClient.print("\n\n");
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SSE Status-Push
// ═══════════════════════════════════════════════════════════════════════════

void ssePushStatus() {
  if (!sseConnected || !sseClient.connected()) return;

  String antA = "---", antB = "---";
  uint8_t ia = portTxAnt[0], ib = portTxAnt[1];
  if (ia > 0 && ia <= MAX_ANTENNAS && antennas[ia-1].valid) antA = antennas[ia-1].name;
  else if (ia > 0) antA = "Ant " + String(ia);
  if (ib > 0 && ib <= MAX_ANTENNAS && antennas[ib-1].valid) antB = antennas[ib-1].name;
  else if (ib > 0) antB = "Ant " + String(ib);

  String activeBand = "---";
  if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid)
    activeBand = bands[lastBandId].name;

  String bandA = "---", bandB = "---";
  if (portBand[0] > 0 && portBand[0] < MAX_BANDS && bands[portBand[0]].valid)
    bandA = bands[portBand[0]].name;
  if (portBand[1] > 0 && portBand[1] < MAX_BANDS && bands[portBand[1]].valid)
    bandB = bands[portBand[1]].name;

  String json = "{";
  json += "\"civAddr\":\"" + String(civAddrStr) + "\",";
  json += "\"civFreq\":\"" + String(civFreqStr) + "\",";
  json += "\"activeBand\":\"" + activeBand + "\",";
  json += "\"agName\":\"" + String(agNameStr) + "\",";
  json += "\"agSerial\":\"" + String(agSerialStr) + "\",";
  json += "\"agFw\":\"" + String(agFwStr) + "\",";
  json += "\"agIP\":\"" + String(agIPStr) + "\",";
  json += "\"agPort\":" + String(agTcpPort) + ",";
  json += "\"agPorts\":" + String(agPorts) + ",";
  json += "\"agAntennas\":" + String(agAntennas) + ",";
  json += "\"agMode\":\"" + String(agModeStr) + "\",";
  json += "\"agUptime\":" + String(agUptime) + ",";
  json += "\"agStatus\":\"" + String(agStatusStr) + "\",";
  json += "\"portA\":\"" + antA + "\",";
  json += "\"portB\":\"" + antB + "\",";
  json += "\"bandA\":\"" + bandA + "\",";
  json += "\"bandB\":\"" + bandB + "\",";
  json += "\"txA\":" + String(portTx[0] ? "true" : "false") + ",";
  json += "\"txB\":" + String(portTx[1] ? "true" : "false") + ",";
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
      ethConnected = false;
      agConnected  = false;
      agConfigDone = false;
      agFound      = false;
      agCfgState   = AGCFG_IDLE;
      agClient.stop();
      snprintf(agStatusStr, sizeof(agStatusStr), "Getrennt");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      webLog("[ETH] Gestoppt");
      ethConnected = false;
      break;
    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  webLog("=== WT32 CI-V -> Antenna Genius Bridge ===");

  Serial2.begin(CIV_BAUD, SERIAL_8N1, CIV_RX_PIN, CIV_TX_PIN);
  webLog("[CIV] UART2 gestartet: %d Baud, RX=GPIO%d TX=GPIO%d",
         CIV_BAUD, CIV_RX_PIN, CIV_TX_PIN);

  WiFi.onEvent(onEthEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN,
            ETH_TYPE, ETH_CLK_MODE);

  memset(bands,    0, sizeof(bands));
  memset(antennas, 0, sizeof(antennas));

  pinMode(CONFLICT_PIN, OUTPUT);
  digitalWrite(CONFLICT_PIN, LOW);
  webLog("[CONF] Konflikt-Ausgang: GPIO%d", CONFLICT_PIN);

  webServer.on("/",       HTTP_GET, handleWebRoot);
  webServer.on("/api",    HTTP_GET, handleApiJson);
  webServer.on("/events", HTTP_GET, handleSSE);
  webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });
}

// ═══════════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  if (ethConnected) {
    webServer.handleClient();
    handleSSEClient();
  }
  if (!ethConnected) return;

  if (!agFound)     { discoverAntennaGenius(); return; }
  if (!agConnected) { connectToAG();           return; }

  handleAGReceive();   // nicht-blockierend AG-Daten lesen & verarbeiten

  if (!agConfigDone) return;

  handleCIV();
  sendKeepalive();
  pollPortStatus();
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG Discovery (UDP Broadcast)
// ═══════════════════════════════════════════════════════════════════════════

void discoverAntennaGenius() {
  static bool udpOpen = false;
  if (!udpOpen) {
    udp.begin(AG_PORT);
    udpOpen = true;
    snprintf(agStatusStr, sizeof(agStatusStr), "Suche...");
    webLog("[DISC] Warte auf AG Broadcast...");
  }

  int pktSize = udp.parsePacket();
  if (pktSize <= 0) return;

  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';
  if (strncmp(buf, "AG ", 3) != 0) return;

  webLog("[DISC] Broadcast: %s", buf);

  // ip=
  char *ipStr = strstr(buf, "ip=");
  if (ipStr) {
    uint8_t a, b, c, d;
    if (sscanf(ipStr + 3, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d) == 4)
      agIP = IPAddress(a, b, c, d);
    else
      agIP = udp.remoteIP();
  } else {
    agIP = udp.remoteIP();
  }
  snprintf(agIPStr, sizeof(agIPStr), "%s", agIP.toString().c_str());

  // port=
  char *pp = strstr(buf, " port=");
  if (pp) sscanf(pp + 6, "%hu", &agTcpPort);

  // v= (Firmware — im Broadcast-Paket, wird ggf. durch Prologue überschrieben)
  char *vp = strstr(buf, " v=");
  if (vp) { char tmp[16]={0}; sscanf(vp + 3, "%15s", tmp); snprintf(agFwStr,  sizeof(agFwStr),  "%s", tmp); }

  // serial=
  char *sp = strstr(buf, "serial=");
  if (sp) { char tmp[24]={0}; sscanf(sp + 7, "%23s", tmp); snprintf(agSerialStr, sizeof(agSerialStr), "%s", tmp); }

  // name=
  char *np = strstr(buf, "name=");
  if (np) { char tmp[48]={0}; sscanf(np + 5, "%47s", tmp); snprintf(agNameStr, sizeof(agNameStr), "%s", tmp); }

  // ports=
  char *prp = strstr(buf, "ports=");
  if (prp) sscanf(prp + 6, "%hhu", &agPorts);

  // antennas=
  char *anp = strstr(buf, "antennas=");
  if (anp) sscanf(anp + 9, "%hhu", &agAntennas);

  // mode=
  char *mp = strstr(buf, "mode=");
  if (mp) { char tmp[12]={0}; sscanf(mp + 5, "%11s", tmp); snprintf(agModeStr, sizeof(agModeStr), "%s", tmp); }

  // uptime=
  char *up = strstr(buf, "uptime=");
  if (up) sscanf(up + 7, "%lu", &agUptime);

  webLog("[DISC] AG gefunden: %s  Name: %s  S/N: %s  FW: %s",
         agIPStr, agNameStr, agSerialStr, agFwStr);
  webLog("[DISC] Port: %d  Radio-Ports: %d  Antennen: %d  Mode: %s  Uptime: %lus",
         agTcpPort, agPorts, agAntennas, agModeStr, agUptime);
  agFound = true;
  udp.stop();
  ssePushStatus();
}

// ═══════════════════════════════════════════════════════════════════════════
//  TCP-Verbindung zum AG aufbauen
// ═══════════════════════════════════════════════════════════════════════════

void connectToAG() {
  if (millis() - agLastConnect < AG_RECONNECT_MS) return;
  agLastConnect = millis();

  webLog("[AG] Verbinde mit %s:%d ...", agIPStr, AG_PORT);
  snprintf(agStatusStr, sizeof(agStatusStr), "Verbinde...");

  if (!agClient.connect(agIP, AG_PORT)) {
    webLog("[AG] Verbindung fehlgeschlagen");
    snprintf(agStatusStr, sizeof(agStatusStr), "Fehler");
    return;
  }

  agClient.setNoDelay(true);
  agConnected  = false;   // erst nach Prologue true setzen
  agConfigDone = false;
  agCfgState   = AGCFG_IDLE;
  agLineBuf    = "";
  agRespLines  = "";
  agSeq        = 1;
  lastPing     = millis();

  webLog("[AG] TCP verbunden, warte auf Prologue...");
  snprintf(agStatusStr, sizeof(agStatusStr), "Prologue...");
  // Prologue wird nicht-blockierend in handleAGReceive() gelesen
  agConnected = true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG nicht-blockierender Empfang
// ═══════════════════════════════════════════════════════════════════════════

void handleAGReceive() {
  if (!agClient.connected()) {
    webLog("[AG] Verbindung getrennt");
    agClient.stop();
    agConnected  = false;
    agConfigDone = false;
    agCfgState   = AGCFG_IDLE;
    snprintf(agStatusStr, sizeof(agStatusStr), "Getrennt");
    ssePushStatus();
    return;
  }

  // Timeout-Prüfung: Config-Schritt hängt zu lange?
  if (agCfgState != AGCFG_IDLE && agCfgState != AGCFG_DONE &&
      agCfgTimeout > 0 && millis() > agCfgTimeout) {
    webLog("[AG] TIMEOUT in Config-Schritt %d – ueberspringe", (int)agCfgState);
    agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
    // Schritt überspringen und mit nächstem fortfahren
    switch (agCfgState) {
      case AGCFG_WAIT_SUB_PORT:
        webLog("[AG] Ueberspringe sub port – starte band list");
        agCfgState   = AGCFG_WAIT_BAND_LIST;
        agCfgSeq     = agSeq;
        sendAGCommand("band list");
        break;
      case AGCFG_WAIT_BAND_LIST:
        webLog("[AG] Ueberspringe band list – starte antenna list");
        agCfgState   = AGCFG_WAIT_ANTENNA_LIST;
        agCfgSeq     = agSeq;
        sendAGCommand("antenna list");
        break;
      case AGCFG_WAIT_ANTENNA_LIST:
        webLog("[AG] Ueberspringe antenna list – starte port get 1");
        agCfgState   = AGCFG_WAIT_PORT1;
        agCfgSeq     = agSeq;
        sendAGCommand("port get 1");
        break;
      case AGCFG_WAIT_PORT1:
        webLog("[AG] Ueberspringe port get 1 – starte port get 2");
        agCfgState   = AGCFG_WAIT_PORT2;
        agCfgSeq     = agSeq;
        sendAGCommand("port get 2");
        break;
      case AGCFG_WAIT_PORT2:
        webLog("[AG] Ueberspringe port get 2 – Config abgeschlossen");
        agCfgState   = AGCFG_DONE;
        agConfigDone = true;
        snprintf(agStatusStr, sizeof(agStatusStr), "OK (unvollst.)");
        ssePushStatus();
        break;
      default:
        agCfgState = AGCFG_DONE;
        break;
    }
  }

  while (agClient.available()) {
    char c = agClient.read();
    if (c == '\r') continue;
    if (c != '\n') { agLineBuf += c; continue; }
    String line = agLineBuf;
    agLineBuf = "";
    if (line.length() == 0) continue;
    agProcessLine(line);
  }
}

// ─── Einzelne AG-Zeile verarbeiten ────────────────────────────────────────

void agProcessLine(const String &line) {
  // Prologue: "V4.0.22 AG" — kommt direkt nach Connect
  if (line.startsWith("V") && line.indexOf(" AG") > 0) {
    char fw[16] = {0};
    sscanf(line.c_str() + 1, "%15s", fw);
    snprintf(agFwStr, sizeof(agFwStr), "%s", fw);
    webLog("[AG] Prologue: %s", line.c_str());
    // Jetzt Konfiguration starten
    agStartConfig();
    return;
  }

  // Status-Nachricht: S0|<message>
  if (line.startsWith("S0|")) {
    webLog("[AG] Status: %s", line.c_str());
    agHandleStatusMessage(line);
    return;
  }

  // Antwort auf ein Kommando: R<seq>|<hex>|<message>
  if (line.startsWith("R")) {
    agHandleConfigResponse(line);
    return;
  }

  webLog("[AG] ?: %s", line.c_str());
}

// ─── Konfigurationssequenz starten ────────────────────────────────────────
// Reihenfolge laut Protokoll-Doku:
//  1. sub port all → Port-Statusänderungen abonnieren (Parameter "all" Pflicht!)
//     sub antenna existiert NICHT – nur: antenna, group, output, port, relay
//  2. band list    → Bänder einlesen
//  3. antenna list → Antennen einlesen
//  4. port get 1   → initialer Port-A-Status
//  5. port get 2   → initialer Port-B-Status

void agStartConfig() {
  webLog("[AG] Starte Konfigurationssequenz...");
  snprintf(agStatusStr, sizeof(agStatusStr), "Konfiguriere");
  bandCount    = 0;
  antennaCount = 0;
  memset(bands,    0, sizeof(bands));
  memset(antennas, 0, sizeof(antennas));
  agRespLines  = "";
  agCfgState   = AGCFG_WAIT_SUB_PORT;
  agCfgSeq     = agSeq;
  agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
  sendAGCommand("sub port all");   // "all" ist Pflichtparameter laut API-Doku!
}

// ─── Config-Antworten verarbeiten (nicht-blockierend) ─────────────────────
// Das AG-Protokoll: jeder Antwortblock endet mit einer Zeile R<seq>|0|
// (leere Message-Teil). Mehrzeilige Antworten (band list etc.) haben
// mehrere R<seq>|0|<daten> Zeilen und enden mit R<seq>|0| (leer).

void agHandleConfigResponse(const String &line) {
  int p1 = line.indexOf('|');
  if (p1 < 0) return;
  uint8_t seq = (uint8_t)line.substring(1, p1).toInt();

  int p2 = line.indexOf('|', p1 + 1);
  if (p2 < 0) return;
  String hexResp = line.substring(p1 + 1, p2);
  hexResp.trim();
  String message = line.substring(p2 + 1);

  bool isError = (hexResp != "0" && hexResp != "00000000");
  if (isError) {
    webLog("[AG] Fehler seq=%d code=%s msg='%s'",
           seq, hexResp.c_str(), message.c_str());
  }

  // Antworten ausserhalb der Config-Sequenz
  if (seq != agCfgSeq || agCfgState == AGCFG_DONE) {
    if (message.startsWith("port ")) parsePortLine(message);
    return;
  }

  // Timeout zurücksetzen da eine Antwort kam
  agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;

  switch (agCfgState) {

    case AGCFG_WAIT_SUB_PORT:
      // Bei Fehler trotzdem weitermachen (z.B. schon abonniert)
      if (isError)
        webLog("[AG] sub port Warnung – fahre fort");
      else
        webLog("[AG] sub port all OK");
      agCfgState = AGCFG_WAIT_BAND_LIST;
      agCfgSeq   = agSeq;
      agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
      sendAGCommand("band list");
      break;

    case AGCFG_WAIT_BAND_LIST:
      if (message.length() > 0) {
        parseBandListLine(message);
      } else {
        // Leere Message = Ende des Blocks
        webLog("[AG] band list: %d Baender geladen", bandCount);
        for (uint8_t i = 0; i < MAX_BANDS; i++)
          if (bands[i].valid && bands[i].freqStop > 0)
            webLog("     Band %2d: %-12s %.3f-%.3f MHz",
                   bands[i].id, bands[i].name,
                   bands[i].freqStart, bands[i].freqStop);
        agCfgState   = AGCFG_WAIT_ANTENNA_LIST;
        agCfgSeq     = agSeq;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
        sendAGCommand("antenna list");
      }
      break;

    case AGCFG_WAIT_ANTENNA_LIST:
      if (message.length() > 0) {
        parseAntennaListLine(message);
      } else {
        webLog("[AG] antenna list: %d Antennen geladen", antennaCount);
        for (uint8_t i = 0; i < MAX_ANTENNAS; i++)
          if (antennas[i].valid)
            webLog("     Ant  %2d: %-20s TX-Mask: %04X",
                   antennas[i].id, antennas[i].name, antennas[i].txMask);
        agCfgState   = AGCFG_WAIT_PORT1;
        agCfgSeq     = agSeq;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
        sendAGCommand("port get 1");
      }
      break;

    case AGCFG_WAIT_PORT1:
      if (message.startsWith("port ")) parsePortLine(message);
      if (message.length() == 0) {
        agCfgState   = AGCFG_WAIT_PORT2;
        agCfgSeq     = agSeq;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
        sendAGCommand("port get 2");
      }
      break;

    case AGCFG_WAIT_PORT2:
      if (message.startsWith("port ")) parsePortLine(message);
      if (message.length() == 0) {
        agCfgState   = AGCFG_DONE;
        agConfigDone = true;
        snprintf(agStatusStr, sizeof(agStatusStr), "OK");
        webLog("[AG] Konfiguration abgeschlossen. Warte auf CI-V...");
        ssePushStatus();
      }
      break;

    default:
      if (message.startsWith("port ")) parsePortLine(message);
      break;
  }
}

// ─── Status-Nachrichten verarbeiten ───────────────────────────────────────
// Format: S0|<message>
// Bekannte Messages:
//   port 1 auto=1 source=AUTO band=3 rxant=1 txant=1 tx=0 inhibit=0
//   antenna reload
//   output ...
//   group ...
//   relay ...

void agHandleStatusMessage(const String &line) {
  // Nachrichten-Teil nach "S0|"
  String msg = line.substring(3);

  if (msg.startsWith("port ")) {
    parsePortLine(msg);
    ssePushStatus();
    return;
  }

  if (msg == "antenna reload") {
    webLog("[AG] Antennen-Konfiguration geaendert, lade neu...");
    // Antennen neu einlesen
    agCfgState = AGCFG_WAIT_ANTENNA_LIST;
    agCfgSeq   = agSeq;
    sendAGCommand("antenna list");
    return;
  }

  // output/group/relay: nur loggen
  webLog("[AG] Status ignoriert: %s", msg.c_str());
}

// ─── AG-Kommando senden ────────────────────────────────────────────────────

void sendAGCommand(const char *cmd) {
  if (!agClient.connected()) return;
  char buf[128];
  snprintf(buf, sizeof(buf), "C%d|%s\r", agSeq, cmd);
  agClient.print(buf);
  webLog("[AG] >> C%d|%s", agSeq, cmd);
  agSeq++;
  if (agSeq > 255) agSeq = 1;
}

// ─── Keepalive (ping) ──────────────────────────────────────────────────────

void sendKeepalive() {
  if (millis() - lastPing < AG_KEEPALIVE_MS) return;
  lastPing = millis();
  sendAGCommand("ping");
}

// ─── Port-Status aktiv pollen ──────────────────────────────────────────────

void pollPortStatus() {
  if (!agConfigDone) return;
  if (millis() - lastPortPoll < PORT_POLL_MS) return;
  lastPortPoll = millis();
  sendAGCommand("port get 1");
  sendAGCommand("port get 2");
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG-Antwort-Parser Hilfsfunktionen
// ═══════════════════════════════════════════════════════════════════════════

// Band-Zeile parsen: "band 1 name=160m freq_start=1.600000 freq_stop=2.200000"
void parseBandListLine(const String &s) {
  if (!s.startsWith("band ")) return;
  uint8_t id = 0;
  sscanf(s.c_str(), "band %hhu", &id);
  if (id >= MAX_BANDS) return;

  char name[16] = {0};
  double fs = 0, fe = 0;
  char *np  = strstr(s.c_str(), "name=");
  char *fsp = strstr(s.c_str(), "freq_start=");
  char *fep = strstr(s.c_str(), "freq_stop=");
  if (np)  sscanf(np  + 5, "%15s",  name);
  if (fsp) sscanf(fsp + 11, "%lf", &fs);
  if (fep) sscanf(fep + 10, "%lf", &fe);

  bands[id].id        = id;
  strncpy(bands[id].name, name, 15);
  bands[id].freqStart = fs;
  bands[id].freqStop  = fe;
  bands[id].valid     = true;
  if (fe > 0) bandCount++;
}

// Antennen-Zeile parsen: "antenna 1 name=Yagi tx=0007 rx=0007 inband=0000"
void parseAntennaListLine(const String &s) {
  if (!s.startsWith("antenna ")) return;
  uint8_t id = 0;
  sscanf(s.c_str(), "antenna %hhu", &id);
  if (id == 0 || id > MAX_ANTENNAS) return;

  char     name[32] = {0};
  uint16_t tx = 0, rx = 0;
  char *np  = strstr(s.c_str(), "name=");
  char *txp = strstr(s.c_str(), "tx=");
  char *rxp = strstr(s.c_str(), "rx=");
  if (np)  sscanf(np  + 5, "%31s",  name);
  if (txp) sscanf(txp + 3, "%hx", &tx);
  if (rxp) sscanf(rxp + 3, "%hx", &rx);

  uint8_t idx = id - 1;
  antennas[idx].id     = id;
  strncpy(antennas[idx].name, name, 31);
  antennas[idx].txMask = tx;
  antennas[idx].rxMask = rx;
  antennas[idx].valid  = true;
  antennaCount++;
}

// Port-Zeile parsen: "port 1 auto=1 source=AUTO band=3 rxant=1 txant=1 tx=0 inhibit=0"
void parsePortLine(const String &s) {
  if (!s.startsWith("port ")) return;
  uint8_t portId = 0;
  sscanf(s.c_str(), "port %hhu", &portId);
  if (portId < 1 || portId > 2) return;
  uint8_t idx = portId - 1;

  uint8_t txant = 0, rxant = 0, band = 0, tx = 0;
  char *tp = strstr(s.c_str(), "txant=");
  char *rp = strstr(s.c_str(), "rxant=");
  char *bp = strstr(s.c_str(), "band=");
  char *xp = strstr(s.c_str(), " tx=");   // Leerzeichen davor um "txant=" zu vermeiden
  if (tp) sscanf(tp + 6, "%hhu", &txant);
  if (rp) sscanf(rp + 6, "%hhu", &rxant);
  if (bp) sscanf(bp + 5, "%hhu", &band);
  if (xp) sscanf(xp + 4, "%hhu", &tx);

  if (portTxAnt[idx] != txant || portRxAnt[idx] != rxant ||
      portBand[idx]  != band  || portTx[idx]    != (bool)tx) {
    webLog("[AG] Port %d: band=%d txant=%d rxant=%d tx=%d",
           portId, band, txant, rxant, tx);
    portTxAnt[idx] = txant;
    portRxAnt[idx] = rxant;
    portBand[idx]  = band;
    portTx[idx]    = (bool)tx;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  AG Port schalten
// ═══════════════════════════════════════════════════════════════════════════

void setAGPort(uint8_t portId, uint8_t bandId, uint8_t antId) {
  if (!agConnected) return;
  char cmd[64];
  snprintf(cmd, sizeof(cmd),
           "port set %d auto=0 source=MANUAL band=%d rxant=%d txant=%d",
           portId, bandId, antId, antId);
  sendAGCommand(cmd);
}

void checkAndSignalConflict(uint8_t wantedAntId) {
  uint8_t otherIdx   = (AG_RADIO_PORT == 1) ? 1 : 0;
  uint8_t otherTxAnt = portTxAnt[otherIdx];
  bool conflict = (wantedAntId != 0) && (otherTxAnt == wantedAntId);
  if (conflict != conflictActive) {
    conflictActive = conflict;
    digitalWrite(CONFLICT_PIN, conflict ? HIGH : LOW);
    if (conflict)
      webLog("[CONF] *** KONFLIKT: Antenne %d bereits von Port %d belegt! GPIO%d=HIGH ***",
             wantedAntId, (AG_RADIO_PORT == 1) ? 2 : 1, CONFLICT_PIN);
    else
      webLog("[CONF] Konflikt aufgeloest. GPIO%d=LOW", CONFLICT_PIN);
    ssePushStatus();
  }
}

uint8_t selectAntenna(uint8_t bandId) {
  uint16_t bandBit = (1u << bandId);
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (!antennas[i].valid) continue;
    if (antennas[i].txMask & bandBit) return antennas[i].id;
  }
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (antennas[i].valid) return antennas[i].id;
  }
  return 0;
}

int8_t freqToBandId(double freqMHz) {
  for (uint8_t i = 1; i < MAX_BANDS; i++) {
    if (!bands[i].valid || bands[i].freqStop <= 0) continue;
    if (freqMHz >= bands[i].freqStart && freqMHz <= bands[i].freqStop)
      return (int8_t)i;
  }
  return -1;
}

// ═══════════════════════════════════════════════════════════════════════════
//  CI-V Empfang und Verarbeitung
// ═══════════════════════════════════════════════════════════════════════════

void handleCIV() {
  // Bytes aus UART2 einlesen
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    // Puffer-Reset wenn zu lang (Framing-Fehler)
    if (civLen >= CIV_BUF_SIZE) civLen = 0;

    // Auf ersten Preamble-Byte warten
    if (civLen == 0 && b != CIV_PREAMBLE) continue;

    civBuf[civLen++] = b;

    if (b == CIV_EOM) {
      processCIVFrame();
      civLen = 0;
    }
  }

  // Aktives Polling wenn seit CIV_BROADCAST_TIMEOUT_MS kein Broadcast
  if (civAddr != 0x00 &&
      millis() - lastCivRx  > CIV_BROADCAST_TIMEOUT_MS &&
      millis() - lastCivPoll > CIV_POLL_MS) {
    lastCivPoll = millis();
    requestCIVFreq();
  }
}

// CI-V Frequenzabfrage aktiv senden (Cmd 0x03 an Transceiver)
void requestCIVFreq() {
  // FE FE <dst> <src> 03 FD
  uint8_t frame[6] = {
    CIV_PREAMBLE, CIV_PREAMBLE,
    civAddr, CIV_CONTROLLER,
    0x03, CIV_EOM
  };
  Serial2.write(frame, sizeof(frame));
}

void processCIVFrame() {
  if (civLen < 6) return;
  if (civBuf[0] != CIV_PREAMBLE || civBuf[1] != CIV_PREAMBLE) return;
  if (civBuf[civLen - 1] != CIV_EOM) return;

  uint8_t dst = civBuf[2];
  uint8_t src = civBuf[3];
  uint8_t cmd = civBuf[4];

  // Transceiver-Adresse beim ersten Frame lernen
  if (civAddr == 0x00 && src != CIV_CONTROLLER && src != 0x00) {
    civAddr = src;
    snprintf(civAddrStr, sizeof(civAddrStr), "0x%02X", civAddr);
    webLog("[CIV] Transceiver-Adresse gelernt: %s", civAddrStr);
    ssePushStatus();
  }

  // Nur Frequenz-Frames verarbeiten (Cmd 0x00 = Broadcast, 0x03 = Poll-Antwort)
  if (cmd != 0x00 && cmd != 0x03) return;

  // Nur Frames die an uns oder Broadcast gerichtet sind
  if (dst != CIV_CONTROLLER && dst != 0x00) return;

  // Mindestens 5 Frequenz-Bytes nötig
  if (civLen < 11) return;

  uint64_t freqHz  = decodeCIVFreq(&civBuf[5]);
  double   freqMHz = freqHz / 1e6;
  if (freqMHz < 0.01) return;

  lastCivRx = millis();  // Zeitstempel aktualisieren
  snprintf(civFreqStr, sizeof(civFreqStr), "%.6f MHz", freqMHz);
  civFreqMHz = freqMHz;

  // Band bestimmen
  int8_t bandId = freqToBandId(freqMHz);

  // Frequenz-Push bei jeder Änderung (nicht nur Bandwechsel)
  static double lastPushedFreq = 0.0;
  if (freqMHz != lastPushedFreq) {
    lastPushedFreq = freqMHz;
    ssePushStatus();
  }

  if (bandId < 0) {
    if (lastBandId != bandId) {
      webLog("[CIV] %.6f MHz - kein passendes Band", freqMHz);
      lastBandId = bandId;
    }
    return;
  }

  if (bandId == lastBandId) return;

  lastBandId = bandId;
  webLog("[CIV] %.6f MHz -> Band %d (%s)", freqMHz, bandId, bands[bandId].name);

  uint8_t antId = selectAntenna(bandId);
  if (antId == 0) {
    webLog("[CIV] Keine Antenne fuer Band %d konfiguriert", bandId);
    return;
  }

  webLog("[AG] Schalte Port %d: Band=%d (%s) Ant=%d (%s)",
         AG_RADIO_PORT, bandId, bands[bandId].name,
         antId, antennas[antId-1].name);

  checkAndSignalConflict(antId);
  setAGPort(AG_RADIO_PORT, bandId, antId);
  ssePushStatus();
}

uint64_t decodeCIVFreq(const uint8_t *data) {
  uint64_t freq = 0, mult = 1;
  for (int i = 0; i < 5; i++) {
    freq += (data[i] & 0x0F) * mult;        mult *= 10;
    freq += ((data[i] >> 4) & 0x0F) * mult; mult *= 10;
  }
  return freq;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Web-Handler
// ═══════════════════════════════════════════════════════════════════════════

void handleSSE() {
  if (sseConnected) sseClient.stop();
  sseClient    = webServer.client();
  sseConnected = true;

  sseClient.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/event-stream\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: keep-alive\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n"
  );

  // Log-Ringpuffer nachliefern
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

void handleSSEClient() {
  static uint32_t lastSsePing = 0;
  if (!sseConnected) return;
  if (!sseClient.connected()) { sseConnected = false; return; }
  if (millis() - lastSsePing > 15000) {
    lastSsePing = millis();
    sseClient.print(": ping\n\n");
  }
}

void handleApiJson() {
  String antA = "---", antB = "---";
  uint8_t ia = portTxAnt[0], ib = portTxAnt[1];
  if (ia > 0 && ia <= MAX_ANTENNAS && antennas[ia-1].valid) antA = antennas[ia-1].name;
  else if (ia > 0) antA = "Ant " + String(ia);
  if (ib > 0 && ib <= MAX_ANTENNAS && antennas[ib-1].valid) antB = antennas[ib-1].name;
  else if (ib > 0) antB = "Ant " + String(ib);

  String activeBand = "---";
  if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid)
    activeBand = bands[lastBandId].name;

  String json = "{";
  json += "\"civAddr\":\"" + String(civAddrStr) + "\",";
  json += "\"civFreq\":\"" + String(civFreqStr) + "\",";
  json += "\"activeBand\":\"" + activeBand + "\",";
  json += "\"agName\":\"" + String(agNameStr) + "\",";
  json += "\"agSerial\":\"" + String(agSerialStr) + "\",";
  json += "\"agFw\":\"" + String(agFwStr) + "\",";
  json += "\"agIP\":\"" + String(agIPStr) + "\",";
  json += "\"agPort\":" + String(agTcpPort) + ",";
  json += "\"agPorts\":" + String(agPorts) + ",";
  json += "\"agAntennas\":" + String(agAntennas) + ",";
  json += "\"agMode\":\"" + String(agModeStr) + "\",";
  json += "\"agUptime\":" + String(agUptime) + ",";
  json += "\"agStatus\":\"" + String(agStatusStr) + "\",";
  json += "\"portA\":\"" + antA + "\",";
  json += "\"portB\":\"" + antB + "\",";
  json += "\"txA\":" + String(portTx[0] ? "true" : "false") + ",";
  json += "\"txB\":" + String(portTx[1] ? "true" : "false") + ",";
  json += "\"conflict\":" + String(conflictActive ? "true" : "false") + ",";
  json += "\"bands\":[";
  bool first = true;
  for (uint8_t i = 0; i < MAX_BANDS; i++) {
    if (!bands[i].valid || bands[i].freqStop <= 0) continue;
    if (!first) json += ","; first = false;
    json += "{\"id\":" + String(bands[i].id) + ",";
    json += "\"name\":\"" + String(bands[i].name) + "\",";
    json += "\"start\":" + String(bands[i].freqStart, 3) + ",";
    json += "\"stop\":"  + String(bands[i].freqStop,  3) + "}";
  }
  json += "],\"antennas\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (!antennas[i].valid) continue;
    if (!first) json += ","; first = false;
    json += "{\"id\":" + String(antennas[i].id) + ",";
    json += "\"name\":\"" + String(antennas[i].name) + "\",";
    json += "\"txMask\":" + String(antennas[i].txMask) + "}";
  }
  json += "]}";

  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", json);
}

void handleWebRoot() {
  String html = R"rawhtml(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WT32 CI-V Bridge</title>
<style>
  :root {
    --bg:#0f1117;--card:#1a1d27;--border:#2a2d3e;
    --accent:#4f8ef7;--green:#3ecf8e;--red:#f87171;
    --yellow:#fbbf24;--text:#e2e8f0;--muted:#94a3b8;
    --mono:'Courier New',monospace;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;font-size:14px;padding:16px}
  h1{font-size:1.2rem;color:var(--accent);margin-bottom:16px;border-bottom:1px solid var(--border);padding-bottom:8px}
  h2{font-size:.85rem;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:10px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;margin-bottom:12px}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:14px}
  .kv{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid var(--border)}
  .kv:last-child{border-bottom:none}
  .kv .label{color:var(--muted)}
  .kv .val{font-weight:600;font-family:var(--mono);font-size:.9rem}
  .badge{display:inline-block;padding:2px 8px;border-radius:99px;font-size:.75rem;font-weight:700}
  .badge-green{background:#14532d;color:var(--green)}
  .badge-red{background:#450a0a;color:var(--red)}
  .badge-yellow{background:#451a03;color:var(--yellow)}
  .badge-blue{background:#1e3a5f;color:var(--accent)}
  .badge-muted{background:#1e2235;color:var(--muted)}
  .badge-tx{background:#7c2d12;color:#fb923c}
  #conflict-banner{display:none;background:#450a0a;border:1px solid var(--red);
    color:var(--red);border-radius:8px;padding:10px 14px;margin-bottom:12px;font-weight:700}
  table{width:100%;border-collapse:collapse;font-size:.82rem}
  th{color:var(--muted);text-align:left;padding:4px 6px;border-bottom:1px solid var(--border)}
  td{padding:4px 6px;border-bottom:1px solid var(--border)}
  tr:last-child td{border-bottom:none}
  td.active{color:var(--green);font-weight:700}
  #log{width:100%;height:260px;background:#0a0c13;color:#7dd3fc;border:1px solid var(--border);
    border-radius:6px;padding:8px;font-family:var(--mono);font-size:.75rem;
    resize:vertical;overflow-y:scroll;white-space:pre-wrap;word-break:break-all}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
  .dot-green{background:var(--green);box-shadow:0 0 6px var(--green)}
  .dot-red{background:var(--red);box-shadow:0 0 6px var(--red)}
  .dot-yellow{background:var(--yellow);box-shadow:0 0 6px var(--yellow)}
  .dot-muted{background:var(--muted)}
  footer{margin-top:12px;color:var(--muted);font-size:.75rem;text-align:center}
</style>
</head>
<body>
<h1>&#128225; WT32-ETH01 &mdash; CI-V &rarr; Antenna Genius Bridge</h1>
<div id="conflict-banner">&#9888; ANTENNEN-KONFLIKT: Ziel-Antenne bereits von anderem Port belegt! GPIO4 = HIGH</div>
<div class="grid">
  <div class="card">
    <h2>Funkger&auml;t (CI-V)</h2>
    <div class="kv"><span class="label">CI-V Adresse</span><span class="val" id="civAddr">---</span></div>
    <div class="kv"><span class="label">Frequenz</span><span class="val" id="civFreq">---</span></div>
    <div class="kv"><span class="label">Aktives Band</span><span id="activeBand"><span class="badge badge-muted">---</span></span></div>
  </div>
  <div class="card">
    <h2>Antenna Genius</h2>
    <div class="kv"><span class="label">Name</span><span class="val" id="agName">---</span></div>
    <div class="kv"><span class="label">IP-Adresse</span><span class="val" id="agIP">---</span></div>
    <div class="kv"><span class="label">TCP-Port</span><span class="val" id="agPort">---</span></div>
    <div class="kv"><span class="label">Seriennummer</span><span class="val" id="agSerial">---</span></div>
    <div class="kv"><span class="label">Firmware</span><span class="val" id="agFw">---</span></div>
    <div class="kv"><span class="label">Radio-Ports</span><span class="val" id="agPorts">---</span></div>
    <div class="kv"><span class="label">Antennen-Ports</span><span class="val" id="agAntennas">---</span></div>
    <div class="kv"><span class="label">Stack-Mode</span><span id="agModeWrap"><span class="val" id="agMode">---</span></span></div>
    <div class="kv"><span class="label">Uptime</span><span class="val" id="agUptime">---</span></div>
    <div class="kv"><span class="label">Status</span><span id="agStatusWrap"><span class="dot dot-muted"></span><span id="agStatus">---</span></span></div>
  </div>
  <div class="card">
    <h2>Port-Belegung</h2>
    <div class="kv"><span class="label">Port A &ndash; Antenne</span><span id="portA"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Port A &ndash; Band</span><span id="bandA"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Port A &ndash; TX</span><span id="txA"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Port B &ndash; Antenne</span><span id="portB"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Port B &ndash; Band</span><span id="bandB"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Port B &ndash; TX</span><span id="txB"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">Konflikt</span><span id="conflictBadge"><span class="badge badge-muted">---</span></span></div>
  </div>
</div>
<div class="grid">
  <div class="card">
    <h2>Band-Konfiguration (AG)</h2>
    <table><thead><tr><th>#</th><th>Name</th><th>Start MHz</th><th>Stop MHz</th></tr></thead>
    <tbody id="bandTable"><tr><td colspan="4" style="color:var(--muted)">Lade...</td></tr></tbody></table>
  </div>
  <div class="card">
    <h2>Antennen-Konfiguration (AG)</h2>
    <table><thead><tr><th>#</th><th>Name</th><th>TX-Maske</th></tr></thead>
    <tbody id="antTable"><tr><td colspan="3" style="color:var(--muted)">Lade...</td></tr></tbody></table>
  </div>
</div>
<div class="card">
  <h2>Debug-Log <span id="sseState" style="font-size:.7rem;margin-left:8px"></span></h2>
  <div id="log"></div>
</div>
<footer>WT32-ETH01 &bull; Live via SSE &bull; Aktualisierung automatisch</footer>
<script>
const MAX_LOG=300;
let logLines=[];
let activeBandName='';
function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
function appendLog(line){
  logLines.push(line);
  if(logLines.length>MAX_LOG)logLines.shift();
  const el=document.getElementById('log');
  el.textContent=logLines.join('\n');
  el.scrollTop=el.scrollHeight;
}
function dotClass(s){
  if(s==='OK'||s==='Verbunden')return'dot-green';
  if(s==='Getrennt'||s==='Fehler')return'dot-red';
  if(s==='Suche...'||s==='Verbinde...'||s==='Konfiguriere'||s==='Prologue...')return'dot-yellow';
  return'dot-muted';
}
function badgeAnt(v){return v==='---'?`<span class="badge badge-muted">---</span>`:`<span class="badge badge-blue">${esc(v)}</span>`}
function badgeBand(v,active){
  if(v==='---')return`<span class="badge badge-muted">---</span>`;
  return`<span class="badge ${active?'badge-green':'badge-blue'}">${esc(v)}</span>`;
}
function fmtUptime(s){
  if(!s||s===0)return'---';
  const d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),
        m=Math.floor((s%3600)/60),sec=s%60;
  if(d>0)return`${d}d ${h}h ${m}m`;
  if(h>0)return`${h}h ${m}m ${sec}s`;
  if(m>0)return`${m}m ${sec}s`;
  return`${sec}s`;
}
function applyStatus(d){
  activeBandName=d.activeBand;
  document.getElementById('civAddr').textContent=d.civAddr;
  document.getElementById('civFreq').textContent=d.civFreq;
  const ab=document.getElementById('activeBand');
  ab.innerHTML=d.activeBand!=='---'?`<span class="badge badge-blue">${esc(d.activeBand)}</span>`:`<span class="badge badge-muted">---</span>`;
  document.getElementById('agName').textContent=d.agName;
  document.getElementById('agIP').textContent=d.agIP;
  if(document.getElementById('agPort'))
    document.getElementById('agPort').textContent=d.agPort>0?':'+d.agPort:'---';
  document.getElementById('agSerial').textContent=d.agSerial||'---';
  document.getElementById('agFw').textContent=d.agFw||'---';
  if(document.getElementById('agPorts'))
    document.getElementById('agPorts').textContent=d.agPorts>0?d.agPorts:'---';
  if(document.getElementById('agAntennas'))
    document.getElementById('agAntennas').textContent=d.agAntennas>0?d.agAntennas:'---';
  if(document.getElementById('agMode')){
    const mode=d.agMode||'---';
    const mw=document.getElementById('agModeWrap');
    if(mode==='master')mw.innerHTML=`<span class="badge badge-green">master</span>`;
    else if(mode==='slave')mw.innerHTML=`<span class="badge badge-blue">slave</span>`;
    else mw.innerHTML=`<span class="val">${esc(mode)}</span>`;
  }
  if(document.getElementById('agUptime'))
    document.getElementById('agUptime').textContent=fmtUptime(d.agUptime);
  const sw=document.getElementById('agStatusWrap');
  sw.innerHTML=`<span class="dot ${dotClass(d.agStatus)}"></span><span>${esc(d.agStatus)}</span>`;
  document.getElementById('portA').innerHTML=badgeAnt(d.portA);
  document.getElementById('portB').innerHTML=badgeAnt(d.portB);
  document.getElementById('bandA').innerHTML=badgeBand(d.bandA||'---',d.bandA===d.activeBand);
  document.getElementById('bandB').innerHTML=badgeBand(d.bandB||'---',d.bandB===d.activeBand);
  document.getElementById('txA').innerHTML=d.txA?`<span class="badge badge-tx">&#128308; SENDEN</span>`:`<span class="badge badge-muted">RX</span>`;
  document.getElementById('txB').innerHTML=d.txB?`<span class="badge badge-tx">&#128308; SENDEN</span>`:`<span class="badge badge-muted">RX</span>`;
  const cb=document.getElementById('conflictBadge');
  const banner=document.getElementById('conflict-banner');
  if(d.conflict){cb.innerHTML=`<span class="badge badge-red">&#9888; KONFLIKT</span>`;banner.style.display='block';}
  else{cb.innerHTML=`<span class="badge badge-green">OK</span>`;banner.style.display='none';}
  document.querySelectorAll('#bandTable tr').forEach(tr=>{
    if(!tr.cells[1])return;
    const active=tr.cells[1].textContent===activeBandName;
    for(let c of tr.cells)c.className=active?'active':'';
  });
}
function loadConfig(){
  fetch('/api').then(r=>r.json()).then(d=>{
    const bt=document.getElementById('bandTable');
    if(d.bands&&d.bands.length){
      bt.innerHTML=d.bands.map(b=>{
        const active=b.name===activeBandName;
        return`<tr><td class="${active?'active':''}">${b.id}</td><td class="${active?'active':''}">${esc(b.name)}</td><td>${b.start.toFixed(3)}</td><td>${b.stop.toFixed(3)}</td></tr>`;
      }).join('');
    }else{bt.innerHTML='<tr><td colspan="4" style="color:var(--muted)">Keine Daten</td></tr>';}
    const at=document.getElementById('antTable');
    if(d.antennas&&d.antennas.length){
      at.innerHTML=d.antennas.map(a=>`<tr><td>${a.id}</td><td>${esc(a.name)}</td><td style="font-family:var(--mono)">${a.txMask.toString(16).padStart(4,'0').toUpperCase()}</td></tr>`).join('');
    }else{at.innerHTML='<tr><td colspan="3" style="color:var(--muted)">Keine Daten</td></tr>';}
    applyStatus(d);
  }).catch(()=>{});
}
function connectSSE(){
  const state=document.getElementById('sseState');
  const es=new EventSource('/events');
  es.addEventListener('status',e=>{try{applyStatus(JSON.parse(e.data));}catch(ex){}});
  es.addEventListener('log',e=>{appendLog(e.data);});
  es.onopen=()=>{state.innerHTML='<span class="badge badge-green">Live</span>';};
  es.onerror=()=>{state.innerHTML='<span class="badge badge-red">Unterbrochen &ndash; reconnect...</span>';};
}
loadConfig();
connectSSE();
setInterval(loadConfig,30000);
</script>
</body>
</html>)rawhtml";

  webServer.sendHeader("Cache-Control","no-cache");
  webServer.send(200,"text/html; charset=utf-8",html);
}
