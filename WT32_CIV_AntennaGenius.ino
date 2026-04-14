/*
 * WT32-ETH01 — CI-V Bus → Antenna Genius Bridge
 * ================================================
 * Hardware : WT32-ETH01 (ESP32 + LAN8720)
 * CI-V Bus : UART2  (GPIO5 = RX2, GPIO17 = TX2)
 *            Open-Collector → 1 kΩ Pull-up auf 3.3 V empfohlen.
 *
 * Ethernet : DHCP, AG-Discovery via UDP-Broadcast Port 9007
 * AG API   : TCP Port 9007 (ASCII-Protokoll)
 *
 * Display  : EA DOGL128W-6 (ST7565, 128×64, Software-SPI)
 *            Bibliothek: U8g2 by olikraus (Arduino Library Manager)
 *            SI=GPIO15  SCK=GPIO14  CS=GPIO12  A0=GPIO2  RST=GPIO33
 *
 * Encoder  : Drehencoder mit Taster
 *            A=GPIO39  B=GPIO36  BTN=GPIO35
 *            Alle input-only → externe 10 kΩ Pull-ups an 3.3 V zwingend!
 *            GPIO36/39 teilen 270 pF interne Kapazität → B wird per Polling
 *            gelesen (kein ISR) um Crosstalk zu vermeiden.
 *
 * FlexRadio: VITA-49 Discovery-Emulation (UDP/TCP Port 4992)
 *            AG erkennt CI-V-Port als FlexRadio und setzt tx=1
 *            wenn der Icom-TRX aktiv ist → SmartSDR sieht Konflikt
 *
 * Ablauf:
 *  1. Ethernet per DHCP initialisieren
 *  2. AG per UDP-Broadcast suchen → IP + Port aus Discovery-Paket
 *  3. TCP-Verbindung zum AG (Port aus Discovery, Fallback 9007)
 *  4. sub port all → Port-Statusänderungen abonnieren
 *  5. band list + antenna list → Konfiguration einlesen
 *  6. CI-V-Frequenz empfangen oder aktiv pollen (Cmd 0x03)
 *     → Band bestimmen → port set senden
 *  7. CI-V-Timeout (5 s): Port freigeben, GPIO4 HIGH, CI-V Inhibit ON
 *  8. FlexRadio-Emulation: Discovery-Broadcast alle 2 s,
 *     TCP-Server sendet Interlock-Status bei PTT-Änderung
 *
 * Sicherheitsfunktionen:
 *  - GPIO4 HIGH:      TX-Inhibit-Hardwareleitung zum TRX
 *  - CI-V Cmd 16/66: TX-Inhibit über CI-V (neuere Icom-TRX)
 *  - FlexRadio PTT:  state=PTT_REQUESTED wenn TRX aktiv → AG sperrt Port B
 *  Alle drei werden synchron über setConflict() gesteuert.
 *
 * AG-Protokoll-Hinweise:
 *  - Kommando-Terminator: \r\n (nicht nur \r)
 *  - write() + flush() verwenden, NICHT print()
 *  - Antwortblock endet mit R<seq>|0| (leere Message = Blockende)
 *  - agCfgSeq IMMER nach sendAGCommand() als agSeq-1 setzen!
 *  - sub port all: Parameter "all" ist Pflicht; sub antenna existiert nicht
 *
 * Konfigurierbare Parameter (alle als #define):
 *  DISPLAY_ENABLED       1/0   Display + Encoder aktivieren
 *  FLEX_EMULATION_ENABLED 1/0  FlexRadio-Emulation aktivieren
 *  CIV_BAUD              19200 Baudrate CI-V
 *  CIV_TIMEOUT_MS        5000  Inaktivitäts-Timeout TRX
 *  AG_RADIO_PORT         1     Genutzter AG-Port (1=A, 2=B)
 *  ETH_POWER_PIN         16    LAN8720 Reset (-1 auf manchen Boards)
 *  FLEX_MODEL/SERIAL     ...   Gerätekennzeichnung in AG-flex-list
 *  DISP_CONTRAST         22    Display-Kontrast 0-63
 *
 * Bibliotheken (Board: esp32 by Espressif ≥ 2.x):
 *  - ETH.h, WiFiUdp.h, WiFiClient.h, WiFiServer.h,
 *    WebServer.h  (alle im ESP32-Arduino-Core enthalten)
 *  - U8g2lib.h   (nur wenn DISPLAY_ENABLED 1)
 *
 * Board-Einstellungen Arduino IDE:
 *  Board: ESP32 Dev Module  |  Flash: QIO 4MB
 */

#include <ETH.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WebServer.h>

// ─── Display & Encoder aktivieren ───────────────────────────────────────────
// Auf 0 setzen um Display und Encoder vollständig zu deaktivieren
// (kein #include, keine Pin-Belegung, kein Code-Overhead)
#define DISPLAY_ENABLED  1

#if DISPLAY_ENABLED
#include <U8g2lib.h>
#include <SPI.h>
#endif

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
#define CIV_TIMEOUT_MS  5000    // nach 5 s ohne CI-V → TRX abgeschaltet → Port freigeben

// ─── Antenna Genius ─────────────────────────────────────────────────────────
#define AG_PORT         9007
#define AG_KEEPALIVE_MS 5000
#define AG_RECONNECT_MS 15000
#define AG_CFG_TIMEOUT_MS 8000  // max. Wartezeit pro Config-Schritt
#define PORT_POLL_MS    3000    // Port-Status aktiv abfragen alle N ms
#define MAX_BANDS       16
#define MAX_ANTENNAS    16
#define AG_RADIO_PORT   1       // Radio-Port A = 1

// ─── Konflikt-Ausgang ────────────────────────────────────────────────────────
#define CONFLICT_PIN    4

// ─── Display (EA DOGL128W-6, ST7565, Software-SPI) ──────────────────────────
// Alle Pins sind am WT32-ETH01 frei verfügbar.
// GPIO2  : Vorsicht beim Flashen (Strapping-Pin) – nach dem Flash normal nutzbar.
// GPIO35/36: Input-Only – daher ideal für Encoder-Eingänge (kein Output nötig).
#define DISP_MOSI_PIN   15   // SI  des DOGL128
#define DISP_SCK_PIN    14   // SCL des DOGL128
#define DISP_CS_PIN     12   // CSB des DOGL128 (active low)
#define DISP_A0_PIN      2   // A0  des DOGL128 (RS: 0=Cmd, 1=Data)
#define DISP_RST_PIN    33   // RST des DOGL128 (active low)

// ─── Drehencoder ─────────────────────────────────────────────────────────────
#define ENC_A_PIN       39   // Encoder Kanal A  (input-only, ext. Pull-up, ISR)
#define ENC_B_PIN       36   // Encoder Kanal B  (input-only, ext. Pull-up)
#define ENC_BTN_PIN     35   // Encoder Taster   (input-only, ext. Pull-up)
// Hinweis: GPIO35/36/39 sind input-only ohne internen Pull-up.
// Externe Pull-up-Widerstände (10 kΩ) an 3.3 V sind zwingend erforderlich!
// GPIO36/39 haben 270 pF interne Kapazität → Encoder-B wird per Polling gelesen
// (kein ISR auf GPIO36) um möglichen Crosstalk zu vermeiden.
#define ENC_DEBOUNCE_MS  5   // Entprellzeit Taster in ms
#define MENU_TIMEOUT_MS 10000 // Menü schließt sich nach 10 s Inaktivität

// ─── Web-Server ──────────────────────────────────────────────────────────────
#define WEB_PORT        80
#define LOG_BUF_LINES   300      // Ringpuffer für Web-Log (inkl. Zeitstempel)

// ─── FlexRadio-Emulation (PTT-Interlock für Antenna Genius) ─────────────────
// Emuliert das FlexRadio VITA-49-Discovery-Protokoll damit der AG unseren
// CI-V-Port als "FLEX"-Radio erkennt und tx=1 setzt wenn der TRX aktiv ist.
// Der AG nutzt dies für die Konfliktverhinderung mit SmartSDR an Port B.
// Auf 0 setzen um die Emulation zu deaktivieren.
#define FLEX_EMULATION_ENABLED  1

#define FLEX_TCP_PORT      4992   // FlexRadio API-Port
#define FLEX_UDP_PORT      4992   // Discovery-Broadcast-Port
#define FLEX_DISCOVERY_MS  2000   // Discovery-Broadcast-Intervall
// Geräte-Identifikation die der AG in seiner flex list anzeigt
#define FLEX_MODEL         "FLEX-6300"
#define FLEX_SERIAL        "CIV-Bridge-1"
#define FLEX_NICKNAME      "CIV-Bridge"
#define FLEX_VERSION       "3.3.15"

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
static uint32_t      agCfgTimeout = 0;  // 0 = inaktiv; >0 = Ablaufzeitpunkt in ms

// Verzögerter Config-Start nach AG-Prologue (nicht-blockierend, ersetzt delay(200))
static uint32_t agConfigStartAt = 0;  // 0 = kein Start ausstehend; >0 = Startzeitpunkt

// Sequenznummer für AG-Kommandos (1–255, rollt über)
static uint8_t  agSeq    = 1;
static uint32_t lastPing = 0;

// AG Zeilen-Empfangspuffer (nicht-blockierend)
static String   agLineBuf;

// Gesammelte Antwortzeilen für laufendes Konfigurations-Kommando

// Port-Status
static uint8_t  portTxAnt[2]   = {0, 0};
static uint8_t  portRxAnt[2]   = {0, 0};
static uint8_t  portBand[2]    = {0, 0};
static bool     portTx[2]      = {false, false};
static bool     conflictActive = false;
static uint32_t lastPortPoll   = 0;

// CI-V
static uint8_t  civBuf[CIV_BUF_SIZE];
static uint8_t  civLen        = 0;
static uint8_t  civAddr       = 0x00;
static int8_t   lastBandId    = -1;
static uint32_t lastCivRx     = 0;   // Zeitstempel letzter CI-V Frequenzempfang
static uint32_t lastCivPoll   = 0;   // Zeitstempel letztes aktives Polling
static double   civFreqMHz    = 0.0;
static bool     civPortReleased = false; // true wenn Port nach Timeout freigegeben wurde

// Web-Server & SSE
WebServer webServer(WEB_PORT);
static String   logLines[LOG_BUF_LINES];
static uint8_t  logHead  = 0;
static uint16_t logTotal = 0;
static WiFiClient sseClient;
static bool       sseConnected = false;

#if FLEX_EMULATION_ENABLED
// FlexRadio-Emulation
static WiFiServer  flexTcpServer(FLEX_TCP_PORT);
// FlexRadio TCP-Server: bis zu FLEX_MAX_CLIENTS Clients gleichzeitig
// (typisch: AG auf Port 4992 + SmartSDR auf Port 4992)
#define FLEX_MAX_CLIENTS 4
struct FlexClient {
  WiFiClient  tcp;
  bool        connected  = false;
  char        lineBuf[128];
  uint32_t    lineBufLen = 0;
  bool        txActive   = false; // zuletzt gesendeter PTT-Status an diesen Client
};
static FlexClient   flexClients[FLEX_MAX_CLIENTS];
static WiFiUDP      flexDiscUdp;
static uint32_t     flexLastDiscovery = 0;
static bool         flexTxActive      = false; // globaler PTT-Zustand
static uint8_t      flexPktCount      = 0;     // rollierender VITA-Paketzähler
#endif

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

// ─── Display & Encoder ───────────────────────────────────────────────────────
#if DISPLAY_ENABLED

// U8g2: Software-SPI, Single-Page-Rendering (128 Byte RAM statt 1 KB)
// Rotation U8G2_R2 = 180° (je nach Einbaulage anpassen: R0/R1/R2/R3)
U8G2_ST7565_EA_DOGM128_1_4W_SW_SPI u8g2(
  U8G2_R2,
  DISP_SCK_PIN, DISP_MOSI_PIN,
  DISP_CS_PIN,  DISP_A0_PIN,
  DISP_RST_PIN
);

// Splash-Screen Bitmap: 128x64 px, 1 bpp, LSB-first (U8g2-Format)
// Quelle: Silo_Zeichnung_4_2_200x128.bmp, skaliert 100x64, zentriert
static const uint8_t splashBitmap[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x40, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0xF8, 0xC3, 0xF1, 0xF0, 0xFC, 0xC1, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0xF8, 0xC7, 0x79, 0x70, 0xFC, 0xC3, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0xF8, 0xC7, 0x39, 0x78, 0xFC, 0xC7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0xF8, 0xCF, 0x3D, 0x7C, 0xFC, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x38, 0xCF, 0x1F, 0x7C, 0x1C, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0D, 0x08, 0x00, 0x18, 0xCE, 0x0F, 0x7E, 0x1C, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x1C, 0xCE, 0x0F, 0x7E, 0x1C, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xA0, 0x4B, 0x02, 0x00, 0x1C, 0xCE, 0x07, 0x77, 0x1C, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xE0, 0x4F, 0x02, 0x00, 0x1C, 0xCE, 0x07, 0x73, 0x0C, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xEC, 0x7F, 0x03, 0x00, 0x1C, 0xEE, 0x87, 0x73, 0x0E, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1C, 0xC0, 0x03, 0x00, 0x1C, 0xEF, 0x8F, 0xFF, 0x0E, 0xE7, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x08, 0x00, 0x03, 0x00, 0x1C, 0xE7, 0x8F, 0xFF, 0x8E, 0x67, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x07, 0x00, 0x9C, 0xE7, 0x9E, 0xFF, 0xCE, 0x73, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0xFC, 0xE7, 0x9C, 0x7F, 0xFE, 0x73, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0xFC, 0xE3, 0x3C, 0x38, 0xFE, 0x71, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0xFE, 0xE1, 0x38, 0x38, 0xFE, 0x71, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0xFE, 0x60, 0x78, 0x38, 0xFE, 0x70, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x10, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x32, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x00, 0xFE, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xC4, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xE4, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xF6, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1E, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x23, 0x3E, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define SPLASH_W 128
#define SPLASH_H  64

// Display-Kontrast 0–63 (typisch 20–30 für DOGL128 mit 3.3 V)
#define DISP_CONTRAST   22

// Menü-Zustände
enum MenuState {
  MENU_STATUS,      // Normalanzeige: Frequenz / Band / Antenne
  MENU_ANT_SELECT,  // Antennen-Auswahl für aktuelles Band
  MENU_CONFIRM      // Bestätigungsdialog
};

static MenuState    menuState      = MENU_STATUS;
static uint8_t      menuCursor     = 0;  // ausgewähltes Element
static uint8_t      menuAntCount   = 0;  // Antennen für aktuelles Band
static uint8_t      menuAntIds[MAX_ANTENNAS]; // IDs der wählbaren Antennen
static uint32_t     menuLastAction = 0;  // letzter Encoder-/Tasterdruck (ms)
static bool         dispNeedsRedraw = true;

// Encoder-State (ISR-sicher)
static volatile int8_t  encDelta   = 0;  // akkumulierte Schritte seit letztem Loop
static uint32_t         encBtnTime = 0;

// ISR für Encoder Kanal A (im Loop ausgewertet wegen SPI-Konflikte)
// Einfaches Gray-Code-Decoding
static void IRAM_ATTR encISR() {
  static uint8_t last = 0;
  uint8_t a = digitalRead(ENC_A_PIN);
  uint8_t b = digitalRead(ENC_B_PIN);
  uint8_t cur = (a << 1) | b;
  // Gray-Code Tabelle: +1 oder -1 je nach Übergang
  static const int8_t table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  encDelta += table[(last << 2) | cur];
  last = cur;
}

#endif // DISPLAY_ENABLED

// ─── Forward Declarations ────────────────────────────────────────────────────
void webLog(const char *fmt, ...);
void ssePushStatus();
void handleWebRoot();
void handleApiJson();
void handleLogExport();
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
void releaseAGPort();
void sendCIVTxInhibit(bool inhibit);
void setConflict(bool active, const char *reason);
void checkAndSignalConflict(uint8_t wantedAntId);
void parseBandListLine(const String &line);
void parseAntennaListLine(const String &line);
void parsePortLine(const String &line);
#if FLEX_EMULATION_ENABLED
void flexSetup();
void flexLoop();
void flexSendDiscovery();
void flexHandleTcpClient();
void flexProcessLine(uint8_t idx, const char *line);
void flexSendLineTo(uint8_t idx, const char *line);
void flexSendInterlockStatus(bool transmitting);
void flexSendLine(const char *line);
#endif
#if DISPLAY_ENABLED
void displaySetup();
void displayLoop();
void displayDrawStatus();
void displayDrawAntMenu();
void displayDrawConfirm();
void menuBuildAntList();
void menuHandleEncoder();
void menuSelectAntenna(uint8_t antId);
void displaySetInvert(bool invert);
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  Logging
// ═══════════════════════════════════════════════════════════════════════════

void webLog(const char *fmt, ...) {
  // Zeitstempel mm:ss.mmm seit Boot voranstellen
  uint32_t ms  = millis();
  uint32_t sec = ms / 1000;
  char tsbuf[12];
  snprintf(tsbuf, sizeof(tsbuf), "%02lu:%02lu.%03lu ",
           sec / 60, sec % 60, ms % 1000);

  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  int len = strlen(buf);
  while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) buf[--len] = '\0';

  // Zeitstempel + Nachricht zusammensetzen
  char entry[270];
  snprintf(entry, sizeof(entry), "%s%s", tsbuf, buf);

  Serial.println(entry);
  logLines[logHead] = String(entry);
  logHead = (logHead + 1) % LOG_BUF_LINES;
  logTotal++;
  if (sseConnected && sseClient.connected()) {
    sseClient.print("event: log\ndata: ");
    sseClient.print(entry);
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
  json += "\"conflict\":" + String(conflictActive ? "true" : "false") + ",";
  json += "\"civReleased\":" + String(civPortReleased ? "true" : "false") + ",";
#if FLEX_EMULATION_ENABLED
  { uint8_t _fc=0; for(uint8_t i=0;i<FLEX_MAX_CLIENTS;i++) if(flexClients[i].connected)_fc++;
  json += "\"flexConn\":" + String(_fc > 0 ? "true" : "false") + ","; }
  json += "\"flexPtt\":" + String(flexTxActive ? "true" : "false");
#else
  json += "\"flexConn\":false,\"flexPtt\":false";
#endif
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

#if DISPLAY_ENABLED
  displaySetup();
#endif

#if FLEX_EMULATION_ENABLED
  flexSetup();
#endif

  webServer.on("/",       HTTP_GET, handleWebRoot);
  webServer.on("/api",    HTTP_GET, handleApiJson);
  webServer.on("/events", HTTP_GET, handleSSE);
  webServer.on("/log",    HTTP_GET, handleLogExport);
  webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });
}

// ═══════════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  if (ethConnected) {
    webServer.handleClient();
    handleSSEClient();
#if FLEX_EMULATION_ENABLED
    flexLoop();
#endif
  }
  if (!ethConnected) return;

  if (!agFound)     { discoverAntennaGenius(); return; }
  if (!agConnected) { connectToAG();           return; }

  handleAGReceive();   // nicht-blockierend AG-Daten lesen & verarbeiten

#if DISPLAY_ENABLED
  // Encoder auch während Config-Phase auswerten (Menü immer bedienbar)
  menuHandleEncoder();
#endif

  if (!agConfigDone) return;

  handleCIV();
  sendKeepalive();
  pollPortStatus();

#if DISPLAY_ENABLED
  displayLoop();
#endif
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

  static uint8_t failCount = 0;

  uint16_t port = (agTcpPort > 0) ? agTcpPort : AG_PORT;
  webLog("[AG] Verbinde mit %s:%d ...", agIPStr, port);
  snprintf(agStatusStr, sizeof(agStatusStr), "Verbinde...");

  if (!agClient.connect(agIP, port)) {
    failCount++;
    webLog("[AG] Verbindung fehlgeschlagen (Versuch %d)", failCount);
    snprintf(agStatusStr, sizeof(agStatusStr), "Fehler");
    // Nach 5 fehlgeschlagenen Versuchen AG neu suchen
    if (failCount >= 5) {
      webLog("[AG] Zu viele Fehler – suche AG neu via Broadcast");
      failCount  = 0;
      agFound    = false;
      agConnected = false;
    }
    return;
  }

  failCount = 0;
  agClient.setNoDelay(true);
  agConnected  = false;
  agConfigDone = false;
  agCfgState   = AGCFG_IDLE;
  agLineBuf    = "";
  agSeq        = 1;
  lastPing     = millis();

  webLog("[AG] TCP verbunden, warte auf Prologue...");
  snprintf(agStatusStr, sizeof(agStatusStr), "Prologue...");
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

  // Daten lesen – solange etwas verfügbar ist
  bool gotData = false;
  while (agClient.available()) {
    gotData = true;
    char c = agClient.read();
    if (c == '\r') continue;
    if (c != '\n') { agLineBuf += c; continue; }
    String line = agLineBuf;
    agLineBuf = "";
    if (line.length() == 0) continue;
    agProcessLine(line);
  }

  // Verzögerter Config-Start nach Prologue (ersetzt delay(200), nicht-blockierend)
  if (agConfigStartAt > 0 && millis() >= agConfigStartAt) {
    agConfigStartAt = 0;
    agStartConfig();
    return;
  }

  // Timeout nur prüfen wenn KEINE Daten kamen und wir auf eine Antwort warten.
  // Timeout-Uhr läuft erst ab dem Moment wo das Kommando gesendet wurde
  // (wird in agStartConfig/agHandleConfigResponse gesetzt).
  if (!gotData &&
      agCfgState != AGCFG_IDLE && agCfgState != AGCFG_DONE &&
      agCfgTimeout > 0 && millis() > agCfgTimeout) {

    webLog("[AG] TIMEOUT Config-Schritt %d – ueberspringe", (int)agCfgState);
    agCfgTimeout = 0;  // Einmalig auslösen, nicht wiederholen

    switch (agCfgState) {
      case AGCFG_WAIT_SUB_PORT:
        webLog("[AG] Ueberspringe sub port – starte band list");
        agCfgState   = AGCFG_WAIT_BAND_LIST;
        sendAGCommand("band list");
        agCfgSeq     = agSeq - 1;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
        break;
      case AGCFG_WAIT_BAND_LIST:
        webLog("[AG] Ueberspringe band list – starte antenna list");
        agCfgState   = AGCFG_WAIT_ANTENNA_LIST;
        sendAGCommand("antenna list");
        agCfgSeq     = agSeq - 1;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
        break;
      case AGCFG_WAIT_ANTENNA_LIST:
        webLog("[AG] Ueberspringe antenna list – Config abgeschlossen");
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
}

// ─── Einzelne AG-Zeile verarbeiten ────────────────────────────────────────

void agProcessLine(const String &line) {
  // Prologue: "V4.0.22 AG" — kommt direkt nach Connect
  if (line.startsWith("V") && line.indexOf(" AG") > 0) {
    char fw[16] = {0};
    sscanf(line.c_str() + 1, "%15s", fw);
    snprintf(agFwStr, sizeof(agFwStr), "%s", fw);
    webLog("[AG] Prologue: %s", line.c_str());
    // 200 ms Pause NICHT blockierend: agConfigStartAt setzen,
    // handleAGReceive() prüft diesen Timer in jedem Loop-Durchlauf
    agConfigStartAt = millis() + 200;
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
//  Port-Status kommt automatisch als S0|port nach sub port all.

void agStartConfig() {
  webLog("[AG] Starte Konfigurationssequenz...");
  snprintf(agStatusStr, sizeof(agStatusStr), "Konfiguriere");
  bandCount    = 0;
  antennaCount = 0;
  memset(bands,    0, sizeof(bands));
  memset(antennas, 0, sizeof(antennas));
  agCfgState   = AGCFG_WAIT_SUB_PORT;
  sendAGCommand("sub port all");
  agCfgSeq     = agSeq - 1;  // agSeq wurde in sendAGCommand bereits inkrementiert
  agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
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
      if (isError)
        webLog("[AG] sub port Warnung – fahre fort");
      else
        webLog("[AG] sub port all OK");
      agCfgState = AGCFG_WAIT_BAND_LIST;
      sendAGCommand("band list");
      agCfgSeq     = agSeq - 1;
      agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
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
        sendAGCommand("antenna list");
        agCfgSeq     = agSeq - 1;
        agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;
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
        // Port-Status wurde bereits via S0|port nach sub port all empfangen.
        // Zusaetzliche port get Abfragen werden im laufenden Betrieb durch
        // pollPortStatus() alle PORT_POLL_MS Millisekunden erledigt.
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
    agCfgState = AGCFG_WAIT_ANTENNA_LIST;
    sendAGCommand("antenna list");
    agCfgSeq   = agSeq - 1;
    return;
  }

  // output/group/relay: nur loggen
  webLog("[AG] Status ignoriert: %s", msg.c_str());
}

// ─── AG-Kommando senden ────────────────────────────────────────────────────

void sendAGCommand(const char *cmd) {
  if (!agClient.connected()) return;
  char buf[128];
  // Protokoll-Doku: Terminator = 0x0D (\r)
  // Einige AG-Firmwareversionen erwarten \r\n — wir senden beides zur Sicherheit
  snprintf(buf, sizeof(buf), "C%d|%s\r\n", agSeq, cmd);
  agClient.write((const uint8_t*)buf, strlen(buf));
  agClient.flush();
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

// ─── Antenne freigeben (Port auf "kein Band / keine Antenne" zurücksetzen) ──
// Wird aufgerufen wenn der CI-V Bus seit CIV_TIMEOUT_MS keine Frequenz mehr
// geliefert hat → TRX vermutlich abgeschaltet.
// Setzt Port auf band=0 rxant=0 txant=0 und aktiviert zur Sicherheit GPIO4.

void releaseAGPort() {
  webLog("[CIV] Timeout: TRX nicht mehr aktiv – gebe Port %d frei", AG_RADIO_PORT);

  lastBandId   = -1;
  civFreqMHz   = 0.0;
  snprintf(civFreqStr, sizeof(civFreqStr), "---");

  // Port auf Nullzustand setzen: kein Band, keine Antenne
  if (agConnected) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd),
             "port set %d auto=1 source=AUTO band=0 rxant=0 txant=0",
             AG_RADIO_PORT);
    sendAGCommand(cmd);
    webLog("[AG] Port %d freigegeben (auto=1 source=AUTO band=0 ant=0)", AG_RADIO_PORT);
  }

  // GPIO4 HIGH + CI-V TX Inhibit als Sicherheitssperre
  setConflict(true, "TRX Timeout");

  ssePushStatus();
}

// ─── CI-V TX Inhibit senden (Cmd 0x16, Subcmd 0x66) ──────────────────────
// Wird zusätzlich zu GPIO4 gesendet um neuere Icom-TRX hardwareseitig
// am Senden zu hindern. Ältere Transceiver ignorieren das unbekannte
// Subkommando stillschweigend.
//
// Frame: FE FE <trx_addr> E0 16 66 <00/01> FD
//   00 = TX Inhibit OFF (TRX darf senden)
//   01 = TX Inhibit ON  (TRX gesperrt)
//
// civAddr muss bekannt sein (wird aus dem ersten empfangenen CI-V-Frame
// gelernt). Ist civAddr noch 0x00 (TRX unbekannt), wird kein Frame gesendet.

void sendCIVTxInhibit(bool inhibit) {
  if (civAddr == 0x00) {
    webLog("[CIV] TX Inhibit nicht sendbar: TRX-Adresse unbekannt");
    return;
  }
  uint8_t frame[8] = {
    CIV_PREAMBLE, CIV_PREAMBLE,   // FE FE
    civAddr,                       // Zieladresse (TRX)
    CIV_CONTROLLER,                // Quelladresse (wir)
    0x16,                          // Command: Set various functions
    0x66,                          // Subcommand: TX Inhibit
    inhibit ? 0x01 : 0x00,         // Data: 01=ON, 00=OFF
    CIV_EOM                        // FD
  };
  Serial2.write(frame, sizeof(frame));
  webLog("[CIV] TX Inhibit %s gesendet (Cmd 16/66 an Adresse 0x%02X)",
         inhibit ? "ON" : "OFF", civAddr);
}

// ─── Zentrale Konflikt/Sicherheitssperre ─────────────────────────────────
// Einzige Stelle die GPIO4 und CI-V TX Inhibit setzt — beide immer synchron.

void setConflict(bool active, const char *reason) {
  if (conflictActive == active) return;
  conflictActive = active;
  digitalWrite(CONFLICT_PIN, active ? HIGH : LOW);
  if (active)
    webLog("[CONF] Sperre aktiv:     GPIO%d=HIGH (%s)", CONFLICT_PIN, reason);
  else
    webLog("[CONF] Sperre aufgehoben: GPIO%d=LOW  (%s)", CONFLICT_PIN, reason);
  sendCIVTxInhibit(active);
#if DISPLAY_ENABLED
  displaySetInvert(active);
  dispNeedsRedraw = true;
#endif
  ssePushStatus();
}

void checkAndSignalConflict(uint8_t wantedAntId) {
  uint8_t otherIdx   = (AG_RADIO_PORT == 1) ? 1 : 0;
  uint8_t otherTxAnt = portTxAnt[otherIdx];
  bool conflict = (wantedAntId != 0) && (otherTxAnt == wantedAntId);
  // setConflict() hat internen Frühausstieg bei gleichem Zustand —
  // explizite Prüfung hier verhindert dennoch unnötigen Log-Eintrag
  // beim ersten Bandwechsel wenn noch gar kein Konflikt vorlag.
  if (conflict)
    setConflict(true,  "Antenne von anderem Port belegt");
  else if (conflictActive)
    setConflict(false, "Konflikt aufgeloest");
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

  // CI-V-Timeout: TRX seit CIV_TIMEOUT_MS nicht mehr gehört → Port freigeben
  // Gilt nur wenn zuvor eine Adresse gelernt und ein Band geschaltet wurde.
  if (civAddr != 0x00 && lastCivRx > 0 &&
      millis() - lastCivRx > CIV_TIMEOUT_MS &&
      !civPortReleased) {
    civPortReleased = true;
    releaseAGPort();
    return;
  }

  // Aktives Polling wenn seit CIV_BROADCAST_TIMEOUT_MS kein Broadcast —
  // aber nur solange der Port NICHT bereits wegen Timeout freigegeben wurde
  if (!civPortReleased &&
      civAddr != 0x00 &&
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

  // War der Port wegen Timeout freigegeben? Dann Sicherheitssperre aufheben
  // und normalen Betrieb wieder aufnehmen
  if (civPortReleased) {
    civPortReleased = false;
    setConflict(false, "TRX wieder aktiv");
  }
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

// ─── Log-Export als Textdatei ─────────────────────────────────────────────
// GET /log  →  alle Einträge als text/plain, UTF-8
// Content-Disposition: attachment → Browser öffnet Speichern-Dialog

void handleLogExport() {
  char fname[40];
  snprintf(fname, sizeof(fname), "wt32-bridge-%lus.log", millis() / 1000);

  // Inhalt vollständig aufbauen — ESP32 WebServer sendet alles in einem HTTP-Response
  String body;
  body.reserve(8192);

  char hdr[256];
  snprintf(hdr, sizeof(hdr),
    "# WT32-ETH01 CI-V -> Antenna Genius Bridge\r\n"
    "# IP: %s  Uptime: %lu s  Eintraege: %u\r\n"
    "# AG: %s (%s)  FW: %s\r\n"
    "#\r\n",
    ETH.localIP().toString().c_str(),
    millis() / 1000,
    (logTotal < LOG_BUF_LINES) ? logTotal : (uint16_t)LOG_BUF_LINES,
    agNameStr, agIPStr, agFwStr);
  body += hdr;

  uint16_t count = (logTotal < LOG_BUF_LINES) ? logTotal : LOG_BUF_LINES;
  uint8_t  start = (logTotal < LOG_BUF_LINES) ? 0 : logHead;
  for (uint16_t i = 0; i < count; i++) {
    uint8_t idx = (start + i) % LOG_BUF_LINES;
    body += logLines[idx];
    body += "\r\n";
  }

  webServer.sendHeader("Content-Disposition",
                       String("attachment; filename=\"") + fname + "\"");
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "text/plain; charset=utf-8", body);
  webLog("[WEB] Log-Export: %d Zeilen als %s (%u Bytes)", count, fname, body.length());
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
  json += "\"civReleased\":" + String(civPortReleased ? "true" : "false") + ",";
#if FLEX_EMULATION_ENABLED
  { uint8_t _fc=0; for(uint8_t i=0;i<FLEX_MAX_CLIENTS;i++) if(flexClients[i].connected)_fc++;
  json += "\"flexConn\":" + String(_fc > 0 ? "true" : "false") + ","; }
  json += "\"flexPtt\":" + String(flexTxActive ? "true" : "false") + ",";
#else
  json += "\"flexConn\":false,\"flexPtt\":false,";
#endif
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
    <div class="kv"><span class="label">TRX-Status</span><span id="civReleasedWrap"><span class="badge badge-muted">---</span></span></div>
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
    <div class="kv"><span class="label">FlexRadio</span><span id="flexConnWrap"><span class="badge badge-muted">---</span></span></div>
    <div class="kv"><span class="label">PTT-Interlock</span><span id="flexPttWrap"><span class="badge badge-muted">---</span></span></div>
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
  <h2>Debug-Log <span id="sseState" style="font-size:.7rem;margin-left:8px"></span>
    <a href="/log" download style="float:right;font-size:.75rem;padding:3px 10px;background:var(--border);color:var(--fg);border-radius:4px;text-decoration:none;border:1px solid var(--muted)">&#11015; Log herunterladen</a>
  </h2>
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
  // TRX-Status (civReleased = Port wurde wegen Timeout freigegeben)
  const crw=document.getElementById('civReleasedWrap');
  if(crw){
    if(d.civReleased)
      crw.innerHTML=`<span class="badge badge-red">&#128308; TRX inaktiv &ndash; Sperre aktiv</span>`;
    else if(d.civAddr&&d.civAddr!=='---')
      crw.innerHTML=`<span class="badge badge-green">&#128994; TRX aktiv</span>`;
    else
      crw.innerHTML=`<span class="badge badge-muted">---</span>`;
  }
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
  // FlexRadio-Verbindungsstatus
  const fcw=document.getElementById('flexConnWrap');
  if(fcw){
    if(d.flexConn)
      fcw.innerHTML=`<span class="badge badge-green">&#10003; Verbunden</span>`;
    else
      fcw.innerHTML=`<span class="badge badge-muted">nicht verbunden</span>`;
  }
  // PTT-Interlock-Status
  const fpw=document.getElementById('flexPttWrap');
  if(fpw){
    if(d.flexPtt)
      fpw.innerHTML=`<span class="badge badge-tx">&#128308; PTT_REQUESTED</span>`;
    else if(d.flexConn)
      fpw.innerHTML=`<span class="badge badge-green">READY</span>`;
    else
      fpw.innerHTML=`<span class="badge badge-muted">---</span>`;
  }
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

// ═══════════════════════════════════════════════════════════════════════════
//  Display & Encoder (nur aktiv wenn DISPLAY_ENABLED = 1)
// ═══════════════════════════════════════════════════════════════════════════

#if DISPLAY_ENABLED

// ─── Hilfsfunktionen ──────────────────────────────────────────────────────

// Gibt die Antennen-ID zurück die aktuell auf dem betrachteten Radio-Port liegt
static uint8_t currentAntId() {
  return portTxAnt[AG_RADIO_PORT - 1];
}

// Kürzt einen String auf maxLen Zeichen + '\0' und fügt '..' an wenn nötig
static void truncStr(char *dst, const char *src, uint8_t maxLen) {
  strncpy(dst, src, maxLen);
  dst[maxLen] = '\0';
  if (strlen(src) > maxLen) {
    dst[maxLen-1] = '.';
    dst[maxLen-2] = '.';
  }
}

// ─── Display-Inversion (Hardware-Befehl ST7565) ───────────────────────────
// 0xA7 = Display invertieren  (Fehlerfall: Pixel-Inversion auf Controller-Ebene)
// 0xA6 = Normaldarstellung    (Fehler behoben)
// Kein Neuzeichnen nötig — wirkt sofort auf den sichtbaren Displayinhalt.

void displaySetInvert(bool invert) {
  u8g2.sendF("c", invert ? 0xA7 : 0xA6);
}

// ─── Setup ────────────────────────────────────────────────────────────────

void displaySetup() {
  // Encoder-Pins
  // GPIO35/36/39: input-only, kein interner Pull-up → ext. 10 kΩ an 3.3 V nötig
  pinMode(ENC_A_PIN,   INPUT);   // ext. Pull-up erforderlich
  pinMode(ENC_B_PIN,   INPUT);   // ext. Pull-up erforderlich
  pinMode(ENC_BTN_PIN, INPUT);   // ext. Pull-up erforderlich

  // ISR auf beide Flanken von Kanal A (GPIO39)
  // Kanal B (GPIO36) wird per Polling gelesen um 36/39-Crosstalk zu vermeiden
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, CHANGE);

  // Display initialisieren
  u8g2.begin();
  u8g2.setContrast(DISP_CONTRAST);
  u8g2.setFontMode(1);   // transparenter Hintergrund

  // Begrüßungsbildschirm mit Splash-Bitmap
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(0, 0, SPLASH_W, SPLASH_H, splashBitmap);
  } while (u8g2.nextPage());

  menuLastAction = millis();
  webLog("[DISP] Display initialisiert (DOGL128W-6, ST7565)");
}

// ─── Hauptschleife Display ─────────────────────────────────────────────────

void displayLoop() {
  // Menü-Timeout → zurück zur Statusanzeige
  if (menuState != MENU_STATUS &&
      millis() - menuLastAction > MENU_TIMEOUT_MS) {
    menuState     = MENU_STATUS;
    dispNeedsRedraw = true;
  }

  if (!dispNeedsRedraw) return;
  dispNeedsRedraw = false;

  switch (menuState) {
    case MENU_STATUS:     displayDrawStatus();   break;
    case MENU_ANT_SELECT: displayDrawAntMenu();  break;
    case MENU_CONFIRM:    displayDrawConfirm();  break;
  }
}

// ─── Statusanzeige (Normalanzeige) ────────────────────────────────────────
//
// ┌────────────────────────────────┐
// │ 14.225.000 MHz                 │  ← Frequenz (groß)
// │ Band: 20m                      │
// │ Ant:  Beam_OB11-3              │
// │────────────────────────────────│
// │ AG: OK  Port A  [kein Konflik] │
// └────────────────────────────────┘

void displayDrawStatus() {
  char buf[24];

  u8g2.firstPage();
  do {
    // ── Zeile 1: Frequenz ──
    u8g2.setFont(u8g2_font_7x14B_tf);
    if (civFreqMHz > 0.001) {
      // Format: 14.225.000 MHz
      uint32_t kHz = (uint32_t)(civFreqMHz * 1000.0 + 0.5);
      uint16_t mhz  = kHz / 1000;
      uint16_t frac = kHz % 1000;
      snprintf(buf, sizeof(buf), "%u.%03u MHz", mhz, frac);
    } else {
      strcpy(buf, "--- MHz");
    }
    u8g2.drawStr(0, 13, buf);

    u8g2.setFont(u8g2_font_6x10_tf);

    // ── Zeile 2: Band ──
    if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid) {
      snprintf(buf, sizeof(buf), "Band: %s", bands[lastBandId].name);
    } else {
      strcpy(buf, "Band: ---");
    }
    u8g2.drawStr(0, 26, buf);

    // ── Zeile 3: Antenne ──
    uint8_t aid = currentAntId();
    char antName[18] = "---";
    if (aid > 0 && aid <= MAX_ANTENNAS && antennas[aid-1].valid)
      truncStr(antName, antennas[aid-1].name, 17);
    snprintf(buf, sizeof(buf), "Ant:  %s", antName);
    u8g2.drawStr(0, 37, buf);

    // ── Trennlinie ──
    u8g2.drawHLine(0, 41, 128);

    // ── Zeile 4: Status ──
    u8g2.setFont(u8g2_font_5x7_tf);

    // AG-Status links
    snprintf(buf, sizeof(buf), "AG:%s", agStatusStr);
    u8g2.drawStr(0, 50, buf);

    // Konflikt rechts
    if (conflictActive) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(80, 43, 48, 9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(82, 51, "KONFLIKT");
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(82, 51, "kein Konfl.");
    }

    // ── Zeile 5: Hinweis auf Menü ──
    if (agConfigDone) {
      u8g2.drawStr(0, 62, "Drehen: Ant wahlen  [Btn]");
    } else {
      snprintf(buf, sizeof(buf), "IP: %s", agIPStr[0] ? agIPStr : "...");
      u8g2.drawStr(0, 62, buf);
    }

  } while (u8g2.nextPage());
}

// ─── Antennen-Auswahlmenü ─────────────────────────────────────────────────
//
// ┌────────────────────────────────┐
// │ Antenne wählen [20m]           │
// │ > Beam_OB11-3          [aktiv] │
// │   Dipol_160-80-40              │
// │   GP_10m                       │
// │────────────────────────────────│
// │ [Btn] = Auswählen   Drehgeber  │
// └────────────────────────────────┘

// Maximale Anzahl Einträge die gleichzeitig sichtbar sind
#define ANT_MENU_ROWS  4

void displayDrawAntMenu() {
  char buf[22];
  uint8_t curAntId = currentAntId();

  // Scroll-Offset: menuCursor immer im sichtbaren Bereich halten
  static uint8_t scrollOffset = 0;
  if (menuCursor < scrollOffset)
    scrollOffset = menuCursor;
  if (menuCursor >= scrollOffset + ANT_MENU_ROWS)
    scrollOffset = menuCursor - ANT_MENU_ROWS + 1;

  u8g2.firstPage();
  do {
    // ── Titelzeile ──
    u8g2.setFont(u8g2_font_6x10_tf);
    char bandName[8] = "---";
    if (lastBandId >= 0 && lastBandId < MAX_BANDS && bands[lastBandId].valid)
      strncpy(bandName, bands[lastBandId].name, 7);
    snprintf(buf, sizeof(buf), "Ant wahlen [%s]", bandName);
    u8g2.drawStr(0, 9, buf);
    u8g2.drawHLine(0, 11, 128);

    // ── Antennen-Liste ──
    u8g2.setFont(u8g2_font_5x7_tf);
    for (uint8_t i = 0; i < ANT_MENU_ROWS; i++) {
      uint8_t listIdx = scrollOffset + i;
      if (listIdx >= menuAntCount) break;

      uint8_t aid = menuAntIds[listIdx];
      uint8_t y   = 22 + i * 10;
      bool    isSelected = (listIdx == menuCursor);
      bool    isActive   = (aid == curAntId);

      if (isSelected) {
        u8g2.drawBox(0, y - 8, 128, 9);
        u8g2.setDrawColor(0);
      }

      // Cursor-Pfeil
      u8g2.drawStr(0, y, isSelected ? ">" : " ");

      // Antennenname (max 15 Zeichen)
      char aname[16] = "---";
      if (aid > 0 && aid <= MAX_ANTENNAS && antennas[aid-1].valid)
        truncStr(aname, antennas[aid-1].name, 15);
      u8g2.drawStr(6, y, aname);

      // [akt] Markierung wenn das die aktuell geschaltete Antenne ist
      if (isActive) {
        u8g2.drawStr(96, y, "[akt]");
      }

      if (isSelected) u8g2.setDrawColor(1);
    }

    // ── Scrollbalken ──
    if (menuAntCount > ANT_MENU_ROWS) {
      uint8_t barH = (ANT_MENU_ROWS * 40) / menuAntCount;
      uint8_t barY = 12 + (scrollOffset * 40) / menuAntCount;
      u8g2.drawBox(126, 12, 2, barH);
      u8g2.drawFrame(126, barY, 2, barH);
    }

    // ── Fußzeile ──
    u8g2.drawHLine(0, 53, 128);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 63, "[Btn]=Wahlen  Drehen=Navig.");

  } while (u8g2.nextPage());
}

// ─── Bestätigungsdialog ────────────────────────────────────────────────────

void displayDrawConfirm() {
  if (menuCursor >= menuAntCount) { menuState = MENU_ANT_SELECT; return; }
  uint8_t aid = menuAntIds[menuCursor];
  char aname[20] = "---";
  if (aid > 0 && aid <= MAX_ANTENNAS && antennas[aid-1].valid)
    truncStr(aname, antennas[aid-1].name, 19);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Antenne schalten?");
    u8g2.drawHLine(0, 13, 128);
    u8g2.drawStr(4, 26, aname);

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 40, "Port A wird umgeschaltet.");

    u8g2.drawHLine(0, 50, 128);
    // Zwei Buttons: [JA] links, [NEIN] rechts
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr( 4, 63, "[Btn]=JA");
    u8g2.drawStr(72, 63, "Drehen=NEIN");
  } while (u8g2.nextPage());
}

// ─── Antennen-Liste für aktuelles Band aufbauen ────────────────────────────

void menuBuildAntList() {
  menuAntCount = 0;
  if (lastBandId < 0) {
    // Kein Band → alle gültigen Antennen anzeigen
    for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
      if (antennas[i].valid && menuAntCount < MAX_ANTENNAS)
        menuAntIds[menuAntCount++] = antennas[i].id;
    }
    return;
  }
  uint16_t bandBit = (1u << lastBandId);
  for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
    if (!antennas[i].valid) continue;
    if (antennas[i].txMask & bandBit) {
      menuAntIds[menuAntCount++] = antennas[i].id;
    }
  }
  // Fallback: wenn keine Antenne für das Band konfiguriert, alle zeigen
  if (menuAntCount == 0) {
    for (uint8_t i = 0; i < MAX_ANTENNAS; i++) {
      if (antennas[i].valid && menuAntCount < MAX_ANTENNAS)
        menuAntIds[menuAntCount++] = antennas[i].id;
    }
  }
}

// ─── Antenne tatsächlich schalten ─────────────────────────────────────────

void menuSelectAntenna(uint8_t antId) {
  if (antId == 0 || antId > MAX_ANTENNAS) return;
  uint8_t bandId = (lastBandId >= 0) ? (uint8_t)lastBandId : portBand[AG_RADIO_PORT - 1];
  webLog("[DISP] Manuelle Antennenwahl: Ant %d (%s)",
         antId,
         (antId <= MAX_ANTENNAS && antennas[antId-1].valid) ? antennas[antId-1].name : "?");
  checkAndSignalConflict(antId);
  setAGPort(AG_RADIO_PORT, bandId, antId);
  ssePushStatus();
}

// ─── Encoder & Taster auswerten ───────────────────────────────────────────

void menuHandleEncoder() {
  // ── Drehgeber ──
  int8_t delta = 0;
  noInterrupts();
  delta    = encDelta;
  encDelta = 0;
  interrupts();

  // Nur auf volle Rastschritte reagieren (je nach Encoder 2 oder 4 Pulse/Raste)
  static int8_t accumulator = 0;
  accumulator += delta;
  int8_t steps = accumulator / 2;   // bei 2 Pulsen/Raste; bei 4 Pulsen → / 4
  accumulator %= 2;

  if (steps != 0) {
    menuLastAction  = millis();
    dispNeedsRedraw = true;

    switch (menuState) {
      case MENU_STATUS:
        // Drehen öffnet Antennen-Menü
        menuBuildAntList();
        // Cursor auf aktuelle Antenne setzen
        menuCursor = 0;
        for (uint8_t i = 0; i < menuAntCount; i++) {
          if (menuAntIds[i] == currentAntId()) { menuCursor = i; break; }
        }
        menuState = MENU_ANT_SELECT;
        break;

      case MENU_ANT_SELECT:
        if (steps > 0) {
          if (menuCursor + 1 < menuAntCount) menuCursor++;
        } else {
          if (menuCursor > 0) menuCursor--;
        }
        break;

      case MENU_CONFIRM:
        // Jede Drehbewegung → Abbrechen
        menuState = MENU_ANT_SELECT;
        break;
    }
  }

  // ── Taster ──
  static bool     btnLastState = HIGH;
  static uint32_t btnPressTime = 0;
  bool btnNow = digitalRead(ENC_BTN_PIN);  // LOW = gedrückt (ext. Pull-up)

  if (btnNow == LOW && btnLastState == HIGH) {
    btnPressTime = millis();               // Flanke erkannt
  }
  if (btnNow == HIGH && btnLastState == LOW) {
    if (millis() - btnPressTime >= ENC_DEBOUNCE_MS) {
      // Gültiger Tastendruck
      menuLastAction  = millis();
      dispNeedsRedraw = true;

      switch (menuState) {
        case MENU_STATUS:
          // Knopf im Status-Screen → Menü öffnen
          menuBuildAntList();
          menuCursor = 0;
          for (uint8_t i = 0; i < menuAntCount; i++) {
            if (menuAntIds[i] == currentAntId()) { menuCursor = i; break; }
          }
          menuState = MENU_ANT_SELECT;
          break;

        case MENU_ANT_SELECT:
          // Antenne vorgewählt → Bestätigungsdialog
          menuState = MENU_CONFIRM;
          break;

        case MENU_CONFIRM:
          // Bestätigt → Antenne schalten, zurück zur Statusanzeige
          if (menuCursor < menuAntCount)
            menuSelectAntenna(menuAntIds[menuCursor]);
          menuState = MENU_STATUS;
          break;
      }
    }
  }
  btnLastState = btnNow;
}

#endif // DISPLAY_ENABLED

// ═══════════════════════════════════════════════════════════════════════════
//  FlexRadio-Emulation
//  Emuliert das minimal nötige FlexRadio-Protokoll damit der Antenna Genius
//  unseren CI-V-Port als "FLEX"-Radio erkennt und bei aktivem TRX tx=1 setzt.
//
//  Protokoll-Ablauf:
//  1. WT32 sendet alle 2 s einen VITA-49 Discovery-Broadcast (UDP 4992)
//  2. AG empfängt Broadcast → erkennt "FlexRadio" im Netz
//  3. AG baut TCP-Verbindung auf Port 4992 auf
//  4. WT32 sendet Prologue: Versionslinie + Handle
//  5. AG sendet "sub tx all" → WT32 antwortet mit aktuellem Interlock-Status
//  6. Bei CI-V-Aktivität: state=PTT_REQUESTED/TRANSMITTING
//     Bei CI-V-Timeout:   state=READY (→ AG gibt Port frei für SmartSDR)
// ═══════════════════════════════════════════════════════════════════════════

#if FLEX_EMULATION_ENABLED

// ─── VITA-49 Discovery-Broadcast ─────────────────────────────────────────
// Paketstruktur (Big-Endian, 32-Bit-Wörter):
//  Word 0: Header   (pkt_type=3=ExtDataWithStream, c=1, t=1, tsi=0, tsf=0)
//  Word 1: Stream ID (0x00000000 für Discovery)
//  Word 2: OUI      (0x001C2D00 = FlexRadio OUI in oberen 24 Bit)
//  Word 3: Class    (InformationClassCode=0x0000, PacketClassCode=0xFFFF)
//  Word 4..N: Payload als UTF-8 ASCII, auf 32-Bit-Wortgrenze gepaddet

void flexSendDiscovery() {
  // Payload zusammenbauen
  char localIP[20];
  snprintf(localIP, sizeof(localIP), "%s", ETH.localIP().toString().c_str());

  char payload[256];
  int plen = snprintf(payload, sizeof(payload),
    "model=%s ip=%s port=%d serial=%s status=Available "
    "version=%s nickname=%s max_panadapters=1 max_slices=2 "
    "available_panadapters=1 available_slices=2 "
    "inuse_ip= inuse_host= discovery_protocol_version=3.3.15",
    FLEX_MODEL, localIP, FLEX_TCP_PORT,
    FLEX_SERIAL, FLEX_VERSION, FLEX_NICKNAME);

  // Payload auf 32-Bit-Wortgrenze auffüllen (Space-Padding)
  while (plen % 4 != 0) payload[plen++] = ' ';
  payload[plen] = '\0';

  // packet_size in 32-Bit-Wörtern:
  // 1 (Header) + 1 (StreamID) + 2 (ClassID) + plen/4 (Payload)
  uint16_t pkt_size = 1 + 1 + 2 + (plen / 4);

  // Header-Word aufbauen
  // pkt_type=3 (ExtDataWithStream), c=1, t=0, tsi=0, tsf=0
  uint32_t hdr = ((uint32_t)3 << 28)      // pkt_type
               | ((uint32_t)1 << 27)      // c (ClassID present)
               | ((uint32_t)flexPktCount++ << 16) // packet_count (4 bit, rollt über)
               | pkt_size;
  flexPktCount &= 0x0F;

  uint8_t buf[320];
  int idx = 0;

  // Word 0: Header (Big-Endian)
  buf[idx++] = (hdr >> 24) & 0xFF;
  buf[idx++] = (hdr >> 16) & 0xFF;
  buf[idx++] = (hdr >>  8) & 0xFF;
  buf[idx++] =  hdr        & 0xFF;

  // Word 1: Stream ID = 0
  buf[idx++] = 0; buf[idx++] = 0; buf[idx++] = 0; buf[idx++] = 0;

  // Word 2: OUI = 0x001C2D (FlexRadio), obere 8 Bit = 0
  buf[idx++] = 0x00;
  buf[idx++] = 0x1C;
  buf[idx++] = 0x2D;
  buf[idx++] = 0x00;

  // Word 3: InformationClassCode=0x0000, PacketClassCode=0xFFFF
  buf[idx++] = 0x00; buf[idx++] = 0x00;
  buf[idx++] = 0xFF; buf[idx++] = 0xFF;

  // Payload
  memcpy(buf + idx, payload, plen);
  idx += plen;

  // Als Broadcast senden — flexDiscUdp ist statisch, kein Socket-Open/Close nötig
  flexDiscUdp.beginPacket(IPAddress(255,255,255,255), FLEX_UDP_PORT);
  flexDiscUdp.write(buf, idx);
  flexDiscUdp.endPacket();
}

// ─── Zeile an einen bestimmten Client senden ──────────────────────────────

void flexSendLine(const char *line) {
  // Broadcast an alle verbundenen Clients (wird intern aufgerufen)
  // Direktaufruf mit Client-Index für gezielte Antworten
  for (uint8_t i = 0; i < FLEX_MAX_CLIENTS; i++) {
    if (!flexClients[i].connected) continue;
    flexClients[i].tcp.print(line);
    flexClients[i].tcp.print("\n");
    flexClients[i].tcp.flush();
  }
  webLog("[FLEX] >> %s", line);
}

void flexSendLineTo(uint8_t idx, const char *line) {
  if (!flexClients[idx].connected) return;
  flexClients[idx].tcp.print(line);
  flexClients[idx].tcp.print("\n");
  flexClients[idx].tcp.flush();
  webLog("[FLEX:%d] >> %s", idx, line);
}

// ─── Interlock-Status an alle Clients senden ─────────────────────────────

void flexSendInterlockStatus(bool transmitting) {
  char line[96];
  if (transmitting) {
    snprintf(line, sizeof(line),
      "S00000000|interlock state=PTT_REQUESTED source=SW reason= tx_allowed=1");
  } else {
    snprintf(line, sizeof(line),
      "S00000000|interlock state=READY source= reason= tx_allowed=1");
  }
  flexTxActive = transmitting;
  // An alle verbundenen Clients senden, txActive pro Client merken
  for (uint8_t i = 0; i < FLEX_MAX_CLIENTS; i++) {
    if (!flexClients[i].connected) continue;
    flexClients[i].tcp.print(line);
    flexClients[i].tcp.print("\n");
    flexClients[i].tcp.flush();
    flexClients[i].txActive = transmitting;
  }
  webLog("[FLEX] >> %s", line);
  webLog("[FLEX] Interlock: %s (civAddr=0x%02X lastCivRx=%lums ago civReleased=%d)",
         transmitting ? "PTT_REQUESTED" : "READY",
         civAddr,
         lastCivRx > 0 ? millis() - lastCivRx : 0,
         civPortReleased);
}

// ─── Eingehende TCP-Zeile verarbeiten ─────────────────────────────────────

void flexProcessLine(uint8_t idx, const char *line) {
  webLog("[FLEX:%d] << %s", idx, line);

  if (line[0] != 'C') return;
  const char *pipe = strchr(line, '|');
  if (!pipe) return;
  int seqNum = atoi(line + 1);
  const char *cmd = pipe + 1;
  char reply[64];

  // "keepalive enable" → OK, dann sofort initialen Interlock-Status senden
  if (strncmp(cmd, "keepalive", 9) == 0) {
    snprintf(reply, sizeof(reply), "R%d|00000000|", seqNum);
    flexSendLineTo(idx, reply);
    bool txNow = (!civPortReleased && civAddr != 0x00 && lastCivRx > 0);
    // Nur an diesen Client senden (er kennt unseren Status noch nicht)
    char status[96];
    if (txNow) {
      snprintf(status, sizeof(status),
        "S00000000|interlock state=PTT_REQUESTED source=SW reason= tx_allowed=1");
    } else {
      snprintf(status, sizeof(status),
        "S00000000|interlock state=READY source= reason= tx_allowed=1");
    }
    flexSendLineTo(idx, status);
    flexClients[idx].txActive = txNow;
    webLog("[FLEX:%d] Interlock nach keepalive: %s", idx,
           txNow ? "PTT_REQUESTED" : "READY");
    return;
  }

  // "client ip" → mit unserer IP antworten
  if (strncmp(cmd, "client ip", 9) == 0) {
    snprintf(reply, sizeof(reply), "R%d|00000000|%s", seqNum,
             ETH.localIP().toString().c_str());
    flexSendLineTo(idx, reply);
    return;
  }

  // "sub tx all" → Status an diesen Client, OK-Antwort
  if (strncmp(cmd, "sub tx", 6) == 0) {
    bool txNow = (!civPortReleased && civAddr != 0x00 && lastCivRx > 0);
    char status[96];
    if (txNow) {
      snprintf(status, sizeof(status),
        "S00000000|interlock state=PTT_REQUESTED source=SW reason= tx_allowed=1");
    } else {
      snprintf(status, sizeof(status),
        "S00000000|interlock state=READY source= reason= tx_allowed=1");
    }
    flexSendLineTo(idx, status);
    flexClients[idx].txActive = txNow;
    snprintf(reply, sizeof(reply), "R%d|00000000|", seqNum);
    flexSendLineTo(idx, reply);
    webLog("[FLEX:%d] sub tx: Status gesendet (%s)", idx,
           txNow ? "PTT_REQUESTED" : "READY");
    return;
  }

  // Alle anderen Kommandos mit leerem OK beantworten
  snprintf(reply, sizeof(reply), "R%d|00000000|", seqNum);
  flexSendLineTo(idx, reply);
}

// ─── TCP-Clients verarbeiten (alle Slots) ─────────────────────────────────

void flexHandleTcpClient() {
  // Neuen Client annehmen wenn ein Slot frei ist
  WiFiClient newClient = flexTcpServer.available();
  if (newClient) {
    uint8_t slot = FLEX_MAX_CLIENTS; // ungültig
    for (uint8_t i = 0; i < FLEX_MAX_CLIENTS; i++) {
      if (!flexClients[i].connected) { slot = i; break; }
    }
    if (slot < FLEX_MAX_CLIENTS) {
      flexClients[slot].tcp       = newClient;
      flexClients[slot].connected = true;
      flexClients[slot].lineBufLen= 0;
      flexClients[slot].txActive  = false;
      webLog("[FLEX:%d] Client verbunden von %s",
             slot, flexClients[slot].tcp.remoteIP().toString().c_str());
      // Prologue: Version + Handle
      flexSendLineTo(slot, "V" FLEX_VERSION);
      flexSendLineTo(slot, "H00000001");
    } else {
      webLog("[FLEX] Kein freier Client-Slot — Verbindung abgewiesen");
      newClient.stop();
    }
  }

  // Alle aktiven Clients verarbeiten
  for (uint8_t i = 0; i < FLEX_MAX_CLIENTS; i++) {
    if (!flexClients[i].connected) continue;

    // Verbindung verloren?
    if (!flexClients[i].tcp.connected()) {
      webLog("[FLEX:%d] Client getrennt", i);
      flexClients[i].tcp.stop();
      flexClients[i].connected  = false;
      flexClients[i].lineBufLen = 0;
      flexClients[i].txActive   = false;
      continue;
    }

    // Bytes lesen
    while (flexClients[i].tcp.available()) {
      char c = flexClients[i].tcp.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (flexClients[i].lineBufLen > 0) {
          flexClients[i].lineBuf[flexClients[i].lineBufLen] = '\0';
          flexProcessLine(i, flexClients[i].lineBuf);
          flexClients[i].lineBufLen = 0;
        }
        continue;
      }
      if (flexClients[i].lineBufLen < sizeof(flexClients[i].lineBuf) - 1)
        flexClients[i].lineBuf[flexClients[i].lineBufLen++] = c;
    }
  }
}

// ─── PTT-Status-Update bei Zustandsänderung senden ────────────────────────

void flexUpdatePttStatus() {
  bool txNow = (!civPortReleased && civAddr != 0x00 && lastCivRx > 0);
  if (txNow == flexTxActive) return;
  webLog("[FLEX] PTT-Status geändert: %s", txNow ? "PTT_REQUESTED" : "READY");
  flexSendInterlockStatus(txNow);
}

// ─── Setup ────────────────────────────────────────────────────────────────

void flexSetup() {
  flexTcpServer.begin();
  webLog("[FLEX] FlexRadio-Emulation aktiv (TCP Port %d, UDP Discovery Port %d)",
         FLEX_TCP_PORT, FLEX_UDP_PORT);
}

// ─── Hauptschleife ────────────────────────────────────────────────────────

void flexLoop() {
  // Discovery-Broadcast regelmäßig senden
  if (millis() - flexLastDiscovery >= FLEX_DISCOVERY_MS) {
    flexLastDiscovery = millis();
    flexSendDiscovery();
  }

  // TCP-Client verarbeiten
  flexHandleTcpClient();

  // PTT-Status bei Änderung senden
  flexUpdatePttStatus();
}

#endif // FLEX_EMULATION_ENABLED
