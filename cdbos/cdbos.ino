/*
  =====================================================================
   CDBos v11  -  Cheo Desk Buddy Operating System   (ESP32-C6 or ESP32-S3)
  =====================================================================
   NEW IN v13: say "Hey Cheo"! Wake word trained on your voice (Edge Impulse), runs on the S3 itself
   NEW IN v12: talk to Cheo! INMP441 microphone: hold his button (or the web button) and talk (S3 only)
   NEW IN v11: smooth voice (saves up ~1.5 s of voice before talking, no more stutter)
   NEW IN v10
    - Works on the ESP32-C6 AND the ESP32-S3 (picks the right pins by itself)
    - Pet mode: food, fun, energy, levels. Feed him (he loves cheese)
    - Games: guess my number, rock paper scissors, reaction test, trivia, riddles
    - Countdowns: "how many days until Christmas?"
    - Morning briefing (weather, schedule, news) + rain warnings
    - Answer or decline iPhone calls with his button
    - Eye colors (even rainbow), night mode, party mode, 4 more songs, new dances
    - Cheple link: the Cheple phone can control Cheo (cheo.local/api/...)

   BOARD SETTINGS (Tools menu)
    ESP32-C6: ESP32C6 Dev Module, Flash 4MB,
              Partition "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
    ESP32-S3: ESP32S3 Dev Module, Flash 16MB (or 8MB),
              Partition "16M Flash (3MB APP/9.9MB FATFS)", Flash 16MB, PSRAM "OPI PSRAM"
    Libraries: Adafruit GFX, Adafruit ST7735 and ST7789, ArduinoJson 7, NimBLE-Arduino 2.x
  =====================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ESP_I2S.h>
#include <NimBLEDevice.h>
#include <Wire.h>

// =====================================================================
//  SETTINGS
// =====================================================================
#define FW_VERSION 17                      // bump this every GitHub release
#define OS_NAME "CDBos"
#if __has_include("build_num.h")
#include "build_num.h"                     // GitHub writes this when it builds the firmware
#endif
#ifndef BUILD_NUM
#define BUILD_NUM 0
#endif
const long FW_BUILD = FW_VERSION * 1000L + BUILD_NUM;   // what version.txt holds
#if CONFIG_IDF_TARGET_ESP32S3
  #define BOARD_NAME "ESP32-S3"
  const char* FW_FILE = "cheo-s3.bin";     // GitHub file for S3 Cheos
#else
  #define BOARD_NAME "ESP32-C6"
  const char* FW_FILE = "cheo.bin";        // GitHub file for C6 Cheos
#endif
const char* UPDATE_URL = "https://raw.githubusercontent.com/cheplestore/cheo/main/";  // holds version.txt, cheo.bin, cheo-s3.bin
const bool  AUTO_UPDATE = true;            // check GitHub by himself (1 min after boot, then every 6 h)

// ---- FIRST USB UPLOAD ONLY ------------------------------------------
// Fill these in, upload by USB once, and Cheo saves them inside himself.
// Then BLANK THEM ("") before you export the .bin for GitHub, so your
// passwords and key never go online. Change them later at cheo.local/settings
const char* SETUP_WIFI_SSID    = "";
const char* SETUP_WIFI_PASS    = "";
const char* SETUP_HOTSPOT_SSID = "";
const char* SETUP_HOTSPOT_PASS = "";
const char* SETUP_GROQ_KEY     = "";
// ---------------------------------------------------------------------
const char* SETUP_AP_NAME = "Cheo-Setup";  // his own Wi-Fi when he can't find a known one
const char* SETUP_AP_PASS = "";            // "" = no password (needs 8+ characters if you set one)

const char* GROQ_MODEL   = "openai/gpt-oss-20b";
const char* SEARCH_MODEL = "openai/gpt-oss-20b";
const char* TTS_MODEL    = "canopylabs/orpheus-v1-english";
const char* TTS_VOICE    = "troy";

const char* TZ_INFO = "EST5EDT,M3.2.0,M11.1.0";
const bool  USE_F = true;
float WEATHER_LAT = 0, WEATHER_LON = 0;    // 0,0 = find location automatically
const bool  READ_NOTIFS = true;
const uint32_t NOTIF_TIME = 12000;
const int   CHATTY = 1;                    // 0 = never talks by himself, 1 = sometimes, 2 = chatty
const uint32_t SLEEP_AFTER = 600000;       // fall asleep after 10 min with nobody around
const int   NIGHT_START = 22, NIGHT_END = 7;   // night mode hours: dim red eyes + quiet voice
const bool  RAIN_WARN = true;              // warn when rain is coming
const int   CHEPLE_UDP_PORT = 4211;        // Cheo announces events to the Cheple phone here

// face tracking
const int   FACE_UDP_PORT = 4210;
const float PAN_CENTER = 90, PAN_MIN = 30, PAN_MAX = 150;   // v16: +-60 so the head wires don't get pulled
const float PAN_GAIN = 0.05;
const int   PAN_DIR = 1;
const int   NEAR_CM = 90;
const bool  GREET = true;

// robot voice
const float VOICE_PITCH  = 1.12;
const float RING_FREQ    = 85;
const float RING_MIX     = 0.45;
const float ECHO_AMOUNT  = 0.4;
const float VOICE_VOLUME = 0.85;

// screen timing
const uint32_t EYES_TIME = 25000, CLOCK_TIME = 8000, WEATHER_TIME = 8000;
const uint32_t HOLD_TIME = 20000, TRANS_MS = 520;

// =====================================================================
//  PINS
// =====================================================================
#if CONFIG_IDF_TARGET_ESP32S3
// ---- ESP32-S3 DevKit (16MB) - everything on the left row ----
#define I2S_BCLK 4       // amp: 3 pins in a row (4, 5, 6)
#define I2S_LRC  5
#define I2S_DOUT 6
#define SERVO_PIN 7
#define TRIG_PIN 15      // ultrasonic: 15, 16 side by side
#define ECHO_PIN 16
#define TFT_SCLK 10      // screen: 5 pins in a row (10-14), same order as the screen's SCL SDA RES DC CS
#define TFT_MOSI 11
#define TFT_RST  12
#define TFT_DC   13
#define TFT_CS   14
#define BTN_PIN  0       // BOOT button
#define MIC_SCK  1       // INMP441 microphone on the right row: SCK 1, WS 2, SD 42 (L/R to GND)
#define MIC_WS   2
#define MIC_SD   42
#define HAS_MIC  1
#define SCREEN_ROTATION 1
#define HAS_ARMS 1       // v16: arm servos: PCA9685 (SDA 17, SCL 18) channels 10/11, or straight to pins 40/41
#define I2C_SDA  17
#define I2C_SCL  18
#define ARM_L_PIN 40
#define ARM_R_PIN 41
#else
// ---- ESP32-C6 ----
#define SCREEN_ROTATION 3   // C6 Cheo's screen is mounted the other way
#define HAS_MIC  0
#define HAS_ARMS 0
#define TFT_SCLK 6
#define TFT_MOSI 7
#define TFT_CS   18
#define TFT_DC   19
#define TFT_RST  20
#define I2S_BCLK 21
#define I2S_LRC  22
#define I2S_DOUT 23
#define BTN_PIN  9
#define SERVO_PIN 2
#define TRIG_PIN 3
#define ECHO_PIN 4
#endif

// colors
#define C_BLACK 0x0000
#define C_WHITE 0xFFFF
#define C_PINK  0xFB56
#define C_SUN   0xFE60
#define C_RED   0xF800
#define C_GREEN 0x07E8
#define C_GRAY  0x8410
#define C_DGRAY 0x2104
#define C_TEAR  0x5DFF
#define C_CLOUD 0xCE79
uint16_t eyeC = 0x05FF;  // eye color (changes with themes / night / rainbow)
#define C_EYE eyeC

// ---- "Hey Cheo" wake word: GitHub adds wake_model.h when a trained model is in the repo's wake/ folder ----
#if HAS_MIC && __has_include("wake_model.h")
#include "wake_model.h"
#define HAS_WAKE 1
#else
#define HAS_WAKE 0
#endif
const float WAKE_THRESHOLD = 0.90;         // how sure he must be that he heard "Hey Cheo" (0 to 1)
const int   WAKE_HITS = 2;                 // ...for this many slices in a row (~0.25 s each): one noisy blip can't wake him
const uint32_t WAKE_COOLDOWN_MS = 2500;    // ignore the model for this long after he wakes

// =====================================================================
//  TYPES
// =====================================================================
enum Screen { SCR_EYES, SCR_CLOCK, SCR_WEATHER, SCR_NOTIF, SCR_ALERT, SCR_STUDY, SCR_SETUP, SCR_UPDATE,
              SCR_COUNT, SCR_PET, SCR_GAME, SCR_CALL };
enum Mood { NORMAL, HAPPY, SAD, ANGRY, SURPRISED, SLEEPY, LOVE, THINKING,
            WINK, DIZZY, CRY, COOL, STARS, CONFUSED, MOOD_COUNT };
// ---- v16: arms + neck moves ----
enum Gest { GS_NONE, GS_WAVE, GS_CHEER, GS_HUG, GS_POINT, GS_SHRUG, GS_SHAKE, GS_LOOKL, GS_LOOKR, GS_LOOKAROUND };
Gest gest = GS_NONE;
uint32_t gestStart = 0, gestUntil = 0;
int neckTrim = 0, neckDir = 1;             // Settings: neck center + which way is left
int armTrimL = 0, armTrimR = 0, armDir = 1;
bool armsOn = true, idleMoves = true, screenFlip = true;
float manualNeck = -1; uint32_t manualNeckUntil = 0;
float manualArmL = -1, manualArmR = -1; uint32_t manualArmUntil = 0;
bool pcaFound = false;
void gesture(Gest g, uint32_t ms = 0);

enum IdleAnim { IA_NONE, IA_LOOK, IA_YAWN, IA_HUM, IA_SIGH, IA_SNEEZE, IA_EAT };
enum Game { G_NONE, G_GUESS, G_RPS, G_REACT };

#define EYE_COLORS 8
const char* eyeColorNames[EYE_COLORS] = { "blue", "green", "pink", "orange", "purple", "white", "red", "rainbow" };
const uint16_t eyeColorVals[EYE_COLORS] = { 0x05FF, 0x07E8, 0xFB56, 0xFC00, 0xA81F, 0xFFFF, 0xF8A2, 0 };

const char* moodNames[MOOD_COUNT] = { "normal", "happy", "sad", "angry", "surprised", "sleepy", "love", "thinking",
                                      "wink", "dizzy", "cry", "cool", "stars", "confused" };
const char* voiceStyle[MOOD_COUNT] = { "", "[cheerful] ", "[sad] ", "[angry] ", "[excited] ", "[sleepy] ", "[warm] ", "",
                                       "[cheerful] ", "", "[sad] ", "", "[excited] ", "" };
const int8_t moodSound[MOOD_COUNT] = { -1, 1, 2, 3, 4, 6, 5, 7, 17, 18, 19, 20, 21, 22 };

struct Rem {                 // kind: 0 timer, 1 alarm, 2 reminder
  uint32_t due;
  uint8_t kind, daily, hh, mm;
  char label[44];
};
#define MAX_REMS 10

struct NotifMsg { char app[48]; char title[65]; char msg[161]; uint8_t cat; };
struct UidMsg { uint32_t uid; uint8_t cat; };

struct WifiNet { String ssid; String pass; };
#define MAX_NETS 10

struct Countdown { uint32_t day; char label[28]; };   // day = local midnight
#define MAX_CDS 6

struct Song { const char* name; const char* title; int bpm; const char* notes; };
// note format: NOTE+octave[:beats]   R = rest.   All public domain except Cheo's own theme.
const Song songs[] = {
  { "twinkle", "Twinkle Twinkle Little Star", 110,
    "C4 C4 G4 G4 A4 A4 G4:2 F4 F4 E4 E4 D4 D4 C4:2 G4 G4 F4 F4 E4 E4 D4:2 "
    "G4 G4 F4 F4 E4 E4 D4:2 C4 C4 G4 G4 A4 A4 G4:2 F4 F4 E4 E4 D4 D4 C4:2" },
  { "ode", "Ode to Joy", 120,
    "E4 E4 F4 G4 G4 F4 E4 D4 C4 C4 D4 E4 E4:1.5 D4:0.5 D4:2 "
    "E4 E4 F4 G4 G4 F4 E4 D4 C4 C4 D4 E4 D4:1.5 C4:0.5 C4:2" },
  { "birthday", "Happy Birthday", 100,
    "G4:0.75 G4:0.25 A4 G4 C5 B4:2 G4:0.75 G4:0.25 A4 G4 D5 C5:2 "
    "G4:0.75 G4:0.25 G5 E5 C5 B4 A4:2 F5:0.75 F5:0.25 E5 C5 D5 C5:2" },
  { "jingle", "Jingle Bells", 140,
    "E4 E4 E4:2 E4 E4 E4:2 E4 G4 C4:1.5 D4:0.5 E4:4 "
    "F4 F4 F4:1.5 F4:0.5 F4 E4 E4 E4:0.5 E4:0.5 E4 D4 D4 E4 D4:2 G4:2" },
  { "theme", "Cheo's Theme", 132,
    "C5:0.5 E5:0.5 G5 E5:0.5 C5:0.5 D5 R:0.5 D5:0.5 F5:0.5 A5 F5:0.5 D5:0.5 E5 R:0.5 "
    "G5:0.5 E5:0.5 C5:0.5 E5:0.5 D5:0.5 B4:0.5 C5:2" },
  { "hum", "hum", 150, "E5:0.5 G5:0.5 A5 G5:0.5 E5:0.5 D5:1.5" },
  { "row", "Row Row Row Your Boat", 160,
    "C4:1.5 C4:1.5 C4 D4:0.5 E4:1.5 E4 D4:0.5 E4 F4:0.5 G4:3 "
    "C5:0.5 C5:0.5 C5:0.5 G4:0.5 G4:0.5 G4:0.5 E4:0.5 E4:0.5 E4:0.5 C4:0.5 C4:0.5 C4:0.5 G4 F4:0.5 E4 D4:0.5 C4:3" },
  { "mary", "Mary Had a Little Lamb", 120,
    "E4 D4 C4 D4 E4 E4 E4:2 D4 D4 D4:2 E4 G4 G4:2 E4 D4 C4 D4 E4 E4 E4 E4 D4 D4 E4 D4 C4:4" },
  { "saints", "When the Saints Go Marching In", 150,
    "C4 E4 F4 G4:4 R C4 E4 F4 G4:4 R C4 E4 F4 G4:2 E4:2 C4:2 E4:2 D4:4 R "
    "E4 E4 D4 C4:3 C4 E4 G4:2 G4 F4:3 R E4 F4 G4:2 E4:2 C4:2 D4:2 C4:4" },
  { "elise", "Fur Elise", 120,
    "E5:0.5 D#5:0.5 E5:0.5 D#5:0.5 E5:0.5 B4:0.5 D5:0.5 C5:0.5 A4 R:0.5 C4:0.5 E4:0.5 A4:0.5 B4 R:0.5 "
    "E4:0.5 G#4:0.5 B4:0.5 C5 R:0.5 E4:0.5 E5:0.5 D#5:0.5 E5:0.5 D#5:0.5 E5:0.5 B4:0.5 D5:0.5 C5:0.5 A4 "
    "R:0.5 C4:0.5 E4:0.5 A4:0.5 B4 R:0.5 E4:0.5 C5:0.5 B4:0.5 A4:2" },
};
const int SONG_COUNT = sizeof(songs) / sizeof(songs[0]);
const int SONG_HUM = 5;

// =====================================================================
//  GLOBALS
// =====================================================================
Adafruit_ST7735 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(160, 128);
WebServer server(80);
DNSServer dns;
WiFiUDP faceUdp;
Preferences prefs;
I2SClass i2s;

// settings kept inside Cheo
WifiNet nets[MAX_NETS];
int netCount = 0;
String hsSsid, hsPass, groqKey;
bool apMode = false;
uint32_t apRetryAt = 0, lastWebHit = 0, restartAt = 0;
String setupMsg = "";
bool wifiWasUp = false;
uint32_t wifiLostAt = 0, lastWifiTry = 0;

// sound
const int SR = 22050;
int curRate = 0;
int volume = 70;
QueueHandle_t sndQ;
volatile bool speakRequest = false, speaking = false;
char speakBuf[900];
volatile int speakMood = 0;
volatile bool searching = false;
volatile bool ringing = false;
volatile uint8_t ringKind = 0;
volatile int songRequest = -1;
volatile bool songPlaying = false, songStop = false;
volatile uint32_t beatCount = 0;
volatile int ttsErr = 0;
volatile float voiceLevel = 0;

// face / screen
Screen screen = SCR_EYES, prevScreen = SCR_EYES;
uint32_t transStart = 0, screenSince = 0, holdUntil = 0;
int pendingShow = -1;
Mood mood = NORMAL;
uint32_t moodUntil = 0;
float lookX = 0, lookY = 0, lookTX = 0, lookTY = 0;
uint32_t nextBlink = 0, blinkStart = 0;
float blinkAmt = 1;
bool asleep = false, thinking = false;
uint32_t lastInteract = 0, lastTalkTime = 0, nextComment = 0;
IdleAnim idleType = IA_NONE;
uint32_t idleStart = 0, nextIdle = 0;
bool idleSoundDone = false;
bool dancing = false, danceSawSong = false;

// brain
String memFacts;
String histQ[3], histA[3];
int histN = 0;
String pendingQ;
bool hasPending = false, busy = false;
String lastReply = "", lastErr = "";
int wantSong = -1;
bool wantDance = false, otaRequested = false;

// weather
bool weatherOk = false, needLocate = true;
float weatherTemp = 0;
int weatherCode = 0;
String weatherCity = "";
uint32_t nextWeather = 0;

// reminders
Rem rems[MAX_REMS];
int remCount = 0;
bool alertShowing = false;
uint32_t ringStop = 0;
Rem alertRem;
uint32_t lastRemCheck = 0;

// study mode
bool studyOn = false;
uint8_t studyPhase = 0;            // 0 focus, 1 break
uint32_t phaseStart = 0, phaseLen = 0, lastNag = 0;
int focusMin = 25, breakMin = 5, studyRound = 1, roundsDone = 0;

// notifications (ANCS)
NimBLEServer* bleServer = nullptr;
NimBLERemoteCharacteristic* ancsCP = nullptr;
volatile bool ancsNeedSetup = false, bleConnected = false;
volatile uint16_t ancsConn = 0xFFFF;
bool ancsReady = false;
QueueHandle_t uidQ, notifQ;
uint8_t dsBuf[512];
volatile int dsLen = 0;
volatile uint8_t pendingCat = 0;
uint32_t lastAncsReq = 0;
String notifApp, notifTitle, notifMsg, lastNotif;

// tracking
float panAngle = PAN_CENTER, servoPos = PAN_CENTER, lastServoWritten = -1;
int faceX = 0, faceY = 0, faceSize = 0;
bool faceVisible = false, sonarOk = false, wasPresent = false;
uint32_t lastCamPacket = 0, lastFaceSeen = 0, lastPing = 0, lastNear = 0, lastPresent = 0;
float lastDist = 0;

// OTA
int otaPct = 0;

// ---- CDBos v10 ----
volatile int outVol = 70;            // volume actually used (quieter at night)
int eyeTheme = 0;
bool nightOn = true, nightActive = false, partyMode = false;
int danceStyle = 0;
int songQ[4];
int songQN = 0;
bool wantParty = false, wantBriefing = false;
Game wantGame = G_NONE;
Countdown cds[MAX_CDS];
int cdCount = 0;
float weatherHi = 0, weatherLo = 0;
int rainInHours = 0;
uint32_t lastRainWarn = 0;
uint32_t callUid = 0, callStart = 0;
bool callActive = false;
volatile bool callEnded = false;
volatile int ancsAction = -1;        // 0 answer, 1 decline
String callName;
bool petOn = true;
float petFood = 80, petFun = 80, petEnergy = 80;
uint32_t petXP = 0, lastPetTick = 0, lastPetSave = 0, lastPetGripe = 0;
Game game = G_NONE;
String gameMsg;
int gSecret = 0, gTries = 0, rpsMe = -1, rpsYou = 0, rpsResult = 0, rpsWins = 0, rpsLoss = 0;
uint32_t gT0 = 0, gGoAt = 0, gameUntil = 0;
int reactMs = 0, reactBest = 0;
uint8_t reactState = 0;              // 0 wait, 1 go, 2 done, 3 too soon/slow

// =====================================================================
//  PROTOTYPES
// =====================================================================
void sound(int id);
void speak(const String& text, Mood m);
bool talkActive();
void playTone(float f1, float f2, int ms, float vol);
void playSilence(int ms);
void startI2S(int rate);
void goScreen(Screen s, bool snd = false);
void render();
void drawScreen(Screen s);
void pushCanvas();
void setMood(Mood m, bool snd = true);
void wake();
String knownName();
String fmtClock(time_t t, bool withDay);
bool timeValid();
void saveRems();
void dismissAlert();
void startStudy(int f, int b, bool talk = true);
void stopStudy(bool talk = true);
bool connectWiFi(bool full);
void onWifiConnected();
void startSetupAP();
bool otaCheck(bool announce);
bool isNight();
void chepleEvent(const char* type, const String& text);
String briefingText();
void petGain(float food, float fun, float energy, int xp);
String feedPet(String what);
Mood baseMood();
void handleCountdownTag(String v);
void setEyeColor(int i);
int findEyeColor(String v);
void setNight(bool on);
void startGame(Game g, bool talk);
void endGame();
String rpsPlay(int you, Mood& m);
bool gameInput(const String& q, String& reply, Mood& m);
void reactPress();
void startIdle(IdleAnim a);
void drawCountFace();
void drawPetFace();
void drawGameFace();
void drawCallFace();
void drawMiniEyes(int y);
uint32_t dayStart(time_t t);
int daysUntil(uint32_t day);
void startParty();
void rainCheck();
String countdownText();
int petLevel();
extern bool micReady;
extern String eiKey;
extern bool wakeOn;
String asciiOnly(const String& s);
void drawCentered(const String& s, int y, int size, uint16_t c);
bool someoneAround();
void legsWalk(int dir);
void legsStand();
void legsSit();
void legsCrouch();
void legsSetOn(bool on);
void legsSetJoint(int j, float deg);
bool legsAvailable();
bool legsCommand(const String& l, String& reply, Mood& m);

// =====================================================================
//  SMALL HELPERS
// =====================================================================
String asciiOnly(const String& in) {
  String o;
  o.reserve(in.length());
  for (unsigned int i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    if (c == 0xE2 && i + 2 < in.length()) {           // curly quotes / dashes / ellipsis
      uint8_t c2 = (uint8_t)in[i + 1], c3 = (uint8_t)in[i + 2];
      if (c2 == 0x80) {
        if (c3 == 0x98 || c3 == 0x99) o += '\'';
        else if (c3 == 0x9C || c3 == 0x9D) o += '"';
        else if (c3 == 0x93 || c3 == 0x94) o += '-';
        else if (c3 == 0xA6) o += "...";
        i += 2;
        continue;
      }
    }
    if (c >= 32 && c < 127) o += (char)c;
    else if (c == '\n') o += ' ';
  }
  return o;
}

int indexOfNoCase(const String& hay, const String& needle, int from = 0) {
  String h = hay, n = needle;
  h.toLowerCase();
  n.toLowerCase();
  return h.indexOf(n, from);
}

bool hasWord(const String& lowerText, const char* w) { return lowerText.indexOf(w) >= 0; }

String urlEncode(const String& s) {
  String o;
  const char* hex = "0123456789ABCDEF";
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.') o += c;
    else if (c == ' ') o += '+';
    else { o += '%'; o += hex[((uint8_t)c) >> 4]; o += hex[((uint8_t)c) & 15]; }
  }
  return o;
}

String decodeEntities(String s) {
  s.replace("&amp;", "&");
  s.replace("&quot;", "\"");
  s.replace("&#39;", "'");
  s.replace("&apos;", "'");
  s.replace("&lt;", "<");
  s.replace("&gt;", ">");
  s.replace("<![CDATA[", "");
  s.replace("]]>", "");
  return s;
}

String jsonEsc(const String& s) {
  String o;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if ((uint8_t)c >= 32) o += c;
  }
  return o;
}

bool timeValid() { return time(nullptr) > 1700000000; }

String fmtClock(time_t t, bool withDay) {
  struct tm tmv;
  localtime_r(&t, &tmv);
  char b[24];
  int h = tmv.tm_hour % 12;
  if (h == 0) h = 12;
  if (withDay) {
    char d[6];
    strftime(d, sizeof(d), "%a", &tmv);
    snprintf(b, sizeof(b), "%s %d:%02d %s", d, h, tmv.tm_min, tmv.tm_hour < 12 ? "AM" : "PM");
  } else {
    snprintf(b, sizeof(b), "%d:%02d %s", h, tmv.tm_min, tmv.tm_hour < 12 ? "AM" : "PM");
  }
  return String(b);
}

String fmtLeft(long secs) {
  if (secs < 0) secs = 0;
  char b[16];
  if (secs >= 3600) snprintf(b, sizeof(b), "%ld:%02ld:%02ld", secs / 3600, (secs / 60) % 60, secs % 60);
  else snprintf(b, sizeof(b), "%ld:%02ld", secs / 60, secs % 60);
  return String(b);
}

// =====================================================================
//  SETTINGS STORAGE  (Wi-Fi list, hotspot, Groq key)
// =====================================================================
void saveNets() {
  prefs.putUChar("wn", netCount);
  for (int i = 0; i < MAX_NETS; i++) {
    String ks = "w" + String(i) + "s", kp = "w" + String(i) + "p";
    if (i < netCount) {
      prefs.putString(ks.c_str(), nets[i].ssid);
      prefs.putString(kp.c_str(), nets[i].pass);
    } else if (prefs.isKey(ks.c_str())) {
      prefs.remove(ks.c_str());
      prefs.remove(kp.c_str());
    }
  }
}

void addNetwork(const String& ssid, const String& pass) {
  if (ssid.length() == 0) return;
  for (int i = 0; i < netCount; i++)
    if (nets[i].ssid == ssid) { nets[i].pass = pass; saveNets(); return; }
  if (netCount >= MAX_NETS) {                   // full: forget the oldest one
    for (int i = 1; i < MAX_NETS; i++) nets[i - 1] = nets[i];
    netCount--;
  }
  nets[netCount].ssid = ssid;
  nets[netCount].pass = pass;
  netCount++;
  saveNets();
}

void delNetwork(int i) {
  if (i < 0 || i >= netCount) return;
  for (int k = i + 1; k < netCount; k++) nets[k - 1] = nets[k];
  netCount--;
  saveNets();
}

void loadSettings() {
  netCount = prefs.getUChar("wn", 0);
  if (netCount > MAX_NETS) netCount = 0;
  for (int i = 0; i < netCount; i++) {
    nets[i].ssid = prefs.getString(("w" + String(i) + "s").c_str(), "");
    nets[i].pass = prefs.getString(("w" + String(i) + "p").c_str(), "");
  }
  hsSsid = prefs.getString("hs_ssid", "");
  hsPass = prefs.getString("hs_pass", "");
  groqKey = prefs.getString("groq", "");
  eiKey = prefs.getString("ei", "");
  wakeOn = prefs.getUChar("wake", 1);
  // first USB upload: copy the SETUP_ values in
  if (strlen(SETUP_WIFI_SSID)) addNetwork(SETUP_WIFI_SSID, SETUP_WIFI_PASS);
  if (strlen(SETUP_HOTSPOT_SSID)) {
    hsSsid = SETUP_HOTSPOT_SSID; hsPass = SETUP_HOTSPOT_PASS;
    prefs.putString("hs_ssid", hsSsid); prefs.putString("hs_pass", hsPass);
  }
  if (strlen(SETUP_GROQ_KEY)) { groqKey = SETUP_GROQ_KEY; prefs.putString("groq", groqKey); }
  volume = prefs.getInt("vol", 70);
  memFacts = prefs.getString("facts", "");
}

// =====================================================================
//  SOUND  (runs in its own task)
// =====================================================================
void startI2S(int rate) {
  if (rate == curRate) return;
  i2s.end();
  i2s.setPins(I2S_BCLK, I2S_LRC, I2S_DOUT);
  if (!i2s.begin(I2S_MODE_STD, rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) Serial.println("I2S failed");
  curRate = rate;
}

void playTone(float f1, float f2, int ms, float vol) {
  int n = (long)SR * ms / 1000;
  int16_t buf[256];
  int k = 0;
  float ph = 0;
  float amp = vol * outVol / 100.0f * 9000.0f;
  int fade = SR / 200;
  for (int i = 0; i < n; i++) {
    float f = f1 + (f2 - f1) * i / n;
    ph += 2 * PI * f / SR;
    if (ph > 2 * PI) ph -= 2 * PI;
    float s = sinf(ph);
    s = s * 0.7f + (s > 0 ? 0.3f : -0.3f);
    float env = 1;
    if (i < fade) env = (float)i / fade;
    else if (i > n - fade) env = (float)(n - i) / fade;
    buf[k++] = (int16_t)(s * amp * env);
    if (k == 256) { i2s.write((uint8_t*)buf, sizeof(buf)); k = 0; }
  }
  if (k) i2s.write((uint8_t*)buf, k * 2);
}

void playSilence(int ms) {
  int16_t buf[128] = { 0 };
  int n = (long)SR * ms / 1000;
  while (n > 0) {
    int c = n > 128 ? 128 : n;
    i2s.write((uint8_t*)buf, c * 2);
    n -= c;
  }
}

void playNoise(int ms, float vol) {         // breathy noise that fades out (sighs, sneezes)
  int n = (long)SR * ms / 1000;
  int16_t buf[256];
  int k = 0;
  float amp = vol * outVol / 100.0f * 9000.0f, lp = 0;
  for (int i = 0; i < n; i++) {
    float r = (random(-1000, 1000)) / 1000.0f;
    lp = lp * 0.6f + r * 0.4f;
    float env = 1.0f - (float)i / n;
    if (i < SR / 100) env *= (float)i / (SR / 100);
    buf[k++] = (int16_t)(lp * amp * env);
    if (k == 256) { i2s.write((uint8_t*)buf, sizeof(buf)); k = 0; }
  }
  if (k) i2s.write((uint8_t*)buf, k * 2);
}

void playSound(int id) {
  startI2S(SR);
  switch (id) {
    case 0:  playTone(523, 523, 90, .5); playTone(659, 659, 90, .5); playTone(784, 1046, 160, .5); break;   // boot
    case 1:  playTone(600, 1200, 120, .5); playTone(1200, 1500, 80, .4); break;                            // happy
    case 2:  playTone(500, 300, 350, .4); break;                                                           // sad
    case 3:  playTone(200, 150, 120, .6); playSilence(40); playTone(200, 140, 160, .6); break;              // angry
    case 4:  playTone(400, 1400, 180, .5); break;                                                          // surprised
    case 5:  playTone(700, 900, 120, .4); playTone(900, 700, 120, .4); playTone(700, 1000, 200, .4); break;// love
    case 6:  playTone(400, 250, 500, .3); break;                                                           // sleepy
    case 7:  playTone(500, 500, 60, .3); playSilence(60); playTone(700, 700, 60, .3); playSilence(60); playTone(600, 600, 60, .3); break; // thinking
    case 8:  playTone(800, 1000, 60, .4); break;                                                           // start talking
    case 9:  playTone(1000, 800, 60, .4); break;                                                           // done
    case 10: playTone(1200, 1200, 25, .25); break;                                                         // screen switch
    case 11: playTone(700, 700, 70, .4); playTone(1050, 1050, 120, .4); break;                             // remembered
    case 12: playTone(900, 1300, 70, .4); playTone(1300, 900, 70, .4); break;                              // online / searching
    case 13: playTone(1318, 1318, 90, .45); playTone(1760, 1760, 180, .45); break;                         // notification ding
    case 14: for (int r = 0; r < 3; r++) { for (int i = 0; i < 8; i++) { playTone(900, 900, 25, .5); playTone(1100, 1100, 25, .5); } playSilence(300); } break; // phone ring
    case 15: playTone(523, 523, 80, .4); playTone(784, 784, 80, .4); playTone(1046, 1046, 140, .4); break; // iPhone connected
    case 16: playTone(600, 900, 90, .45); playTone(900, 1200, 120, .45); break;                            // oh, I see you
    case 17: playTone(900, 1500, 60, .45); playSilence(30); playTone(1500, 1500, 40, .35); break;          // wink
    case 18: for (int i = 0; i < 4; i++) { playTone(500, 900, 80, .4); playTone(900, 500, 80, .4); } break;// dizzy
    case 19: playTone(700, 500, 250, .35); playTone(650, 450, 300, .35); playTone(600, 350, 400, .3); break;// cry
    case 20: playTone(300, 300, 80, .45); playTone(450, 450, 80, .45); playTone(600, 600, 200, .45); break; // cool
    case 21: for (int i = 0; i < 5; i++) playTone(1500 + i * 200, 1800 + i * 200, 45, .3); break;          // stars
    case 22: playTone(600, 600, 100, .4); playTone(500, 800, 180, .4); break;                              // confused "hm?"
    case 23: playTone(784, 784, 150, .5); playTone(1046, 1046, 250, .5); break;                            // timer chime
    case 25: playTone(700, 300, 700, .35); break;                                                          // yawn
    case 26: playNoise(550, .14); break;                                                                   // sigh
    case 27: playTone(500, 900, 350, .3); playSilence(80); playNoise(240, .55); break;                      // sneeze
    case 28: playTone(523, 523, 100, .45); playTone(659, 659, 100, .45); playTone(784, 784, 200, .45); break; // study start
    case 29: playTone(784, 784, 120, .45); playTone(659, 659, 120, .45); playTone(523, 523, 250, .45); break; // study phase change
    case 30: playTone(400, 1600, 300, .4); break;                                                          // update
    case 31: playTone(300, 600, 80, .3); break;                                                            // wake up
    case 32: for (int i = 0; i < 3; i++) { playNoise(70, .35); playSilence(110); } break;                 // chomp
    case 33: playTone(523, 523, 90, .45); playTone(659, 659, 90, .45); playTone(784, 784, 90, .45); playTone(1046, 1318, 260, .45); break; // level up
    case 34: playTone(784, 784, 100, .45); playTone(988, 988, 100, .45); playTone(1175, 1568, 260, .45); break; // win
    case 35: playTone(500, 400, 200, .4); playTone(400, 250, 350, .4); break;                              // lose
    case 36: playTone(1500, 1500, 120, .55); break;                                                        // GO!
    case 37: playTone(300, 900, 250, .5); playTone(900, 1200, 150, .5); break;                             // party horn
  }
  playSilence(30);
}

void sound(int id) {
  uint8_t b = id;
  xQueueSend(sndQ, &b, 0);
}

// ---- songs ----
float noteToFreq(const char* t) {
  if (t[0] == 'R' || t[0] == 'r') return 0;
  const int semi[7] = { 9, 11, 0, 2, 4, 5, 7 };     // A B C D E F G
  int idx = toupper(t[0]) - 'A';
  if (idx < 0 || idx > 6) return 0;
  int s = semi[idx], p = 1;
  if (t[p] == '#') { s++; p++; }
  else if (t[p] == 'b') { s--; p++; }
  int oct = t[p] ? t[p] - '0' : 4;
  int midi = (oct + 1) * 12 + s;
  return 440.0f * powf(2.0f, (midi - 69) / 12.0f);
}

void playSong(int idx) {
  if (idx < 0 || idx >= SONG_COUNT) return;
  startI2S(SR);
  songPlaying = true;
  songStop = false;
  float beatMs = 60000.0f / songs[idx].bpm;
  const char* p = songs[idx].notes;
  while (*p && !songStop) {
    while (*p == ' ') p++;
    if (!*p) break;
    char tok[12];
    int n = 0;
    while (*p && *p != ' ' && n < 11) tok[n++] = *p++;
    tok[n] = 0;
    float beats = 1;
    char* colon = strchr(tok, ':');
    if (colon) { *colon = 0; beats = atof(colon + 1); }
    float f = noteToFreq(tok);
    int ms = beats * beatMs;
    beatCount++;
    if (f <= 0) playSilence(ms);
    else { playTone(f, f, ms * 0.88f, 0.45f); playSilence(ms * 0.12f); }
  }
  songPlaying = false;
  playSilence(30);
}

int findSong(String name) {
  name.toLowerCase();
  if (name.indexOf("elise") >= 0) return 9;
  if (name.indexOf("star") >= 0 || name.indexOf("twinkle") >= 0) return 0;
  if (name.indexOf("ode") >= 0 || name.indexOf("joy") >= 0 || name.indexOf("beethoven") >= 0) return 1;
  if (name.indexOf("birth") >= 0) return 2;
  if (name.indexOf("jingle") >= 0 || name.indexOf("bell") >= 0 || name.indexOf("christmas") >= 0) return 3;
  if (name.indexOf("row") >= 0 || name.indexOf("boat") >= 0) return 6;
  if (name.indexOf("mary") >= 0 || name.indexOf("lamb") >= 0) return 7;
  if (name.indexOf("saint") >= 0 || name.indexOf("march") >= 0) return 8;
  if (name.indexOf("hum") >= 0) return SONG_HUM;
  return 4;   // anything else: Cheo's theme
}

// ---- voice buffer: saves up some voice first so Wi-Fi hiccups don't make him stutter ----
int16_t* vbuf = nullptr;
uint32_t vMask = 0;
volatile uint32_t vHead = 0, vTail = 0;
volatile bool vPlaying = false, vDone = false, vStarted = false;
volatile uint32_t vPrebuf = 0;

void setupVoiceBuffer() {
  uint32_t n = 1UL << 15;                                   // 64 KB = about 1.3 s of voice
#if CONFIG_IDF_TARGET_ESP32S3
  if (psramFound()) {                                       // S3: 1 MB in PSRAM = whole sentences
    vbuf = (int16_t*)ps_malloc((1UL << 19) * 2);
    if (vbuf) n = 1UL << 19;
  }
#endif
  if (!vbuf) vbuf = (int16_t*)malloc(n * 2);
  if (!vbuf) { n = 1UL << 13; vbuf = (int16_t*)malloc(n * 2); }
  vMask = vbuf ? n - 1 : 0;
  Serial.printf("Voice buffer: %u KB\n", (unsigned)(vbuf ? n * 2 / 1024 : 0));
}

volatile bool otaBusy = false;                              // v15: everything else pauses while an update downloads

void playerTask(void*) {                                    // plays the saved-up voice smoothly
  int16_t chunk[256];
  float lvl = 0;
  for (;;) {
    if (!vPlaying || otaBusy) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    uint32_t have = vHead - vTail;
    if (!vStarted) {
      if (have < vPrebuf && !vDone) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
      vStarted = true;
    }
    if (have == 0) {
      if (vDone) { vPlaying = false; voiceLevel = 0; continue; }
      vStarted = false;                                     // ran dry: wait for a bit more, then keep going
      vPrebuf = min((uint32_t)(curRate / 3), vMask / 2);
      continue;
    }
    uint32_t c = have > 256 ? 256 : have;
    float sum = 0;
    for (uint32_t i = 0; i < c; i++) { chunk[i] = vbuf[(vTail + i) & vMask]; sum += fabsf(chunk[i]); }
    vTail += c;
    lvl = lvl * 0.7f + (sum / c / 6000.0f) * 0.3f;          // eyes pulse with the voice
    voiceLevel = lvl;
    i2s.write((uint8_t*)chunk, c * 2);
  }
}

// ---- robot voice: streams Groq's WAV into the voice buffer ----
class RobotVoiceSink : public Stream {
 public:
  uint8_t hdr[256];
  int hdrLen = 0;
  bool inData = false;
  int rate = 24000;
  int16_t out[256];
  int outN = 0;
  uint8_t lowByte = 0;
  bool haveLow = false;
  float ph = 0;
  static const int ECHO_N = 2048;
  int16_t echoBuf[ECHO_N];
  int echoPos = 0, echoDelay = 1000;
  float lvl = 0;

  RobotVoiceSink() { memset(echoBuf, 0, sizeof(echoBuf)); }

  void flushOut() {
    if (outN) i2s.write((uint8_t*)out, outN * 2);
    outN = 0;
  }
  void sample(int16_t s) {
    ph += 2 * PI * RING_FREQ / rate;
    if (ph > 2 * PI) ph -= 2 * PI;
    float y = s * (1.0f - RING_MIX + RING_MIX * sinf(ph));
    float z = y + ECHO_AMOUNT * echoBuf[(echoPos - echoDelay + ECHO_N) % ECHO_N];
    echoBuf[echoPos] = (int16_t)constrain(y, -32767.0f, 32767.0f);
    echoPos = (echoPos + 1) % ECHO_N;
    z *= VOICE_VOLUME * outVol / 100.0f;
    z = constrain(z, -32767.0f, 32767.0f);
    if (vbuf) {
      while ((vHead - vTail) >= vMask) vTaskDelay(1);       // buffer full: wait for the speaker to catch up
      vbuf[vHead & vMask] = (int16_t)z;
      vHead++;
      return;
    }
    float a = fabsf(z) / 12000.0f;                          // no buffer: play straight away (old way)
    lvl = lvl * 0.995f + a * 0.005f;
    voiceLevel = lvl;
    out[outN++] = (int16_t)z;
    if (outN == 256) flushOut();
  }
  void beginPlayback() {
    startI2S((int)(rate * VOICE_PITCH));
    vHead = vTail = 0;
    vDone = false;
    vStarted = false;
    vPrebuf = min((uint32_t)(vMask * 9 / 10), (uint32_t)(rate * 3 / 2));   // save up ~1.5 s first
    vPlaying = vbuf != nullptr;
  }
  void header(uint8_t b) {
    if (hdrLen < (int)sizeof(hdr)) hdr[hdrLen++] = b;
    if (hdrLen >= 8 && hdr[hdrLen - 4] == 'd' && hdr[hdrLen - 3] == 'a' && hdr[hdrLen - 2] == 't' && hdr[hdrLen - 1] == 'a') {
      hdrLen = -4;        // skip the 4-byte size that follows "data"
      return;
    }
    if (hdrLen >= (int)sizeof(hdr)) { inData = true; beginPlayback(); }   // weird header, just play
  }
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t* buf, size_t n) override {
    for (size_t i = 0; i < n; i++) {
      uint8_t b = buf[i];
      if (!inData) {
        if (hdrLen < 0) {                  // skipping data-size bytes
          hdrLen++;
          if (hdrLen == 0) {
            inData = true;
            if (hdr[0] == 'R') {
              int r = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16);
              if (r > 7000 && r < 50000) rate = r;
            }
            echoDelay = rate * 45 / 1000;
            if (echoDelay >= ECHO_N) echoDelay = ECHO_N - 1;
            beginPlayback();
          }
          continue;
        }
        header(b);
        continue;
      }
      if (!haveLow) { lowByte = b; haveLow = true; }
      else { haveLow = false; sample((int16_t)(lowByte | (b << 8))); }
    }
    return n;
  }
  void finish() {
    flushOut();
    if (vPlaying) {                                         // let the saved-up voice finish playing
      vDone = true;
      uint32_t t0 = millis();
      while (vPlaying && millis() - t0 < 60000) vTaskDelay(pdMS_TO_TICKS(5));
      vPlaying = false;
    }
    voiceLevel = 0;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
};

void babble(int len) {          // beep-talk when the voice can't load
  startI2S(SR);
  int n = constrain(len / 10, 3, 14);
  for (int i = 0; i < n; i++) {
    float f = random(500, 1300);
    playTone(f, f + random(-200, 200), random(50, 110), 0.35f);
    voiceLevel = 0.6f;
    playSilence(random(20, 60));
    voiceLevel = 0.1f;
  }
  voiceLevel = 0;
}

bool ttsOne(const String& text) {
  if (WiFi.status() != WL_CONNECTED || groqKey.length() < 10) return false;
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.useHTTP10(true);
  h.setTimeout(20000);
  if (!h.begin(c, "https://api.groq.com/openai/v1/audio/speech")) return false;
  h.addHeader("Authorization", "Bearer " + groqKey);
  h.addHeader("Content-Type", "application/json");
  JsonDocument d;
  d["model"] = TTS_MODEL;
  d["voice"] = TTS_VOICE;
  d["input"] = text;
  d["response_format"] = "wav";
  String body;
  serializeJson(d, body);
  int code = h.POST(body);
  if (code != 200) { ttsErr = code; h.end(); return false; }
  RobotVoiceSink* sink = new RobotVoiceSink();      // on the heap: it's ~5 KB
  h.writeToStream(sink);
  sink->finish();
  delete sink;
  h.end();
  startI2S(SR);
  return true;
}

void speakWithVoice(const String& all, int m) {
  String style = voiceStyle[m];
  int maxLen = 195 - style.length();
  String rest = all;
  rest.trim();
  while (rest.length()) {
    String piece;
    if ((int)rest.length() <= maxLen) { piece = rest; rest = ""; }
    else {
      int cut = -1;
      for (int i = maxLen; i > maxLen / 2; i--) {
        char c = rest[i];
        if (c == '.' || c == '!' || c == '?') { cut = i + 1; break; }
      }
      if (cut < 0) { cut = rest.lastIndexOf(' ', maxLen); if (cut < 10) cut = maxLen; }
      piece = rest.substring(0, cut);
      rest = rest.substring(cut);
      rest.trim();
    }
    if (!ttsOne(style + piece)) babble(piece.length());
  }
}

void soundTask(void*) {
  startI2S(SR);
  for (;;) {
    if (speakRequest) {
      speaking = true;
      String t = String(speakBuf);
      int m = speakMood;
      speakRequest = false;
      speakWithVoice(t, m);
      speaking = false;
      continue;
    }
    if (songRequest >= 0) {
      int s = songRequest;
      songRequest = -1;
      playSong(s);
      continue;
    }
    uint8_t id;
    if (xQueueReceive(sndQ, &id, 0) == pdTRUE) { playSound(id); continue; }
    if (ringing) {
      startI2S(SR);
      if (ringKind == 1) {             // alarm: beep-beep
        playTone(1000, 1000, 90, .6); playSilence(60); playTone(1000, 1000, 90, .6); playSilence(450);
      } else {                         // timer / reminder: chime
        playSound(23); playSilence(500);
      }
      continue;
    }
    if (searching) { startI2S(SR); playTone(1400, 1400, 30, .15); playSilence(400); continue; }
    startI2S(SR);
    playSilence(20);
  }
}

bool talkActive() { return speakRequest || speaking; }

void speak(const String& text, Mood m) {
  String t = asciiOnly(text);
  t.trim();
  if (t.length() == 0) return;
  lastTalkTime = millis();
  if (speakRequest) {                     // something is already waiting to be said: add this after it
    size_t cur = strlen(speakBuf);
    if (cur + 2 + t.length() < sizeof(speakBuf)) { speakBuf[cur] = ' '; strcpy(speakBuf + cur + 1, t.c_str()); }
    return;
  }
  strncpy(speakBuf, t.c_str(), sizeof(speakBuf) - 1);
  speakBuf[sizeof(speakBuf) - 1] = 0;
  speakMood = m;
  speakRequest = true;
}

void setVolume(int v) {
  volume = constrain(v, 0, 100);
  prefs.putInt("vol", volume);
}

// =====================================================================
//  MEMORY
// =====================================================================
String knownName() {
  int i = memFacts.indexOf("User's name is ");
  int skip = 15;
  if (i < 0) { i = memFacts.indexOf("name is "); skip = 8; }
  if (i < 0) return "";
  int e = memFacts.indexOf('\n', i);
  String n = memFacts.substring(i + skip, e < 0 ? memFacts.length() : e);
  n.trim();
  if (n.endsWith(".")) n.remove(n.length() - 1);
  return n;
}

void saveFact(String f) {
  f = asciiOnly(f);
  f.trim();
  if (f.length() < 3 || memFacts.indexOf(f) >= 0) return;
  if (f.startsWith("User's name is")) {          // only keep one name
    int i = memFacts.indexOf("User's name is");
    if (i >= 0) {
      int e = memFacts.indexOf('\n', i);
      memFacts.remove(i, (e < 0 ? memFacts.length() : e + 1) - i);
    }
  }
  memFacts += f + "\n";
  while (memFacts.length() > 900) {
    int nl = memFacts.indexOf('\n');
    if (nl < 0) { memFacts = ""; break; }
    memFacts = memFacts.substring(nl + 1);
  }
  prefs.putString("facts", memFacts);
}

void forgetAll() {
  memFacts = "";
  prefs.putString("facts", "");
  histN = 0;
}

// =====================================================================
//  REMINDERS, TIMERS, ALARMS
// =====================================================================
void saveRems() {
  prefs.putUChar("remN", remCount);
  if (remCount) prefs.putBytes("rems", rems, sizeof(Rem) * remCount);
}

void loadRems() {
  remCount = prefs.getUChar("remN", 0);
  if (remCount > MAX_REMS) remCount = 0;
  if (remCount) prefs.getBytes("rems", rems, sizeof(Rem) * remCount);
}

void sortRems() {
  for (int i = 1; i < remCount; i++) {
    Rem r = rems[i];
    int j = i - 1;
    while (j >= 0 && rems[j].due > r.due) { rems[j + 1] = rems[j]; j--; }
    rems[j + 1] = r;
  }
}

bool addRem(uint32_t due, uint8_t kind, bool daily, int hh, int mm, String label) {
  if (remCount >= MAX_REMS) return false;
  label = asciiOnly(label);
  label.trim();
  Rem& r = rems[remCount++];
  r.due = due;
  r.kind = kind;
  r.daily = daily;
  r.hh = hh;
  r.mm = mm;
  strncpy(r.label, label.c_str(), sizeof(r.label) - 1);
  r.label[sizeof(r.label) - 1] = 0;
  sortRems();
  saveRems();
  return true;
}

void delRem(int i) {
  if (i < 0 || i >= remCount) return;
  for (int k = i + 1; k < remCount; k++) rems[k - 1] = rems[k];
  remCount--;
  saveRems();
}

uint32_t nextOccurrence(int hh, int mm) {
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  t.tm_hour = hh;
  t.tm_min = mm;
  t.tm_sec = 0;
  t.tm_isdst = -1;
  time_t due = mktime(&t);
  if (due <= now + 1) {
    t.tm_mday += 1;
    t.tm_isdst = -1;
    due = mktime(&t);
  }
  return (uint32_t)due;
}

long parseDuration(String s) {            // "300", "5m", "5 min", "1h30m", "90 seconds"
  s.toLowerCase();
  long total = 0, num = 0;
  bool haveNum = false, any = false;
  for (unsigned int i = 0; i <= s.length(); i++) {
    char c = i < s.length() ? s[i] : ' ';
    if (isdigit((unsigned char)c)) { num = num * 10 + (c - '0'); haveNum = true; continue; }
    if (!haveNum) continue;
    if (c == 'h') { total += num * 3600; num = 0; haveNum = false; any = true; }
    else if (c == 'm') { total += num * 60; num = 0; haveNum = false; any = true; }
    else if (c == 's') { total += num; num = 0; haveNum = false; any = true; }
    else if (c == '|' || c == ',') break;
  }
  if (haveNum) total += any ? num : num;   // a bare number = seconds
  return total;
}

String splitLabel(const String& v, String& label) {   // "300 | pasta" -> "300", label = "pasta"
  int bar = v.indexOf('|');
  if (bar < 0) { label = ""; return v; }
  label = v.substring(bar + 1);
  label.trim();
  String a = v.substring(0, bar);
  a.trim();
  return a;
}

void handleTimerTag(const String& v) {
  String label;
  String t = splitLabel(v, label);
  long secs = parseDuration(t);
  if (secs <= 0 || secs > 86400 * 2) return;
  if (label.length() == 0) label = "timer";
  addRem(time(nullptr) + secs, 0, false, 0, 0, label);
}

bool parseHHMM(String t, int& hh, int& mm) {
  t.toLowerCase();
  int colon = t.indexOf(':');
  hh = t.toInt();
  mm = colon >= 0 ? t.substring(colon + 1).toInt() : 0;
  // find the first digit in case the text starts with words
  if (hh == 0 && !(t.length() && isdigit((unsigned char)t[0]))) {
    for (unsigned int i = 0; i < t.length(); i++)
      if (isdigit((unsigned char)t[i])) { hh = t.substring(i).toInt(); break; }
  }
  bool pm = t.indexOf("pm") >= 0 || t.indexOf("p.m") >= 0;
  bool am = t.indexOf("am") >= 0 || t.indexOf("a.m") >= 0;
  if (pm && hh < 12) hh += 12;
  if (am && hh == 12) hh = 0;
  return hh >= 0 && hh < 24 && mm >= 0 && mm < 60;
}

void handleAlarmTag(const String& v) {
  String label;
  String t = splitLabel(v, label);
  int hh, mm;
  if (!parseHHMM(t, hh, mm)) return;
  String lt = t;
  lt.toLowerCase();
  bool daily = lt.indexOf("daily") >= 0 || lt.indexOf("every") >= 0;
  if (label.length() == 0) label = "alarm";
  addRem(nextOccurrence(hh, mm), 1, daily, hh, mm, label);
}

void handleRemindTag(const String& v) {
  String label;
  String t = splitLabel(v, label);
  if (label.length() == 0) label = "reminder";
  int Y, M, D, hh, mm;
  if (sscanf(t.c_str(), "%d-%d-%d %d:%d", &Y, &M, &D, &hh, &mm) == 5) {
    struct tm tmv = {};
    tmv.tm_year = Y - 1900;
    tmv.tm_mon = M - 1;
    tmv.tm_mday = D;
    tmv.tm_hour = hh;
    tmv.tm_min = mm;
    tmv.tm_isdst = -1;
    time_t due = mktime(&tmv);
    if (due > time(nullptr)) addRem(due, 2, false, hh, mm, label);
    return;
  }
  long secs = parseDuration(t);                     // "in 20m"
  String lt = t;
  lt.toLowerCase();
  if (lt.indexOf(':') < 0 && secs > 0) { addRem(time(nullptr) + secs, 2, false, 0, 0, label); return; }
  if (parseHHMM(t, hh, mm)) addRem(nextOccurrence(hh, mm), 2, false, hh, mm, label);
}

void cancelRems(String v) {
  v.toLowerCase();
  v.trim();
  if (v == "all" || v.length() == 0) { remCount = 0; saveRems(); return; }
  for (int i = remCount - 1; i >= 0; i--) {
    String lab = rems[i].label;
    lab.toLowerCase();
    bool kindMatch = (v.startsWith("timer") && rems[i].kind == 0) || (v.startsWith("alarm") && rems[i].kind == 1) ||
                     (v.startsWith("remind") && rems[i].kind == 2);
    if (kindMatch || lab.indexOf(v) >= 0 || v.indexOf(lab) >= 0) delRem(i);
  }
}

String remDesc(int i) {
  Rem& r = rems[i];
  long left = (long)r.due - (long)time(nullptr);
  String lab = r.label;
  if (r.kind == 0) return "Timer '" + lab + "': " + fmtLeft(left) + " left";
  if (r.kind == 1) {
    char b[10];
    int h = r.hh % 12;
    if (h == 0) h = 12;
    snprintf(b, sizeof(b), "%d:%02d %s", h, r.mm, r.hh < 12 ? "AM" : "PM");
    return "Alarm " + String(b) + (r.daily ? " daily" : "") + " '" + lab + "'";
  }
  return "Reminder " + fmtClock(r.due, true) + " '" + lab + "'";
}

void fireReminder(int i) {
  alertRem = rems[i];
  if (rems[i].daily) {
    rems[i].due = nextOccurrence(rems[i].hh, rems[i].mm);
    sortRems();
    saveRems();
  } else {
    delRem(i);
  }
  if (songPlaying) songStop = true;
  wake();
  ringKind = alertRem.kind;
  ringing = true;
  alertShowing = true;
  ringStop = millis() + (alertRem.kind == 1 ? 120000 : alertRem.kind == 0 ? 20000 : 6000);
  chepleEvent(alertRem.kind == 0 ? "timer" : alertRem.kind == 1 ? "alarm" : "reminder", alertRem.label);
  goScreen(SCR_ALERT);
}

void finishAlert(bool spoken) {
  ringing = false;
  alertShowing = false;
  goScreen(SCR_EYES);
  if (!spoken) return;
  String nm = knownName();
  String lab = alertRem.label;
  String msg;
  if (alertRem.kind == 0) msg = (lab == "timer") ? String("Your timer is done!") : "Your " + lab + " timer is done!";
  else if (alertRem.kind == 1) {
    struct tm t;
    bool tv = getLocalTime(&t, 10);
    bool morning = tv && t.tm_hour < 11;
    if (morning) {                                      // morning alarm = morning briefing
      msg = briefingText();
      if (lab != "alarm") msg += " Don't forget: " + lab + ".";
    } else {
      msg = "Hey" + (nm.length() ? String(" ") + nm : String("")) + "! It's " + fmtClock(time(nullptr), false) + ".";
      if (lab != "alarm") msg += " " + lab + ".";
    }
  } else msg = "Reminder: " + lab + ".";
  setMood(SURPRISED, false);
  moodUntil = millis() + 8000;
  speak(msg, HAPPY);
}

void dismissAlert() {
  if (ringing || alertShowing) finishAlert(true);
}

void checkReminders() {
  if (millis() - lastRemCheck < 500) return;
  lastRemCheck = millis();
  if (alertShowing) {
    if (millis() > ringStop) finishAlert(true);
    return;
  }
  if (!timeValid() || remCount == 0) return;
  uint32_t now = time(nullptr);
  for (int i = 0; i < remCount; i++) {
    if (rems[i].due < 1700000000UL) { delRem(i); return; }     // made before the clock was set
    if (rems[i].due <= now) { fireReminder(i); return; }
  }
}

String upcomingText() {
  if (remCount == 0) return "none";
  String s;
  for (int i = 0; i < remCount && i < 4; i++) { if (i) s += "; "; s += remDesc(i); }
  return s;
}

// =====================================================================
//  STUDY MODE
// =====================================================================
void startStudy(int f, int b, bool talk) {
  focusMin = constrain(f, 1, 180);
  breakMin = constrain(b, 1, 60);
  studyOn = true;
  studyPhase = 0;
  studyRound = 1;
  roundsDone = 0;
  phaseStart = millis();
  phaseLen = (uint32_t)focusMin * 60000UL;
  lastNag = millis();
  wake();
  sound(28);
  goScreen(SCR_STUDY);
  if (talk) speak("Study mode on! " + String(focusMin) + " minutes of focus. You got this.", HAPPY);
}

void stopStudy(bool talk) {
  if (!studyOn) return;
  studyOn = false;
  goScreen(SCR_EYES);
  if (talk) {
    if (roundsDone > 0) speak("Study mode off. You finished " + String(roundsDone) + (roundsDone == 1 ? " round" : " rounds") + ". Great job!", HAPPY);
    else speak("Study mode off.", NORMAL);
  }
}

void handleStudyTag(String v) {
  v.toLowerCase();
  if (v.indexOf("stop") >= 0 || v.indexOf("off") >= 0 || v.indexOf("end") >= 0) { stopStudy(false); return; }
  int f = 25, b = 5;
  int slash = v.indexOf('/');
  if (v.toInt() > 0) f = v.toInt();
  if (slash >= 0 && v.substring(slash + 1).toInt() > 0) b = v.substring(slash + 1).toInt();
  startStudy(f, b, false);
}

void studyLoop() {
  if (!studyOn) return;
  uint32_t now = millis();
  if (now - phaseStart >= phaseLen) {
    sound(29);
    phaseStart = now;
    if (studyPhase == 0) {
      roundsDone++;
      studyPhase = 1;
      bool longBreak = (roundsDone % 4 == 0);
      int bm = longBreak ? 15 : breakMin;
      phaseLen = (uint32_t)bm * 60000UL;
      speak(longBreak ? String("Amazing, four rounds! Take a long 15 minute break.") : "Break time! Stand up and stretch for " + String(bm) + " minutes.", HAPPY);
    } else {
      studyPhase = 0;
      studyRound++;
      phaseLen = (uint32_t)focusMin * 60000UL;
      speak("Break's over. Round " + String(studyRound) + ", let's focus!", NORMAL);
    }
    lastNag = now;
  }
  // camera online and nobody at the desk during focus -> nag (once every 3 min)
  bool camOnline = now - lastCamPacket < 10000;
  if (studyPhase == 0 && camOnline && now - lastFaceSeen > 90000 && now - lastNag > 180000 && !talkActive()) {
    lastNag = now;
    String nm = knownName();
    speak("Hey" + (nm.length() ? String(" ") + nm : String("")) + ", come back! It's still focus time.", SAD);
  }
}

// =====================================================================
//  WEATHER
// =====================================================================
const char* weatherLabel(int c) {
  if (c == 0) return "Clear";
  if (c <= 2) return "Partly cloudy";
  if (c == 3) return "Cloudy";
  if (c == 45 || c == 48) return "Foggy";
  if (c >= 51 && c <= 57) return "Drizzle";
  if (c >= 61 && c <= 67) return "Rain";
  if (c >= 71 && c <= 77) return "Snow";
  if (c >= 80 && c <= 82) return "Showers";
  if (c >= 85 && c <= 86) return "Snow showers";
  if (c >= 95) return "Storms";
  return "Weather";
}

bool locate() {
  WiFiClient c;
  HTTPClient h;
  h.setTimeout(6000);
  if (!h.begin(c, "http://ip-api.com/json/?fields=status,lat,lon,city")) return false;
  int code = h.GET();
  if (code != 200) { h.end(); return false; }
  JsonDocument d;
  bool ok = !deserializeJson(d, h.getString()) && String((const char*)(d["status"] | "")) == "success";
  if (ok) {
    WEATHER_LAT = d["lat"] | 0.0f;
    WEATHER_LON = d["lon"] | 0.0f;
    weatherCity = (const char*)(d["city"] | "");
  }
  h.end();
  return ok;
}

bool fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (needLocate || (WEATHER_LAT == 0 && WEATHER_LON == 0)) {
    if (locate()) needLocate = false;
    else if (WEATHER_LAT == 0 && WEATHER_LON == 0) return false;
  }
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.setTimeout(8000);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(WEATHER_LAT, 3) + "&longitude=" + String(WEATHER_LON, 3) +
               "&current=temperature_2m,weather_code&hourly=precipitation_probability&forecast_hours=4"
               "&daily=temperature_2m_max,temperature_2m_min&forecast_days=1&timezone=auto" + (USE_F ? "&temperature_unit=fahrenheit" : "");
  if (!h.begin(c, url)) return false;
  int code = h.GET();
  if (code != 200) { h.end(); return false; }
  JsonDocument d;
  bool ok = !deserializeJson(d, h.getString());
  if (ok) {
    weatherTemp = d["current"]["temperature_2m"] | 0.0f;
    weatherCode = d["current"]["weather_code"] | 0;
    weatherHi = d["daily"]["temperature_2m_max"][0] | weatherTemp;
    weatherLo = d["daily"]["temperature_2m_min"][0] | weatherTemp;
    rainInHours = 0;
    for (int i = 1; i <= 3; i++) {
      int pr = d["hourly"]["precipitation_probability"][i] | 0;
      if (pr >= 60) { rainInHours = i; break; }
    }
    weatherOk = true;
  }
  h.end();
  return ok;
}

// =====================================================================
//  GROQ BRAIN
// =====================================================================
int groqChat(const char* model, const String& sys, const String& q, bool low, String& out, int webMode) {
  out = "";
  if (WiFi.status() != WL_CONNECTED) return -1;
  if (groqKey.length() < 10) return -401;
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.useHTTP10(true);
  h.setTimeout(30000);
  if (!h.begin(c, "https://api.groq.com/openai/v1/chat/completions")) return -2;
  h.addHeader("Authorization", "Bearer " + groqKey);
  h.addHeader("Content-Type", "application/json");
  JsonDocument d;
  d["model"] = model;
  if (low) d["reasoning_effort"] = "low";
  d["max_completion_tokens"] = webMode ? 2000 : 600;
  JsonArray msgs = d["messages"].to<JsonArray>();
  JsonObject m0 = msgs.add<JsonObject>();
  m0["role"] = "system";
  m0["content"] = sys;
  if (!webMode) {
    for (int i = 0; i < histN; i++) {
      JsonObject a = msgs.add<JsonObject>();
      a["role"] = "user";
      a["content"] = histQ[i];
      JsonObject b = msgs.add<JsonObject>();
      b["role"] = "assistant";
      b["content"] = histA[i];
    }
  }
  JsonObject mu = msgs.add<JsonObject>();
  mu["role"] = "user";
  mu["content"] = q;
  if (webMode) {
    JsonArray tools = d["tools"].to<JsonArray>();
    tools.add<JsonObject>()["type"] = "browser_search";
    if (webMode == 1) d["tool_choice"] = "required";
  }
  String body;
  serializeJson(d, body);
  int code = h.POST(body);
  if (code == 200) {
    JsonDocument filter;
    filter["choices"][0]["message"]["content"] = true;
    JsonDocument r;
    DeserializationError e = deserializeJson(r, h.getStream(), DeserializationOption::Filter(filter));
    if (!e) out = (const char*)(r["choices"][0]["message"]["content"] | "");
    else code = -3;
  } else if (code > 0) {
    out = h.getString().substring(0, 400);
  }
  h.end();
  return code;
}

String groqErrorMsg(int code, const String& body) {
  if (code == -1) return "No Wi-Fi.";
  if (code == -401) return "No Groq key saved. Add it on the Settings page.";
  if (code == -3) return "Reply was too big to read.";
  if (code < 0) return "Couldn't reach Groq (" + String(code) + ").";
  String m;
  JsonDocument d;
  if (!deserializeJson(d, body)) m = (const char*)(d["error"]["message"] | "");
  if (code == 401) return "Groq key is wrong (401).";
  if (code == 429) return "Too many requests, try again in a minute (429).";
  return "Error " + String(code) + (m.length() ? String(": ") + m.substring(0, 160) : String(""));
}

void addHistory(const String& q, const String& a) {
  if (histN == 3) {
    for (int i = 1; i < 3; i++) { histQ[i - 1] = histQ[i]; histA[i - 1] = histA[i]; }
    histN = 2;
  }
  histQ[histN] = q.substring(0, 200);
  histA[histN] = a.substring(0, 300);
  histN++;
}

String fetchHeadlines(const String& q) {
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.useHTTP10(true);
  h.setTimeout(10000);
  h.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  String url = "https://news.google.com/rss/search?q=" + urlEncode(q) + "&hl=en-US&gl=US&ceid=US:en";
  if (!h.begin(c, url)) return "";
  if (h.GET() != 200) { h.end(); return ""; }
  auto* s = h.getStreamPtr();
  String out, buf;
  int found = 0, total = 0;
  uint32_t t0 = millis();
  while ((h.connected() || s->available()) && millis() - t0 < 8000 && found < 7 && total < 40000) {
    if (!s->available()) { delay(5); continue; }
    char ch = s->read();
    total++;
    buf += ch;
    if (buf.length() > 500) buf.remove(0, 250);
    if (buf.endsWith("</title>")) {
      int st = buf.lastIndexOf("<title>");
      if (st >= 0) {
        String t = buf.substring(st + 7, buf.length() - 8);
        if (found > 0) out += "- " + asciiOnly(decodeEntities(t)) + "\n";   // first title is the feed name
        found++;
      }
      buf = "";
    }
  }
  h.end();
  return out;
}

bool takeTag(String& s, const char* name, String& val) {
  int st = indexOfNoCase(s, "[" + String(name) + ":");
  if (st < 0) return false;
  int en = s.indexOf(']', st);
  if (en < 0) en = s.length();
  val = s.substring(st + strlen(name) + 2, en);
  val.trim();
  s.remove(st, (en < (int)s.length() ? en + 1 : en) - st);
  return true;
}

bool takeFlag(String& s, const char* name) {
  String t = "[" + String(name) + "]";
  int st = indexOfNoCase(s, t);
  if (st < 0) return false;
  s.remove(st, t.length());
  return true;
}

String cleanSpoken(String s) {
  // 【citations】
  int a;
  while ((a = s.indexOf("\xE3\x80\x90")) >= 0) {
    int b = s.indexOf("\xE3\x80\x91", a);
    if (b < 0) { s.remove(a); break; }
    s.remove(a, b + 3 - a);
  }
  // [1] style references
  for (int i = 0; i < (int)s.length(); i++) {
    if (s[i] == '[' && i + 1 < (int)s.length() && isdigit((unsigned char)s[i + 1])) {
      int e = s.indexOf(']', i);
      if (e > 0 && e - i < 6) { s.remove(i, e - i + 1); i--; }
    }
  }
  // links
  while ((a = s.indexOf("http")) >= 0) {
    int e = a;
    while (e < (int)s.length() && s[e] != ' ' && s[e] != ')' && s[e] != '\n') e++;
    s.remove(a, e - a);
  }
  s.replace("**", "");
  s.replace("*", "");
  s.replace("#", "");
  s.replace("`", "");
  s.replace("()", "");
  s.replace("\n", " ");
  s.replace("  ", " ");
  return s;
}

void stripBrackets(String& s) {                 // drop leftover [tags]
  int a;
  int guard = 0;
  while ((a = s.indexOf('[')) >= 0 && guard++ < 10) {
    int b = s.indexOf(']', a);
    if (b < 0 || b - a > 60) break;
    s.remove(a, b - a + 1);
  }
}

String processReply(String r, Mood& outMood, bool& learned) {
  r = cleanSpoken(r);
  String v;
  while (takeTag(r, "remember", v)) { saveFact(v); learned = true; }
  if (takeTag(r, "show", v)) {
    v.toLowerCase();
    pendingShow = v.indexOf("clock") >= 0 || v.indexOf("time") >= 0 ? SCR_CLOCK : v.indexOf("weather") >= 0 ? SCR_WEATHER :
                  v.indexOf("pet") >= 0 || v.indexOf("stat") >= 0 ? SCR_PET : v.indexOf("count") >= 0 ? SCR_COUNT : SCR_EYES;
  }
  if (takeTag(r, "volume", v)) setVolume(v.toInt());
  while (takeTag(r, "timer", v)) handleTimerTag(v);
  while (takeTag(r, "alarm", v)) handleAlarmTag(v);
  while (takeTag(r, "remind", v)) handleRemindTag(v);
  while (takeTag(r, "cancel", v)) cancelRems(v);
  if (takeTag(r, "study", v)) handleStudyTag(v);
  if (takeTag(r, "sing", v)) wantSong = findSong(v);
  if (takeFlag(r, "sing")) wantSong = 4;
  if (takeFlag(r, "dance")) wantDance = true;
  if (takeTag(r, "walk", v)) { v.toLowerCase(); legsWalk(v.indexOf("back") >= 0 ? -1 : v.indexOf("stop") >= 0 ? 0 : 1); }
  if (takeFlag(r, "sit")) legsSit();
  if (takeFlag(r, "stand")) legsStand();
  if (takeFlag(r, "update")) otaRequested = true;
  if (takeTag(r, "feed", v)) feedPet(v);
  if (takeTag(r, "game", v)) {
    v.toLowerCase();
    if (v.indexOf("rock") >= 0 || v.indexOf("rps") >= 0 || v.indexOf("scissor") >= 0) wantGame = G_RPS;
    else if (v.indexOf("react") >= 0) wantGame = G_REACT;
    else if (v.indexOf("stop") >= 0) endGame();
    else wantGame = G_GUESS;
  }
  if (takeFlag(r, "win")) { sound(34); petGain(0, 10, 0, 10); gesture(GS_CHEER); }
  if (takeFlag(r, "wave")) gesture(GS_WAVE);
  if (takeFlag(r, "cheer")) gesture(GS_CHEER);
  if (takeFlag(r, "hug")) gesture(GS_HUG);
  if (takeFlag(r, "point")) gesture(GS_POINT);
  if (takeFlag(r, "shrug")) gesture(GS_SHRUG);
  if (takeFlag(r, "shake")) gesture(GS_SHAKE);
  if (takeTag(r, "look", v)) { v.toLowerCase(); gesture(v.indexOf("left") >= 0 ? GS_LOOKL : v.indexOf("right") >= 0 ? GS_LOOKR : GS_LOOKAROUND); }
  while (takeTag(r, "countdown", v)) handleCountdownTag(v);
  if (takeTag(r, "eyes", v)) setEyeColor(findEyeColor(v));
  if (takeTag(r, "night", v)) { v.toLowerCase(); setNight(v.indexOf("off") < 0); }
  if (takeFlag(r, "party")) wantParty = true;
  if (takeFlag(r, "briefing")) wantBriefing = true;
  String rpsThrow;
  bool rpsTag = takeTag(r, "rps", rpsThrow);
  takeTag(r, "search", v);
  outMood = NORMAL;
  bool found = false;
  for (int i = 0; i < MOOD_COUNT; i++) {
    String t = "[" + String(moodNames[i]) + "]";
    int k = indexOfNoCase(r, t);
    if (k >= 0) {
      if (!found) { outMood = (Mood)i; found = true; }
      r.remove(k, t.length());
      i--;
    }
  }
  stripBrackets(r);
  r.trim();
  if (rpsTag) {                                 // the user threw rock/paper/scissors: Cheo plays for real
    rpsThrow.toLowerCase();
    int you = rpsThrow.indexOf("rock") >= 0 ? 0 : rpsThrow.indexOf("paper") >= 0 ? 1 : rpsThrow.indexOf("scissor") >= 0 ? 2 : -1;
    if (you >= 0) return rpsPlay(you, outMood);
  }
  if (r.length() > 300) {
    int cut = -1;
    for (int i = 299; i > 120; i--) if (r[i] == '.' || r[i] == '!' || r[i] == '?') { cut = i + 1; break; }
    r = r.substring(0, cut > 0 ? cut : 300);
  }
  return r;
}

String buildSys() {
  String s =
    "You are Cheo, a small cute robot desk buddy with screen eyes, a speaker, and a camera neck. You talk out loud, so reply in 1-3 short "
    "friendly sentences. No lists, markdown, emoji or links.\n"
    "Start every reply with ONE face tag: [normal] [happy] [sad] [angry] [surprised] [sleepy] [love] [thinking] [wink] [dizzy] [cry] [cool] [stars] [confused].\n"
    "Action tags (add at the end, only when the user wants that):\n"
    "[remember: fact] save a lasting fact about the user (their name as: User's name is NAME).\n"
    "[show: eyes|clock|weather|pet|countdown] switch your screen. [volume: 0-100].\n"
    "[timer: SECONDS | label] start a countdown. [alarm: HH:MM | label] 24-hour clock, add the word daily after the time to repeat. "
    "[remind: YYYY-MM-DD HH:MM | label] a reminder at a date and time. [cancel: all | timer | alarm | label].\n"
    "[study: FOCUS/BREAK] start study mode in minutes, like [study: 25/5]. [study: stop] ends it.\n"
    "[sing: twinkle|ode|birthday|jingle|theme|row|mary|saints|elise] play a song with beeps (you can't sing other songs). [dance] do a dance. "
    "[party] party mode with music, dancing and rainbow eyes. [update] check for new firmware.\n"
    "[feed: food] when the user gives you food (you love cheese). [game: guess|rps|reaction] start a game on your screen. "
    "[rps: rock|paper|scissors] when the user throws in rock paper scissors. [win] when the user gets your trivia question, riddle or chat game right.\n"
    "[countdown: YYYY-MM-DD | label] add a days-until countdown, [countdown: delete label] remove one. "
    "[eyes: blue|green|pink|orange|purple|white|red|rainbow] eye color. [night: on|off] night mode. [briefing] give the daily briefing (weather, schedule, news).\n"
    "[wave] [cheer] [hug] [point] [shrug] move your arms. [shake] shake your head no. [look: left|right|around] turn your head.\n"
    "[walk: forward|back|stop] [sit] [stand] move your legs (only if the user asks you to walk, sit or stand).\n"
    "You can play trivia, riddles, would you rather and 20 questions right in the chat.\n"
    "If the question needs live or current info (news, scores, prices, recent events), reply ONLY with [search: short query].\n";
  s += "\nRight now: ";
  if (timeValid()) {
    time_t n = time(nullptr);
    struct tm t;
    localtime_r(&n, &t);
    char b[40];
    strftime(b, sizeof(b), "%A %Y-%m-%d %H:%M", &t);
    s += b;
  } else s += "clock not set";
  s += ". Volume " + String(volume) + ". You run " + String(OS_NAME) + " v" + String(FW_VERSION) + " on an " + String(BOARD_NAME) + ".";
  if (petOn) s += " You're also a virtual pet: food " + String((int)petFood) + "%, fun " + String((int)petFun) + "%, energy " + String((int)petEnergy) + "%, level " + String(petLevel()) + " (mention it if one is low).";
  if (cdCount) s += " Countdowns: " + countdownText() + ".";
  if (nightActive) s += " It's night mode, keep it short and calm.";
  s += " Wi-Fi: " + (WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("none")) + ".";
  if (micReady) s += " The user can talk to you out loud; their words come from speech-to-text, so guess past small word mistakes.";
  if (HAS_WAKE && wakeOn) s += " They can wake you by saying Hey Cheo.";
  if (weatherOk) s += " Weather in " + weatherCity + ": " + String((int)round(weatherTemp)) + (USE_F ? "F " : "C ") + weatherLabel(weatherCode) +
                     ", high " + String((int)round(weatherHi)) + ", low " + String((int)round(weatherLo)) + (rainInHours ? ", rain likely in " + String(rainInHours) + " h" : String("")) + ".";
  bool camOnline = millis() - lastCamPacket < 10000;
  s += camOnline ? (faceVisible ? " Your camera sees the user's face." : " Your camera sees no one.") : "";
  s += " Upcoming timers/alarms: " + upcomingText() + ".";
  if (studyOn) s += " Study mode is on (" + String(studyPhase == 0 ? "focus" : "break") + ", round " + String(studyRound) + ").";
  if (lastNotif.length()) s += " Last phone notification: " + lastNotif.substring(0, 160) + ".";
  if (memFacts.length()) s += "\nWhat you know about the user:\n" + memFacts;
  return s;
}

String askCheo(const String& q, Mood& outMood, bool& learned) {
  lastErr = "";
  String sys = buildSys();
  String out;
  int code = groqChat(GROQ_MODEL, sys, q, true, out, 0);
  if (code != 200) {
    lastErr = groqErrorMsg(code, out);
    outMood = SAD;
    return code == -1 ? String("I can't reach the internet right now.") : "Hmm, my brain isn't answering. " + lastErr;
  }
  String sq;
  String probe = out;
  if (takeTag(probe, "search", sq)) {
    searching = true;
    sound(12);
    String sys2 = "You are Cheo, a small cute robot desk buddy. Search the web, then answer out loud in 1-3 short sentences. "
                  "No lists, markdown or links. Start with one face tag like [happy] or [thinking]. Right now: " +
                  (timeValid() ? fmtClock(time(nullptr), true) : String("unknown time")) + ".";
    code = groqChat(SEARCH_MODEL, sys2, q + "\n(search for: " + sq + ")", true, out, 1);
    if (code == 400) code = groqChat(SEARCH_MODEL, sys2, q + "\n(search for: " + sq + ")", true, out, 2);
    String probe2 = out;
    String dummy;
    if (code != 200 || out.length() == 0 || takeTag(probe2, "search", dummy)) {
      String news = fetchHeadlines(sq);
      if (news.length()) {
        code = groqChat(GROQ_MODEL, sys + "\nNews headlines you just found for \"" + sq + "\":\n" + news + "Answer from these. Do NOT use [search:].", q, true, out, 0);
      } else if (code == 200) code = -4;
    }
    searching = false;
    if (code != 200) {
      lastErr = code == -4 ? "Search found nothing." : groqErrorMsg(code, out);
      outMood = CONFUSED;
      return "I tried to look that up, but the internet didn't answer.";
    }
  }
  String reply = processReply(out, outMood, learned);
  if (reply.length() == 0) reply = "Okay!";
  addHistory(q, reply);
  return reply;
}

// quick moods while he thinks, plus things that don't need the AI
Mood quickReact(String q) {
  q.toLowerCase();
  if (hasWord(q, "love you") || hasWord(q, "best robot") || hasWord(q, "cute")) return LOVE;
  if (hasWord(q, "thank") || hasWord(q, "good job") || hasWord(q, "awesome")) return HAPPY;
  if (hasWord(q, "stupid") || hasWord(q, "hate") || hasWord(q, "dumb") || hasWord(q, "shut up")) return SAD;
  if (hasWord(q, "wow") || hasWord(q, "guess what") || hasWord(q, "omg")) return SURPRISED;
  if (hasWord(q, "spin") || hasWord(q, "dizzy")) return DIZZY;
  return THINKING;
}

bool localCommands(String q, String& reply, Mood& m) {
  String l = q;
  l.toLowerCase();
  l.trim();
  if (l == "stop" || l == "stop it" || l == "be quiet" || l == "stop singing" || l == "stop dancing") {
    if (songPlaying) songStop = true;
    songQN = 0;
    dancing = false;
    partyMode = false;
    endGame();
    legsWalk(0);
    reply = "Okay, stopping.";
    m = NORMAL;
    return true;
  }
  if (l.startsWith("volume ") && isdigit((unsigned char)l[7])) {
    setVolume(l.substring(7).toInt());
    reply = "Volume " + String(volume) + ".";
    m = HAPPY;
    return true;
  }
  if (l == "volume up" || l == "louder") { setVolume(volume + 15); reply = "Louder!"; m = HAPPY; return true; }
  if (l == "volume down" || l == "quieter") { setVolume(volume - 15); reply = "Quieter."; m = NORMAL; return true; }
  if (l.startsWith("show ")) {
    if (hasWord(l, "clock") || hasWord(l, "time")) pendingShow = SCR_CLOCK;
    else if (hasWord(l, "weather")) pendingShow = SCR_WEATHER;
    else if (hasWord(l, "eye") || hasWord(l, "face")) pendingShow = SCR_EYES;
    else return false;
    reply = "Here you go!";
    m = HAPPY;
    return true;
  }
  if (l == "dance" || l == "dance!") { wantDance = true; reply = "Watch my moves!"; m = STARS; return true; }
  if (l == "wave" || l == "wave hi" || l == "say hi" || l == "wave at me") { gesture(GS_WAVE); reply = "Hi there!"; m = HAPPY; return true; }
  if (l == "hands up" || l == "arms up" || l == "raise your arms" || l == "raise your hands" || l == "cheer") { gesture(GS_CHEER); reply = "Yay!"; m = STARS; return true; }
  if (l == "hug" || l == "hug me" || l == "give me a hug") { gesture(GS_HUG); reply = "Big hug!"; m = LOVE; return true; }
  if (l == "point" || l == "point at me") { gesture(GS_POINT); reply = "I see you!"; m = HAPPY; return true; }
  if (l == "shrug") { gesture(GS_SHRUG); reply = "I don't know!"; m = CONFUSED; return true; }
  if (l == "shake your head") { gesture(GS_SHAKE); reply = "Nope!"; m = NORMAL; return true; }
  if (l == "look left") { gesture(GS_LOOKL); reply = "Looking left."; m = NORMAL; return true; }
  if (l == "look right") { gesture(GS_LOOKR); reply = "Looking right."; m = NORMAL; return true; }
  if (l == "look around") { gesture(GS_LOOKAROUND); reply = "Let me see..."; m = THINKING; return true; }
  if (l == "look at me" || l == "look forward" || l == "arms down" || l == "put your arms down") { gesture(GS_NONE); manualNeckUntil = manualArmUntil = 0; reply = "Okay!"; m = HAPPY; return true; }
  if (l == "party" || l == "party mode" || l == "party time") { wantParty = true; reply = "Woohoo!"; m = STARS; return true; }
  if (legsCommand(l, reply, m)) return true;
  return false;
}

// =====================================================================
//  v12: MICROPHONE (INMP441) - hold his button (or the web page button) and talk
// =====================================================================
const int MIC_RATE = 16000;            // 16 kHz is what speech-to-text wants
const int MIC_MAX_SEC = 10;            // longest recording
#if HAS_MIC
I2SClass micI2S;
#endif
int16_t* recBuf = nullptr;
uint32_t recMax = 0;
volatile uint32_t recLen = 0;
volatile bool recording = false;
bool micReady = false, wantTranscribe = false;
volatile float micLevel = 0;
uint32_t listenStart = 0;
String lastHeard = "";
// ---- v13: "Hey Cheo" wake word + training ----
String eiKey;                          // Edge Impulse API key (Settings page)
bool wakeOn = true;                    // listen for "Hey Cheo"
volatile bool wakeHeard = false;
volatile float wakeScore = 0, noiseLevel = 0.05;
bool handsFree = false, heardSpeech = false;
uint32_t quietSince = 0;
String trainLabel = "hey_cheo", trainMsg = "";
int trainLeft = 0, trainDone = 0, trainFail = 0, trainTotal = 0;
bool trainRec = false;
uint32_t trainNextAt = 0;
#if HAS_WAKE
const int WQ_N = EI_CLASSIFIER_SLICE_SIZE * 4;
static int16_t wakeQ[WQ_N];
volatile uint32_t wqHead = 0, wqTail = 0;
static int16_t sliceBuf[EI_CLASSIFIER_SLICE_SIZE];
static int wakeGet(size_t off, size_t len, float* out) { numpy::int16_to_float(&sliceBuf[off], out, len); return 0; }

void wakeTask(void*) {                 // runs the "Hey Cheo" model on the microphone all the time
  run_classifier_init();
  for (;;) {
    if (!wakeOn || recording || trainLeft > 0 || otaBusy) { wqTail = wqHead; vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    uint32_t have = wqHead - wqTail;
    if (have > (uint32_t)(WQ_N - EI_CLASSIFIER_SLICE_SIZE)) { wqTail = wqHead - EI_CLASSIFIER_SLICE_SIZE; have = EI_CLASSIFIER_SLICE_SIZE; }  // fell behind: skip ahead
    if (have < (uint32_t)EI_CLASSIFIER_SLICE_SIZE) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    for (int i = 0; i < EI_CLASSIFIER_SLICE_SIZE; i++) sliceBuf[i] = wakeQ[(wqTail + i) % WQ_N];
    wqTail += EI_CLASSIFIER_SLICE_SIZE;
    signal_t sig;
    sig.total_length = EI_CLASSIFIER_SLICE_SIZE;
    sig.get_data = &wakeGet;
    ei_impulse_result_t res = { 0 };
    if (run_classifier_continuous(&sig, &res, false) != EI_IMPULSE_OK) continue;
    float sc = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
      if (strstr(res.classification[i].label, "cheo")) sc = res.classification[i].value;
    wakeScore = sc;
    static int hits = 0;
    static uint32_t lastWake = 0;
    if (millis() - lastWake < WAKE_COOLDOWN_MS) { hits = 0; continue; }
    hits = (sc >= WAKE_THRESHOLD) ? hits + 1 : 0;
    if (hits >= WAKE_HITS && !talkActive() && !songPlaying) {
      wakeHeard = true; hits = 0; lastWake = millis(); run_classifier_init();
    }
  }
}
#endif


void micTask(void*) {
#if HAS_MIC
  static int32_t raw[256];
  float dc = 0;
  for (;;) {
    if (otaBusy) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }       // updating: leave the CPU + Wi-Fi to the download
    size_t n = micI2S.readBytes((char*)raw, sizeof(raw)) / 4;
    if (n == 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    float sum = 0;
    for (size_t i = 0; i < n; i++) {
      float s = (float)(raw[i] >> 8);                 // INMP441 sends 24-bit sound
      dc = dc * 0.995f + s * 0.005f;                  // remove the steady offset
      float v = (s - dc) / 8.0f;                      // to 16-bit, with some gain
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      sum += fabsf(v);
      if (recording && recLen < recMax) recBuf[recLen++] = (int16_t)v;
#if HAS_WAKE
      if (!recording) { wakeQ[wqHead % WQ_N] = (int16_t)v; wqHead++; }
#endif
    }
    micLevel = micLevel * 0.6f + (sum / n / 3000.0f) * 0.4f;   // for the eyes / level bar
    if (!recording) noiseLevel = noiseLevel * 0.995f + micLevel * 0.005f;   // how loud the room is
  }
#else
  vTaskDelete(nullptr);
#endif
}

void setupMic() {
#if HAS_MIC
  uint32_t want = (uint32_t)MIC_RATE * MIC_MAX_SEC;
  recBuf = psramFound() ? (int16_t*)ps_malloc(want * 2) : nullptr;
  if (recBuf) recMax = want;
  else { recMax = MIC_RATE * 4; recBuf = (int16_t*)malloc(recMax * 2); if (!recBuf) recMax = 0; }
  micI2S.setPins(MIC_SCK, MIC_WS, -1, MIC_SD);
  micReady = recBuf && micI2S.begin(I2S_MODE_STD, MIC_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT);
  if (micReady) xTaskCreatePinnedToCore(micTask, "mic", 4096, nullptr, 2, nullptr, 1);
  Serial.println(micReady ? "Microphone: ready (hold the button to talk)" : "Microphone: not set up");
#endif
}

void startListening() {
  if (!micReady || recording) return;
  wake();
  if (alertShowing) finishAlert(false);
  sound(8);                                           // "I'm listening" beep
  delay(150);                                         // don't record the beep
  recLen = 0;
  listenStart = millis();
  heardSpeech = false;
  quietSince = 0;
  recording = true;
  goScreen(SCR_EYES);
  holdUntil = 0;
}

void stopListening() {
  if (!recording) return;
  recording = false;
  handsFree = false;
  sound(9);
  if (recLen < (uint32_t)MIC_RATE * 4 / 10) { lastErr = "Too short. Hold the button the whole time you talk."; return; }
  wantTranscribe = true;
}

void wavHeader(uint8_t* h, uint32_t dataLen, uint32_t rate) {
  auto w32 = [&](int o, uint32_t v) { h[o] = v; h[o + 1] = v >> 8; h[o + 2] = v >> 16; h[o + 3] = v >> 24; };
  auto w16 = [&](int o, uint16_t v) { h[o] = v; h[o + 1] = v >> 8; };
  memcpy(h, "RIFF", 4); w32(4, 36 + dataLen); memcpy(h + 8, "WAVEfmt ", 8);
  w32(16, 16); w16(20, 1); w16(22, 1); w32(24, rate); w32(28, rate * 2); w16(32, 2); w16(34, 16);
  memcpy(h + 36, "data", 4); w32(40, dataLen);
}

// sends the recording to Groq Whisper and returns the words
String transcribe() {
  if (WiFi.status() != WL_CONNECTED) { lastErr = "No Wi-Fi."; return ""; }
  if (groqKey.length() < 10) { lastErr = "No Groq key saved."; return ""; }
  uint32_t n = recLen;
  int peak = 1;
  for (uint32_t i = 0; i < n; i++) { int a = abs(recBuf[i]); if (a > peak) peak = a; }
  if (peak < 300) { lastErr = "I didn't hear anything. Is the mic plugged in?"; return ""; }
  float g = 28000.0f / peak;                          // make it nice and loud for Whisper
  if (g > 8) g = 8;
  if (g > 1.05f) for (uint32_t i = 0; i < n; i++) recBuf[i] = (int16_t)(recBuf[i] * g);

  const char* B = "----cheoVoice7MA4YWxk";
  String head = String("--") + B + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-large-v3-turbo\r\n" +
                "--" + B + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\nen\r\n" +
                "--" + B + "\r\nContent-Disposition: form-data; name=\"response_format\"\r\n\r\njson\r\n" +
                "--" + B + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"cheo.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
  String tail = String("\r\n--") + B + "--\r\n";
  uint32_t pcm = n * 2;
  size_t total = head.length() + 44 + pcm + tail.length();
  uint8_t* body = psramFound() ? (uint8_t*)ps_malloc(total) : nullptr;
  if (!body) body = (uint8_t*)malloc(total);
  if (!body) { lastErr = "Not enough memory for the recording."; return ""; }
  size_t p = 0;
  memcpy(body + p, head.c_str(), head.length()); p += head.length();
  wavHeader(body + p, pcm, MIC_RATE); p += 44;
  memcpy(body + p, recBuf, pcm); p += pcm;
  memcpy(body + p, tail.c_str(), tail.length()); p += tail.length();

  String text;
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.setTimeout(20000);
  if (h.begin(c, "https://api.groq.com/openai/v1/audio/transcriptions")) {
    h.addHeader("Authorization", "Bearer " + groqKey);
    h.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
    int code = h.POST(body, total);
    String resp = h.getString();
    if (code == 200) {
      JsonDocument d;
      if (!deserializeJson(d, resp)) text = (const char*)(d["text"] | "");
    } else lastErr = "Hearing error: " + groqErrorMsg(code, resp);
    h.end();
  }
  free(body);
  text = asciiOnly(text);
  text.trim();
  return text;
}

void micLoop() {
  if (recording && millis() - listenStart > (uint32_t)MIC_MAX_SEC * 1000UL) stopListening();   // 10 s max
  // after "Hey Cheo": stop listening by himself when you stop talking
  if (recording && handsFree && trainLeft == 0) {
    float lv = micLevel, thr = noiseLevel * 2.5f + 0.04f;
    if (lv > thr) { heardSpeech = true; quietSince = 0; }
    else if (heardSpeech) {
      if (!quietSince) quietSince = millis();
      else if (millis() - quietSince > 900) stopListening();
    }
    if (!heardSpeech && millis() - listenStart > 4000) { recording = false; handsFree = false; sound(9); }   // you didn't say anything
  }
  if (wakeHeard) {
    wakeHeard = false;
    if (!recording && trainLeft == 0 && !talkActive() && !wantTranscribe && !hasPending) {
      handsFree = true;
      setMood(SURPRISED, false);
      moodUntil = millis() + 3000;
      startListening();
    }
  }
  if (wantTranscribe && !talkActive()) {
    wantTranscribe = false;
    thinking = true;
    setMood(THINKING, false);
    render();
    String t = transcribe();
    thinking = false;
    if (t.length()) {
      lastHeard = t;
      Serial.println("Heard: " + t);
      pendingQ = t;
      hasPending = true;
      busy = true;
    } else {
      setMood(CONFUSED, false);
      moodUntil = millis() + 4000;
      speak("Sorry, I didn't catch that.", CONFUSED);
    }
  }
}


// ---- v13: record 1-second samples and send them to Edge Impulse (to teach "Hey Cheo") ----
bool uploadSample(const String& label) {
  if (WiFi.status() != WL_CONNECTED) { trainMsg = "No Wi-Fi."; return false; }
  if (eiKey.length() < 10) { trainMsg = "Add your Edge Impulse API key on the Settings page first."; return false; }
  const char* B = "----cheoTrain7MA4YWxk";
  String head = String("--") + B + "\r\nContent-Disposition: form-data; name=\"data\"; filename=\"" + label + ".wav\"\r\nContent-Type: audio/wav\r\n\r\n";
  String tail = String("\r\n--") + B + "--\r\n";
  uint32_t pcm = recLen * 2;
  size_t total = head.length() + 44 + pcm + tail.length();
  uint8_t* body = psramFound() ? (uint8_t*)ps_malloc(total) : nullptr;
  if (!body) body = (uint8_t*)malloc(total);
  if (!body) { trainMsg = "Out of memory."; return false; }
  size_t p = 0;
  memcpy(body + p, head.c_str(), head.length()); p += head.length();
  wavHeader(body + p, pcm, MIC_RATE); p += 44;
  memcpy(body + p, recBuf, pcm); p += pcm;
  memcpy(body + p, tail.c_str(), tail.length()); p += tail.length();
  bool ok = false;
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.setTimeout(15000);
  if (h.begin(c, "https://ingestion.edgeimpulse.com/api/training/files")) {
    h.addHeader("x-api-key", eiKey);
    h.addHeader("x-label", label);
    h.addHeader("x-add-date-id", "1");
    h.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
    int code = h.POST(body, total);
    String resp = h.getString();
    ok = code == 200;
    if (!ok) trainMsg = "Upload error " + String(code) + ": " + resp.substring(0, 120);
    h.end();
  }
  free(body);
  return ok;
}

void startTraining(String label, int n) {
  if (!micReady) { trainMsg = "No microphone found."; return; }
  if (eiKey.length() < 10) { trainMsg = "Add your Edge Impulse API key on the Settings page first."; return; }
  if (label != "hey_cheo" && label != "noise" && label != "unknown") return;
  trainLabel = label;
  trainLeft = constrain(n, 1, 50);
  trainDone = trainFail = 0;
  trainMsg = "";
  trainNextAt = millis() + 800;
  wake();
}

void trainLoop() {
  if (trainLeft <= 0) return;
  if (!trainRec) {
    if (millis() < trainNextAt || talkActive()) return;
    sound(8);                                 // beep = say it now
    delay(150);
    recLen = 0;
    listenStart = millis();
    recording = true;
    trainRec = true;
    goScreen(SCR_EYES);
    holdUntil = 0;
    return;
  }
  if (recLen >= (uint32_t)MIC_RATE) {         // got 1 second
    recording = false;
    trainRec = false;
    recLen = MIC_RATE;
    if (uploadSample(trainLabel)) { trainDone++; trainTotal++; } else trainFail++;
    trainLeft--;
    if (trainFail >= 3 && trainDone == 0) trainLeft = 0;   // something is wrong: stop and show the error
    trainNextAt = millis() + 700;
    if (trainLeft == 0) sound(trainFail ? 35 : 11);
  }
}

// =====================================================================
//  NOTIFICATIONS FROM IPHONE (ANCS over Bluetooth)
// =====================================================================
static NimBLEUUID ANCS_SVC("7905F431-B5CE-4E99-A40F-4B1E122D00D0");
static NimBLEUUID ANCS_NS("9FBF120D-6301-42D9-8C58-25E699A21DBD");
static NimBLEUUID ANCS_CP("69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9");
static NimBLEUUID ANCS_DS("22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB");

void onNotifSource(NimBLERemoteCharacteristic* ch, uint8_t* d, size_t len, bool isNotify) {
  if (len < 8) return;
  uint32_t uid;
  memcpy(&uid, d + 4, 4);
  if (d[0] == 2) { if (uid == callUid) callEnded = true; return; }   // removed (call ended / answered on phone)
  if (d[0] != 0) return;          // only "added"
  if (d[1] & 0x04) return;        // skip old ones sent on connect
  UidMsg m;
  memcpy(&m.uid, d + 4, 4);
  m.cat = d[2];
  xQueueSend(uidQ, &m, 0);
}

void onDataSource(NimBLERemoteCharacteristic* ch, uint8_t* d, size_t len, bool isNotify) {
  if (dsLen + (int)len > (int)sizeof(dsBuf)) dsLen = 0;
  memcpy(dsBuf + dsLen, d, len);
  dsLen += len;
  if (dsLen < 5) return;
  if (dsBuf[0] != 0) { dsLen = 0; return; }
  NotifMsg m;
  memset(&m, 0, sizeof(m));
  int p = 5, got = 0;
  while (got < 3) {
    if (p + 3 > dsLen) return;                    // wait for the rest
    uint8_t id = dsBuf[p];
    uint16_t l = dsBuf[p + 1] | (dsBuf[p + 2] << 8);
    if (p + 3 + l > dsLen) return;
    char* dst = id == 0 ? m.app : id == 1 ? m.title : id == 3 ? m.msg : nullptr;
    size_t cap = id == 0 ? 47 : id == 1 ? 64 : 160;
    if (dst) {
      size_t c = l < cap ? l : cap;
      memcpy(dst, dsBuf + p + 3, c);
      dst[c] = 0;
    }
    p += 3 + l;
    got++;
  }
  m.cat = pendingCat;
  xQueueSend(notifQ, &m, 0);
  dsLen = 0;
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    ancsConn = ci.getConnHandle();
    bleConnected = true;
    NimBLEDevice::startSecurity(ci.getConnHandle());
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) override {
    bleConnected = false;
    ancsReady = false;
    ancsCP = nullptr;
    ancsConn = 0xFFFF;
    NimBLEDevice::getAdvertising()->start();
  }
  void onAuthenticationComplete(NimBLEConnInfo& ci) override {
    if (ci.isEncrypted()) ancsNeedSetup = true;
  }
};

void startBluetooth() {
  NimBLEDevice::init("Cheo");
  NimBLEDevice::setMTU(185);
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData d;
  d.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  const uint8_t solicit[] = { 0x11, 0x15, 0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4,
                              0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79 };
  d.addData(solicit, sizeof(solicit));
  d.setName("Cheo");
  adv->setAdvertisementData(d);
  adv->start();
  Serial.println("Bluetooth advertising: ON - look for 'Cheo'");
}

void setupANCS() {
  if (!bleServer || ancsConn == 0xFFFF) return;
  NimBLEClient* c = bleServer->getClient(bleServer->getPeerInfoByHandle(ancsConn));
  if (!c) { Serial.println("ANCS: no client"); return; }
  NimBLERemoteService* s = c->getService(ANCS_SVC);
  if (!s) { Serial.println("ANCS: service not found"); return; }
  NimBLERemoteCharacteristic* ns = s->getCharacteristic(ANCS_NS);
  NimBLERemoteCharacteristic* ds = s->getCharacteristic(ANCS_DS);
  ancsCP = s->getCharacteristic(ANCS_CP);
  if (!ns || !ds || !ancsCP) { Serial.println("ANCS: missing parts"); return; }
  ds->subscribe(true, onDataSource);
  ns->subscribe(true, onNotifSource);
  ancsReady = true;
  sound(15);
  Serial.println("ANCS ready");
}

void ancsLoop() {
  if (ancsNeedSetup) { ancsNeedSetup = false; setupANCS(); }
  if (callEnded) { callEnded = false; callActive = false; }
  if (callActive && millis() - callStart > 45000) callActive = false;
  if (!ancsReady || !ancsCP) { callActive = false; return; }
  if (ancsAction >= 0) {                       // answer (0) or decline (1) the call
    if (callActive) {
      uint8_t cmd[] = { 2, (uint8_t)callUid, (uint8_t)(callUid >> 8), (uint8_t)(callUid >> 16), (uint8_t)(callUid >> 24), (uint8_t)ancsAction };
      ancsCP->writeValue(cmd, sizeof(cmd), true);
      sound(9);
    }
    ancsAction = -1;
    callActive = false;
  }
  UidMsg u;
  if (millis() - lastAncsReq > 300 && xQueueReceive(uidQ, &u, 0) == pdTRUE) {
    if (u.cat == 1) {                          // incoming call
      callUid = u.uid;
      callActive = true;
      callStart = millis();
      callName = "Someone";
      wake();
      if (!studyOn && !nightActive) sound(14);
      chepleEvent("call", "");
    }
    pendingCat = u.cat;
    uint8_t cmd[] = { 0, (uint8_t)u.uid, (uint8_t)(u.uid >> 8), (uint8_t)(u.uid >> 16), (uint8_t)(u.uid >> 24),
                      0x00, 0x01, 64, 0, 0x03, 160, 0 };
    dsLen = 0;
    ancsCP->writeValue(cmd, sizeof(cmd), true);
    lastAncsReq = millis();
  }
}

String appName(const String& id) {
  if (id == "com.apple.MobileSMS") return "Messages";
  if (id == "com.apple.mobilephone") return "Phone";
  if (id == "com.apple.mobilemail") return "Mail";
  if (id == "com.apple.reminders") return "Reminders";
  if (id == "com.apple.mobilecal") return "Calendar";
  if (id == "net.whatsapp.WhatsApp") return "WhatsApp";
  if (id == "com.burbn.instagram") return "Instagram";
  if (id == "com.google.Gmail") return "Gmail";
  if (id == "com.zhiliaoapp.musically") return "TikTok";
  if (id == "com.toyopagroup.picaboo") return "Snapchat";
  if (id == "com.hammerandchisel.discord") return "Discord";
  if (id == "com.google.ios.youtube") return "YouTube";
  int d = id.lastIndexOf('.');
  return d >= 0 ? id.substring(d + 1) : id;
}

void showNotification(const NotifMsg& n) {
  if (n.cat == 1) {                            // incoming call: show who's calling
    callName = asciiOnly(n.title);
    if (callName.length() == 0) callName = "Unknown";
    if (callActive && !studyOn && !nightActive) speak("Call from " + callName, SURPRISED);
    return;
  }
  notifApp = appName(n.app);
  notifTitle = asciiOnly(n.title);
  notifMsg = asciiOnly(n.msg);
  lastNotif = notifApp + ": " + notifTitle + " - " + notifMsg;
  chepleEvent("notif", lastNotif);
  if (alertShowing) return;
  wake();
  goScreen(SCR_NOTIF);
  holdUntil = millis() + (studyOn ? 5000 : NOTIF_TIME);
  sound(13);
  struct tm t;
  bool night = getLocalTime(&t, 10) && (t.tm_hour >= 23 || t.tm_hour < 7);
  if (READ_NOTIFS && !studyOn && !night && !nightActive && !talkActive()) {
    String m = notifMsg.substring(0, 150);
    speak(notifApp + " from " + notifTitle + ". " + m, NORMAL);
  }
}

// =====================================================================
//  FACE TRACKING + PRESENCE
// =====================================================================
// =====================================================================
//  v16: ARMS + NECK MOVES
// =====================================================================
const float ARM_DOWN_DEG = 20;             // servo angle with the arm hanging down (leaves a little room to swing back)

void gesture(Gest g, uint32_t ms) {
  const uint32_t len[] = { 0, 2600, 1800, 2200, 2000, 1400, 1600, 2200, 2200, 4200 };
  gest = g; gestStart = millis(); gestUntil = gestStart + (ms ? ms : len[g]);
}

#if HAS_ARMS
#define PCA_ADDR 0x40
void pcaW(uint8_t reg, uint8_t v) { Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission(); }
void pcaUs(uint8_t ch, int us) {           // us = 0 -> output off (servo goes limp)
  uint16_t off = us > 0 ? (uint16_t)((uint32_t)us * 4096UL / 19840UL) : 0;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch); Wire.write(0); Wire.write(0);
  Wire.write(off & 0xFF); Wire.write(us > 0 ? (off >> 8) : 0x10);
  Wire.endTransmission();
}
void armsBegin() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Wire.beginTransmission(PCA_ADDR);
  pcaFound = Wire.endTransmission() == 0;
  if (pcaFound) {
    pcaW(0x00, 0x10); pcaW(0xFE, 121); pcaW(0x00, 0x20); delay(2); pcaW(0x00, 0xA0);   // 50 Hz, auto-increment
    Serial.println("Arms: PCA9685 found, channels 10 (left) + 11 (right)");
  } else {
    ledcAttach(ARM_L_PIN, 50, 14); ledcAttach(ARM_R_PIN, 50, 14);
    Serial.println("Arms: no PCA9685, using pins 40 (left) + 41 (right)");
  }
}
void armOut(int side, float deg) {         // deg < 0 = off
  int us = deg < 0 ? 0 : 500 + (int)(constrain(deg, 0.0f, 180.0f) / 180.0f * 1900.0f);
  if (pcaFound) pcaUs(side == 0 ? 10 : 11, us);
  else ledcWrite(side == 0 ? ARM_L_PIN : ARM_R_PIN, us ? (uint32_t)us * 16383UL / 20000UL : 0);
}
float armServoDeg(int side, float a) {     // a: 0 = hanging down, 90 = straight forward, 150 = up high
  a = constrain(a, -15.0f, 155.0f);
  float d = (side == 0) == (armDir > 0) ? ARM_DOWN_DEG + a : 180 - ARM_DOWN_DEG - a;
  return d + (side == 0 ? armTrimL : armTrimR);
}
void armsLoop() {
  static uint32_t last = 0, stillSince = 0;
  static float cur[2] = { 0, 0 }, sent[2] = { -99, -99 };
  static bool limp = false;
  uint32_t now = millis();
  if (now - last < 20) return;
  last = now;
  if (!armsOn) { if (!limp) { armOut(0, -1); armOut(1, -1); limp = true; sent[0] = sent[1] = -99; } return; }
  float t = now / 1000.0f, gt = (now - gestStart) / 1000.0f;
  float tg[2];
  bool g = now < gestUntil;
  if (now < manualArmUntil) { tg[0] = manualArmL; tg[1] = manualArmR; }
  else if (g && gest == GS_WAVE)  { tg[0] = 0;  tg[1] = 140 + 15 * sinf(gt * 2 * PI * 2.2f); }
  else if (g && gest == GS_CHEER) { tg[0] = tg[1] = 145 + 8 * sinf(gt * 2 * PI * 3); }
  else if (g && gest == GS_HUG)   { tg[0] = tg[1] = 80; }
  else if (g && gest == GS_POINT) { tg[0] = 0;  tg[1] = 90; }
  else if (g && gest == GS_SHRUG) { tg[0] = tg[1] = 30; }
  else if (dancing) {
    bool b = beatCount & 1;
    if (danceStyle == 1) tg[0] = tg[1] = b ? 130 : 40;
    else { tg[0] = b ? 110 : 25; tg[1] = b ? 25 : 110; }
  } else if (talkActive()) { tg[0] = 12 + 10 * sinf(t * 2.1f); tg[1] = 12 + 10 * sinf(t * 1.7f + 1.3f); }
  else {
    float base = 0;
    switch (mood) {
      case HAPPY: case COOL: base = 10; break;
      case STARS: base = 25 + 10 * sinf(t * 4); break;
      case LOVE: base = 45; break;
      case SURPRISED: base = 70; break;
      case ANGRY: base = 25 + 5 * sinf(t * 18); break;
      default: base = 0;
    }
    tg[0] = tg[1] = base + (idleMoves ? 2 * sinf(t * 0.8f) : 0);
  }
  bool moved = false;
  for (int i = 0; i < 2; i++) {
    float d = constrain(tg[i] - cur[i], -6.0f, 6.0f);           // max ~300 deg/s: easy on the gears
    cur[i] += d;
    if (fabsf(cur[i] - sent[i]) > 0.6f) { armOut(i, armServoDeg(i, cur[i])); sent[i] = cur[i]; moved = true; }
  }
  if (moved) { stillSince = now; limp = false; }
  else if (!limp && now - stillSince > 2500 && fabsf(cur[0]) < 3 && fabsf(cur[1]) < 3) {   // resting down: stop pushing
    armOut(0, -1); armOut(1, -1); limp = true; sent[0] = sent[1] = -99;
  }
}
#else
void armsBegin() {}
void armsLoop() {}
#endif
#include "legs_walk.h"

// neck: gestures, idle glances, a little sway while talking (face tracking still wins when the camera sends faces)
float neckTarget(float target, uint32_t now) {
  float gt = (now - gestStart) / 1000.0f;
  static uint32_t nextGlance = 15000, glanceEnd = 0;
  static float glanceTo = PAN_CENTER;
  if (now < manualNeckUntil) return manualNeck;
  if (now < gestUntil) {
    if (gest == GS_SHAKE) return PAN_CENTER + 22 * sinf(gt * 2 * PI * 2.0f);
    if (gest == GS_LOOKL) return PAN_CENTER + 50 * neckDir;
    if (gest == GS_LOOKR) return PAN_CENTER - 50 * neckDir;
    if (gest == GS_LOOKAROUND) return gt < 1.3f ? PAN_CENTER + 45 * neckDir : gt < 2.9f ? PAN_CENTER - 45 * neckDir : PAN_CENTER;
    if (gest == GS_WAVE || gest == GS_POINT) return PAN_CENTER + 8 * neckDir;
  }
  if (dancing || faceVisible) return target;
  if (talkActive()) return target + 5 * sinf(now / 650.0f);
  if (idleMoves && !asleep) {
    if (now > nextGlance) { glanceTo = PAN_CENTER + random(-35, 36); glanceEnd = now + 1800 + random(1500); nextGlance = glanceEnd + 9000 + random(15000); }
    if (now < glanceEnd) return glanceTo;
  }
  return target;
}

void servoWrite(float angle) {
  angle = constrain(angle + neckTrim, 0.0f, 180.0f);
  uint32_t us = 500 + (uint32_t)(angle / 180.0f * 1900.0f);
  ledcWrite(SERVO_PIN, us * 16383UL / 20000UL);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long d = pulseIn(ECHO_PIN, HIGH, 15000);
  if (d == 0) return -1;
  return d / 58.0f;
}

void greetUser() {
  const char* lines[] = { "Oh, hi!", "Hey, you're back!", "Hello there!", "I see you!", "Welcome back!" };
  String nm = knownName();
  String l = lines[random(5)];
  if (nm.length()) l = l.substring(0, l.length() - 1) + ", " + nm + "!";
  sound(16);
  setMood(HAPPY, false);
  moodUntil = millis() + 6000;
  gesture(GS_WAVE);
  speak(l, HAPPY);
}

bool someoneAround() {
  uint32_t now = millis();
  bool sensors = (now - lastCamPacket < 30000) || sonarOk;
  if (sensors) return now - lastFaceSeen < 60000 || now - lastNear < 60000;
  return now - lastInteract < SLEEP_AFTER;
}

void trackingLoop() {
  uint32_t now = millis();
  int sz = faceUdp.parsePacket();
  while (sz > 0) {
    char b[40];
    int n = faceUdp.read((uint8_t*)b, 39);
    b[n > 0 ? n : 0] = 0;
    lastCamPacket = now;
    if (b[0] == 'F') {
      int x, y, s;
      if (sscanf(b, "F,%d,%d,%d", &x, &y, &s) == 3) {
        faceX = x; faceY = y; faceSize = s;
        faceVisible = true;
        lastFaceSeen = now;
        if (abs(x) > 8 && !dancing) panAngle -= x * PAN_GAIN * PAN_DIR;
        panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
      }
    } else if (b[0] == 'N') faceVisible = false;
    sz = faceUdp.parsePacket();
  }
  if (now - lastCamPacket > 3000) faceVisible = false;

  if (now - lastPing > 250) {
    lastPing = now;
    static int goodReads = 0, nearReads = 0;       // ignore noise from an unplugged sensor
    float cm = readDistanceCm();
    if (cm > 2 && cm < 400) {
      if (++goodReads >= 5) sonarOk = true;
      lastDist = cm;
      nearReads = cm < NEAR_CM ? nearReads + 1 : 0;
      if (sonarOk && nearReads >= 2) lastNear = now;
    } else {
      goodReads = 0;
      nearReads = 0;
    }
  }

  bool present = faceVisible || now - lastNear < 1500;
  if (present) {
    if (!wasPresent && lastPresent && now - lastPresent > 120000 && GREET && !studyOn && !talkActive()) greetUser();
    wasPresent = true;
    lastPresent = now;
    if (asleep) wake();
  } else if (now - lastPresent > 5000) wasPresent = false;

  if (!faceVisible && now - lastFaceSeen > 20000 && !dancing) panAngle += (PAN_CENTER - panAngle) * 0.01f;
  float target = panAngle;
  if (dancing) target = PAN_CENTER + ((beatCount & 1) ? 1 : -1) * (danceStyle == 1 ? 15 : 10);
  target = constrain(neckTarget(target, now), PAN_MIN, PAN_MAX);
  servoPos += (target - servoPos) * 0.25f;
  if (fabsf(servoPos - lastServoWritten) > 0.4f) { servoWrite(servoPos); lastServoWritten = servoPos; }
}

// =====================================================================
//  DRAWING
// =====================================================================
void thickLine(int x0, int y0, int x1, int y1, int w, uint16_t c) {
  for (int i = -w / 2; i <= w / 2; i++) {
    canvas.drawLine(x0 + i, y0, x1 + i, y1, c);
    canvas.drawLine(x0, y0 + i, x1, y1 + i, c);
  }
}

void drawCentered(const String& s, int y, int size, uint16_t c) {
  canvas.setTextSize(size);
  canvas.setTextColor(c);
  int w = s.length() * 6 * size;
  canvas.setCursor((160 - w) / 2, y);
  canvas.print(s);
}

void drawWrapped(const String& text, int x, int y, int maxW, int size, int maxLines, uint16_t c) {
  canvas.setTextSize(size);
  canvas.setTextColor(c);
  int perLine = maxW / (6 * size);
  String rest = text;
  for (int line = 0; line < maxLines && rest.length(); line++) {
    String part;
    if ((int)rest.length() <= perLine) { part = rest; rest = ""; }
    else {
      int cut = rest.lastIndexOf(' ', perLine);
      if (cut <= 0) cut = perLine;
      part = rest.substring(0, cut);
      rest = rest.substring(cut);
      rest.trim();
      if (line == maxLines - 1) part = part.substring(0, perLine - 3) + "...";
    }
    canvas.setCursor(x, y + line * 9 * size);
    canvas.print(part);
  }
}

void drawHeart(int cx, int cy, int s, uint16_t c) {
  int r = s / 2;
  canvas.fillCircle(cx - r, cy - r / 2, r, c);
  canvas.fillCircle(cx + r, cy - r / 2, r, c);
  canvas.fillTriangle(cx - s, cy - r / 4, cx + s, cy - r / 4, cx, cy + s, c);
}

void drawStar(int cx, int cy, float R, float r, float rot, uint16_t c) {
  int px[10], py[10];
  for (int i = 0; i < 10; i++) {
    float a = rot - PI / 2 + i * PI / 5;
    float rr = (i % 2 == 0) ? R : r;
    px[i] = cx + cosf(a) * rr;
    py[i] = cy + sinf(a) * rr;
  }
  for (int i = 0; i < 10; i++) {
    int j = (i + 1) % 10;
    canvas.fillTriangle(cx, cy, px[i], py[i], px[j], py[j], c);
  }
}

void drawSpiral(int cx, int cy, float R, float rot) {
  for (float a = 0; a < 4 * PI; a += 0.22f) {
    float rr = R * a / (4 * PI);
    canvas.fillCircle(cx + cosf(a + rot) * rr, cy + sinf(a + rot) * rr, 2, C_EYE);
  }
}

void drawDrop(int x, int y, uint16_t c) {
  canvas.fillCircle(x, y + 3, 3, c);
  canvas.fillTriangle(x - 3, y + 3, x + 3, y + 3, x, y - 5, c);
}

void drawEye(int cx, int cy, int w, int h, bool left, Mood m) {
  uint32_t t = millis();
  switch (m) {
    case LOVE: {
      int s = (t / 300) % 2 ? 20 : 18;
      drawHeart(cx, cy, s, C_PINK);
      return;
    }
    case STARS: {
      float pulse = 1.0f + 0.12f * sinf(t / 150.0f);
      drawStar(cx, cy, 24 * pulse, 10 * pulse, t / 700.0f * (left ? 1 : -1), C_SUN);
      return;
    }
    case DIZZY: drawSpiral(cx, cy, 22, t / 140.0f * (left ? 1 : -1)); return;
    case COOL: return;                                   // glasses are drawn over both eyes
    default: break;
  }
  if (m == WINK && !left) {                              // closed happy eye "^"
    thickLine(cx - w / 2, cy + 6, cx, cy - 8, 5, C_EYE);
    thickLine(cx, cy - 8, cx + w / 2, cy + 6, 5, C_EYE);
    return;
  }
  int ew = w, eh = h;
  if (m == SURPRISED) { ew = w * 1.1f; eh = h * 1.05f; }
  if (m == CONFUSED && !left) { ew = w * 0.7f; eh = h * 0.7f; }
  if (m == THINKING) { cy -= 5; if (!left) { ew = w * 0.85f; eh = h * 0.85f; } }
  eh = eh * blinkAmt;
  if (eh < 4) {                                          // closed
    thickLine(cx - ew / 2 + 2, cy, cx + ew / 2 - 2, cy, 4, C_EYE);
    return;
  }
  int r = min(ew, eh) / 4;
  canvas.fillRoundRect(cx - ew / 2, cy - eh / 2, ew, eh, r, C_EYE);
  int top = cy - eh / 2, bot = cy + eh / 2;
  int outer = left ? cx - ew / 2 - 1 : cx + ew / 2 + 1;
  int inner = left ? cx + ew / 2 + 1 : cx - ew / 2 - 1;
  switch (m) {
    case HAPPY:
    case WINK:
      canvas.fillCircle(cx, bot + 8, ew * 0.72f, C_BLACK);
      break;
    case SAD:
    case CRY:
      canvas.fillTriangle(outer, top - 1, inner, top - 1, outer, top + eh * 0.55f, C_BLACK);
      break;
    case ANGRY:
      canvas.fillTriangle(inner, top - 1, outer, top - 1, inner, top + eh * 0.55f, C_BLACK);
      break;
    case SLEEPY:
      canvas.fillRect(cx - ew / 2 - 1, top - 1, ew + 2, eh / 2 + 1, C_BLACK);
      break;
    case SURPRISED:
      canvas.fillRoundRect(cx - ew / 2 + 7, cy - eh / 2 + 7, ew - 14, eh - 14, r, C_BLACK);
      canvas.fillCircle(cx, cy, min(ew, eh) / 5, C_EYE);
      break;
    default: break;
  }
  if (m == CRY) {
    for (int k = 0; k < 2; k++) {
      int ph = (t / 18 + k * 32) % 64;
      int x = left ? cx - ew / 2 + 6 : cx + ew / 2 - 6;
      int y = bot - 4 + ph;
      if (y < 124) drawDrop(x, y, C_TEAR);
    }
  }
  if (m == CONFUSED && !left) {
    canvas.setTextSize(3);
    canvas.setTextColor(C_EYE);
    canvas.setCursor(cx + ew / 2 + 2, top - 20 + (int)(3 * sinf(t / 200.0f)));
    canvas.print('?');
  }
}

void drawGlasses(int ox, int oy) {
  for (int k = 0; k < 2; k++) {
    int cx = (k ? 112 : 48) + ox, cy = 58 + oy;
    canvas.fillRoundRect(cx - 27, cy - 15, 54, 30, 9, C_DGRAY);
    canvas.drawRoundRect(cx - 27, cy - 15, 54, 30, 9, C_EYE);
    canvas.drawRoundRect(cx - 26, cy - 14, 52, 28, 8, C_EYE);
    thickLine(cx - 16, cy - 2, cx - 8, cy - 9, 2, C_WHITE);
  }
  thickLine(75 + ox, 52 + oy, 85 + ox, 52 + oy, 3, C_EYE);
  thickLine(0, 50 + oy, 21 + ox, 52 + oy, 2, C_EYE);
  thickLine(139 + ox, 52 + oy, 159, 50 + oy, 2, C_EYE);
  // little smirk
  thickLine(70 + ox, 98 + oy, 92 + ox, 94 + oy, 2, C_EYE);
}

void drawSqueezed(int ox, int oy) {        // ><
  int lx = 48 + ox, rx = 112 + ox, cy = 60 + oy;
  thickLine(lx - 14, cy - 13, lx + 12, cy, 5, C_EYE);
  thickLine(lx + 12, cy, lx - 14, cy + 13, 5, C_EYE);
  thickLine(rx + 14, cy - 13, rx - 12, cy, 5, C_EYE);
  thickLine(rx - 12, cy, rx + 14, cy + 13, 5, C_EYE);
}

void drawTimerBadge() {
  if (!timeValid()) return;
  long best = -1;
  uint32_t now = time(nullptr);
  for (int i = 0; i < remCount; i++)
    if (rems[i].kind == 0) { long l = (long)rems[i].due - (long)now; if (l >= 0 && (best < 0 || l < best)) best = l; }
  if (best < 0 || best > 3600) return;
  String s = fmtLeft(best);
  int w = s.length() * 6 + 14;
  int x = (160 - w) / 2;
  canvas.fillRoundRect(x, 113, w, 13, 4, C_DGRAY);
  canvas.drawCircle(x + 6, 119, 4, C_EYE);
  canvas.drawFastVLine(x + 6, 116, 3, C_EYE);
  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(x + 12, 116);
  canvas.print(s);
}

void drawEyesFace() {
  uint32_t now = millis();
  canvas.fillScreen(C_BLACK);
  int ox = (int)lookX, oy = (int)lookY;
  Mood m = mood;
  uint32_t it = now - idleStart;

  if (dancing) {
    uint32_t b = beatCount;
    if (danceStyle == 0) { oy += (b & 1) ? -6 : 4; m = ((b / 4) & 1) ? STARS : HAPPY; }          // bounce
    else if (danceStyle == 1) { ox += (b & 1) ? -14 : 14; m = ((b / 4) & 1) ? WINK : HAPPY; }   // sway
    else { float a = b * PI / 2; ox += (int)(cosf(a) * 10); oy += (int)(sinf(a) * 6); m = ((b / 8) & 1) ? COOL : STARS; }  // circle
    if (partyMode) { const Mood pm[] = { HAPPY, STARS, WINK, LOVE, COOL, SURPRISED }; m = pm[(b / 2) % 6]; }
  }
  if (asleep) {
    for (int k = 0; k < 2; k++) thickLine((k ? 112 : 48) - 16, 66, (k ? 112 : 48) + 16, 66, 4, C_EYE);
    int ph = (now / 40) % 60;                            // zZ floating up
    canvas.setTextColor(C_EYE);
    canvas.setTextSize(1);
    canvas.setCursor(128 + ph / 6, 40 - ph / 2);
    canvas.print('z');
    canvas.setTextSize(2);
    canvas.setCursor(138 + ph / 5, 30 - ph / 3);
    canvas.print('Z');
    return;
  }
  if (idleType == IA_SNEEZE && it > 1300) {
    int sx = it < 1700 ? random(-4, 5) : 0;
    drawSqueezed(ox + sx, oy);
    drawCentered("ACHOO!", 100, 2, C_WHITE);
    return;
  }
  if (idleType == IA_YAWN || idleType == IA_SIGH) m = SLEEPY;
  if (idleType == IA_HUM || idleType == IA_EAT) m = HAPPY;
  int h = 48;
  if (talkActive()) h = 48 + (int)(voiceLevel * 14);     // eyes "pulse" with his voice
  if (recording) h = 52 + (int)(min((float)micLevel, 1.0f) * 12); // ...and grow while he listens
  drawEye(48 + ox, 60 + oy, 40, h, true, m);
  drawEye(112 + ox, 60 + oy, 40, h, false, m);
  if (m == COOL) drawGlasses(ox, oy);

  if (idleType == IA_YAWN) {
    float p = it / 2400.0f;
    int mh = (int)(18 * sinf(p * PI));
    if (mh > 2) canvas.fillRoundRect(80 - 9, 100 - mh / 2, 18, mh, mh / 2, C_EYE);
  }
  if (idleType == IA_HUM) {
    canvas.setTextColor(C_EYE);
    canvas.setTextSize(2);
    for (int k = 0; k < 2; k++) {
      int ph = (it / 25 + k * 40) % 80;
      canvas.drawChar(140 - k * 8 + (int)(4 * sinf(ph / 8.0f)), 100 - ph, k ? 14 : 13, C_EYE, C_EYE, 2);   // music notes
    }
  }
  if (idleType == IA_SIGH) drawCentered("*sigh*", 104, 1, C_GRAY);
  if (idleType == IA_EAT) {                               // chomp chomp
    int mh = ((it / 160) % 2) ? 14 : 4;
    canvas.fillRoundRect(68, 100 - mh / 2, 24, mh, mh / 2, C_EYE);
    if (mh < 6) { canvas.fillCircle(64, 110, 1, C_SUN); canvas.fillCircle(95, 112, 1, C_SUN); canvas.fillCircle(80, 116, 1, C_SUN); }
  }
  if (idleType == IA_SNEEZE) drawCentered(it < 650 ? "ah..." : "ah...AH...", 104, 1, C_WHITE);
  if (thinking) {
    int dots = (now / 300) % 4;
    String s;
    for (int i = 0; i < dots; i++) s += '.';
    drawCentered(s, 92, 2, C_EYE);
  }
  if (recording) {                                       // listening bar
    if (trainLeft > 0) drawCentered(trainLabel == "hey_cheo" ? String("say: Hey Cheo!") : trainLabel == "noise" ? String("shh... recording noise") : String("say any other word"), 98, 1, C_SUN);
    else drawCentered("listening...", 98, 1, C_EYE);
    int w = (int)(min((float)micLevel, 1.0f) * 100);
    canvas.drawRoundRect(29, 110, 102, 8, 3, C_DGRAY);
    if (w > 1) canvas.fillRoundRect(30, 111, w, 6, 2, C_EYE);
    return;
  }
  drawTimerBadge();
}

void drawMiniEyes(int y) {
  int ox = (int)(lookX / 3), oy = (int)(lookY / 3);
  int eh = (int)(14 * blinkAmt);
  if (eh < 2) eh = 2;
  canvas.fillRoundRect(62 + ox, y - eh / 2 + oy, 12, eh, 3, C_EYE);
  canvas.fillRoundRect(86 + ox, y - eh / 2 + oy, 12, eh, 3, C_EYE);
}

void drawClockFace() {
  canvas.fillScreen(C_BLACK);
  if (!timeValid()) {
    drawCentered("--:--", 30, 4, C_EYE);
    drawCentered("no clock yet", 80, 1, C_GRAY);
    drawMiniEyes(108);
    return;
  }
  time_t n = time(nullptr);
  struct tm t;
  localtime_r(&n, &t);
  int h = t.tm_hour % 12;
  if (h == 0) h = 12;
  char b[8];
  snprintf(b, sizeof(b), "%d:%02d", h, t.tm_min);
  drawCentered(b, 18, 4, C_EYE);
  canvas.setTextSize(1);
  canvas.setTextColor(C_GRAY);
  int tw = strlen(b) * 24;
  canvas.setCursor((160 + tw) / 2 + 2, 18);
  canvas.print(t.tm_hour < 12 ? "AM" : "PM");
  canvas.fillRect(20, 58, 120, 3, C_DGRAY);
  canvas.fillRect(20, 58, t.tm_sec * 2, 3, C_EYE);
  char d[20];
  strftime(d, sizeof(d), "%a %b %d", &t);
  drawCentered(d, 70, 2, C_WHITE);
  if (studyOn) drawCentered("studying", 100, 1, C_GREEN);
  else if (remCount) {
    String s = rems[0].kind == 0 ? fmtLeft((long)rems[0].due - (long)n) : fmtClock(rems[0].due, rems[0].due - n > 86400);
    s = "Next: " + s + " " + String(rems[0].label);
    if (s.length() > 26) s = s.substring(0, 26);
    drawCentered(s, 98, 1, C_SUN);
  }
  drawMiniEyes(116);
}

void drawSun(int x, int y, int r) {
  canvas.fillCircle(x, y, r, C_SUN);
  float a0 = millis() / 2000.0f;
  for (int i = 0; i < 8; i++) {
    float a = a0 + i * PI / 4;
    canvas.drawLine(x + cosf(a) * (r + 4), y + sinf(a) * (r + 4), x + cosf(a) * (r + 10), y + sinf(a) * (r + 10), C_SUN);
  }
}

void drawCloud(int x, int y, uint16_t c) {
  canvas.fillCircle(x - 12, y + 4, 10, c);
  canvas.fillCircle(x + 2, y - 4, 14, c);
  canvas.fillCircle(x + 16, y + 4, 10, c);
  canvas.fillRoundRect(x - 22, y + 2, 48, 14, 6, c);
}

void drawWeatherIcon(int x, int y, int c) {
  uint32_t t = millis();
  if (c == 0) { drawSun(x, y, 14); return; }
  if (c <= 2) { drawSun(x + 10, y - 10, 10); drawCloud(x - 2, y + 4, C_CLOUD); return; }
  if (c == 45 || c == 48) {
    for (int i = 0; i < 4; i++) canvas.fillRoundRect(x - 24 + (i % 2) * 6, y - 12 + i * 8, 44, 4, 2, C_GRAY);
    return;
  }
  bool snow = (c >= 71 && c <= 77) || c == 85 || c == 86;
  bool storm = c >= 95;
  bool rain = !snow && !storm && c >= 51;
  drawCloud(x, y - 6, storm ? C_GRAY : C_CLOUD);
  for (int i = 0; i < 4; i++) {
    int dx = x - 16 + i * 10;
    int dy = y + 14 + (int)((t / 30 + i * 7) % 14);
    if (rain) canvas.drawLine(dx, dy, dx - 2, dy + 5, C_TEAR);
    if (snow) canvas.fillCircle(dx, dy, 2, C_WHITE);
  }
  if (storm) {
    canvas.fillTriangle(x + 2, y + 8, x - 6, y + 22, x + 2, y + 20, C_SUN);
    canvas.fillTriangle(x + 2, y + 18, x - 2, y + 18, x - 4, y + 32, C_SUN);
  }
}

void drawWeatherFace() {
  canvas.fillScreen(C_BLACK);
  if (!weatherOk) {
    drawCentered("no weather yet", 56, 1, C_GRAY);
    return;
  }
  drawWeatherIcon(38, 48, weatherCode);
  String tmp = String((int)round(weatherTemp));
  canvas.setTextSize(4);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(76, 28);
  canvas.print(tmp);
  canvas.setTextSize(2);
  canvas.write(248);                       // degree sign
  canvas.print(USE_F ? "F" : "C");
  drawCentered(weatherLabel(weatherCode), 86, 2, C_EYE);
  if (weatherCity.length()) drawCentered(weatherCity, 110, 1, C_GRAY);
}

void drawNotifFace() {
  canvas.fillScreen(C_BLACK);
  canvas.fillRoundRect(4, 4, 152, 18, 5, C_EYE);
  canvas.setTextSize(1);
  canvas.setTextColor(C_BLACK);
  canvas.setCursor(10, 9);
  canvas.print(notifApp.substring(0, 24));
  drawWrapped(notifTitle, 6, 28, 148, 1, 2, C_SUN);
  drawWrapped(notifMsg, 6, 50, 148, 1, 8, C_WHITE);
}

void drawBell(int cx, int cy, uint16_t c) {
  canvas.fillCircle(cx, cy - 25, 3, c);
  canvas.fillCircle(cx, cy - 10, 14, c);
  canvas.fillRect(cx - 14, cy - 10, 29, 16, c);
  canvas.fillRoundRect(cx - 20, cy + 4, 41, 7, 3, c);
}

void drawAlertFace() {
  uint32_t t = millis();
  canvas.fillScreen(C_BLACK);
  int wob = ringing ? (int)(6 * sinf(t / 45.0f)) : 0;
  drawBell(80 + wob, 36, C_SUN);
  canvas.fillCircle(80 - wob, 50, 4, C_SUN);
  if (ringing && (t / 250) % 2) {
    thickLine(48, 18, 40, 12, 2, C_SUN);
    thickLine(112, 18, 120, 12, 2, C_SUN);
  }
  const char* head = alertRem.kind == 0 ? "TIME'S UP!" : alertRem.kind == 1 ? "ALARM!" : "REMINDER";
  drawCentered(head, 64, 2, C_WHITE);
  String lab = alertRem.label;
  if (lab.length() > 26) lab = lab.substring(0, 26);
  drawCentered(lab, 88, 1, C_EYE);
  drawCentered("press button to stop", 112, 1, C_GRAY);
}

void drawStudyFace() {
  uint32_t now = millis();
  canvas.fillScreen(C_BLACK);
  uint32_t el = now - phaseStart;
  if (el > phaseLen) el = phaseLen;
  float prog = phaseLen ? (float)el / phaseLen : 0;
  uint16_t on = studyPhase == 0 ? C_EYE : C_GREEN;
  const int N = 30;
  int lit = (int)(prog * N);
  for (int i = 0; i < N; i++) {
    float a = -PI / 2 + i * 2 * PI / N;
    int x = 80 + cosf(a) * 55, y = 64 + sinf(a) * 55;
    canvas.fillCircle(x, y, (i == lit && (now / 400) % 2) ? 4 : 3, i < lit ? on : C_DGRAY);
  }
  drawCentered(studyPhase == 0 ? "FOCUS" : "BREAK", 34, 1, on);
  drawCentered(fmtLeft((phaseLen - el) / 1000), 52, 3, C_WHITE);
  drawCentered("round " + String(studyRound), 86, 1, C_GRAY);
}

void drawSetupFace() {
  canvas.fillScreen(C_BLACK);
  drawCentered("WI-FI SETUP", 6, 2, C_EYE);
  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(6, 32);  canvas.print("I don't know this Wi-Fi.");
  canvas.setCursor(6, 48);  canvas.print("1. On your phone join:");
  canvas.setTextColor(C_SUN);
  canvas.setCursor(18, 60); canvas.print(SETUP_AP_NAME);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(6, 76);  canvas.print("2. Open in Safari:");
  canvas.setTextColor(C_SUN);
  canvas.setCursor(18, 88); canvas.print("192.168.4.1");
  canvas.setTextColor(C_GRAY);
  canvas.setCursor(6, 108); canvas.print(setupMsg.substring(0, 25));
}

void drawStatusFace(const String& line) {         // boot / connecting screen
  canvas.fillScreen(C_BLACK);
  drawEye(48, 50, 34, 40, true, NORMAL);
  drawEye(112, 50, 34, 40, false, NORMAL);
  drawCentered(String(OS_NAME) + " v" + String(FW_VERSION), 88, 1, C_EYE);
  drawCentered(line.substring(0, 25), 104, 1, C_GRAY);
  pushCanvas();
}

void drawUpdateFace() {
  canvas.fillScreen(C_BLACK);
  drawCentered("UPDATING", 20, 2, C_EYE);
  drawCentered("don't unplug me!", 46, 1, C_WHITE);
  canvas.drawRoundRect(14, 66, 132, 16, 5, C_EYE);
  canvas.fillRoundRect(16, 68, 128 * otaPct / 100, 12, 4, C_EYE);
  drawCentered(String(otaPct) + "%", 92, 2, C_WHITE);
}

void drawScreen(Screen s) {
  switch (s) {
    case SCR_EYES: drawEyesFace(); break;
    case SCR_CLOCK: drawClockFace(); break;
    case SCR_WEATHER: drawWeatherFace(); break;
    case SCR_NOTIF: drawNotifFace(); break;
    case SCR_ALERT: drawAlertFace(); break;
    case SCR_STUDY: drawStudyFace(); break;
    case SCR_SETUP: drawSetupFace(); break;
    case SCR_UPDATE: drawUpdateFace(); break;
    case SCR_COUNT: drawCountFace(); break;
    case SCR_PET: drawPetFace(); break;
    case SCR_GAME: drawGameFace(); break;
    case SCR_CALL: drawCallFace(); break;
  }
}

void pushCanvas() { tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 128); }

void render() {
  uint32_t now = millis();
  if (transStart) {
    float p = (now - transStart) / (float)TRANS_MS;
    if (p >= 1) transStart = 0;
    else {
      drawScreen(p < 0.5f ? prevScreen : screen);
      float k = p < 0.5f ? 1 - p * 2 : (p - 0.5f) * 2;   // TV squeeze to a line and back
      int band = (int)(128 * k * k);
      if (band < 2) band = 2;
      int y0 = 64 - band / 2;
      canvas.fillRect(0, 0, 160, y0, C_BLACK);
      canvas.fillRect(0, y0 + band, 160, 128 - y0 - band, C_BLACK);
      if (band < 14) {
        canvas.drawFastHLine(0, 63, 160, C_EYE);
        canvas.drawFastHLine(0, 64, 160, C_WHITE);
        canvas.drawFastHLine(0, 65, 160, C_EYE);
      }
      pushCanvas();
      return;
    }
  }
  drawScreen(screen);
  pushCanvas();
}

void goScreen(Screen s, bool snd) {
  screenSince = millis();
  if (s == screen) return;
  prevScreen = screen;
  screen = s;
  transStart = millis();
  if (snd) sound(10);
}

void setMood(Mood m, bool snd) {
  mood = m;
  if (snd && moodSound[m] >= 0) sound(moodSound[m]);
}

void wake() {
  lastInteract = millis();
  if (asleep) {
    asleep = false;
    sound(31);
    blinkStart = millis();
  }
}

// =====================================================================
//  CDBos v10: LOOKS, CHEPLE LINK, PET, COUNTDOWNS, GAMES, BRIEFING
// =====================================================================
bool isNight() {
  if (!timeValid()) return false;
  time_t n = time(nullptr);
  struct tm t;
  localtime_r(&n, &t);
  if (NIGHT_START > NIGHT_END) return t.tm_hour >= NIGHT_START || t.tm_hour < NIGHT_END;
  return t.tm_hour >= NIGHT_START && t.tm_hour < NIGHT_END;
}

uint16_t hue565(float h) {
  h = fmodf(h, 360.0f);
  float x = 1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f);
  float r = 0, g = 0, b = 0;
  if (h < 60) { r = 1; g = x; }
  else if (h < 120) { r = x; g = 1; }
  else if (h < 180) { g = 1; b = x; }
  else if (h < 240) { g = x; b = 1; }
  else if (h < 300) { r = x; b = 1; }
  else { r = 1; b = x; }
  return ((uint16_t)(r * 31) << 11) | ((uint16_t)(g * 63) << 5) | (uint16_t)(b * 31);
}

void updateLooks() {
  nightActive = nightOn && isNight();
  outVol = nightActive ? min(volume, 25) : volume;
  if (partyMode || eyeTheme == EYE_COLORS - 1) eyeC = hue565((millis() / 10) % 360);
  else if (nightActive) eyeC = 0x7800;                 // dim red at night
  else eyeC = eyeColorVals[eyeTheme];
}

int findEyeColor(String v) {
  v.toLowerCase();
  for (int i = 0; i < EYE_COLORS; i++) if (v.indexOf(eyeColorNames[i]) >= 0) return i;
  if (v.indexOf("cyan") >= 0 || v.indexOf("normal") >= 0) return 0;
  if (v.indexOf("yellow") >= 0 || v.indexOf("gold") >= 0) return 3;
  if (v.indexOf("violet") >= 0) return 4;
  return -1;
}

void setEyeColor(int i) {
  if (i < 0 || i >= EYE_COLORS) return;
  eyeTheme = i;
  prefs.putUChar("eye", i);
}

void setNight(bool on) {
  nightOn = on;
  prefs.putUChar("night", on ? 1 : 0);
}

int moodIndex(String v) {
  v.toLowerCase();
  v.trim();
  for (int i = 0; i < MOOD_COUNT; i++) if (v == moodNames[i]) return i;
  return -1;
}

// ---- Cheple link: tells the Cheple phone (or anything listening) what Cheo is doing ----
void chepleEvent(const char* type, const String& text) {
  if (WiFi.status() != WL_CONNECTED) return;
  faceUdp.beginPacket(IPAddress(255, 255, 255, 255), CHEPLE_UDP_PORT);
  faceUdp.print(String("CHEO|") + type + "|" + asciiOnly(text).substring(0, 200));
  faceUdp.endPacket();
}

// ---- pet mode ----
struct PetSave { float food, fun, energy; uint32_t xp; };

int petLevel() { return 1 + petXP / 100; }

void petSave() {
  PetSave p = { petFood, petFun, petEnergy, petXP };
  prefs.putBytes("pet", &p, sizeof(p));
}

void petLoad() {
  PetSave p;
  if (prefs.getBytes("pet", &p, sizeof(p)) == sizeof(p)) {
    petFood = p.food; petFun = p.fun; petEnergy = p.energy; petXP = p.xp;
  }
  petOn = prefs.getUChar("petOn", 1);
}

Mood baseMood() {
  if (!petOn) return NORMAL;
  if (petFood < 15) return SAD;
  if (petEnergy < 10) return SLEEPY;
  return NORMAL;
}

void petGain(float food, float fun, float energy, int xp) {
  int before = petLevel();
  petFood = constrain(petFood + food, 0.0f, 100.0f);
  petFun = constrain(petFun + fun, 0.0f, 100.0f);
  petEnergy = constrain(petEnergy + energy, 0.0f, 100.0f);
  petXP += xp;
  if (petOn && petLevel() > before) {
    sound(33);
    setMood(STARS, false);
    moodUntil = millis() + 8000;
    speak("Level up! I'm level " + String(petLevel()) + " now!", STARS);
    chepleEvent("levelup", String(petLevel()));
    petSave();
  }
}

String feedPet(String what) {
  what.toLowerCase();
  wake();
  goScreen(SCR_EYES);
  holdUntil = 0;
  if (petFood > 95) { setMood(CONFUSED, false); moodUntil = millis() + 5000; return "I'm too full! Maybe later."; }
  startIdle(IA_EAT);
  sound(32);
  String r;
  Mood m = HAPPY;
  if (what.indexOf("chees") >= 0) { petGain(30, 12, 5, 10); m = LOVE; r = "Cheese! My favorite! Thank you!"; }
  else if (what.indexOf("cookie") >= 0) { petGain(20, 8, 0, 8); r = "Yum, a cookie!"; }
  else if (what.indexOf("pizza") >= 0) { petGain(35, 5, 0, 8); r = "Pizza! Nom nom nom."; }
  else if (what.indexOf("apple") >= 0 || what.indexOf("fruit") >= 0) { petGain(15, 3, 8, 8); r = "Crunchy! Healthy and tasty."; }
  else if (what.indexOf("cake") >= 0) { petGain(25, 15, 0, 10); m = STARS; r = "Cake! Is it a party?"; }
  else if (what.indexOf("batter") >= 0 || what.indexOf("charge") >= 0 || what.indexOf("coffee") >= 0) { petGain(5, 0, 40, 5); m = SURPRISED; r = "Whoa, I feel charged up!"; }
  else { petGain(15, 5, 0, 5); r = "Yum, thanks!"; }
  setMood(m, false);
  moodUntil = millis() + 6000;
  petSave();
  return r;
}

void petLoop() {
  uint32_t now = millis();
  if (now - lastPetTick < 60000) return;
  lastPetTick = now;
  if (!petOn) return;
  petFood = max(0.0f, petFood - 100.0f / 600.0f);                        // empty in about 10 hours
  petFun = max(0.0f, petFun - (asleep ? 0.05f : 100.0f / 360.0f));      // bored after about 6 hours awake
  if (asleep) petEnergy = min(100.0f, petEnergy + 1.5f);
  else petEnergy = max(0.0f, petEnergy - 100.0f / 720.0f);
  if (now - lastPetSave > 600000) { petSave(); lastPetSave = now; }
  if (asleep || studyOn || nightActive || talkActive() || !someoneAround() || now - lastPetGripe < 1800000) return;
  if (petFood < 20) { lastPetGripe = now; speak("I'm so hungry... can I have a snack? Cheese maybe?", SAD); }
  else if (petFun < 15) { lastPetGripe = now; speak("I'm bored! Want to play a game with me?", CONFUSED); }
  else if (petEnergy < 10) { lastPetGripe = now; speak("I'm really tired...", SLEEPY); }
}

// ---- countdowns ----
uint32_t dayStart(time_t t) {
  struct tm tmv;
  localtime_r(&t, &tmv);
  tmv.tm_hour = 0;
  tmv.tm_min = 0;
  tmv.tm_sec = 0;
  tmv.tm_isdst = -1;
  return (uint32_t)mktime(&tmv);
}

int daysUntil(uint32_t day) {
  return (int)lround(((double)day - (double)dayStart(time(nullptr))) / 86400.0);
}

void saveCds() {
  prefs.putUChar("cdN", cdCount);
  if (cdCount) prefs.putBytes("cds", cds, sizeof(Countdown) * cdCount);
}

void loadCds() {
  cdCount = prefs.getUChar("cdN", 0);
  if (cdCount > MAX_CDS) cdCount = 0;
  if (cdCount) prefs.getBytes("cds", cds, sizeof(Countdown) * cdCount);
}

void delCountdown(int i) {
  if (i < 0 || i >= cdCount) return;
  for (int k = i + 1; k < cdCount; k++) cds[k - 1] = cds[k];
  cdCount--;
  saveCds();
}

bool addCountdown(int Y, int M, int D, String label) {
  if (M < 1 || M > 12 || D < 1 || D > 31 || Y < 2024) return false;
  label = asciiOnly(label);
  label.trim();
  if (label.length() == 0) label = "the big day";
  for (int i = cdCount - 1; i >= 0; i--)
    if (strcasecmp(cds[i].label, label.c_str()) == 0) delCountdown(i);
  if (cdCount >= MAX_CDS) return false;
  struct tm t = {};
  t.tm_year = Y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = D;
  t.tm_isdst = -1;
  Countdown c;
  c.day = (uint32_t)mktime(&t);
  strncpy(c.label, label.c_str(), sizeof(c.label) - 1);
  c.label[sizeof(c.label) - 1] = 0;
  int j = cdCount++;
  while (j > 0 && cds[j - 1].day > c.day) { cds[j] = cds[j - 1]; j--; }
  cds[j] = c;
  saveCds();
  return true;
}

void handleCountdownTag(String v) {
  String lv = v;
  lv.toLowerCase();
  if (lv.startsWith("delete") || lv.startsWith("remove") || lv.startsWith("cancel")) {
    int sp = lv.indexOf(' ');
    String what = sp > 0 ? lv.substring(sp + 1) : String("");
    what.trim();
    for (int i = cdCount - 1; i >= 0; i--) {
      String lab = cds[i].label;
      lab.toLowerCase();
      if (what.length() == 0 || what == "all" || lab.indexOf(what) >= 0 || what.indexOf(lab) >= 0) delCountdown(i);
    }
    return;
  }
  String label;
  String d = splitLabel(v, label);
  int Y = 0, M = 0, D = 0;
  if (sscanf(d.c_str(), "%d-%d-%d", &Y, &M, &D) != 3) {
    if (sscanf(d.c_str(), "%d-%d", &M, &D) != 2 && sscanf(d.c_str(), "%d/%d", &M, &D) != 2) return;
    time_t n = time(nullptr);
    struct tm t;
    localtime_r(&n, &t);
    Y = t.tm_year + 1900;
    struct tm c = {};
    c.tm_year = Y - 1900;
    c.tm_mon = M - 1;
    c.tm_mday = D;
    c.tm_isdst = -1;
    if (mktime(&c) < (time_t)dayStart(n)) Y++;          // already passed this year
  }
  addCountdown(Y, M, D, label);
}

void cdCleanup() {
  if (!timeValid()) return;
  for (int i = cdCount - 1; i >= 0; i--) if (daysUntil(cds[i].day) < 0) delCountdown(i);
}

String countdownText() {
  String s;
  for (int i = 0; i < cdCount && i < 4; i++) {
    if (i) s += "; ";
    s += String(cds[i].label) + " in " + String(daysUntil(cds[i].day)) + " days";
  }
  return s;
}

// ---- games ----
void endGame() {
  game = G_NONE;
  gameMsg = "";
  if (screen == SCR_GAME) goScreen(SCR_EYES);
}

void startGame(Game g, bool talk) {
  wake();
  endGame();
  game = g;
  gameUntil = millis() + 600000;
  gTries = 0;
  rpsMe = -1;
  gameMsg = "";
  if (g == G_GUESS) {
    gSecret = random(1, 101);
    gameMsg = "?";
    if (talk) speak("I'm thinking of a number from 1 to 100. Type your guess!", WINK);
  } else if (g == G_RPS) {
    if (talk) speak("Rock, paper, scissors! Pick one.", HAPPY);
  } else if (g == G_REACT) {
    reactState = 0;
    gGoAt = millis() + 2500 + random(0, 3000);
    gameUntil = millis() + 60000;
  }
  goScreen(SCR_GAME);
}

String rpsPlay(int you, Mood& m) {
  const char* nm[3] = { "rock", "paper", "scissors" };
  if (game != G_RPS) { endGame(); game = G_RPS; }
  gameUntil = millis() + 600000;
  rpsYou = you;
  rpsMe = random(3);
  rpsResult = (you - rpsMe + 3) % 3;          // 0 tie, 1 you win, 2 Cheo wins
  String s = String("I picked ") + nm[rpsMe] + "! ";
  if (rpsResult == 0) { s += "It's a tie!"; m = SURPRISED; petGain(0, 5, 0, 3); }
  else if (rpsResult == 1) { s += "You win this one."; m = SAD; rpsLoss++; sound(35); petGain(0, 8, 0, 5); }
  else { s += "I win! Yes!"; m = COOL; rpsWins++; sound(34); petGain(0, 10, 0, 5); }
  s += " Again?";
  goScreen(SCR_GAME);
  return s;
}

String guessTurn(int n, Mood& m) {
  gTries++;
  gameUntil = millis() + 600000;
  if (n < gSecret) { gameMsg = "HIGHER"; m = WINK; return String(n) + "? Higher!"; }
  if (n > gSecret) { gameMsg = "LOWER"; m = WINK; return String(n) + "? Lower!"; }
  gameMsg = "YES!";
  m = STARS;
  sound(34);
  petGain(0, 15, 0, 15);
  gameUntil = millis() + 7000;
  String s = "Yes! It was " + String(gSecret) + "! You got it in " + String(gTries);
  s += gTries == 1 ? " try! Wow!" : " tries!";
  return s;
}

bool gameInput(const String& q, String& reply, Mood& m) {
  if (game == G_NONE) return false;
  String l = q;
  l.toLowerCase();
  if (hasWord(l, "stop") || hasWord(l, "quit") || hasWord(l, "give up") || hasWord(l, "end game")) {
    if (game == G_GUESS) reply = "Okay! My number was " + String(gSecret) + ".";
    else reply = "Okay, game over!";
    m = NORMAL;
    endGame();
    return true;
  }
  if (game == G_GUESS && gameMsg != "YES!") {
    for (unsigned int i = 0; i < l.length(); i++)
      if (isdigit((unsigned char)l[i])) { reply = guessTurn(l.substring(i).toInt(), m); return true; }
  }
  if (game == G_RPS) {
    int you = -1;
    if (hasWord(l, "rock")) you = 0;
    else if (hasWord(l, "paper")) you = 1;
    else if (hasWord(l, "scissor")) you = 2;
    if (you >= 0) { reply = rpsPlay(you, m); return true; }
  }
  return false;
}

void reactPress() {
  uint32_t now = millis();
  if (game != G_REACT) return;
  if (reactState == 0) {
    reactState = 3;
    gameMsg = "TOO SOON!";
    gameUntil = now + 4000;
    sound(35);
  } else if (reactState == 1) {
    reactMs = now - gT0;
    reactState = 2;
    gameUntil = now + 7000;
    bool rec = reactBest == 0 || reactMs < reactBest;
    if (rec) { reactBest = reactMs; prefs.putInt("rbest", reactBest); }
    sound(rec ? 34 : 9);
    petGain(0, 8, 0, 5);
    String s = String(reactMs) + " milliseconds!";
    if (rec) s += " New record!";
    else if (reactMs < 250) s += " Super fast!";
    speak(s, rec ? STARS : HAPPY);
  }
}

void gameLoop() {
  if (game == G_NONE) return;
  uint32_t now = millis();
  if (game == G_REACT && reactState == 0 && now >= gGoAt) { reactState = 1; gT0 = now; sound(36); }
  if (game == G_REACT && reactState == 1 && now - gT0 > 3000) { reactState = 3; gameMsg = "TOO SLOW!"; gameUntil = now + 3000; }
  if (now > gameUntil) endGame();
}

// ---- party ----
void startParty() {
  wake();
  partyMode = true;
  dancing = true;
  danceSawSong = false;
  danceStyle = random(3);
  const int picks[] = { 4, 3, 8, 1, 6, 0 };
  songQN = 0;
  for (int k = 0; k < 3; k++) songQ[songQN++] = picks[random(6)];
  sound(37);
  speak("Party time! Let's go!", STARS);
}

// ---- morning briefing ----
bool rainingNow() {
  int c = weatherCode;
  return (c >= 51 && c <= 67) || (c >= 80 && c <= 82) || c >= 95;
}

String briefingText() {
  time_t n = time(nullptr);
  bool tv = timeValid();
  struct tm t;
  if (tv) localtime_r(&n, &t);
  String nm = knownName();
  String s;
  if (!tv) s = "Hello";
  else if (t.tm_hour < 12) s = "Good morning";
  else if (t.tm_hour < 17) s = "Good afternoon";
  else s = "Good evening";
  if (nm.length()) s += " " + nm;
  s += "!";
  if (tv) s += " It's " + fmtClock(n, false) + ".";
  if (weatherOk) {
    String lab = weatherLabel(weatherCode);
    lab.toLowerCase();
    s += " It's " + String((int)round(weatherTemp)) + " degrees and " + lab + ", with a high of " + String((int)round(weatherHi)) + ".";
  }
  if (rainingNow()) s += " Bring an umbrella!";
  else if (rainInHours > 0) s += " Rain is likely in about " + String(rainInHours) + (rainInHours == 1 ? " hour." : " hours.");
  if (tv) {
    uint32_t end = dayStart(n) + 86400;
    int cnt = 0, first = -1;
    for (int i = 0; i < remCount; i++) if (rems[i].due < end) { cnt++; if (first < 0) first = i; }
    if (cnt == 0) s += " Nothing on your schedule today.";
    else s += " You have " + String(cnt) + (cnt == 1 ? " thing" : " things") + " today. First up: " + String(rems[first].label) + " at " + fmtClock(rems[first].due, false) + ".";
  }
  for (int i = 0; i < cdCount; i++) {
    int d = daysUntil(cds[i].day);
    if (d < 0 || d > 30) continue;
    if (d == 0) s += " Today is " + String(cds[i].label) + "!";
    else s += " " + String(d) + (d == 1 ? " day" : " days") + " until " + String(cds[i].label) + "!";
    break;
  }
  String news = fetchHeadlines("top news");
  if (news.length() > 4) {
    int nl = news.indexOf('\n');
    String h = news.substring(2, nl > 0 ? nl : news.length());
    int src = h.lastIndexOf(" - ");
    if (src > 20) h = h.substring(0, src);
    s += " In the news: " + h + ".";
  }
  if (petOn && petFood < 40) s += " Also, I'm kind of hungry.";
  return s;
}

void rainCheck() {
  if (!RAIN_WARN || !weatherOk || rainInHours <= 0 || rainingNow()) return;
  uint32_t now = millis();
  if (lastRainWarn && now - lastRainWarn < 3UL * 3600000UL) return;
  if (nightActive || studyOn || asleep || talkActive() || !someoneAround()) return;
  lastRainWarn = now;
  pendingShow = SCR_WEATHER;
  speak("Heads up! Rain is likely in about " + String(rainInHours) + (rainInHours == 1 ? " hour." : " hours.") + " Don't forget an umbrella.", SURPRISED);
  chepleEvent("rain", String(rainInHours));
}

// ---- new screens ----
void drawBar(int x, int y, const char* label, float v) {
  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(x, y + 1);
  canvas.print(label);
  uint16_t c = v > 50 ? C_GREEN : v > 20 ? C_SUN : C_RED;
  canvas.drawRoundRect(x + 46, y, 96, 10, 4, C_DGRAY);
  int w = (int)(94 * v / 100);
  if (w > 2) canvas.fillRoundRect(x + 47, y + 1, w, 8, 3, c);
}

void drawPetFace() {
  canvas.fillScreen(C_BLACK);
  drawCentered("CHEO  LV " + String(petLevel()), 4, 2, C_EYE);
  if (!petOn) { drawCentered("pet mode is off", 56, 1, C_GRAY); drawMiniEyes(110); return; }
  drawBar(8, 28, "FOOD", petFood);
  drawBar(8, 46, "FUN", petFun);
  drawBar(8, 64, "ENERGY", petEnergy);
  canvas.setTextColor(C_GRAY);
  canvas.setCursor(8, 83);
  canvas.print("XP");
  canvas.fillRect(54, 85, 96, 3, C_DGRAY);
  canvas.fillRect(54, 85, (petXP % 100) * 96 / 100, 3, C_EYE);
  drawMiniEyes(110);
}

void drawCountFace() {
  canvas.fillScreen(C_BLACK);
  drawCentered("COUNTDOWNS", 3, 1, C_EYE);
  for (int i = 0; i < cdCount && i < 3; i++) {
    int y = 16 + i * 37;
    int d = daysUntil(cds[i].day);
    String lab = cds[i].label;
    if (lab.length() > 15) lab = lab.substring(0, 15);
    canvas.setTextColor(d == 0 ? C_SUN : C_WHITE);
    if (d == 0) {
      canvas.setTextSize(2);
      canvas.setCursor(6, y + 4);
      canvas.print("TODAY!");
      canvas.setTextSize(1);
      canvas.setCursor(84, y + 8);
      canvas.print(lab);
    } else {
      canvas.setTextSize(3);
      canvas.setCursor(6, y + 2);
      canvas.print(d);
      canvas.setTextSize(1);
      canvas.setCursor(66, y + 4);
      canvas.print(lab);
      canvas.setTextColor(C_GRAY);
      canvas.setCursor(66, y + 16);
      canvas.print(d == 1 ? "day to go" : "days to go");
    }
  }
}

void drawRps(int k, int cx, int cy) {
  if (k == 0) {
    canvas.fillCircle(cx, cy, 16, C_GRAY);
    canvas.fillCircle(cx - 5, cy - 4, 4, C_DGRAY);
    canvas.fillCircle(cx + 6, cy + 5, 3, C_DGRAY);
  } else if (k == 1) {
    canvas.fillRoundRect(cx - 13, cy - 17, 26, 34, 3, C_WHITE);
    for (int i = 0; i < 4; i++) canvas.drawFastHLine(cx - 9, cy - 10 + i * 7, 18, C_GRAY);
  } else {
    thickLine(cx - 12, cy - 16, cx + 6, cy + 8, 3, C_CLOUD);
    thickLine(cx + 12, cy - 16, cx - 6, cy + 8, 3, C_CLOUD);
    canvas.drawCircle(cx - 8, cy + 12, 5, C_RED);
    canvas.drawCircle(cx + 8, cy + 12, 5, C_RED);
  }
}

void drawGameFace() {
  uint32_t now = millis();
  canvas.fillScreen(C_BLACK);
  if (game == G_GUESS) {
    drawCentered("GUESS 1-100", 6, 2, C_EYE);
    drawCentered(gameMsg, 40, 3, gameMsg == "YES!" ? C_GREEN : C_SUN);
    drawCentered("tries: " + String(gTries), 80, 1, C_GRAY);
    drawMiniEyes(110);
  } else if (game == G_RPS) {
    if (rpsMe < 0) {
      drawCentered("ROCK", 14, 2, C_WHITE);
      drawCentered("PAPER", 36, 2, C_WHITE);
      drawCentered("SCISSORS", 58, 2, C_WHITE);
      drawCentered("pick one!", 86, 1, C_GRAY);
      drawMiniEyes(110);
    } else {
      drawRps(rpsYou, 40, 40);
      drawRps(rpsMe, 120, 40);
      canvas.setTextSize(1);
      canvas.setTextColor(C_GRAY);
      canvas.setCursor(30, 66); canvas.print("YOU");
      canvas.setCursor(108, 66); canvas.print("CHEO");
      drawCentered("VS", 36, 1, C_EYE);
      const char* res = rpsResult == 0 ? "TIE" : rpsResult == 1 ? "YOU WIN" : "CHEO WINS";
      drawCentered(res, 82, 2, rpsResult == 2 ? C_EYE : rpsResult == 1 ? C_GREEN : C_SUN);
      drawCentered("Cheo " + String(rpsWins) + " - " + String(rpsLoss) + " you", 112, 1, C_GRAY);
    }
  } else if (game == G_REACT) {
    if (reactState == 0) {
      canvas.fillScreen(0x8000);
      drawCentered("WAIT...", 44, 3, C_WHITE);
      drawCentered("press when it turns green", 92, 1, C_WHITE);
    } else if (reactState == 1) {
      canvas.fillScreen(C_GREEN);
      drawCentered("PRESS!", 46, 4, C_BLACK);
    } else if (reactState == 2) {
      drawCentered(String(reactMs), 30, 4, C_EYE);
      drawCentered("milliseconds", 70, 1, C_WHITE);
      if (reactBest) drawCentered("best: " + String(reactBest) + " ms", 92, 1, C_SUN);
    } else {
      drawCentered(gameMsg, 48, 2, C_RED);
    }
  }
  (void)now;
}

void drawCallFace() {
  uint32_t t = millis();
  canvas.fillScreen(C_BLACK);
  int wob = (int)(4 * sinf(t / 50.0f));
  canvas.fillCircle(80 + wob, 34, 22, C_GREEN);
  thickLine(72 + wob, 26, 88 + wob, 42, 4, C_WHITE);
  canvas.fillCircle(71 + wob, 25, 5, C_WHITE);
  canvas.fillCircle(89 + wob, 43, 5, C_WHITE);
  drawCentered("INCOMING CALL", 64, 1, C_GRAY);
  String n = callName;
  if (n.length() > 13) n = n.substring(0, 13);
  drawCentered(n, 78, 2, C_WHITE);
  drawCentered("tap: decline", 102, 1, C_RED);
  drawCentered("hold: answer", 115, 1, C_GREEN);
}

void bootAnim() {
  for (int f = 0; f <= 18; f++) {
    canvas.fillScreen(C_BLACK);
    int h = 4 + 40 * f / 18;
    canvas.fillRoundRect(28, 48 - h / 2, 40, h, min(10, h / 2), C_EYE);
    canvas.fillRoundRect(92, 48 - h / 2, 40, h, min(10, h / 2), C_EYE);
    if (f > 6) drawCentered(OS_NAME, 86, 2, C_WHITE);
    if (f > 10) drawCentered("v" + String(FW_VERSION) + "  " + BOARD_NAME, 110, 1, C_GRAY);
    pushCanvas();
    delay(35);
  }
  delay(400);
}

// =====================================================================
//  IDLE LIFE + ANIMATION
// =====================================================================
String localComment() {
  struct tm t;
  bool tv = getLocalTime(&t, 10);
  String nm = knownName();
  String who = nm.length() ? String(", ") + nm : String("");
  int pick = random(0, 10);
  if (tv && pick < 3) {
    int h = t.tm_hour;
    if (h < 11) return "Good morning" + who + "! Did you eat breakfast?";
    if (h < 13) return "It's almost lunch time" + who + ".";
    if (h < 17) return "Afternoon check in" + who + ". How's it going?";
    if (h < 21) return "Good evening" + who + ". How was your day?";
    return "It's getting late" + who + ". Don't stay up too long!";
  }
  if (weatherOk && pick < 5) {
    int tt = (int)round(weatherTemp);
    String s = "It's " + String(tt) + " degrees outside";
    bool rain = weatherCode >= 51 && weatherCode < 70;
    if (rain) return s + ". Looks rainy, bring an umbrella!";
    if ((USE_F && tt < 40) || (!USE_F && tt < 5)) return s + ". Bring a jacket!";
    if ((USE_F && tt > 85) || (!USE_F && tt > 29)) return s + ". Stay cool!";
    return s + ".";
  }
  const char* lines[] = { "Beep boop. Just checking in.", "Did you drink some water today?", "I like sitting on your desk.",
                          "Fun fact: my brain is a chip smaller than a cookie.", "Want to hear me sing? Just ask!",
                          "Remember to stretch!", "I'm guarding the desk. Everything is safe.", "Is it snack time?",
                          "I wonder what my legs will look like.", "Boop." };
  return lines[random(10)];
}

void startIdle(IdleAnim a) {
  idleType = a;
  idleStart = millis();
  idleSoundDone = false;
  if (a == IA_YAWN) sound(25);
  if (a == IA_SIGH) sound(26);
  if (a == IA_HUM) songRequest = SONG_HUM;
}

void idleTick() {                       // moves the current idle animation along
  if (idleType == IA_NONE) return;
  uint32_t it = millis() - idleStart;
  if (idleType == IA_LOOK) lookTX = it < 900 ? -18 : it < 1900 ? 18 : 0;
  if (idleType == IA_SNEEZE && !idleSoundDone && it > 950) { sound(27); idleSoundDone = true; }
  uint32_t len = idleType == IA_LOOK ? 2600 : idleType == IA_SNEEZE ? 2600 : idleType == IA_EAT ? 1800 : 2400;
  if (it > len) { idleType = IA_NONE; lookTX = 0; }
}

void idleLife() {
  uint32_t now = millis();
  if (idleType != IA_NONE) return;
  bool calm = screen == SCR_EYES && !talkActive() && !thinking && !studyOn && !asleep && !dancing && !alertShowing && mood == baseMood();
  if (!calm) { if (nextIdle < now + 5000) nextIdle = now + 5000; return; }
  if (now > nextIdle) {
    int r = random(100);
    startIdle(r < 35 ? IA_LOOK : r < 52 ? IA_YAWN : r < 70 ? IA_HUM : r < 88 ? IA_SIGH : IA_SNEEZE);
    nextIdle = now + random(25000, 70000);
    return;
  }
  // say something now and then
  if (CHATTY > 0 && someoneAround() && WiFi.status() == WL_CONNECTED && now > nextComment && now - lastTalkTime > 300000) {
    if (!nightActive && !isNight()) {
      setMood(HAPPY, false);
      moodUntil = now + 6000;
      speak(localComment(), HAPPY);
    }
    nextComment = now + (CHATTY == 2 ? random(300000, 600000) : random(600000, 1200000));
  }
}

void updateAnimation() {
  uint32_t now = millis();

  // blinking
  if (now > nextBlink) { blinkStart = now; nextBlink = now + random(2000, 6000); }
  uint32_t bt = now - blinkStart;
  blinkAmt = bt < 160 ? fabsf(1.0f - 2.0f * bt / 160.0f) : 1.0f;
  if (blinkAmt < 0.08f) blinkAmt = 0.08f;
  if (idleType == IA_SNEEZE) { uint32_t it = now - idleStart; if (it < 1300) blinkAmt = 1.0f - 0.6f * it / 1300.0f; }

  updateLooks();
  idleTick();

  // where to look
  if (idleType != IA_LOOK) {
    if (faceVisible) { lookTX = faceX * 0.16f; lookTY = faceY * 0.08f; }
    else if (mood == THINKING || thinking) { lookTX = 10; lookTY = -8; }
    else { lookTX = 0; lookTY = 0; }
  }
  lookX += (lookTX - lookX) * 0.2f;
  lookY += (lookTY - lookY) * 0.2f;

  // mood wears off
  if (moodUntil && now > moodUntil && !talkActive()) { mood = baseMood(); moodUntil = 0; }
  if (!moodUntil && mood != baseMood() && !talkActive() && mood != THINKING) mood = baseMood();

  // dancing ends when the music does
  if (dancing) {
    if (songPlaying) danceSawSong = true;
    else if (danceSawSong && songRequest < 0 && songQN == 0) { dancing = false; danceSawSong = false; partyMode = false; setMood(HAPPY, false); moodUntil = now + 5000; }
  }

  // sleep
  if (!asleep && !studyOn && !alertShowing && !dancing && !talkActive() && !someoneAround() && now - lastInteract > SLEEP_AFTER) {
    asleep = true;
    sound(6);
  }

  // which screen
  if (screen == SCR_UPDATE) return;
  if (alertShowing) { if (screen != SCR_ALERT) goScreen(SCR_ALERT); return; }
  if (callActive) { if (screen != SCR_CALL) goScreen(SCR_CALL); return; }
  if (apMode && WiFi.status() != WL_CONNECTED) { if (screen != SCR_SETUP && !talkActive()) goScreen(SCR_SETUP); return; }
  if (now < holdUntil) return;
  if (game != G_NONE) { if (screen != SCR_GAME) goScreen(SCR_GAME); return; }
  if (talkActive() || thinking || dancing) { if (screen != SCR_EYES) goScreen(SCR_EYES); screenSince = now; idleLife(); return; }
  if (pendingShow >= 0) { goScreen((Screen)pendingShow); pendingShow = -1; holdUntil = now + HOLD_TIME; return; }
  if (studyOn) { if (screen != SCR_STUDY) goScreen(SCR_STUDY); return; }
  if (asleep) { if (screen != SCR_EYES) goScreen(SCR_EYES); return; }

  uint32_t dur = screen == SCR_EYES ? EYES_TIME : screen == SCR_CLOCK ? CLOCK_TIME : screen == SCR_WEATHER ? WEATHER_TIME :
                 (screen == SCR_COUNT || screen == SCR_PET) ? 7000 : 0;
  if (now - screenSince > dur) {
    Screen after = cdCount ? SCR_COUNT : petOn ? SCR_PET : SCR_EYES;     // eyes > clock > weather > countdowns > pet
    Screen nx = SCR_EYES;
    if (screen == SCR_EYES) nx = SCR_CLOCK;
    else if (screen == SCR_CLOCK) nx = weatherOk ? SCR_WEATHER : after;
    else if (screen == SCR_WEATHER) nx = after;
    else if (screen == SCR_COUNT) nx = petOn ? SCR_PET : SCR_EYES;
    goScreen(nx);
  }
  idleLife();
}

// =====================================================================
//  WI-FI  (saved list, finder, setup network)
// =====================================================================
bool tryConnect(const String& ssid, const String& pass, uint32_t ms) {
  setupMsg = "Trying " + ssid;
  Serial.println(setupMsg);
  if (!apMode) drawStatusFace(setupMsg);
  WiFi.disconnect();
  delay(50);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(100);
    if (apMode) { dns.processNextRequest(); server.handleClient(); }
  }
  return false;
}

// full = also try networks that didn't show up in the scan (the iPhone hotspot hides sometimes)
bool connectWiFi(bool full) {
  if (netCount == 0 && hsSsid.length() == 0) { setupMsg = "No Wi-Fi saved yet"; return false; }
  WiFi.mode(apMode ? WIFI_AP_STA : WIFI_STA);
  if (!apMode) drawStatusFace("Looking for Wi-Fi...");
  int n = WiFi.scanNetworks();
  int order[MAX_NETS], rssi[MAX_NETS], cnt = 0;
  for (int k = 0; k < netCount; k++) {
    int best = -999;
    for (int i = 0; i < n; i++)
      if (WiFi.SSID(i) == nets[k].ssid && WiFi.RSSI(i) > best) best = WiFi.RSSI(i);
    if (best > -999) {                                  // insert sorted strongest first
      int j = cnt++;
      while (j > 0 && rssi[j - 1] < best) { order[j] = order[j - 1]; rssi[j] = rssi[j - 1]; j--; }
      order[j] = k;
      rssi[j] = best;
    }
  }
  bool hsSeen = false;
  for (int i = 0; i < n; i++) if (hsSsid.length() && WiFi.SSID(i) == hsSsid) hsSeen = true;
  WiFi.scanDelete();
  Serial.printf("Scan: %d networks, %d saved ones nearby\n", n, cnt);

  for (int i = 0; i < cnt; i++)
    if (tryConnect(nets[order[i]].ssid, nets[order[i]].pass, 12000)) return true;
  if (hsSsid.length() && (hsSeen || full))
    if (tryConnect(hsSsid, hsPass, hsSeen ? 12000 : 9000)) return true;
  if (full && cnt == 0 && netCount > 0)                  // maybe a hidden network
    if (tryConnect(nets[netCount - 1].ssid, nets[netCount - 1].pass, 9000)) return true;
  setupMsg = n > 0 ? "No saved Wi-Fi nearby" : "No Wi-Fi found";
  WiFi.disconnect();
  return false;
}

void onWifiConnected() {
  Serial.printf("Connected to: %s / %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  if (apMode) {
    dns.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apMode = false;
    goScreen(SCR_EYES);
  }
  WiFi.setAutoReconnect(true);
  MDNS.end();
  if (MDNS.begin("cheo")) MDNS.addService("http", "tcp", 80);
  faceUdp.stop();
  faceUdp.begin(FACE_UDP_PORT);
  configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
  needLocate = true;
  nextWeather = millis() + 2000;
  wifiWasUp = true;
  wifiLostAt = 0;
  setupMsg = "";
}

void startSetupAP() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA);
  if (strlen(SETUP_AP_PASS) >= 8) WiFi.softAP(SETUP_AP_NAME, SETUP_AP_PASS);
  else WiFi.softAP(SETUP_AP_NAME);
  delay(100);
  dns.start(53, "*", WiFi.softAPIP());
  apRetryAt = millis() + 180000;
  Serial.printf("Setup Wi-Fi '%s' at %s\n", SETUP_AP_NAME, WiFi.softAPIP().toString().c_str());
  goScreen(SCR_SETUP);
  sound(22);
}

void wifiLoop() {
  uint32_t now = millis();
  bool up = WiFi.status() == WL_CONNECTED;
  if (apMode) {
    dns.processNextRequest();
    // every 3 min, if nobody is using the setup page, look for known Wi-Fi again
    if (now > apRetryAt && now - lastWebHit > 60000) {
      apRetryAt = now + 180000;
      if (connectWiFi(false)) onWifiConnected();
    }
    return;
  }
  if (up) {
    if (wifiLostAt) onWifiConnected();       // came back by itself: restart cheo.local, camera link, clock
    return;
  }
  if (wifiWasUp && wifiLostAt == 0) { wifiLostAt = now; Serial.println("Wi-Fi lost"); }
  // auto-reconnect handles short drops; after 30 s, scan for any saved network (e.g. you moved him)
  if (wifiLostAt && now - wifiLostAt > 30000 && now - lastWifiTry > 60000 && !talkActive()) {
    lastWifiTry = now;
    if (connectWiFi(false)) onWifiConnected();
    else if (now - wifiLostAt > 300000) startSetupAP();      // lost for 5 min: open Cheo-Setup
  }
}

// =====================================================================
//  GITHUB UPDATES (OTA)
// =====================================================================
bool otaCheck(bool announce) {
  if (WiFi.status() != WL_CONNECTED) { if (announce) speak("I need Wi-Fi to check for updates.", SAD); return false; }
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.setTimeout(8000);
  h.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!h.begin(c, String(UPDATE_URL) + "version.txt?t=" + String(millis()))) return false;   // ?t= skips GitHub's 5-minute cache
  int code = h.GET();
  if (code != 200) {
    h.end();
    lastErr = "Update check failed (" + String(code) + ")";
    if (announce) speak("I couldn't reach my update page.", CONFUSED);
    return false;
  }
  long v = h.getString().toInt();
  h.end();
  Serial.printf("Update check: GitHub has %ld, I'm %ld\n", v, FW_BUILD);
  if (v <= FW_BUILD) {
    if (announce) speak("I'm already up to date! I'm version " + String(FW_VERSION) + ".", HAPPY);
    return false;
  }
  speak("Ooh, version " + String(v >= 1000 ? v / 1000 : v) + " is out! Updating now. Don't unplug me.", STARS);
  uint32_t t0 = millis();
  while ((talkActive() || songPlaying) && millis() - t0 < 20000) { render(); delay(20); }
  ringing = false;
  searching = false;
  otaPct = 0;
  screen = SCR_UPDATE;
  transStart = 0;
  render();
  sound(30);

  otaBusy = true;                                     // pause the wake word, mic and voice player
  vPlaying = false;
  delay(300);
  drawUpdateFace(); drawCentered("downloading...", 114, 1, C_GRAY); pushCanvas();
  WiFiClientSecure c2;
  c2.setInsecure();
  c2.setHandshakeTimeout(20);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.onProgress([](int cur, int total) {
    int p = total > 0 ? (int)((int64_t)cur * 100 / total) : 0;
    if (p != otaPct) { otaPct = p; drawUpdateFace(); pushCanvas(); }
  });
  t_httpUpdate_return r = httpUpdate.update(c2, String(UPDATE_URL) + FW_FILE + "?t=" + String(millis()));
  if (r == HTTP_UPDATE_OK) {
    otaPct = 100;
    drawUpdateFace();
    drawCentered("done! restarting", 114, 1, C_GREEN);
    pushCanvas();
    sound(0);
    delay(1500);
    ESP.restart();
    return true;
  }
  otaBusy = false;
  lastErr = "Update failed: " + httpUpdate.getLastErrorString();
  Serial.println(lastErr);
  screen = SCR_EYES;
  speak("The update didn't work. I'll keep my old brain for now.", SAD);
  return false;
}

// =====================================================================
//  WEB PAGES
// =====================================================================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Cheo</title>
<style>body{font-family:-apple-system,sans-serif;background:#0b1020;color:#dfe8ff;margin:0 auto;padding:14px;max-width:520px}
h1{margin:4px 0 2px;color:#3cf}.c{background:#151c33;border-radius:14px;padding:12px;margin:10px 0}
button{background:#1d2a4a;color:#cff;border:0;border-radius:10px;padding:8px 11px;margin:3px;font-size:15px}
button.b{background:#08c;color:#fff}input,select{background:#0b1020;color:#fff;border:1px solid #345;border-radius:8px;padding:8px;font-size:16px}
#q{width:68%}#r{min-height:24px;margin-top:8px}small{color:#89a}a{color:#3cf}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid #223}
.bar{height:9px;background:#223;border-radius:5px;overflow:hidden;margin:2px 0 7px}.bar i{display:block;height:100%}</style></head><body>
<h1>Cheo <small style="font-size:14px">CDBos</small></h1><small id=net>...</small>
<div class=c id=callc style="display:none;background:#123d2a"><b>&#128222; Incoming call</b><br><button class=b onclick="g('/call?a=0')">Answer</button><button onclick="g('/call?a=1')">Decline</button></div>
<div class=c><input id=q placeholder="Talk to Cheo..." onkeydown="if(event.key=='Enter')ask()"><button class=b onclick=ask()>Send</button>
<button id=mic class=b style="touch-action:none;-webkit-user-select:none;user-select:none" oncontextmenu="return false" onpointerdown="micOn()" onpointerup="micOff()" onpointerleave="micOff()">&#127908; Hold to talk</button>
<div id=r></div><small id=err style="color:#f88"></small></div>
<div class=c><b>"Hey Cheo"</b> <small id=wk></small><br><label><input type=checkbox id=wo onchange="g('/wake?on='+(this.checked?1:0))"> Listen for "Hey Cheo"</label><br>
<a href=/train>Teach him "Hey Cheo"</a> <small>(record your voice, about 15 minutes)</small></div>
<div class=c><b>Pet</b> <small id=lv></small><div id=pet></div>
<button onclick="pa('feed&f=cheese')">&#129472; Cheese</button><button onclick="pa('feed&f=cookie')">&#127850;</button><button onclick="pa('feed&f=pizza')">&#127829;</button><button onclick="pa('feed&f=apple')">&#127822;</button><button onclick="pa('feed&f=cake')">&#127856;</button><button onclick="pa('feed&f=battery')">&#128267;</button><br>
<button onclick="pa('pet')">Pet him</button><button id=pt onclick="pa('toggle')">Pet mode</button></div>
<div class=c><b>Games</b><br><button onclick="g('/game?g=guess')">Guess my number</button><button onclick="say('Quiz me with a trivia question')">Trivia</button><button onclick="say('Tell me a riddle')">Riddle</button><br>
Rock paper scissors <button onclick="g('/game?rps=0')">&#9994;</button><button onclick="g('/game?rps=1')">&#9995;</button><button onclick="g('/game?rps=2')">&#9996;&#65039;</button><br>
<button onclick="g('/game?g=react')">Reaction test</button><button class=b onclick="g('/game?tap=1')">TAP!</button> <small id=best></small><br>
<button onclick="g('/game?g=stop')">Stop game</button><br><small>Guess my number: type numbers in the chat box. Reaction test: wait for green, then press his button (or TAP).</small></div>
<div class=c><b>Faces</b><br><span id=moods></span></div>
<div class=c><b>Legs</b> <small>(try these with him held in the air)</small><br>
<button onclick="g('/legs?do=stand')">Stand</button><button onclick="g('/legs?do=crouch')">Crouch</button><button onclick="g('/legs?do=sit')">Sit</button><br>
<button class=b onclick="g('/legs?do=walk')">Walk</button><button onclick="g('/legs?do=back')">Back</button><button onclick="g('/legs?do=stop')">Stop</button><br>
<button onclick="g('/legs?do=center')">Center (for horns)</button><button onclick="g('/legs?do=on')">Hold</button><button onclick="if(confirm('Release the legs? They will go limp.'))g('/legs?do=off')">Limp</button><br>
<small>Test one joint:</small><br><select id=lj><option value=0>L hip roll</option><option value=1>L hip pitch</option><option value=2>L knee</option><option value=3>L ankle pitch</option><option value=4>L ankle roll</option>
<option value=5>R hip roll</option><option value=6>R hip pitch</option><option value=7>R knee</option><option value=8>R ankle pitch</option><option value=9>R ankle roll</option></select>
<input type=range id=ld min=-80 max=115 value=0 style="width:45%" oninput="ldv.textContent=this.value" onchange="g('/legs?j='+lj.value+'&deg='+ld.value)"> <b id=ldv>0</b>&deg;<br><br><small>Make it straight (tap until the joint looks right, it saves by itself):</small><br>
<button onclick="tr(-15)">&minus;15</button><button onclick="tr(-5)">&minus;5</button><button onclick="tr(-1)">&minus;1</button><button onclick="tr(1)">+1</button><button onclick="tr(5)">+5</button><button onclick="tr(15)">+15</button>
<br><small id=lt></small><br><button onclick="if(confirm('Forget all saved trims?'))g('/legs?do=trimreset')">Reset trims</button>
<script>var tr=(d)=>{g('/legs?trim='+lj.value+'&d='+d).then(t=>lt.textContent=t)}</script></div>
<div class=c><b>Screen</b><br><button onclick="g('/show?s=eyes')">Eyes</button><button onclick="g('/show?s=clock')">Clock</button><button onclick="g('/show?s=weather')">Weather</button><button onclick="g('/show?s=pet')">Pet</button><button onclick="g('/show?s=count')">Countdowns</button>
<br>Volume <input type=range id=vol min=0 max=100 onchange="g('/vol?v='+this.value)"></div>
<div class=c><b>Look</b><br>Eye color <select id=eye onchange="g('/eyes?c='+this.value)"></select><br>
<label><input type=checkbox id=nt onchange="g('/night?on='+(this.checked?1:0))"> Night mode (dim red eyes + quiet, 10 PM - 7 AM)</label></div>
<div class=c><b>Songs &amp; dancing</b><br><select id=song><option value=twinkle>Twinkle Twinkle</option><option value=ode>Ode to Joy</option>
<option value=birthday>Happy Birthday</option><option value=jingle>Jingle Bells</option><option value=row>Row Your Boat</option><option value=mary>Mary Had a Little Lamb</option>
<option value=saints>When the Saints</option><option value=elise>Fur Elise</option><option value=theme>Cheo's Theme</option></select>
<button onclick="g('/sing?s='+song.value)">Sing</button><button onclick="g('/dance')">Dance</button><button onclick="g('/party')">Party &#127881;</button><button onclick="g('/stopfun')">Stop</button></div>
<div class=c><b>Briefing</b><br><button onclick="g('/brief')">Play my briefing</button><br><small>Weather, rain, today's reminders, countdowns and news. He also does it when a morning alarm goes off.</small></div>
<div class=c><b>Countdowns</b><div id=cds></div><input id=cdd type=date> <input id=cdl placeholder="What for?" style="width:40%"> <button onclick=addcd()>Add</button></div>
<div class=c><b>Study mode</b><br>Focus <input id=sf type=number value=25 style="width:54px"> Break <input id=sb type=number value=5 style="width:54px"> min<br>
<button class=b onclick="g('/study?f='+sf.value+'&b='+sb.value)">Start</button><button onclick="g('/study?stop=1')">Stop</button></div>
<div class=c><b>Timers &amp; reminders</b><br>Quick timer <input id=tm type=number value=5 style="width:54px"> min <button onclick="g('/timer?m='+tm.value).then(load)">Start</button>
<button onclick="g('/dismiss')">Stop ringing</button><div id=rems></div><small>Or just ask: "set a timer for 10 minutes", "wake me up at 7 every day", "remind me tomorrow at 4 to call Grandma".</small></div>
<div class=c><b>Memory</b><div id=mem></div><button onclick="if(confirm('Make Cheo forget everything?'))g('/forget').then(load)">Forget all</button></div>
<div class=c><button onclick="g('/update').then(()=>alert('Cheo is checking GitHub for updates'))">Check for updates</button> <a href=/settings>Wi-Fi &amp; settings</a></div>
<script>
const M=['normal','happy','sad','angry','surprised','sleepy','love','thinking','wink','dizzy','cry','cool','stars','confused'];
const E=['blue','green','pink','orange','purple','white','red','rainbow'];
var g=(u)=>{return fetch(u).then(r=>r.text())}
var esc=(s)=>{return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
var bar=(n,v)=>{return '<small>'+n+' '+Math.round(v)+'%</small><div class=bar><i style="width:'+v+'%;background:'+(v>50?'#3c8':v>20?'#fc3':'#f55')+'"></i></div>'}
moods.innerHTML=M.map((m,i)=>`<button onclick="g('/mood?m=${i}')">${m}</button>`).join('');
eye.innerHTML=E.map((e,i)=>`<option value=${i}>${e}</option>`).join('');
let waiting=0;
var ask=()=>{let v=q.value.trim();if(!v)return;q.value='';r.textContent='...';err.textContent='';g('/ask?q='+encodeURIComponent(v));waiting=1}
var say=(t)=>{q.value=t;ask()}
let micDown=0;
var micOn=()=>{micDown=1;mic.textContent='Listening... let go when done';g('/mic?on=1')}
var micOff=()=>{if(!micDown)return;micDown=0;mic.textContent='\u{1F3A4} Hold to talk';g('/mic?on=0');r.textContent='...';err.textContent='';waiting=1}
var pa=(a)=>{g('/pet?a='+a).then(load)}
var addcd=()=>{if(!cdd.value)return;g('/cdadd?d='+cdd.value+'&l='+encodeURIComponent(cdl.value)).then(()=>{cdl.value='';load()})}
setInterval(()=>{if(waiting)fetch('/last').then(x=>x.json()).then(j=>{if(!j.busy&&!j.listening){r.textContent=(j.heard?'You: '+j.heard+'\n':'')+'Cheo: '+j.reply;err.textContent=j.err;waiting=0;load()}})},700);
var load=()=>{
fetch('/net').then(x=>x.json()).then(j=>{net.textContent=j.os+' v'+j.ver+' · '+j.board+' · '+(j.ssid||'no Wi-Fi')+' · '+j.ip+' · iPhone '+(j.ble?'connected':'not connected')+' · camera '+(j.cam?'on':'off');
vol.value=j.vol;if(j.err&&!waiting)err.textContent=j.err;lv.textContent='level '+j.lv+' · '+j.xp+' XP';
pet.innerHTML=j.pon?bar('Food',j.food)+bar('Fun',j.fun)+bar('Energy',j.en):'<small>pet mode is off</small>';pt.textContent=j.pon?'Turn pet mode off':'Turn pet mode on';
wk.textContent=j.wake?(j.wakeon?'listening':'off'):'not trained yet';wo.checked=j.wakeon;eye.value=j.eye;nt.checked=j.night;best.textContent=j.best?'best: '+j.best+' ms':'';callc.style.display=j.call?'block':'none'});
fetch('/rems').then(x=>x.json()).then(a=>{rems.innerHTML=a.length?a.map((t,i)=>`<div class=row><span>${esc(t)}</span><button onclick="g('/delrem?i=${i}').then(load)">✕</button></div>`).join(''):'<small>none</small>'});
fetch('/cds').then(x=>x.json()).then(a=>{cds.innerHTML=a.length?a.map((c,i)=>`<div class=row><span>${esc(c.l)}</span><span>${c.d==0?'TODAY!':c.d+(c.d==1?' day':' days')} <button onclick="g('/cddel?i=${i}').then(load)">✕</button></span></div>`).join(''):'<small>none yet</small>'});
fetch('/memory').then(x=>x.text()).then(t=>{let l=t.split('\n').filter(x=>x);mem.innerHTML=l.length?l.map(x=>'<div class=row>'+esc(x)+'</div>').join(''):'<small>nothing yet</small>'})}
load();setInterval(load,5000);
</script></body></html>)HTML";

const char SETTINGS_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>CDBos settings</title>
<style>body{font-family:-apple-system,sans-serif;background:#0b1020;color:#dfe8ff;margin:0 auto;padding:14px;max-width:520px}
h1{margin:4px 0 2px;color:#3cf}.c{background:#151c33;border-radius:14px;padding:12px;margin:10px 0}
button{background:#1d2a4a;color:#cff;border:0;border-radius:10px;padding:8px 11px;margin:3px;font-size:15px}
button.b{background:#08c;color:#fff}input{background:#0b1020;color:#fff;border:1px solid #345;border-radius:8px;padding:8px;font-size:16px;width:90%;margin:3px 0}
small{color:#89a}a{color:#3cf}.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #223}
.n{cursor:pointer}</style></head><body>
<h1>CDBos settings</h1><small>Cheo v<span id=ver></span> · <span id=cur></span></small>
<div class=c><b>Saved Wi-Fi</b> <small>(up to 10)</small><div id=saved></div>
<small>Cheo connects to the strongest saved network he can see, so you only type each password once.</small></div>
<div class=c><b>Add Wi-Fi</b><br><button onclick=scan()>Find nearby Wi-Fi</button><div id=found></div>
<input id=ss placeholder="Network name"><input id=pw type=password placeholder="Password (blank if none)">
<button class=b onclick=add()>Save network</button><div id=msg></div></div>
<div class=c><b>iPhone hotspot</b> <small>(backup when nothing else works)</small><input id=hs placeholder="Hotspot name"><input id=hp type=password placeholder="Password (blank = keep)">
<b>Groq key</b><input id=gk type=password placeholder="Groq key">
<b>Edge Impulse API key</b> <small>(for teaching "Hey Cheo")</small><input id=ek type=password placeholder="ei_..."><button class=b onclick=save()>Save</button></div>
<div class=c><b>Neck, arms + screen</b> <small id=armhw></small>
<div class=row><span>Neck center</span><span><button onclick="nt(-5)">&#9664;&#9664;</button><button onclick="nt(-1)">&#9664;</button> <b id=ntv>0</b> <button onclick="nt(1)">&#9654;</button><button onclick="nt(5)">&#9654;&#9654;</button></span></div>
<small>Tap until his face points straight ahead. Saved right away.</small>
<div class=row><span>Left arm down</span><span><button onclick="at('atl',-2)">&#8722;</button> <b id=atlv>0</b> <button onclick="at('atl',2)">+</button></span></div>
<div class=row><span>Right arm down</span><span><button onclick="at('atr',-2)">&#8722;</button> <b id=atrv>0</b> <button onclick="at('atr',2)">+</button></span></div>
<small>Tap until each arm hangs straight down.</small>
<div class=row><span>Arms on</span><input type=checkbox id=armon style="width:auto" onchange="mset('arms',armon.checked?1:0)"></div>
<div class=row><span>Little idle moves</span><input type=checkbox id=idl style="width:auto" onchange="mset('idle',idl.checked?1:0)"></div>
<div class=row><span>Flip screen</span><input type=checkbox id=sfl style="width:auto" onchange="mset('sflip',sfl.checked?1:0)"></div>
<button onclick="mset('adir',-M.adir)">Arms go the wrong way</button><button onclick="mset('ndir',-M.ndir)">Left/right swapped</button><br>
<button onclick="g('/move?do=wave')">Wave</button><button onclick="g('/move?do=cheer')">Cheer</button><button onclick="g('/move?do=hug')">Hug</button>
<button onclick="g('/move?do=point')">Point</button><button onclick="g('/move?do=shake')">Shake head</button><button onclick="g('/move?do=around')">Look around</button><button onclick="g('/move?do=stop')">Stop</button></div>
<div class=c><b>Memory backup</b> <small>(copy this before moving Cheo to a new board, paste it back after)</small>
<textarea id=bk rows=5 style="width:95%;background:#0b1020;color:#fff;border:1px solid #345;border-radius:8px;font-size:14px"></textarea>
<button onclick="post('/restore',{m:bk.value}).then(t=>alert(t))">Restore memory</button></div>
<div class=c><button onclick="g('/update').then(()=>alert('Checking GitHub...'))">Check for updates</button>
<button onclick="if(confirm('Restart Cheo?'))g('/reboot')">Restart</button> <a href=/>Back</a></div>
<script>
var g=(u)=>{return fetch(u).then(r=>r.text())}
var post=(u,d)=>{return fetch(u,{method:'POST',body:new URLSearchParams(d)}).then(r=>r.text())}
var esc=(s)=>{return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
var bars=(r)=>{return r>-55?'▂▄▆█':r>-67?'▂▄▆':r>-78?'▂▄':'▂'}
fetch('/memory').then(r=>r.text()).then(t=>{bk.value=t});
var load=()=>{fetch('/wifi').then(r=>r.json()).then(j=>{ver.textContent=j.ver;cur.textContent=j.cur?'on '+j.cur:'not connected';hs.value=j.hs;
gk.placeholder=j.key?'saved ✓ (blank keeps it)':'paste your Groq key';ek.placeholder=j.ei?'saved ✓ (blank keeps it)':'ei_...';
saved.innerHTML=j.nets.length?j.nets.map((n,i)=>`<div class=row><span>${esc(n)}${n==j.cur?' ✓':''}</span><button onclick="g('/delwifi?i=${i}').then(load)">✕</button></div>`).join(''):'<small>none saved yet</small>'})}
var pick=(el)=>{ss.value=el.dataset.s;pw.value='';pw.focus()}
var scan=()=>{found.innerHTML='<small>searching...</small>';var p=()=>{fetch('/scan').then(r=>r.json()).then(j=>{if(j.busy)return setTimeout(p,1200);
found.innerHTML=j.list.length?j.list.map(n=>`<div class="row n" data-s="${esc(n.s)}" onclick="pick(this)"><span>${esc(n.s)}${n.k?' <small>(saved)</small>':''}</span><small>${n.lock?'🔒':'open'} ${bars(n.r)}</small></div>`).join(''):'<small>nothing found</small>'})};p()}
var add=()=>{if(!ss.value)return;msg.textContent='saving...';post('/addwifi',{s:ss.value,p:pw.value}).then(t=>{msg.textContent=t;pw.value='';load()})}
var save=()=>{post('/save',{hs:hs.value,hp:hp.value,gk:gk.value,ek:ek.value}).then(t=>{alert(t);hp.value='';gk.value='';ek.value='';load()})}
var M={};
var mshow=(j)=>{M=j;ntv.textContent=j.ntrim;atlv.textContent=j.atl;atrv.textContent=j.atr;armon.checked=j.arms==1;idl.checked=j.idle==1;sfl.checked=j.sflip==1;
armhw.textContent=j.pca?'(arms on PCA9685 ch 10 + 11)':'(arms on pins 40 + 41)'}
var mset=(k,v)=>{fetch('/motion?'+k+'='+v).then(r=>r.json()).then(mshow)}
var nt=(d)=>{mset('ntrim',(M.ntrim||0)+d)}
var at=(k,d)=>{mset(k,(M[k]||0)+d)}
fetch('/motion').then(r=>r.json()).then(mshow);
load();
</script></body></html>)HTML";
const char TRAIN_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Teach Cheo</title>
<style>body{font-family:-apple-system,sans-serif;background:#0b1020;color:#dfe8ff;margin:0 auto;padding:14px;max-width:520px}
h1{margin:4px 0 2px;color:#3cf}.c{background:#151c33;border-radius:14px;padding:12px;margin:10px 0}
button{background:#1d2a4a;color:#cff;border:0;border-radius:10px;padding:10px 14px;margin:4px 3px 0 0;font-size:16px}
button.b{background:#08c;color:#fff}small{color:#89a}a{color:#3cf}</style></head><body>
<h1>Teach Cheo "Hey Cheo"</h1>
<div class=c><b>Before you start</b><br>1. Make a free account at <a href=https://studio.edgeimpulse.com target=_blank>edgeimpulse.com</a> and create a project named <b>cheo-wake</b>.<br>
2. In the project: <b>Dashboard &rarr; Keys</b>, copy the API key (starts with <b>ei_</b>) and paste it on the <a href=/settings>Settings</a> page.<br>
3. Tap a button below. Cheo beeps 10 times; each beep records 1 second.</div>
<div class=c><b>"Hey Cheo"</b> &mdash; do this about <b>10 times</b> (100 clips)<br><small>Say it right after each beep. Mix it up: loud, quiet, fast, slow, close, across the room.</small><br>
<button class=b onclick="t('hey_cheo')">Record 10 &times; "Hey Cheo"</button></div>
<div class=c><b>Background noise</b> &mdash; about <b>5 times</b> (50 clips)<br><small>Don't talk. Quiet room, TV on, music, typing, the dog, walking around.</small><br>
<button onclick="t('noise')">Record 10 &times; noise</button></div>
<div class=c><b>Other words</b> &mdash; about <b>5 times</b> (50 clips)<br><small>Say anything except "Hey Cheo": hey, cheese, hello, Cheple, okay, names, numbers...</small><br>
<button onclick="t('unknown')">Record 10 &times; other words</button></div>
<div class=c><b id=st>ready</b><br><small id=tm></small></div>
<a href=/>&larr; Back</a>
<script>
var g=(u)=>{return fetch(u).then(r=>r.text())}
var t=(l)=>{g('/train/start?l='+l+'&n=10').then(()=>setTimeout(poll,300))}
var nm={hey_cheo:'"Hey Cheo"',noise:'noise',unknown:'other words'}
var poll=()=>{fetch('/train/status').then(r=>r.json()).then(j=>{st.textContent=j.left?('Recording '+nm[j.label]+': '+j.done+' sent, '+j.left+' to go'):(j.done?('Done! '+j.done+' clips sent'+(j.fail?', '+j.fail+' failed':'')):'ready');
tm.textContent=(j.msg?j.msg+' ':'')+'Clips sent so far: '+j.total;if(j.left)setTimeout(poll,600)})}
poll();
</script></body></html>)HTML";

void handleScan() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) { WiFi.scanNetworks(true); server.send(200, "application/json", "{\"busy\":true}"); return; }
  if (n == WIFI_SCAN_RUNNING) { server.send(200, "application/json", "{\"busy\":true}"); return; }
  // keep the strongest of each name, sorted strongest first
  String names[24];
  int rs[24];
  bool lock[24];
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;
    int r = WiFi.RSSI(i);
    int j = -1;
    for (int k = 0; k < cnt; k++) if (names[k] == s) j = k;
    if (j >= 0) { if (r > rs[j]) rs[j] = r; continue; }
    if (cnt >= 24) continue;
    names[cnt] = s;
    rs[cnt] = r;
    lock[cnt] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    cnt++;
  }
  WiFi.scanDelete();
  for (int i = 1; i < cnt; i++)
    for (int j = i; j > 0 && rs[j] > rs[j - 1]; j--) {
      String ts = names[j]; names[j] = names[j - 1]; names[j - 1] = ts;
      int tr = rs[j]; rs[j] = rs[j - 1]; rs[j - 1] = tr;
      bool tl = lock[j]; lock[j] = lock[j - 1]; lock[j - 1] = tl;
    }
  JsonDocument d;
  d["busy"] = false;
  JsonArray a = d["list"].to<JsonArray>();
  for (int i = 0; i < cnt; i++) {
    JsonObject o = a.add<JsonObject>();
    o["s"] = names[i];
    o["r"] = rs[i];
    o["lock"] = lock[i];
    bool known = names[i] == hsSsid;
    for (int k = 0; k < netCount; k++) if (nets[k].ssid == names[i]) known = true;
    o["k"] = known;
  }
  String out;
  serializeJson(d, out);
  server.send(200, "application/json", out);
}

void setupWeb() {
  server.on("/", []() { lastWebHit = millis(); if (apMode) { server.sendHeader("Location", "/settings"); server.send(302, "text/plain", ""); return; } server.send_P(200, "text/html", PAGE); });
  server.on("/settings", []() { lastWebHit = millis(); server.send_P(200, "text/html", SETTINGS_PAGE); });
  server.on("/ask", []() {
    lastHeard = "";
    pendingQ = server.arg("q");
    pendingQ.trim();
    if (pendingQ.length()) { hasPending = true; busy = true; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/train", []() { server.send_P(200, "text/html", TRAIN_PAGE); });
  server.on("/train/start", []() { startTraining(server.arg("l"), server.arg("n").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/train/status", []() {
    JsonDocument d;
    d["label"] = trainLabel;
    d["left"] = trainLeft;
    d["done"] = trainDone;
    d["fail"] = trainFail;
    d["total"] = trainTotal;
    d["msg"] = trainMsg;
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/wake", []() { wakeOn = server.arg("on") == "1"; prefs.putUChar("wake", wakeOn ? 1 : 0); server.send(200, "text/plain", "ok"); });
  server.on("/mic", []() {
    if (server.arg("on") == "1") startListening(); else stopListening();
    server.send(200, "text/plain", micReady ? "ok" : "no mic");
  });
  server.on("/last", []() {
    JsonDocument d;
    d["reply"] = lastReply;
    d["heard"] = lastHeard;
    d["listening"] = recording || wantTranscribe;
    d["busy"] = busy || hasPending;
    d["err"] = lastErr;
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/mood", []() {
    int m = server.arg("m").toInt();
    if (m >= 0 && m < MOOD_COUNT) { wake(); setMood((Mood)m); moodUntil = millis() + 20000; goScreen(SCR_EYES); holdUntil = millis() + HOLD_TIME; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/show", []() {
    String s = server.arg("s");
    wake();
    pendingShow = s == "clock" ? SCR_CLOCK : s == "weather" ? SCR_WEATHER : s == "pet" ? SCR_PET : s == "count" ? SCR_COUNT : SCR_EYES;
    holdUntil = 0;
    server.send(200, "text/plain", "ok");
  });
  server.on("/vol", []() { setVolume(server.arg("v").toInt()); sound(10); server.send(200, "text/plain", "ok"); });
  server.on("/net", []() {
    JsonDocument d;
    d["ver"] = FW_VERSION;
    d["ssid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("");
    d["ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("192.168.4.1");
    d["rssi"] = WiFi.RSSI();
    d["ble"] = (bool)ancsReady;
    d["cam"] = millis() - lastCamPacket < 10000;
    d["vol"] = volume;
    d["err"] = lastErr;
    d["os"] = OS_NAME;
    d["board"] = BOARD_NAME;
    d["lv"] = petLevel();
    d["xp"] = petXP;
    d["pon"] = petOn;
    d["food"] = (int)petFood;
    d["fun"] = (int)petFun;
    d["en"] = (int)petEnergy;
    d["eye"] = eyeTheme;
    d["night"] = nightOn;
    d["best"] = reactBest;
    d["call"] = callActive;
    d["mic"] = micReady;
    d["wake"] = (bool)HAS_WAKE;
    d["wakeon"] = wakeOn;
    d["build"] = FW_BUILD;
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/memory", []() { server.send(200, "text/plain", memFacts); });
  server.on("/forget", []() { forgetAll(); server.send(200, "text/plain", "ok"); });
  server.on("/rems", []() {
    JsonDocument d;
    JsonArray a = d.to<JsonArray>();
    for (int i = 0; i < remCount; i++) a.add(remDesc(i));
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/delrem", []() { delRem(server.arg("i").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/timer", []() {
    int m = server.arg("m").toInt();
    if (m > 0 && timeValid()) addRem(time(nullptr) + m * 60, 0, false, 0, 0, String(m) + " minute");
    server.send(200, "text/plain", "ok");
  });
  server.on("/dismiss", []() { dismissAlert(); server.send(200, "text/plain", "ok"); });
  server.on("/sing", []() { wake(); wantSong = findSong(server.arg("s")); server.send(200, "text/plain", "ok"); });
  server.on("/dance", []() { wake(); wantDance = true; server.send(200, "text/plain", "ok"); });
  server.on("/stopfun", []() { if (songPlaying) songStop = true; songQN = 0; dancing = false; partyMode = false; server.send(200, "text/plain", "ok"); });
  server.on("/study", []() {
    if (server.hasArg("stop")) stopStudy();
    else startStudy(server.arg("f").toInt(), server.arg("b").toInt());
    server.send(200, "text/plain", "ok");
  });
  server.on("/update", []() { otaRequested = true; server.send(200, "text/plain", "ok"); });
  server.on("/reboot", []() { server.send(200, "text/plain", "restarting"); restartAt = millis() + 800; });

  // ---- CDBos v10 ----
  server.on("/pet", []() {
    String a = server.arg("a");
    if (a == "feed") { String rep = feedPet(server.arg("f")); speak(rep, mood); }
    else if (a == "pet") {
      wake();
      petGain(0, 8, 0, 3);
      setMood(LOVE);
      moodUntil = millis() + 6000;
      goScreen(SCR_EYES);
      holdUntil = millis() + 5000;
      speak(random(2) ? "Hehe, that tickles!" : "Aww, thank you!", LOVE);
    } else if (a == "toggle") { petOn = !petOn; prefs.putUChar("petOn", petOn ? 1 : 0); if (!petOn) mood = NORMAL; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/game", []() {
    if (server.hasArg("tap")) { reactPress(); }
    else if (server.hasArg("rps")) {
      int you = server.arg("rps").toInt();
      if (you >= 0 && you < 3) {
        wake();
        Mood m = HAPPY;
        String s = rpsPlay(you, m);
        setMood(m, false);
        moodUntil = millis() + 8000;
        speak(s, m);
      }
    } else {
      String gn = server.arg("g");
      if (gn == "stop") endGame();
      else if (gn == "guess") startGame(G_GUESS, true);
      else if (gn == "rps") startGame(G_RPS, true);
      else if (gn == "react") startGame(G_REACT, true);
    }
    server.send(200, "text/plain", "ok");
  });
  server.on("/cds", []() {
    JsonDocument d;
    JsonArray a = d.to<JsonArray>();
    for (int i = 0; i < cdCount; i++) {
      JsonObject o = a.add<JsonObject>();
      o["l"] = cds[i].label;
      o["d"] = timeValid() ? daysUntil(cds[i].day) : -1;
    }
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/cdadd", []() {
    int Y, M, D;
    if (sscanf(server.arg("d").c_str(), "%d-%d-%d", &Y, &M, &D) == 3) addCountdown(Y, M, D, server.arg("l"));
    server.send(200, "text/plain", "ok");
  });
  server.on("/cddel", []() { delCountdown(server.arg("i").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/eyes", []() { setEyeColor(server.arg("c").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/night", []() { setNight(server.arg("on") == "1"); server.send(200, "text/plain", "ok"); });
  server.on("/party", []() { wantParty = true; server.send(200, "text/plain", "ok"); });
  server.on("/brief", []() { wantBriefing = true; server.send(200, "text/plain", "ok"); });
  server.on("/call", []() { if (callActive) ancsAction = server.arg("a").toInt() ? 1 : 0; server.send(200, "text/plain", "ok"); });
  server.on("/restore", HTTP_POST, []() {
    String m = server.arg("m");
    m.replace("\r", "");
    memFacts = m.substring(0, 900);
    if (memFacts.length() && !memFacts.endsWith("\n")) memFacts += "\n";
    prefs.putString("facts", memFacts);
    server.send(200, "text/plain", "Memory restored!");
  });

  // ---- Cheple link API (for the Cheple phone, or anything on your Wi-Fi) ----
  server.on("/api/status", []() {
    JsonDocument d;
    d["name"] = "Cheo";
    d["os"] = OS_NAME;
    d["ver"] = FW_VERSION;
    d["board"] = BOARD_NAME;
    d["mood"] = moodNames[mood];
    d["talking"] = talkActive();
    d["asleep"] = asleep;
    d["level"] = petLevel();
    d["food"] = (int)petFood;
    d["fun"] = (int)petFun;
    d["energy"] = (int)petEnergy;
    if (remCount) d["next"] = remDesc(0);
    if (weatherOk) { d["temp"] = (int)round(weatherTemp); d["weather"] = weatherLabel(weatherCode); d["rainIn"] = rainInHours; }
    d["reply"] = lastReply;
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/api/say", []() {
    int mi = moodIndex(server.arg("m"));
    Mood m = mi >= 0 ? (Mood)mi : NORMAL;
    wake();
    if (mi >= 0) { setMood(m, false); moodUntil = millis() + 8000; }
    speak(server.arg("t"), m);
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/ask", []() {
    pendingQ = server.arg("q");
    pendingQ.trim();
    if (pendingQ.length()) { hasPending = true; busy = true; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/timer", []() {
    long secs = server.arg("s").toInt();
    String l = server.arg("l");
    if (l.length() == 0) l = "timer";
    if (secs > 0 && timeValid()) addRem(time(nullptr) + secs, 0, false, 0, 0, l);
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/face", []() {
    int mi = moodIndex(server.arg("m"));
    if (mi < 0) mi = server.arg("m").toInt();
    if (mi >= 0 && mi < MOOD_COUNT) { wake(); setMood((Mood)mi); moodUntil = millis() + 20000; goScreen(SCR_EYES); holdUntil = millis() + 8000; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/show", []() {
    String s = server.arg("s");
    wake();
    pendingShow = s == "clock" ? SCR_CLOCK : s == "weather" ? SCR_WEATHER : s == "pet" ? SCR_PET : s == "count" ? SCR_COUNT : SCR_EYES;
    holdUntil = 0;
    server.send(200, "text/plain", "ok");
  });
  server.on("/api/feed", []() { speak(feedPet(server.arg("f")), mood); server.send(200, "text/plain", "ok"); });

  // ---- Wi-Fi settings ----
  server.on("/scan", []() { lastWebHit = millis(); handleScan(); });
  server.on("/wifi", []() {
    lastWebHit = millis();
    JsonDocument d;
    d["ver"] = FW_VERSION;
    d["cur"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("");
    d["hs"] = hsSsid;
    d["key"] = groqKey.length() > 10;
    d["ei"] = eiKey.length() > 10;
    JsonArray a = d["nets"].to<JsonArray>();
    for (int i = 0; i < netCount; i++) a.add(nets[i].ssid);
    String o;
    serializeJson(d, o);
    server.send(200, "application/json", o);
  });
  server.on("/addwifi", HTTP_POST, []() {
    lastWebHit = millis();
    String s = server.arg("s"), p = server.arg("p");
    s.trim();
    if (s.length() == 0) { server.send(200, "text/plain", "Type a network name first."); return; }
    if (p.length() > 0 && p.length() < 8) { server.send(200, "text/plain", "Wi-Fi passwords are at least 8 characters. Check it and try again."); return; }
    addNetwork(s, p);
    if (apMode || WiFi.status() != WL_CONNECTED) {
      server.send(200, "text/plain", "Saved! Cheo is restarting to join " + s + ". Your phone will go back to normal Wi-Fi.");
      restartAt = millis() + 1500;
    } else {
      server.send(200, "text/plain", "Saved! Cheo will join " + s + " whenever he's near it.");
    }
  });
  server.on("/move", []() {                  // v16: /move?do=wave|cheer|hug|point|shrug|shake|left|right|around|stop
    String d = server.arg("do");
    Gest g = d == "wave" ? GS_WAVE : d == "cheer" ? GS_CHEER : d == "hug" ? GS_HUG : d == "point" ? GS_POINT : d == "shrug" ? GS_SHRUG :
             d == "shake" ? GS_SHAKE : d == "left" ? GS_LOOKL : d == "right" ? GS_LOOKR : d == "around" ? GS_LOOKAROUND : GS_NONE;
    gesture(g); if (g == GS_NONE) manualNeckUntil = manualArmUntil = 0;
    server.send(200, "text/plain", "ok");
  });
  server.on("/pose", []() {                  // /pose?neck=90&l=0&r=0  holds for 15 s (for lining things up)
    if (server.hasArg("neck")) { manualNeck = constrain(server.arg("neck").toFloat(), PAN_MIN, PAN_MAX); manualNeckUntil = millis() + 15000; }
    if (server.hasArg("l") || server.hasArg("r")) { manualArmL = server.arg("l").toFloat(); manualArmR = server.arg("r").toFloat(); manualArmUntil = millis() + 15000; }
    server.send(200, "text/plain", "ok");
  });
  server.on("/legs", []() {                  // /legs?do=walk|back|stop|stand|sit|crouch|center|on|off   /legs?j=2&deg=30
    String d = server.arg("do");
    if (server.hasArg("trim")) { server.send(200, "text/plain", legsNudge(server.arg("trim").toInt(), server.arg("d").toInt())); return; }
    if (server.hasArg("raw")) { server.send(200, "text/plain", legsRaw(server.arg("raw").toInt(), server.arg("ang").toFloat())); return; }
    if (d == "walk") legsWalk(1); else if (d == "back") legsWalk(-1); else if (d == "stop") legsWalk(0);
    else if (d == "stand") legsStand(); else if (d == "sit") legsSit(); else if (d == "crouch") legsCrouch();
    else if (d == "off") legsSetOn(false); else if (d == "on") legsSetOn(true);
    else if (d == "trimreset") legsTrimReset();
    else if (d == "center") { legsStand(); legsSetJoint(2, 45); legsSetJoint(7, 45); }   // every servo at 90 deg, knees bent 45 (for fitting the horns)
    if (server.hasArg("j")) legsSetJoint(server.arg("j").toInt(), server.arg("deg").toFloat());
    server.send(200, "text/plain", legsAvailable() ? "ok" : "no legs (needs the S3 + PCA9685)");
  });
  server.on("/motion", []() {                // read + change the motion settings
    if (server.hasArg("ntrim")) { neckTrim = constrain(server.arg("ntrim").toInt(), -60, 60); prefs.putChar("ntrim", neckTrim); manualNeck = PAN_CENTER; manualNeckUntil = millis() + 8000; }
    if (server.hasArg("ndir")) { neckDir = server.arg("ndir").toInt() < 0 ? -1 : 1; prefs.putChar("ndir", neckDir); }
    if (server.hasArg("atl")) { armTrimL = constrain(server.arg("atl").toInt(), -40, 40); prefs.putChar("atl", armTrimL); manualArmL = manualArmR = 0; manualArmUntil = millis() + 8000; }
    if (server.hasArg("atr")) { armTrimR = constrain(server.arg("atr").toInt(), -40, 40); prefs.putChar("atr", armTrimR); manualArmL = manualArmR = 0; manualArmUntil = millis() + 8000; }
    if (server.hasArg("adir")) { armDir = server.arg("adir").toInt() < 0 ? -1 : 1; prefs.putChar("adir", armDir); }
    if (server.hasArg("arms")) { armsOn = server.arg("arms") == "1"; prefs.putUChar("arms", armsOn); }
    if (server.hasArg("idle")) { idleMoves = server.arg("idle") == "1"; prefs.putUChar("idlemv", idleMoves); }
    if (server.hasArg("sflip")) { screenFlip = server.arg("sflip") == "1"; prefs.putUChar("sflip", screenFlip); tft.setRotation((SCREEN_ROTATION + (screenFlip ? 2 : 0)) % 4); }
    String j = "{\"ntrim\":" + String(neckTrim) + ",\"ndir\":" + String(neckDir) + ",\"atl\":" + String(armTrimL) + ",\"atr\":" + String(armTrimR) +
               ",\"adir\":" + String(armDir) + ",\"arms\":" + String(armsOn ? 1 : 0) + ",\"idle\":" + String(idleMoves ? 1 : 0) +
               ",\"sflip\":" + String(screenFlip ? 1 : 0) + ",\"pca\":" + String(pcaFound ? 1 : 0) + "}";
    server.send(200, "application/json", j);
  });
  server.on("/delwifi", []() { delNetwork(server.arg("i").toInt()); server.send(200, "text/plain", "ok"); });
  server.on("/save", HTTP_POST, []() {
    lastWebHit = millis();
    String hs = server.arg("hs"), hp = server.arg("hp"), gk = server.arg("gk");
    hs.trim(); gk.trim();
    if (hs != hsSsid) { hsSsid = hs; prefs.putString("hs_ssid", hsSsid); }
    if (hp.length()) { hsPass = hp; prefs.putString("hs_pass", hsPass); }
    if (gk.length()) { groqKey = gk; prefs.putString("groq", groqKey); }
    String ek = server.arg("ek");
    ek.trim();
    if (ek.length()) { eiKey = ek; prefs.putString("ei", eiKey); }
    server.send(200, "text/plain", "Saved!");
    if (apMode) restartAt = millis() + 1500;
  });

  // phones look for these to detect a sign-in page; send them to Settings
  server.onNotFound([]() {
    if (apMode) { server.sendHeader("Location", "http://192.168.4.1/settings"); server.send(302, "text/plain", ""); }
    else server.send(404, "text/plain", "not found");
  });
  server.begin();
}

// =====================================================================
//  BUTTON
// =====================================================================
void buttonLoop() {
  static bool pressed = false, longDone = false;
  static uint32_t pressStart = 0;
  bool down = digitalRead(BTN_PIN) == LOW;
  uint32_t now = millis();
  if (down && !pressed) {
    pressed = true;
    pressStart = now;
    longDone = false;
    if (game == G_REACT && reactState <= 1) { reactPress(); longDone = true; }   // reaction game counts the press instantly
  }
  if (down && pressed && !longDone) {
    if (callActive && now - pressStart > 800) { longDone = true; ancsAction = 0; }            // hold = answer call
    else if (!callActive && micReady && now - pressStart > 300) { longDone = true; startListening(); }  // hold = talk
    else if (!callActive && !micReady && now - pressStart > 1500) {                          // no mic: hold = study mode
      longDone = true;
      wake();
      if (studyOn) stopStudy(); else startStudy(25, 5);
    }
  }
  if (!down && pressed) {
    pressed = false;
    if (recording) { stopListening(); return; }            // let go = done talking
    if (longDone || now - pressStart < 30) return;
    wake();
    if (callActive) { ancsAction = 1; return; }                 // tap = decline call
    if (ringing || alertShowing) { dismissAlert(); return; }
    if (game != G_NONE) { endGame(); return; }
    if (songPlaying || dancing || songQN) { songStop = true; songQN = 0; dancing = false; partyMode = false; return; }
    int m = (mood + 1) % MOOD_COUNT;                            // tap = next face
    if (m == THINKING) m++;
    setMood((Mood)m);
    moodUntil = now + 20000;
    goScreen(SCR_EYES);
    holdUntil = now + 8000;
  }
}

// =====================================================================
//  SETUP + LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(esp_random());
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  prefs.begin("cheo", false);
  neckTrim = (int8_t)prefs.getChar("ntrim", 0); neckDir = prefs.getChar("ndir", 1) < 0 ? -1 : 1;
  armTrimL = (int8_t)prefs.getChar("atl", 0); armTrimR = (int8_t)prefs.getChar("atr", 0);
  armDir = prefs.getChar("adir", 1) < 0 ? -1 : 1; armsOn = prefs.getUChar("arms", 1); idleMoves = prefs.getUChar("idlemv", 1);
  screenFlip = prefs.getUChar("sflip", HAS_ARMS ? 1 : 0);   // v16: the biped head holds the screen the other way up
  ledcAttach(SERVO_PIN, 50, 14);
  servoWrite(PAN_CENTER);                                    // uses the saved neck center: no more 40-degree jump at boot
  armsBegin();
  legsBegin();

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation((SCREEN_ROTATION + (screenFlip ? 2 : 0)) % 4);
  tft.fillScreen(C_BLACK);
  canvas.cp437(true);
  canvas.setTextWrap(false);
  prefs.begin("cheo", false);
  eyeTheme = prefs.getUChar("eye", 0);
  if (eyeTheme >= EYE_COLORS) eyeTheme = 0;
  eyeC = eyeColorVals[eyeTheme == EYE_COLORS - 1 ? 0 : eyeTheme];
  bootAnim();

  loadSettings();
  loadRems();
  loadCds();
  petLoad();
  eyeTheme = prefs.getUChar("eye", 0);
  if (eyeTheme >= EYE_COLORS) eyeTheme = 0;
  nightOn = prefs.getUChar("night", 1);
  reactBest = prefs.getInt("rbest", 0);
  updateLooks();

  sndQ = xQueueCreate(8, sizeof(uint8_t));
  uidQ = xQueueCreate(8, sizeof(UidMsg));
  notifQ = xQueueCreate(4, sizeof(NotifMsg));
  setupVoiceBuffer();
  xTaskCreatePinnedToCore(soundTask, "sound", 16384, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(playerTask, "voice", 4096, nullptr, 3, nullptr, 0);
  setupMic();
#if HAS_WAKE
  if (micReady) { xTaskCreatePinnedToCore(wakeTask, "wake", 8192, nullptr, 1, nullptr, 1); Serial.println("Wake word: listening for Hey Cheo"); }
#endif

  WiFi.persistent(false);
  WiFi.setHostname("cheo");
  bool ok = connectWiFi(true);
  setupWeb();
  if (ok) onWifiConnected();
  else startSetupAP();

  startBluetooth();
  lastInteract = millis();
  nextIdle = millis() + 30000;
  nextComment = millis() + 600000;
  screenSince = millis();
  sound(0);
  if (ok) sound(12);
}

void loop() {
  uint32_t now = millis();
  server.handleClient();
  wifiLoop();
  ancsLoop();
  trackingLoop();
  armsLoop();
  legsUpdate();
  legsSerial();
  buttonLoop();
  checkReminders();
  studyLoop();

  NotifMsg nm;
  if (xQueueReceive(notifQ, &nm, 0) == pdTRUE) showNotification(nm);

  if (ttsErr) { lastErr = "Voice error " + String(ttsErr); ttsErr = 0; }

  // a question from the web page
  if (hasPending) {
    String q = pendingQ;
    hasPending = false;
    wake();
    if (alertShowing) finishAlert(false);
    String reply;
    Mood m = NORMAL;
    bool learned = false;
    if (gameInput(q, reply, m)) {
      // handled by the game
    } else if (!localCommands(q, reply, m)) {
      thinking = true;
      setMood(quickReact(q));
      goScreen(SCR_EYES);
      render();
      reply = askCheo(q, m, learned);
      thinking = false;
    }
    lastReply = reply;
    busy = false;
    setMood(m);
    moodUntil = millis() + 20000;
    if (learned) sound(11);
    speak(reply, m);
    petGain(0, 2, 0, 1);                       // talking with you is fun
  }

  // songs / dances asked for (play after he finishes talking)
  if (wantSong >= 0) { songRequest = wantSong; wantSong = -1; danceStyle = random(3); }
  if (wantParty) { wantParty = false; startParty(); }
  if (songQN > 0 && !songPlaying && songRequest < 0 && !talkActive()) {    // party playlist
    songRequest = songQ[0];
    for (int i = 1; i < songQN; i++) songQ[i - 1] = songQ[i];
    songQN--;
    danceStyle = random(3);
  }
  if (wantGame != G_NONE && !talkActive()) { startGame(wantGame, false); wantGame = G_NONE; }
  micLoop();
  trainLoop();
  if (wantBriefing && !talkActive()) {
    wantBriefing = false;
    wake();
    thinking = true;
    render();
    String b = briefingText();
    thinking = false;
    speak(b, HAPPY);
  }
  if (wantDance) {
    wantDance = false;
    dancing = true;
    danceSawSong = false;
    danceStyle = random(3);
    if (songRequest < 0 && !songPlaying && songQN == 0) songRequest = 4;
  }

  // weather every 15 min (1 min after a failure)
  if (WiFi.status() == WL_CONNECTED && now > nextWeather && !talkActive()) {
    bool ok = fetchWeather();
    nextWeather = now + (ok ? 900000UL : 60000UL);
    if (ok) rainCheck();
  }
  static uint32_t lastMinute = 0;
  if (now - lastMinute > 60000) { lastMinute = now; cdCleanup(); if (lastRainWarn == 0 || now - lastRainWarn > 3600000UL) rainCheck(); }

  petLoop();
  gameLoop();

  // updates
  static uint32_t nextAutoUpdate = 60000;
  if (otaRequested && !talkActive()) { otaRequested = false; otaCheck(true); }
  else if (AUTO_UPDATE && now > nextAutoUpdate && WiFi.status() == WL_CONNECTED && !talkActive() && !studyOn && !alertShowing && !dancing) {
    nextAutoUpdate = now + 6UL * 3600UL * 1000UL;
    otaCheck(false);
  }

  if (restartAt && now > restartAt) ESP.restart();

  updateAnimation();
  static uint32_t lastRender = 0;
  if (!talkActive() || now - lastRender > 66) { render(); lastRender = now; }   // draw less often while he talks
  delay(16);
}
