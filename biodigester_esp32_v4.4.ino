// ============================================================
//  Biodigester Monitor — ESP32 Firmware  v4.2
//  5 Sensor Thermocouple (TC1-TC5)
// ============================================================

#include <Adafruit_MAX31856.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================================
//  KONFIGURASI JARINGAN
// ============================================================
const char* ssid      = "Biodigester2025";
const char* password  = "z3r0w4sTe###";
const char* serverUrl = "https://api-rmc.vercel.app/api/tempMonitor2";
const char* apiKey    = "sk_x8dTslb8U5yRPTL6sDxKMLe4wFgI9lVw";

// ============================================================
//  KONFIGURASI SENSOR — 5 TC
// ============================================================
#define NUM_TC  5

const int TC_CS_PINS[NUM_TC] = {
  15,   // TC1 → Tmp_CmbJ1
   2,   // TC2 → Tmp_PrlJ1_1
   4,   // TC3 → Tmp_PrlJ1_2
   5,   // TC4 → Tmp_PrlJ1_3
  13    // TC5 → Tmp_PrlJ1_4
};

const char* TC_JSON_KEY[NUM_TC] = {
  "Tmp_CmbJ1",
  "Tmp_PrlJ1_1",
  "Tmp_PrlJ1_2",
  "Tmp_PrlJ1_3",
  "Tmp_PrlJ1_4" 
};

#define SENSOR_FAULT_VALUE  0

// ============================================================
//  KONFIGURASI FILTER NOISE
// ============================================================
#define FILTER_WINDOW_SIZE     10
#define SPIKE_THRESHOLD        15
#define SPIKE_MAX_CONSECUTIVE   3
#define FILTER_MIN_VALID_TEMP  -50
#define FILTER_MAX_VALID_TEMP  1370

// ============================================================
//  KONFIGURASI PIN
// ============================================================
#define pinMCU_SrlRX1  32
#define pinMCU_SrlTX1  33
#define pinMCU_SrlRX2  16
#define pinMCU_SrlTX2  17
#define pinMCU_GP_IO1  26
#define pinMCU_GP_IO2  25
#define pinMCU_GP_I_1  35
#define pinMCU_GP_I_2  34
#define pinMCU_GP_I_3  39
#define pinMCU_GP_I_4  36

// ============================================================
//  KONFIGURASI TIMING
// ============================================================
#define TC_CONVERSION_TIME_MS    160UL
#define HMI_DATETIME_PERIOD_MS   500UL
#define LFS_POLL_PERIOD_MS      1000UL
#define SERVER_POST_PERIOD_MS  60000UL
#define WIFI_RETRY_PERIOD_MS   10000UL
#define HTTP_CONNECT_TIMEOUT_MS  8000

const unsigned long conLng_LFSTmePrdGetDta = LFS_POLL_PERIOD_MS;
const unsigned long conLng_DMDTmePrdTmpVal = HMI_DATETIME_PERIOD_MS;
const unsigned long conLng_DbsTmePrdTmpVal = SERVER_POST_PERIOD_MS;
const unsigned long conLng_WFiTmePrdChkCnt = WIFI_RETRY_PERIOD_MS;

// ============================================================
//  KONFIGURASI EEPROM
// ============================================================
#define EEPROM_SIZE          24    // [B3] was 20
#define EEPROM_CRC_ADDR      20    // [B3] was 16
#define EEPROM_DEFAULT_LIMIT 60

// ============================================================
//  NTP
// ============================================================
const char* conChr_SrvNtp    = "pool.ntp.org";
const long  conLng_GMTOfs    = 7 * 3600;
const int   conInt_DayLghOfs = 0;

// ============================================================
//  PROTOKOL SERIAL
// ============================================================
const char conChr_SfxLF_ = 0x0a;
const char conChr_SfxCR_ = 0x0d;
const char conChr_SfxFF_ = 0xff;

// ============================================================
//  KONSTANTA PANEL
// ============================================================
const int conInt_PnlMan = 0;
const int conInt_PnlLFS = 1;
const int conInt_PnlStg = 2;

// ============================================================
//  SENSOR STATE MACHINE
// ============================================================
enum SensorState { TC_IDLE, TC_TRIGGERED, TC_READY_READ };

// ============================================================
//  FILTER NOISE
// ============================================================
struct SensorFilter {
  int  window[FILTER_WINDOW_SIZE];
  int  windowIndex;
  int  windowCount;
  int  lastAccepted;
  int  spikeCount;
  bool initialized;

  void begin() {
    windowIndex  = 0;
    windowCount  = 0;
    spikeCount   = 0;
    lastAccepted = 0;
    initialized  = false;
    memset(window, 0, sizeof(window));
  }

  bool addSample(int raw, int &out) {
    if (raw < FILTER_MIN_VALID_TEMP || raw > FILTER_MAX_VALID_TEMP) {
      out = initialized ? lastAccepted : 0;
      return false;
    }
    if (initialized) {
      if (abs(raw - lastAccepted) > SPIKE_THRESHOLD) {
        spikeCount++;
        if (spikeCount <= SPIKE_MAX_CONSECUTIVE) {
          out = getFiltered();
          return false;
        }
        spikeCount = 0;
      } else {
        spikeCount = 0;
      }
    }
    window[windowIndex] = raw;
    windowIndex = (windowIndex + 1) % FILTER_WINDOW_SIZE;
    if (windowCount < FILTER_WINDOW_SIZE) windowCount++;
    lastAccepted = raw;
    initialized  = true;
    out = getFiltered();
    return true;
  }

  int getMedian() {
    if (!windowCount) return 0;
    int tmp[FILTER_WINDOW_SIZE];
    memcpy(tmp, window, windowCount * sizeof(int));
    for (int i = 1; i < windowCount; i++) {
      int k = tmp[i], j = i - 1;
      while (j >= 0 && tmp[j] > k) { tmp[j+1] = tmp[j]; j--; }
      tmp[j+1] = k;
    }
    return tmp[windowCount / 2];
  }

  int getMean() {
    if (!windowCount) return 0;
    long s = 0;
    for (int i = 0; i < windowCount; i++) s += window[i];
    return (int)(s / windowCount);
  }

  int getFiltered() {
    if (!initialized) return 0;
    return (getMedian() + getMean()) / 2;
  }
};

// ============================================================
//  STRUKTUR SENSOR
// ============================================================
struct SensorTC {
  Adafruit_MAX31856* driver;
  SensorFilter       filter;
  SensorState        state;
  unsigned long      triggerTime;
  int  rawValue;
  int  filteredValue;
  int  limitValue;
  bool limitFlag;
  bool limitFlagPrev;
  bool isOnline;
  bool isFault;
  int  sensorIndex;

  void begin(int csPin, int idx) {
    driver        = new Adafruit_MAX31856(csPin);
    state         = TC_IDLE;
    triggerTime   = 0;
    rawValue      = 0;
    filteredValue = 0;
    limitValue    = EEPROM_DEFAULT_LIMIT;
    limitFlag     = false;
    limitFlagPrev = false;
    isOnline      = false;
    isFault       = false;
    sensorIndex   = idx;
    filter.begin();
  }

  bool checkFault() {
    uint8_t fault = driver->readFault();
    isFault = (fault != 0);
    if (isFault) {
      int n = sensorIndex + 1;
      if (fault & MAX31856_FAULT_CJRANGE) Serial.printf("[TC%d] Fault: Cold junction out of range\n", n);
      if (fault & MAX31856_FAULT_TCRANGE) Serial.printf("[TC%d] Fault: Thermocouple out of range\n", n);
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.printf("[TC%d] Fault: Cold junction temp high\n", n);
      if (fault & MAX31856_FAULT_CJLOW)   Serial.printf("[TC%d] Fault: Cold junction temp low\n", n);
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.printf("[TC%d] Fault: Thermocouple temp high\n", n);
      if (fault & MAX31856_FAULT_TCLOW)   Serial.printf("[TC%d] Fault: Thermocouple temp low\n", n);
      if (fault & MAX31856_FAULT_OVUV)    Serial.printf("[TC%d] Fault: Overvoltage/Undervoltage\n", n);
      if (fault & MAX31856_FAULT_OPEN)    Serial.printf("[TC%d] Fault: OPEN CIRCUIT — cek kabel!\n", n);
    }
    return isFault;
  }
};

struct SensorTC sensors[NUM_TC];

// ============================================================
//  UNION KONVERSI
// ============================================================
union ArrayToInteger {
  byte     varAry_Dta[4];
  uint32_t varInt_Dta;
};

// ============================================================
//  FREERTOS HTTP TASK — Core 0
// ============================================================
struct HttpPayload {
  char json[512];
};

QueueHandle_t     httpQueue;
SemaphoreHandle_t wifiMutex;

void Task_HttpPost(void* param) {
  HttpPayload payload;
  for (;;) {
    if (xQueueReceive(httpQueue, &payload, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[HTTP] Skip — WiFi not connected");
        continue;
      }
      bool success = false;
      int  attempt = 0;
      while (!success && attempt < 3) {
        attempt++;
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        http.useHTTP10(true);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        if (!http.begin(client, serverUrl)) {
          Serial.printf("[HTTP] begin() gagal (attempt %d)\n", attempt);
          vTaskDelay(pdMS_TO_TICKS(2000));
          continue;
        }
        http.setTimeout(HTTP_CONNECT_TIMEOUT_MS);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Accept",       "application/json");
        if (apiKey && apiKey[0] != '\0') {
          http.addHeader("X-API-KEY", apiKey);
        }
        Serial.printf("[HTTP] POST attempt %d: %s\n", attempt, payload.json);
        int code = http.POST(payload.json);
        Serial.printf("[HTTP] Response code: %d\n", code);
        if (code > 0) {
          Serial.printf("[HTTP] Response: %s\n", http.getString().c_str());
          success = true;
        } else {
          Serial.printf("[HTTP] Error: %s\n", http.errorToString(code).c_str());
          http.end();
          if (attempt < 3) vTaskDelay(pdMS_TO_TICKS(2000));
        }
        http.end();
      }
      if (!success) Serial.println("[HTTP] WARN: POST gagal semua 3x retry");
    }
  }
}

// ============================================================
//  LABEL HMI — 5 sensor
// ============================================================
const char* TC_HMI_LABEL[NUM_TC] = {
  "PANEL_MAIN.N_TmpTC1", "PANEL_MAIN.N_TmpTC2",
  "PANEL_MAIN.N_TmpTC3", "PANEL_MAIN.N_TmpTC4",
  "PANEL_MAIN.N_TmpTC5"
};
const char* TC_STG_H_LABEL[NUM_TC] = {
  "PANEL_STNG.H_TmpTC1Lmt", "PANEL_STNG.H_TmpTC2Lmt",
  "PANEL_STNG.H_TmpTC3Lmt", "PANEL_STNG.H_TmpTC4Lmt",
  "PANEL_STNG.H_TmpTC5Lmt"
};
const char* TC_STG_N_LABEL[NUM_TC] = {
  "PANEL_STNG.N_TmpTC1Lmt", "PANEL_STNG.N_TmpTC2Lmt",
  "PANEL_STNG.N_TmpTC3Lmt", "PANEL_STNG.N_TmpTC4Lmt",
  "PANEL_STNG.N_TmpTC5Lmt"
};

// ============================================================
//  VARIABEL GLOBAL
// ============================================================
ArrayToInteger cnv_TmpTC1Lmt, cnv_TmpTC2Lmt, cnv_TmpTC3Lmt,
               cnv_TmpTC4Lmt, cnv_TmpTC5Lmt;
ArrayToInteger* cnvLmt[NUM_TC] = {
  &cnv_TmpTC1Lmt, &cnv_TmpTC2Lmt, &cnv_TmpTC3Lmt,
  &cnv_TmpTC4Lmt, &cnv_TmpTC5Lmt
};

// Legacy variables
int  varInt_TmpTC1, varInt_TmpTC2, varInt_TmpTC3, varInt_TmpTC4, varInt_TmpTC5;
int  varInt_TmpTC1Lmt, varInt_TmpTC2Lmt, varInt_TmpTC3Lmt,
     varInt_TmpTC4Lmt, varInt_TmpTC5Lmt;
bool varBol_TmpTC1, varBol_TmpTC2, varBol_TmpTC3, varBol_TmpTC4, varBol_TmpTC5;
bool varBol_TmpTC1Lmt, varBol_TmpTC2Lmt, varBol_TmpTC3Lmt,
     varBol_TmpTC4Lmt, varBol_TmpTC5Lmt;
bool varBol_TrgOneShtTC1, varBol_TrgOneShtTC2, varBol_TrgOneShtTC3,
     varBol_TrgOneShtTC4, varBol_TrgOneShtTC5;
bool varBol_TmpTCn    = false;
bool varBol_TmpTCnLmt = false;
int  varInt_TmpTC1OvH = 100;
bool varBol_TmpTC1OvH = false;

unsigned long varLng_LFSTmeStrGetDta=0, varLng_LFSTmeCrnGetDta;
unsigned long varLng_DMDTmeStrTmpVal=0, varLng_DMDTmeCrnTmpVal;
unsigned long varLng_DbsTmeStrTmpVal=0, varLng_DbsTmeCrnTmpVal;
unsigned long varLng_WFiTmeStrChkCnt=0, varLng_WFiTmeCrnChkCnt=0;

bool varBol_WFiCnt = false;

String varStr_SrlRcvCmdP0_LFS    = "";
String varStr_SrlRcvCmdP0_LFSDta = "";
String varStr_SrlRcvCmdP1_HMI    = "";
String varStr_SrlRcvCmdP2_Rjc    = "";
bool   varBol_SrlSndCmdP0_LFS    = false;
bool   varBol_SrlSndCmdP1_HMILFS = false;
bool   varBol_SrlSndCmdP1_HMI    = false;
bool   varBol_SrlSndCmdP2_Rjc    = false;

int     varInt_PgeNmb = conInt_PnlMan;
boolean varBol_RL1=false, varBol_RL2=false, varBol_RL3=false, varBol_RL4=false;
boolean varBol_RL5=false, varBol_RL6=false, varBol_RL7=false, varBol_RL8=false;

String varStr_VolLF1="00.000", varStr_VolLF2="00.000", varStr_VolLF3="00.000";
String varStr_VolLF4="00.000", varStr_VolLF5="00.000", varStr_VolLF6="00.000";

int varInt_TypVol=1, varInt_TypTmp=1, varInt_DteTmeSts=1;
const int conInt_TypVolLF1=1, conInt_TypVolLF2=2, conInt_TypVolLF3=3;
const int conInt_TypVolLF4=4, conInt_TypVolLF5=5, conInt_TypVolLF6=6;
const int conInt_TypTmpTC1=1, conInt_TypTmpTC2=2, conInt_TypTmpTC3=3, conInt_TypTmpTC4=4;

char varChr_GetDteCur[11];
char varChr_GetTmeCur[9];
struct tm* p_tm;

HardwareSerial hrdWre_SrlP1_HMI(1);
HardwareSerial hrdWre_SrlP2_Rjc(2);

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void HMISfx();
void EEPROMWR_(); void EEPROMRD_();
bool EEPROM_ValidateCRC(); void EEPROM_WriteCRC();
uint32_t CRC32_Calculate(uint8_t*, size_t);
void HMITmpTC1Lmt(); void HMITmpTC2Lmt(); void HMITmpTC3Lmt();
void HMITmpTC4Lmt(); void HMITmpTC5Lmt();
void WiFi_Config(); void Wifi_Handle(); void Wifi_SendData();
void syncLegacyVars();
void Task_SensorRead(unsigned long now);
void Task_HMIDatetime(unsigned long now);
void Task_LFSPoll(unsigned long now);
void ProcessHMICommand(); void ProcessRjcCommand(); void ProcessLFSData();
void RjcRly2_0Off(); void RjcRly2_0On_();
void RjcRly2_1Off(); void RjcRly2_1On_();
void RjcRly2_2Off(); void RjcRly2_2On_();
void RjcRly2_3Off(); void RjcRly2_3On_();
void RjcRly2_4Off(); void RjcRly2_4On_();
void RjcRly2_5Off(); void RjcRly2_5On_();
void RjcRly2_6Off(); void RjcRly2_6On_();
void RjcRly2_7Off(); void RjcRly2_7On_();
void Bol_TmpTCnTru();
void _HMISendLimit(int i);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  hrdWre_SrlP1_HMI.begin(9600, SERIAL_8N1, pinMCU_SrlRX1, pinMCU_SrlTX1);
  hrdWre_SrlP2_Rjc.begin(9600, SERIAL_8N1, pinMCU_SrlRX2, pinMCU_SrlTX2);

  for (int i = 0; i < NUM_TC; i++) {
    sensors[i].begin(TC_CS_PINS[i], i);
    if (!sensors[i].driver->begin()) {
      Serial.printf("[INIT] TC%d OFFLINE (CS=%d)\n", i+1, TC_CS_PINS[i]);
      sensors[i].isOnline = false;
    } else {
      sensors[i].driver->setThermocoupleType(MAX31856_TCTYPE_K);
      sensors[i].driver->setConversionMode(MAX31856_ONESHOT_NOWAIT);
      sensors[i].isOnline = true;
      Serial.printf("[INIT] TC%d OK (CS=%d)\n", i+1, TC_CS_PINS[i]);
    }
  }

  while (Serial.available())            Serial.read();
  while (hrdWre_SrlP1_HMI.available()) hrdWre_SrlP1_HMI.read();
  while (hrdWre_SrlP2_Rjc.available()) hrdWre_SrlP2_Rjc.read();

  varStr_SrlRcvCmdP0_LFS.reserve(64);
  varStr_SrlRcvCmdP0_LFSDta.reserve(64);
  varStr_SrlRcvCmdP1_HMI.reserve(64);
  varStr_SrlRcvCmdP2_Rjc.reserve(64);

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROMRD_();
  if (!EEPROM_ValidateCRC()) {
    Serial.println("[INIT] EEPROM CRC invalid — reset default");
    for (int i = 0; i < NUM_TC; i++) cnvLmt[i]->varInt_Dta = EEPROM_DEFAULT_LIMIT;
    EEPROMWR_();
  }

  for (int i = 0; i < NUM_TC; i++) sensors[i].limitValue = cnvLmt[i]->varInt_Dta;
  cnv_TmpTC1Lmt.varInt_Dta = cnvLmt[0]->varInt_Dta;
  cnv_TmpTC2Lmt.varInt_Dta = cnvLmt[1]->varInt_Dta;
  cnv_TmpTC3Lmt.varInt_Dta = cnvLmt[2]->varInt_Dta;
  cnv_TmpTC4Lmt.varInt_Dta = cnvLmt[3]->varInt_Dta;
  cnv_TmpTC5Lmt.varInt_Dta = cnvLmt[4]->varInt_Dta;
  varInt_TmpTC1Lmt = cnvLmt[0]->varInt_Dta;
  varInt_TmpTC2Lmt = cnvLmt[1]->varInt_Dta;
  varInt_TmpTC3Lmt = cnvLmt[2]->varInt_Dta;
  varInt_TmpTC4Lmt = cnvLmt[3]->varInt_Dta;
  varInt_TmpTC5Lmt = cnvLmt[4]->varInt_Dta;    
  HMITmpTC1Lmt(); HMITmpTC2Lmt(); HMITmpTC3Lmt();
  HMITmpTC4Lmt(); HMITmpTC5Lmt();

  wifiMutex = xSemaphoreCreateMutex();
  httpQueue  = xQueueCreate(4, sizeof(HttpPayload));

  xTaskCreatePinnedToCore(
    Task_HttpPost, "HttpPostTask",
    8192, NULL, 1, NULL, 0
  );

  WiFi_Config();
  configTime(conLng_GMTOfs, conInt_DayLghOfs, conChr_SrvNtp);

  varInt_TypVol = conInt_TypVolLF1;
  varInt_TypTmp = conInt_TypTmpTC1;

  Serial.println("[INIT] v4.2 — 5 sensor, semua bug fixed");
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  unsigned long now = millis();

  Wifi_Handle();
  Task_SensorRead(now);

  bool anyLimit = false;
  for (int i = 0; i < NUM_TC; i++) anyLimit |= sensors[i].limitFlag;
  if (anyLimit != varBol_TmpTCnLmt || varBol_TmpTCn) {
    varBol_TmpTCnLmt = anyLimit;
    varBol_TmpTCn    = false;
    hrdWre_SrlP2_Rjc.print(anyLimit ? "*O011*O013" : "*O001*O003");
    hrdWre_SrlP2_Rjc.write(0x0d);
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == conChr_SfxCR_) {
      if (varInt_PgeNmb == conInt_PnlLFS || varInt_PgeNmb == conInt_PnlMan)
        varBol_SrlSndCmdP1_HMILFS = true;
    } else {
      varStr_SrlRcvCmdP0_LFS += c;
    }
  }

  Task_LFSPoll(now);
  Task_HMIDatetime(now);

  while (hrdWre_SrlP1_HMI.available()) {
    char c = (char)hrdWre_SrlP1_HMI.read();
    if      (c == conChr_SfxLF_) {}
    else if (c == conChr_SfxFF_) { varStr_SrlRcvCmdP1_HMI = ""; }
    else if (c == conChr_SfxCR_) { varBol_SrlSndCmdP2_Rjc = true; break; }
    else                          { varStr_SrlRcvCmdP1_HMI += c; }
  }
  if (varBol_SrlSndCmdP2_Rjc) ProcessHMICommand();

  while (hrdWre_SrlP2_Rjc.available()) {
    char c = (char)hrdWre_SrlP2_Rjc.read();
    if (c == conChr_SfxCR_) varBol_SrlSndCmdP1_HMI = true;
    else                     varStr_SrlRcvCmdP2_Rjc += c;
  }
  if (varBol_SrlSndCmdP1_HMI) ProcessRjcCommand();

  if (varBol_SrlSndCmdP1_HMILFS) ProcessLFSData();

  varLng_DbsTmeCrnTmpVal = now;
  if (now - varLng_DbsTmeStrTmpVal >= conLng_DbsTmePrdTmpVal) {
    Wifi_SendData();
  }
}

// ============================================================
//  TASK: SENSOR STATE MACHINE
// ============================================================
void Task_SensorRead(unsigned long now) {
  for (int i = 0; i < NUM_TC; i++) {
    SensorTC& s = sensors[i];
    if (!s.isOnline) continue;

    switch (s.state) {
      case TC_IDLE:
        s.driver->triggerOneShot();
        s.triggerTime = now;
        s.state       = TC_TRIGGERED;
        break;

      case TC_TRIGGERED:
        if (now - s.triggerTime >= TC_CONVERSION_TIME_MS)
          s.state = TC_READY_READ;
        break;

      case TC_READY_READ:
        if (!s.driver->conversionComplete()) break;

        if (s.checkFault()) {
          s.rawValue      = SENSOR_FAULT_VALUE;
          s.filteredValue = SENSOR_FAULT_VALUE;
          s.isFault       = true;
          hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
          hrdWre_SrlP1_HMI.print(".val=0");
          HMISfx();
          hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
          hrdWre_SrlP1_HMI.print(".pco=63488");
          HMISfx();
          s.state = TC_IDLE;
          syncLegacyVars();
          break;
        }

        {
          s.isFault = false;
          int raw   = (int)s.driver->readThermocoupleTemperature();
          int filtered = 0;
          bool ok = s.filter.addSample(raw, filtered);
          if (!ok) Serial.printf("[Filter TC%d] Rejected raw=%d, using=%d\n", i+1, raw, filtered);

          s.rawValue      = raw;
          s.filteredValue = filtered;

          bool prevLimit = s.limitFlag;
          s.limitFlag    = (filtered >= s.limitValue);
          if (s.limitFlag != prevLimit) {
            hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
            hrdWre_SrlP1_HMI.print(".pco=");
            hrdWre_SrlP1_HMI.print(s.limitFlag ? 63488 : 2024);
            HMISfx();
          }

          hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
          hrdWre_SrlP1_HMI.print(".val=");
          hrdWre_SrlP1_HMI.print(filtered);
          HMISfx();

          if (i == 0) {
            bool prevOvH = varBol_TmpTC1OvH;
            varBol_TmpTC1OvH = (filtered >= varInt_TmpTC1OvH);
            if (varBol_TmpTC1OvH != prevOvH) {
              hrdWre_SrlP2_Rjc.print(varBol_TmpTC1OvH ? "*O017" : "*O007");
              hrdWre_SrlP2_Rjc.write(0x0d);
            }
          }

          s.state = TC_IDLE;
          syncLegacyVars();
        }
        break;
    }
  }

  varBol_TrgOneShtTC1 = (sensors[0].state != TC_IDLE);
  varBol_TrgOneShtTC2 = (sensors[1].state != TC_IDLE);
  varBol_TrgOneShtTC3 = (sensors[2].state != TC_IDLE);
  varBol_TrgOneShtTC4 = (sensors[3].state != TC_IDLE);
  varBol_TrgOneShtTC5 = (sensors[4].state != TC_IDLE);
}

// ============================================================
//  TASK: HMI DATETIME
// ============================================================
void Task_HMIDatetime(unsigned long now) {
  varLng_DMDTmeCrnTmpVal = now;
  if (now - varLng_DMDTmeStrTmpVal < conLng_DMDTmePrdTmpVal) return;
  varLng_DMDTmeStrTmpVal = now;

  time_t t = time(nullptr);
  p_tm = localtime(&t);

  sprintf(varChr_GetDteCur, "%02d-%02d-%04d",
          p_tm->tm_mday, p_tm->tm_mon+1, p_tm->tm_year+1900);
  sprintf(varChr_GetTmeCur, "%02d:%02d:%02d",
          p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

  hrdWre_SrlP1_HMI.print(F("L_DteTme.txt=\""));
  hrdWre_SrlP1_HMI.print(varChr_GetDteCur);
  hrdWre_SrlP1_HMI.print("  ");
  hrdWre_SrlP1_HMI.print(varChr_GetTmeCur);
  hrdWre_SrlP1_HMI.print("\"");
  HMISfx();
}

// ============================================================
//  TASK: LFS POLL
// ============================================================
void Task_LFSPoll(unsigned long now) {
  varLng_LFSTmeCrnGetDta = now;
  if (now - varLng_LFSTmeStrGetDta < conLng_LFSTmePrdGetDta) return;
  varLng_LFSTmeStrGetDta = now;

  static const char* cmds[] = {
    "#ID:2-GET","#ID:3-GET","#ID:4-GET",
    "#ID:5-GET","#ID:6-GET","#ID:1-GET"
  };
  static int idx = 0;
  Serial.println(cmds[idx]);
  varInt_TypVol = idx + 1;
  idx = (idx + 1) % 6;
}

// ============================================================
//  WIFI
// ============================================================
void WiFi_Config() {
  Serial.println("[WiFi] Starting...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid, password);
  varLng_WFiTmeStrChkCnt = millis();
}

void Wifi_Handle() {
  wl_status_t status = WiFi.status();
  unsigned long now  = millis();

  if (status == WL_CONNECTED && !varBol_WFiCnt) {
    varBol_WFiCnt = true;
    Serial.print("[WiFi] Connected: ");
    Serial.println(WiFi.localIP());
    hrdWre_SrlP2_Rjc.print("*O010");
    hrdWre_SrlP2_Rjc.write(0x0d);
  }
  if (status != WL_CONNECTED && varBol_WFiCnt) {
    varBol_WFiCnt = false;
    Serial.println("[WiFi] Lost — reconnecting");
    hrdWre_SrlP2_Rjc.print("*O000");
    hrdWre_SrlP2_Rjc.write(0x0d);
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    varLng_WFiTmeStrChkCnt = now;
  }
  varLng_WFiTmeCrnChkCnt = now;
  if (!varBol_WFiCnt && (now - varLng_WFiTmeStrChkCnt >= conLng_WFiTmePrdChkCnt)) {
    Serial.println("[WiFi] Retry...");
    WiFi.begin(ssid, password);
    varLng_WFiTmeStrChkCnt = now;
  }
}

// ============================================================
//  HTTP SEND
// ============================================================
void Wifi_SendData() {
  varLng_DbsTmeStrTmpVal = varLng_DbsTmeCrnTmpVal;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] Skip — WiFi tidak terhubung");
    return;
  }

  auto getSensorVal = [](int idx) -> int {
    if (!sensors[idx].isOnline) return SENSOR_FAULT_VALUE;
    if (sensors[idx].isFault)   return SENSOR_FAULT_VALUE;
    return sensors[idx].filteredValue;
  };

  HttpPayload payload;
  snprintf(payload.json, sizeof(payload.json),
    "{"
      "\"Tmp_IcnJ1\":\"0\","      
      "\"Tmp_DctJ1\":\"0\","      
      "\"Tmp_CmbJ1\":\"%d\","     
      "\"Tmp_PrlJ1_1\":\"%d\","   
      "\"Tmp_PrlJ1_2\":\"%d\","   
      "\"Tmp_PrlJ1_3\":\"%d\","   
      "\"Tmp_PrlJ1_4\":\"%d\""    
    "}",
    getSensorVal(0),
    getSensorVal(1),
    getSensorVal(2),
    getSensorVal(3),
    getSensorVal(4)
  );

  Serial.printf("[HTTP] Enqueue: %s\n", payload.json);

  if (xQueueSend(httpQueue, &payload, 0) != pdTRUE) {
    Serial.println("[HTTP] WARN: queue penuh, payload di-drop");
  }
}

// ============================================================
//  PROSES COMMAND HMI
// ============================================================
void ProcessHMICommand() {
  const String& cmd = varStr_SrlRcvCmdP1_HMI;

  if      (cmd=="*IB00") RjcRly2_0Off(); else if (cmd=="*IB10") RjcRly2_0On_();
  else if (cmd=="*IB01") RjcRly2_1Off(); else if (cmd=="*IB11") RjcRly2_1On_();
  else if (cmd=="*IB02") RjcRly2_2Off(); else if (cmd=="*IB12") RjcRly2_2On_();
  else if (cmd=="*IB03") RjcRly2_3Off(); else if (cmd=="*IB13") RjcRly2_3On_();
  else if (cmd=="*IB04") RjcRly2_4Off(); else if (cmd=="*IB14") RjcRly2_4On_();
  else if (cmd=="*IB05") RjcRly2_5Off(); else if (cmd=="*IB15") RjcRly2_5On_();
  else if (cmd=="*IB06") RjcRly2_6Off(); else if (cmd=="*IB16") RjcRly2_6On_();
  else if (cmd=="*IB07") RjcRly2_7Off(); else if (cmd=="*IB17") RjcRly2_7On_();

  else if (cmd=="*IR10") { hrdWre_SrlP1_HMI.print(F("X_VolLF1.val=0")); HMISfx(); Serial.println("#ID:1-RESET"); }
  else if (cmd=="*IR11") { hrdWre_SrlP1_HMI.print(F("X_VolLF2.val=0")); HMISfx(); Serial.println("#ID:2-RESET"); }
  else if (cmd=="*IR12") { hrdWre_SrlP1_HMI.print(F("X_VolLF3.val=0")); HMISfx(); Serial.println("#ID:3-RESET"); }
  else if (cmd=="*IR13") { hrdWre_SrlP1_HMI.print(F("X_VolLF4.val=0")); HMISfx(); Serial.println("#ID:4-RESET"); }
  else if (cmd=="*IR14") { hrdWre_SrlP1_HMI.print(F("X_VolLF5.val=0")); HMISfx(); Serial.println("#ID:5-RESET"); }
  else if (cmd=="*IR15") { hrdWre_SrlP1_HMI.print(F("X_VolLF6.val=0")); HMISfx(); Serial.println("#ID:6-RESET"); }

  else if (cmd=="*IP00") varInt_PgeNmb = conInt_PnlMan;
  else if (cmd=="*IP01") varInt_PgeNmb = conInt_PnlLFS;
  else if (cmd=="*IP02") varInt_PgeNmb = conInt_PnlStg;

  else if (cmd.substring(0,5) == "*IL00") {
    char d[2];
    for (int s = 0; s < NUM_TC; s++) {
      for (int b = 0; b < 4; b++) {
        int pos = 5 + s*4 + b;
        cmd.substring(pos, pos+1).toCharArray(d, 2);
        cnvLmt[s]->varAry_Dta[b] = d[0];
      }
      sensors[s].limitValue = cnvLmt[s]->varInt_Dta;
    }
    varInt_TmpTC1Lmt = cnv_TmpTC1Lmt.varInt_Dta = cnvLmt[0]->varInt_Dta;
    varInt_TmpTC2Lmt = cnv_TmpTC2Lmt.varInt_Dta = cnvLmt[1]->varInt_Dta;
    varInt_TmpTC3Lmt = cnv_TmpTC3Lmt.varInt_Dta = cnvLmt[2]->varInt_Dta;
    varInt_TmpTC4Lmt = cnv_TmpTC4Lmt.varInt_Dta = cnvLmt[3]->varInt_Dta;
    varInt_TmpTC5Lmt = cnv_TmpTC5Lmt.varInt_Dta = cnvLmt[4]->varInt_Dta;
    EEPROMWR_();
    Serial.println("[EEPROM] Limit updated");
  }

  varStr_SrlRcvCmdP1_HMI = "";
  varBol_SrlSndCmdP2_Rjc = false;
}

// ============================================================
//  PROSES COMMAND REJECTOR
// ============================================================
void ProcessRjcCommand() {
  const String& cmd = varStr_SrlRcvCmdP2_Rjc;

  struct RjcEntry {
    const char* code; bool* flag;
    void(*off)(); void(*on)();
    const char* hmiOff; const char* hmiOn;
  } relays[] = {
    {"*I110",&varBol_RL1,RjcRly2_0Off,RjcRly2_0On_,"PANEL_LFSM.B_RL1.val=0","PANEL_LFSM.B_RL1.val=1"},
    {"*I111",&varBol_RL2,RjcRly2_1Off,RjcRly2_1On_,"PANEL_LFSM.B_RL2.val=0","PANEL_LFSM.B_RL2.val=1"},
    {"*I112",&varBol_RL3,RjcRly2_2Off,RjcRly2_2On_,"PANEL_LFSM.B_RL3.val=0","PANEL_LFSM.B_RL3.val=1"},
    {"*I113",&varBol_RL4,RjcRly2_3Off,RjcRly2_3On_,"PANEL_LFSM.B_RL4.val=0","PANEL_LFSM.B_RL4.val=1"},
    {"*I114",&varBol_RL5,RjcRly2_4Off,RjcRly2_4On_,"PANEL_LFSM.B_RL5.val=0","PANEL_LFSM.B_RL5.val=1"},
    {"*I115",&varBol_RL6,RjcRly2_5Off,RjcRly2_5On_,"PANEL_LFSM.B_RL6.val=0","PANEL_LFSM.B_RL6.val=1"},
    {"*I116",&varBol_RL7,RjcRly2_6Off,RjcRly2_6On_,"PANEL_LFSM.B_RL7.val=0","PANEL_LFSM.B_RL7.val=1"},
    {"*I117",&varBol_RL8,RjcRly2_7Off,RjcRly2_7On_,"PANEL_LFSM.B_RL8.val=0","PANEL_LFSM.B_RL8.val=1"},
  };

  for (auto& r : relays) {
    if (cmd == r.code) {
      if (*r.flag) { r.off(); hrdWre_SrlP1_HMI.print(r.hmiOff); }
      else         { r.on();  hrdWre_SrlP1_HMI.print(r.hmiOn);  }
      HMISfx();
      goto rjc_done;
    }
  }

  if      (cmd=="*I302") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS1.bco=63488")); HMISfx(); }
  else if (cmd=="*I312") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS1.bco=2024"));  HMISfx(); }
  else if (cmd=="*I303") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS2.bco=63488")); HMISfx(); }
  else if (cmd=="*I313") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS2.bco=2024"));  HMISfx(); }
  else if (cmd=="*I304") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS3.bco=63488")); HMISfx(); }
  else if (cmd=="*I314") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS3.bco=2024"));  HMISfx(); }
  else if (cmd=="*I305") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS4.bco=63488")); HMISfx(); }
  else if (cmd=="*I315") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS4.bco=2024"));  HMISfx(); }
  else if (cmd=="*I306") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS5.bco=63488")); HMISfx(); }
  else if (cmd=="*I316") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS5.bco=2024"));  HMISfx(); }
  else if (cmd=="*I307") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS6.bco=63488")); HMISfx(); }
  else if (cmd=="*I317") { hrdWre_SrlP1_HMI.print(F("PANEL_MAIN.L_SS6.bco=2024"));  HMISfx(); }

  rjc_done:
  varStr_SrlRcvCmdP2_Rjc = "";
  varBol_SrlSndCmdP1_HMI = false;
}

// ============================================================
//  PROSES DATA LFS
// ============================================================
void ProcessLFSData() {
  varStr_SrlRcvCmdP0_LFSDta =
    varStr_SrlRcvCmdP0_LFS.substring(15,17) +
    varStr_SrlRcvCmdP0_LFS.substring(18,21);

  struct { const char* id; String* vol; const char* hmi; } m[] = {
    {"#ID:1",&varStr_VolLF1,"X_VolLF1.val="},
    {"#ID:2",&varStr_VolLF2,"X_VolLF2.val="},
    {"#ID:3",&varStr_VolLF3,"X_VolLF3.val="},
    {"#ID:4",&varStr_VolLF4,"X_VolLF4.val="},
    {"#ID:5",&varStr_VolLF5,"X_VolLF5.val="},
    {"#ID:6",&varStr_VolLF6,"X_VolLF6.val="},
  };
  for (auto& e : m) {
    if (varStr_SrlRcvCmdP0_LFS.substring(0,5) == e.id) {
      *e.vol = varStr_SrlRcvCmdP0_LFS.substring(15,21);
      hrdWre_SrlP1_HMI.print(e.hmi);
      hrdWre_SrlP1_HMI.print(varStr_SrlRcvCmdP0_LFSDta);
      HMISfx();
      break;
    }
  }
  varStr_SrlRcvCmdP0_LFS    = "";
  varStr_SrlRcvCmdP0_LFSDta = "";
  varBol_SrlSndCmdP1_HMILFS = false;
}

// ============================================================
//  SYNC LEGACY VARIABLES
// ============================================================
void syncLegacyVars() {
  varInt_TmpTC1    = sensors[0].isFault ? 0 : sensors[0].filteredValue;
  varInt_TmpTC2    = sensors[1].isFault ? 0 : sensors[1].filteredValue;
  varInt_TmpTC3    = sensors[2].isFault ? 0 : sensors[2].filteredValue;
  varInt_TmpTC4    = sensors[3].isFault ? 0 : sensors[3].filteredValue;
  varInt_TmpTC5    = sensors[4].isFault ? 0 : sensors[4].filteredValue; 
  varBol_TmpTC1Lmt = sensors[0].limitFlag;
  varBol_TmpTC2Lmt = sensors[1].limitFlag;
  varBol_TmpTC3Lmt = sensors[2].limitFlag;
  varBol_TmpTC4Lmt = sensors[3].limitFlag;
  varBol_TmpTC5Lmt = sensors[4].limitFlag;                            
  varInt_TmpTC1Lmt = sensors[0].limitValue;
  varInt_TmpTC2Lmt = sensors[1].limitValue;
  varInt_TmpTC3Lmt = sensors[2].limitValue;
  varInt_TmpTC4Lmt = sensors[3].limitValue;
  varInt_TmpTC5Lmt = sensors[4].limitValue;                      
}

// ============================================================
//  HMI HELPERS
// ============================================================
void HMISfx() {
  hrdWre_SrlP1_HMI.write(0xff);
  hrdWre_SrlP1_HMI.write(0xff);
  hrdWre_SrlP1_HMI.write(0xff);
}

void Bol_TmpTCnTru() {
  for (int i = 0; i < NUM_TC; i++) sensors[i].limitFlagPrev = true;
  varBol_TmpTCn = true;
}

void _HMISendLimit(int i) {
  hrdWre_SrlP1_HMI.print(TC_STG_H_LABEL[i]);
  hrdWre_SrlP1_HMI.print(".val=");
  hrdWre_SrlP1_HMI.print(cnvLmt[i]->varInt_Dta);
  HMISfx();
  hrdWre_SrlP1_HMI.print(TC_STG_N_LABEL[i]);
  hrdWre_SrlP1_HMI.print(".val=");
  hrdWre_SrlP1_HMI.print(cnvLmt[i]->varInt_Dta);
  HMISfx();
}
void HMITmpTC1Lmt() { _HMISendLimit(0); }
void HMITmpTC2Lmt() { _HMISendLimit(1); }
void HMITmpTC3Lmt() { _HMISendLimit(2); }
void HMITmpTC4Lmt() { _HMISendLimit(3); }
void HMITmpTC5Lmt() { _HMISendLimit(4); }

// ============================================================
//  EEPROM CRC32
// ============================================================
uint32_t CRC32_Calculate(uint8_t* d, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
  }
  return ~crc;
}

bool EEPROM_ValidateCRC() {
  uint8_t raw[20];                                    
  for (int i = 0; i < 20; i++) raw[i] = EEPROM.read(i);
  uint32_t stored = 0;
  for (int i = 0; i < 4; i++)
    stored |= (uint32_t)EEPROM.read(EEPROM_CRC_ADDR+i) << (i*8);
  return stored == CRC32_Calculate(raw, 20);             
}

void EEPROM_WriteCRC() {
  uint8_t raw[20];                                   
  for (int i = 0; i < 20; i++) raw[i] = EEPROM.read(i);
  uint32_t crc = CRC32_Calculate(raw, 20);           
  for (int i = 0; i < 4; i++)
    EEPROM.write(EEPROM_CRC_ADDR+i, (crc >> (i*8)) & 0xFF);
}

void EEPROMWR_() {
  for (int i = 0; i < NUM_TC; i++) {
    int b = i * 4;
    for (int j = 0; j < 4; j++) EEPROM.write(b+j, cnvLmt[i]->varAry_Dta[j]);
  }
  EEPROM_WriteCRC();
  EEPROM.commit();
}

void EEPROMRD_() {
  for (int i = 0; i < NUM_TC; i++) {
    int b = i * 4;
    for (int j = 0; j < 4; j++) cnvLmt[i]->varAry_Dta[j] = EEPROM.read(b+j);
  }
}

// ============================================================
//  RELAY CONTROL
// ============================================================
void RjcRly2_0Off() { hrdWre_SrlP2_Rjc.print("*O200"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL1=false; }
void RjcRly2_0On_() { hrdWre_SrlP2_Rjc.print("*O210"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL1=true;  }
void RjcRly2_1Off() { hrdWre_SrlP2_Rjc.print("*O201"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL2=false; }
void RjcRly2_1On_() { hrdWre_SrlP2_Rjc.print("*O211"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL2=true;  }
void RjcRly2_2Off() { hrdWre_SrlP2_Rjc.print("*O202"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL3=false; }
void RjcRly2_2On_() { hrdWre_SrlP2_Rjc.print("*O212"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL3=true;  }
void RjcRly2_3Off() { hrdWre_SrlP2_Rjc.print("*O203"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL4=false; }
void RjcRly2_3On_() { hrdWre_SrlP2_Rjc.print("*O213"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL4=true;  }
void RjcRly2_4Off() { hrdWre_SrlP2_Rjc.print("*O204"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL5=false; }
void RjcRly2_4On_() { hrdWre_SrlP2_Rjc.print("*O214"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL5=true;  }
void RjcRly2_5Off() { hrdWre_SrlP2_Rjc.print("*O205"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL6=false; }
void RjcRly2_5On_() { hrdWre_SrlP2_Rjc.print("*O215"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL6=true;  }
void RjcRly2_6Off() { hrdWre_SrlP2_Rjc.print("*O206"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL7=false; }
void RjcRly2_6On_() { hrdWre_SrlP2_Rjc.print("*O216"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL7=true;  }
void RjcRly2_7Off() { hrdWre_SrlP2_Rjc.print("*O207"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL8=false; }
void RjcRly2_7On_() { hrdWre_SrlP2_Rjc.print("*O217"); hrdWre_SrlP2_Rjc.write(0x0d); varBol_RL8=true;  }
