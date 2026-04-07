/*  ========================================================
    CONTROLE UNIVERSAL IR - ESP32  v5.2.0
    ========================================================
    NOVIDADES v5.2.0 (em relação à v5.1.2):

      [NOVO]    Seleção automática do melhor sinal RAW:
                - Captura 3 vezes o mesmo sinal
                - Calcula desvio padrão de cada captura
                - Salva automaticamente a de menor ruído
                - Para protocolos reconhecidos (Midea/NEC)
                  encerra já na 1ª captura (bits limpos)
                - Frontend mostra progresso 1/3, 2/3, 3/3

      [FIX]     Midea e Elgin adicionados ao sendACNative
                (não pedia mais captura desnecessária)

    MODIFICADO v5.1.2:
      - Fix gap idle inicial removido antes de salvar RAW
      - Fix gap idle ignorado no envio do RAW aprendido

    MODIFICADO v5.1.1:
      - RAW aprendido enviado 3x
      - MIDEA aprendido enviado 2x
      - Delay 100ms entre envios

    HARDWARE:
      GPIO 4  -> LED IR  (resistor 100-220 ohm)
      GPIO 18 -> Receptor IR (TSOP1738 / VS1838B)
      

    PRIORIDADE DO SINAL AC:
      1. RAW capturado (hasRaw) - enviado 3 vezes
      2. MIDEA capturado (hasMidea) - enviado 2 vezes
      3. Biblioteca nativa
    ======================================================== */

#define FW_VERSION "5.2.0"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <ArduinoOTA.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

#include <ir_Midea.h>
#include <ir_Samsung.h>
#include <ir_LG.h>
#include <ir_Daikin.h>
#include <ir_Fujitsu.h>
#include <ir_Mitsubishi.h>
#include <ir_Panasonic.h>
#include <ir_Carrier.h>
#include <ir_Whirlpool.h>
#include <ir_Gree.h>
#include <ir_Hitachi.h>

const uint16_t PIN_TX = 4;
const uint16_t PIN_RX = 18;

IRsend         irsend(PIN_TX);
IRrecv         irrecv(PIN_RX, 1200, 100, true);
decode_results irResult;

IRMideaAC      acMidea(PIN_TX);
IRSamsungAc    acSamsung(PIN_TX);
IRLgAc         acLG(PIN_TX);
IRDaikinESP    acDaikin(PIN_TX);
IRFujitsuAC    acFujitsu(PIN_TX, fujitsu_ac_remote_model_t::ARRAH2E);
IRMitsubishiAC acMitsubishi(PIN_TX);
IRPanasonicAc  acPanasonic(PIN_TX);
IRCarrierAc64  acCarrier(PIN_TX);
IRWhirlpoolAc  acWhirlpool(PIN_TX);
IRGreeAC       acGree(PIN_TX);
IRHitachiAc    acHitachi(PIN_TX);

WebServer   server(80);
Preferences nvsRaw;
Preferences nvsMeta;
Preferences nvsRooms;

struct Room { String name, acBrand, tvBrand; };
Room rooms[3] = {
  {"Sala",     "midea",   "samsung"},
  {"Quarto 1", "lg",      "lg"},
  {"Quarto 2", "samsung", "sony"}
};

// ============================================================
//  LEARN STATE — 3 buffers para seleção do melhor sinal
// ============================================================
#define LEARN_CAPTURES 3
#define LEARN_BUF_SIZE 1200

struct LearnState {
  bool   active    = false;
  bool   captured  = false;   // true = todas as capturas concluídas
  String brand, cmd, label;
  int    room = -1;

  // 3 buffers independentes
  uint16_t buf[LEARN_CAPTURES][LEARN_BUF_SIZE];
  uint16_t len[LEARN_CAPTURES];
  float    desvio[LEARN_CAPTURES];
  int      capturaAtual = 0;  // 0..2 (qual captura aguardamos)
  int      melhorIdx    = 0;  // índice do buffer com menor desvio

  // info do protocolo (preenchida na 1ª captura ou na que reconhecer)
  decode_type_t proto   = decode_type_t::UNKNOWN;
  uint64_t      value64 = 0;
  uint8_t       bits    = 0;
};
LearnState learn;

// ============================================================
//  MULTI-NETWORK WIFI  (sem WiFiManager)
// ============================================================
#define WIFI_NAMESPACE "wifi-nets"
#define MAX_NETWORKS   5
#define AP_SSID        "IR-Remote"
#define AP_PASS        "12345678"
#define AP_IP          "192.168.4.1"

void startAP() {
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, apMask);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP] SSID: %s  IP: %s\n", AP_SSID, AP_IP);
}

void saveNetwork(const String& ssid, const String& pass) {
  Preferences p;
  p.begin(WIFI_NAMESPACE, false);
  int count = p.getInt("count", 0);
  for (int i = 0; i < count; i++) {
    if (p.getString(("s" + String(i)).c_str(), "") == ssid) {
      p.putString(("p" + String(i)).c_str(), pass);
      p.end();
      Serial.printf("[WiFi] Senha atualizada para: %s\n", ssid.c_str());
      return;
    }
  }
  if (count < MAX_NETWORKS) {
    p.putString(("s" + String(count)).c_str(), ssid);
    p.putString(("p" + String(count)).c_str(), pass);
    p.putInt("count", count + 1);
    Serial.printf("[WiFi] Rede salva (%d/%d): %s\n", count + 1, MAX_NETWORKS, ssid.c_str());
  } else {
    Serial.println("[WiFi] Limite de redes atingido (5).");
  }
  p.end();
}

void deleteNetwork(const String& ssid) {
  Preferences p;
  p.begin(WIFI_NAMESPACE, false);
  int count = p.getInt("count", 0);
  for (int i = 0; i < count; i++) {
    if (p.getString(("s" + String(i)).c_str(), "") == ssid) {
      for (int j = i; j < count - 1; j++) {
        String ns = p.getString(("s" + String(j + 1)).c_str(), "");
        String np = p.getString(("p" + String(j + 1)).c_str(), "");
        p.putString(("s" + String(j)).c_str(), ns);
        p.putString(("p" + String(j)).c_str(), np);
      }
      p.remove(("s" + String(count - 1)).c_str());
      p.remove(("p" + String(count - 1)).c_str());
      p.putInt("count", count - 1);
      Serial.printf("[WiFi] Rede removida: %s\n", ssid.c_str());
      break;
    }
  }
  p.end();
}

bool tryKnownNetworks() {
  Preferences p;
  p.begin(WIFI_NAMESPACE, true);
  int count = p.getInt("count", 0);
  p.end();
  if (count == 0) return false;
  Serial.printf("[WiFi] %d rede(s) salva(s). Tentando STA...\n", count);
  for (int i = 0; i < count; i++) {
    Preferences p2;
    p2.begin(WIFI_NAMESPACE, true);
    String ssid = p2.getString(("s" + String(i)).c_str(), "");
    String pass = p2.getString(("p" + String(i)).c_str(), "");
    p2.end();
    if (ssid.isEmpty()) continue;
    Serial.printf("[WiFi] Tentando (%d/%d): %s\n", i + 1, count, ssid.c_str());
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) { delay(300); Serial.print("."); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Conectado: %s  IP: %s\n", ssid.c_str(), WiFi.localIP().toString().c_str());
      return true;
    }
    WiFi.disconnect(); delay(300);
  }
  Serial.println("[WiFi] Nenhuma rede conhecida disponível. Usando só o AP.");
  return false;
}

bool connectToNetwork(const String& ssid, const String& pass) {
  Serial.printf("[WiFi] Conectando em: %s\n", ssid.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) { delay(300); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("[WiFi] Falhou.");
  WiFi.disconnect();
  return false;
}

// ============================================================
//  NVS KEYS  (hash 32 bits)
// ============================================================
String nvKey(const String& brand, const String& cmd) {
  uint32_t h = 5381;
  for (char c : brand) h = ((h << 5) + h) ^ (uint8_t)c;
  h ^= 0xABCD;
  for (char c : cmd)   h = ((h << 5) + h) ^ (uint8_t)c;
  char hex[9]; snprintf(hex, sizeof(hex), "%08X", h);
  return String(hex);
}
String necKey(const String& brand, const String& cmd) {
  uint32_t h = 5381;
  for (char c : brand) h = ((h << 5) + h) ^ (uint8_t)c;
  h ^= 0xABCD;
  for (char c : cmd)   h = ((h << 5) + h) ^ (uint8_t)c;
  char hex[7]; snprintf(hex, sizeof(hex), "N%05X", h & 0xFFFFF);
  return String(hex);
}
String mideaKey(const String& brand, const String& cmd) {
  uint32_t h = 5381;
  for (char c : brand) h = ((h << 5) + h) ^ (uint8_t)c;
  h ^= 0xCDEF;
  for (char c : cmd)   h = ((h << 5) + h) ^ (uint8_t)c;
  char hex[7]; snprintf(hex, sizeof(hex), "M%05X", h & 0xFFFFF);
  return String(hex);
}

// --- RAW helpers ---
bool hasRaw(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", true); bool ok = nvsRaw.isKey(nvKey(brand, cmd).c_str()); nvsRaw.end(); return ok;
}
uint16_t loadRaw(const String& brand, const String& cmd, uint16_t* buf, uint16_t maxLen) {
  nvsRaw.begin("raw", true);
  String json = nvsRaw.getString(nvKey(brand, cmd).c_str(), "");
  nvsRaw.end();
  if (json.length() < 3) return 0;
  uint16_t n = 0; int s = json.indexOf('[') + 1;
  while (s > 0 && n < maxLen) {
    int e = json.indexOf(',', s); if (e < 0) e = json.indexOf(']', s); if (e < 0) break;
    buf[n++] = (uint16_t)json.substring(s, e).toInt(); s = e + 1;
    if (json.charAt(e) == ']') break;
  }
  return n;
}
uint16_t loadRawByKey(const String& key, uint16_t* buf, uint16_t maxLen) {
  nvsRaw.begin("raw", true);
  String json = nvsRaw.getString(key.c_str(), "");
  nvsRaw.end();
  if (json.length() < 3) return 0;
  uint16_t n = 0; int s = json.indexOf('[') + 1;
  while (s > 0 && n < maxLen) {
    int e = json.indexOf(',', s); if (e < 0) e = json.indexOf(']', s); if (e < 0) break;
    buf[n++] = (uint16_t)json.substring(s, e).toInt(); s = e + 1;
    if (json.charAt(e) == ']') break;
  }
  return n;
}
void saveRawNVS(const String& brand, const String& cmd, const uint16_t* buf, uint16_t len) {
  String json = "[";
  for (uint16_t i = 0; i < len; i++) { json += String(buf[i]); if (i < len - 1) json += ","; }
  json += "]";
  nvsRaw.begin("raw", false); nvsRaw.putString(nvKey(brand, cmd).c_str(), json); nvsRaw.end();
}
void deleteRawNVS(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", false); nvsRaw.remove(nvKey(brand, cmd).c_str()); nvsRaw.end();
}
void deleteAllRawNVS() { nvsRaw.begin("raw", false); nvsRaw.clear(); nvsRaw.end(); }

// --- NEC helpers ---
bool hasNEC(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", true); bool ok = nvsRaw.isKey(necKey(brand, cmd).c_str()); nvsRaw.end(); return ok;
}
uint32_t loadNEC(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", true); uint32_t v = nvsRaw.getULong(necKey(brand, cmd).c_str(), 0); nvsRaw.end(); return v;
}
void saveNECNVS(const String& brand, const String& cmd, uint32_t value) {
  nvsRaw.begin("raw", false); nvsRaw.putULong(necKey(brand, cmd).c_str(), value); nvsRaw.end();
}
void deleteNECNVS(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", false); nvsRaw.remove(necKey(brand, cmd).c_str()); nvsRaw.end();
}

// --- MIDEA helpers ---
bool hasMidea(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", true); bool ok = nvsRaw.isKey(mideaKey(brand, cmd).c_str()); nvsRaw.end(); return ok;
}
uint64_t loadMidea(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", true); uint64_t v = nvsRaw.getULong64(mideaKey(brand, cmd).c_str(), 0); nvsRaw.end(); return v;
}
void saveMideaNVS(const String& brand, const String& cmd, uint64_t value) {
  nvsRaw.begin("raw", false); nvsRaw.putULong64(mideaKey(brand, cmd).c_str(), value); nvsRaw.end();
}
void deleteMideaNVS(const String& brand, const String& cmd) {
  nvsRaw.begin("raw", false); nvsRaw.remove(mideaKey(brand, cmd).c_str()); nvsRaw.end();
}

// ============================================================
//  DISPOSITIVOS GENERICOS
// ============================================================
#define MAX_GENERIC 30
struct GenericDevice { String brand, cmd, label, icon; int room; };

int genericTotal() {
  nvsMeta.begin("meta", true); int t = nvsMeta.getInt("gtotal", 0); nvsMeta.end(); return t;
}
GenericDevice genericGet(int idx) {
  GenericDevice d; nvsMeta.begin("meta", true);
  d.brand = nvsMeta.getString(("gb"+String(idx)).c_str(),"");
  d.cmd   = nvsMeta.getString(("gc"+String(idx)).c_str(),"");
  d.label = nvsMeta.getString(("gl"+String(idx)).c_str(),"");
  d.icon  = nvsMeta.getString(("gi"+String(idx)).c_str(),"");
  d.room  = nvsMeta.getInt   (("gr"+String(idx)).c_str(),0);
  nvsMeta.end(); return d;
}
void genericSave(const String& brand, const String& cmd, const String& label, int room, const String& icon="") {
  int total = genericTotal(); nvsMeta.begin("meta", false);
  for (int i = 0; i < total; i++) {
    String b = nvsMeta.getString(("gb"+String(i)).c_str(),"");
    String c = nvsMeta.getString(("gc"+String(i)).c_str(),"");
    if (b == brand && c == cmd) {
      nvsMeta.putString(("gl"+String(i)).c_str(), label);
      nvsMeta.putInt   (("gr"+String(i)).c_str(), room);
      if (icon.length() > 0) nvsMeta.putString(("gi"+String(i)).c_str(), icon);
      nvsMeta.end(); return;
    }
  }
  if (total < MAX_GENERIC) {
    nvsMeta.putString(("gb"+String(total)).c_str(), brand);
    nvsMeta.putString(("gc"+String(total)).c_str(), cmd);
    nvsMeta.putString(("gl"+String(total)).c_str(), label);
    nvsMeta.putString(("gi"+String(total)).c_str(), icon);
    nvsMeta.putInt   (("gr"+String(total)).c_str(), room);
    nvsMeta.putInt("gtotal", total + 1);
  }
  nvsMeta.end();
}
void genericEdit(int idx, const String& label, const String& icon) {
  nvsMeta.begin("meta", false);
  nvsMeta.putString(("gl"+String(idx)).c_str(), label);
  if (icon.length() > 0) nvsMeta.putString(("gi"+String(idx)).c_str(), icon);
  nvsMeta.end();
}
void genericDelete(int idx) {
  int total = genericTotal(); nvsMeta.begin("meta", false);
  for (int i = idx; i < total - 1; i++) {
    nvsMeta.putString(("gb"+String(i)).c_str(), nvsMeta.getString(("gb"+String(i+1)).c_str(),""));
    nvsMeta.putString(("gc"+String(i)).c_str(), nvsMeta.getString(("gc"+String(i+1)).c_str(),""));
    nvsMeta.putString(("gl"+String(i)).c_str(), nvsMeta.getString(("gl"+String(i+1)).c_str(),""));
    nvsMeta.putString(("gi"+String(i)).c_str(), nvsMeta.getString(("gi"+String(i+1)).c_str(),""));
    nvsMeta.putInt   (("gr"+String(i)).c_str(), nvsMeta.getInt   (("gr"+String(i+1)).c_str(),0));
  }
  nvsMeta.remove(("gb"+String(total-1)).c_str()); nvsMeta.remove(("gc"+String(total-1)).c_str());
  nvsMeta.remove(("gl"+String(total-1)).c_str()); nvsMeta.remove(("gi"+String(total-1)).c_str());
  nvsMeta.remove(("gr"+String(total-1)).c_str()); nvsMeta.putInt("gtotal", total - 1);
  nvsMeta.end();
}

// ============================================================
//  HELPERS
// ============================================================
void printRawSerial(const String& label, const uint16_t* buf, uint16_t len,
                    const String& proto, uint64_t val, uint8_t bits) {
  Serial.println("\n========================================");
  Serial.printf("SINAL: %s\n", label.c_str());
  Serial.printf("Protocolo: %s  Bits: %d  Amostras: %d\n", proto.c_str(), bits, len);
  if (val != 0) {
    if (bits <= 32) Serial.printf("Valor HEX: 0x%08X\n", (uint32_t)val);
    else Serial.printf("Valor HEX: 0x%012llX\n", val);
  }
  if (buf && len > 0) {
    Serial.println("--- C Array ---");
    Serial.print("uint16_t raw[] = {");
    for (uint16_t i = 0; i < len; i++) {
      if (i % 12 == 0) Serial.print("\n  ");
      Serial.print(buf[i]); if (i < len-1) Serial.print(",");
    }
    Serial.println("\n};");
    Serial.printf("uint16_t rawLen = %d;\n", len);
    Serial.println("irsend.sendRaw(raw, rawLen, 38);");
  }
  Serial.println("========================================\n");
}

String buildCodeJson(const String& proto, uint64_t val, uint8_t bits,
                     const uint16_t* buf, uint16_t len) {
  char hexVal[20] = "N/A";
  if (val != 0) {
    if (bits <= 32) snprintf(hexVal, sizeof(hexVal), "0x%08X", (uint32_t)val);
    else snprintf(hexVal, sizeof(hexVal), "0x%012llX", (unsigned long long)val);
  }
  String rawArr = "uint16_t raw[] = {";
  for (uint16_t i = 0; i < len; i++) {
    rawArr += String(buf[i]); if (i < len-1) rawArr += ",";
    if ((i+1) % 12 == 0 && i < len-1) rawArr += "\\n  ";
  }
  rawArr += "};\\nuint16_t rawLen = " + String(len) + ";\\nirsend.sendRaw(raw, rawLen, 38);";
  String rawJson = "[";
  for (uint16_t i = 0; i < len; i++) { rawJson += String(buf[i]); if (i < len-1) rawJson += ","; }
  rawJson += "]";
  return "{\"ok\":true"
         ",\"proto\":\"" + proto + "\""
         ",\"bits\":" + String(bits) +
         ",\"len\":" + String(len) +
         ",\"hex\":\"" + String(hexVal) + "\""
         ",\"raw_c\":\"" + rawArr + "\""
         ",\"raw_json\":" + rawJson + "}";
}

String jsonEscape(const String& s) {
  String out = "";
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s.charAt(i);
    if      (c == '"')  out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

// ============================================================
//  CSS — PROGMEM
// ============================================================
const char CSS[] PROGMEM = R"CSSRAW(
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{font-family:Nunito,sans-serif;background:#F2F3F7;color:#1A1D2E;min-height:100vh;max-width:430px;margin:0 auto}
.hdr{background:#fff;padding:48px 20px 14px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #E8EAF2;position:sticky;top:0;z-index:100}
.hdr h1{font-size:22px;font-weight:900}.hdr p{font-size:12px;color:#8B8FA8;margin-top:2px;font-weight:700}
.online{display:flex;align-items:center;gap:6px;background:#E6FBF5;border-radius:100px;padding:6px 14px;font-size:12px;font-weight:800;color:#00C48C}
.dot{width:7px;height:7px;background:#00C48C;border-radius:50%;animation:blink 2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
.rtabs{background:#fff;display:flex;overflow-x:auto;border-bottom:2px solid #E8EAF2;scrollbar-width:none;padding:0 16px}
.rtabs::-webkit-scrollbar{display:none}
.rtab{flex-shrink:0;padding:14px 16px 12px;font-size:14px;font-weight:800;color:#8B8FA8;background:none;border:none;border-bottom:3px solid transparent;margin-bottom:-2px;cursor:pointer;font-family:Nunito,sans-serif;white-space:nowrap}
.rtab.active{color:#5B5FFF;border-bottom-color:#5B5FFF}
.rpanel{display:none;padding:16px 16px 100px}
.device-toggle{display:grid;grid-template-columns:repeat(auto-fit,minmax(90px,1fr));background:#fff;border-radius:16px;padding:4px;margin-bottom:16px;box-shadow:0 4px 24px rgba(0,0,0,.06)}
.dtog{padding:10px 8px;border-radius:12px;border:none;background:none;font-size:12px;font-weight:800;color:#8B8FA8;cursor:pointer;font-family:Nunito,sans-serif;transition:all .2s;white-space:nowrap}
.dtog.active{background:#5B5FFF;color:#fff;box-shadow:0 4px 12px rgba(91,95,255,.3)}
.sub{display:none}
.ac-main{background:linear-gradient(135deg,#5B5FFF,#8B5FFF);border-radius:24px;padding:24px;margin-bottom:16px;box-shadow:0 8px 32px rgba(91,95,255,.3)}
.ac-brand-tag{display:inline-block;background:rgba(255,255,255,.18);border-radius:100px;padding:4px 14px;font-size:11px;font-weight:900;color:rgba(255,255,255,.95);letter-spacing:1px;text-transform:uppercase;margin-bottom:16px}
.temp-display{display:flex;align-items:flex-start;margin-bottom:8px}
.temp-num{font-size:96px;font-weight:900;color:#fff;line-height:1;letter-spacing:-4px}
.temp-deg{font-size:28px;font-weight:700;color:rgba(255,255,255,.7);margin-top:14px}
.temp-controls{display:flex;gap:12px;margin-bottom:20px}
.tc-btn{width:52px;height:52px;border-radius:50%;border:none;background:rgba(255,255,255,.22);color:#fff;font-size:30px;cursor:pointer;display:flex;align-items:center;justify-content:center;font-family:Nunito,sans-serif}
.ac-actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.ac-on{padding:15px;border-radius:14px;border:none;background:#fff;color:#5B5FFF;font-size:14px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}
.ac-off{padding:15px;border-radius:14px;border:2px solid rgba(255,255,255,.35);background:transparent;color:#fff;font-size:14px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}
.section-title{font-size:11px;font-weight:900;letter-spacing:1.5px;text-transform:uppercase;color:#8B8FA8;margin:18px 0 10px}
.pills-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:4px}
.pill{padding:10px 16px;border-radius:100px;border:2px solid #E8EAF2;background:#fff;color:#8B8FA8;font-size:13px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif}
.pill.sel-mode{background:#EEEEFF;border-color:#5B5FFF;color:#5B5FFF}
.pill.sel-fan{background:#FFF5E6;border-color:#FF9500;color:#FF9500}
.tv-wrap{background:#fff;border-radius:24px;padding:20px;box-shadow:0 4px 24px rgba(0,0,0,.06)}
.tv-brand-row{display:flex;align-items:center;justify-content:space-between;margin-bottom:18px;padding-bottom:14px;border-bottom:1px solid #E8EAF2}
.tv-brand-name{font-size:15px;font-weight:900;text-transform:uppercase;letter-spacing:1px}
.tv-brand-label{font-size:11px;color:#8B8FA8;font-weight:700}
.tv-power-row{margin-bottom:12px}
.tv-pow{width:100%;padding:16px;border-radius:16px;border:none;background:#FF4D4D;color:#fff;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;box-shadow:0 4px 14px rgba(255,77,77,.3)}
.tv-grid-2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}
.tvb{padding:14px 8px;border-radius:14px;border:none;background:#F2F3F7;color:#1A1D2E;font-size:12px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif;display:flex;flex-direction:column;align-items:center;gap:4px}
.tvb-icon{font-size:20px}
.tvb.accent-btn{background:#EEEEFF;color:#5B5FFF}
.dpad-wrap{display:flex;flex-direction:column;align-items:center;gap:8px;margin:14px 0;padding:16px;background:#F2F3F7;border-radius:20px}
.dpad-row{display:flex;gap:8px;align-items:center}
.dp-btn{width:62px;height:62px;border-radius:16px;border:none;background:#fff;color:#1A1D2E;font-size:24px;cursor:pointer;display:flex;align-items:center;justify-content:center;box-shadow:0 2px 8px rgba(0,0,0,.07)}
.dp-ok{width:62px;height:62px;border-radius:50%;border:none;background:#5B5FFF;color:#fff;font-size:14px;font-weight:900;cursor:pointer;box-shadow:0 4px 14px rgba(91,95,255,.4);font-family:Nunito,sans-serif}
.gen-wrap{background:#fff;border-radius:24px;padding:20px;box-shadow:0 4px 24px rgba(0,0,0,.06)}
.gen-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.gen-btn{padding:20px 8px;border-radius:16px;border:2px solid #E8EAF2;background:#fff;color:#1A1D2E;font-size:13px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif;display:flex;flex-direction:column;align-items:center;gap:6px;transition:.15s;text-align:center;line-height:1.3}
.gen-btn:active{opacity:.7;transform:scale(.97)}
.gen-btn:hover{border-color:#5B5FFF;background:#EEEEFF;color:#5B5FFF}
.gen-btn-icon{font-size:28px}
.gen-empty{text-align:center;color:#8B8FA8;font-size:13px;font-weight:700;padding:28px 0}
.bnav{position:fixed;bottom:0;left:50%;transform:translateX(-50%);width:100%;max-width:430px;background:rgba(255,255,255,.96);backdrop-filter:blur(20px);border-top:1px solid #E8EAF2;display:flex;z-index:200;padding:6px 0 max(10px,env(safe-area-inset-bottom));overflow-x:auto}
.bnav-btn{flex:1;min-width:60px;display:flex;flex-direction:column;align-items:center;gap:3px;padding:8px 4px;border:none;background:none;color:#8B8FA8;font-size:10px;font-weight:800;letter-spacing:.3px;text-transform:uppercase;font-family:Nunito,sans-serif;text-decoration:none;cursor:pointer;white-space:nowrap}
.bnav-btn.active{color:#5B5FFF}
.bnav-icon{font-size:22px}
.modal-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);z-index:300;align-items:flex-end;justify-content:center}
.modal-overlay.open{display:flex}
.modal{background:#fff;border-radius:28px 28px 0 0;padding:28px 20px 40px;width:100%;max-width:430px}
.modal-title{font-size:18px;font-weight:900;margin-bottom:4px}
.modal-sub{font-size:13px;color:#8B8FA8;font-weight:700;margin-bottom:24px}
.ir-ring{width:72px;height:72px;border-radius:50%;border:3px solid #5B5FFF;margin:0 auto 16px;display:flex;align-items:center;justify-content:center;font-size:20px;font-weight:900;font-family:Nunito,sans-serif}
.ir-ring.wait{animation:ring-pulse 1.2s infinite}
@keyframes ring-pulse{0%,100%{box-shadow:0 0 0 0 rgba(91,95,255,.5)}70%{box-shadow:0 0 0 14px rgba(91,95,255,0)}}
.ir-ring.done{border-color:#00C48C;background:#E6FBF5}
.modal-msg{text-align:center;font-size:14px;font-weight:700;color:#8B8FA8;margin-bottom:20px}
.modal-btns{display:flex;gap:10px}
.mbtn{flex:1;padding:15px;border-radius:14px;border:none;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}
.mbtn-save{background:#5B5FFF;color:#fff}
.mbtn-cancel{background:#F2F3F7;color:#8B8FA8}
.mbtn:disabled{opacity:.4}
#toast{position:fixed;bottom:85px;left:50%;transform:translateX(-50%) translateY(8px);background:#1A1D2E;color:#fff;font-weight:800;font-size:13px;padding:12px 24px;border-radius:100px;opacity:0;transition:all .25s;pointer-events:none;z-index:999;white-space:nowrap}
#toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
#toast.ok{background:#00C48C}#toast.err{background:#FF4D4D}#toast.warn{background:#FF9500}
.nw-bar{display:none;align-items:center;justify-content:space-between;background:#FFF8E6;border:1.5px solid #FFD666;border-radius:14px;padding:10px 14px;margin-top:12px;animation:fadeSlideIn .35s ease}
@keyframes fadeSlideIn{from{opacity:0;transform:translateY(-6px)}to{opacity:1;transform:translateY(0)}}
.nw-bar.show{display:flex}
.nw-txt{font-size:12px;font-weight:800;color:#996600}
.nw-btn{padding:8px 14px;border-radius:10px;border:none;background:#FF9500;color:#fff;font-size:12px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;white-space:nowrap}
.nw-dismiss{background:none;border:none;font-size:18px;cursor:pointer;color:#FFAA33;padding:0 0 0 8px;line-height:1}
)CSSRAW";

// ============================================================
//  CODE VIEWER JS — PROGMEM
// ============================================================
const char CODE_VIEWER_JS[] PROGMEM = R"JSRAW(
var codeData=null;
function showCode(d){
  codeData=d;
  document.getElementById('info-proto').textContent=d.proto+(d.bits>0?' · '+d.bits+' bits':'');
  document.getElementById('info-hex').textContent=d.hex;
  document.getElementById('info-len').textContent=d.len>0?d.len+' amostras RAW':'(protocolo compacto)';
  document.getElementById('code-c').textContent=d.raw_c.replace(/\\n/g,'\n');
  var arr=d.raw_json,lines=[],row=[];
  for(var i=0;i<arr.length;i++){row.push(arr[i]);if(row.length===12){lines.push('  '+row.join(', '));row=[];}}
  if(row.length)lines.push('  '+row.join(', '));
  document.getElementById('code-json').textContent='[\n'+lines.join(',\n')+'\n]';
  document.getElementById('code-panel').style.display='block';
  switchView('hex');
  document.getElementById('code-panel').scrollIntoView({behavior:'smooth'});
}
function switchView(v){
  ['hex','c','json'].forEach(function(x){
    document.getElementById('view-'+x).style.display=(x===v)?'block':'none';
    var b=document.getElementById('vt-'+x);
    if(b){b.style.background=x===v?'#5B5FFF':'transparent';
    b.style.color=x===v?'#fff':'#8B8FA8';
    b.style.border=x===v?'none':'2px solid #E8EAF2';}
  });
}
function copyCode(type){
  if(!codeData)return;
  var text='';
  if(type==='hex')
    text='Protocolo: '+codeData.proto+'\nValor HEX: '+codeData.hex+(codeData.len>0?'\nAmostras: '+codeData.len:'');
  else if(type==='c')
    text=codeData.raw_c.replace(/\\n/g,'\n');
  else{
    var arr=codeData.raw_json,lines=[],row=[];
    for(var i=0;i<arr.length;i++){row.push(arr[i]);if(row.length===12){lines.push('  '+row.join(', '));row=[];}}
    if(row.length)lines.push('  '+row.join(', '));
    text='[\n'+lines.join(',\n')+'\n]';
  }
  if(navigator.clipboard)
    navigator.clipboard.writeText(text).then(function(){toast('Copiado!','ok');});
  else{
    var ta=document.createElement('textarea');ta.value=text;document.body.appendChild(ta);
    ta.select();document.execCommand('copy');document.body.removeChild(ta);toast('Copiado!','ok');
  }
}
)JSRAW";

// ============================================================
//  AC helpers nativos
// ============================================================
stdAc::opmode_t strToMode(const String& m) {
  if (m=="cool") return stdAc::opmode_t::kCool; if (m=="heat") return stdAc::opmode_t::kHeat;
  if (m=="dry")  return stdAc::opmode_t::kDry;  if (m=="fan")  return stdAc::opmode_t::kFan;
  return stdAc::opmode_t::kAuto;
}
stdAc::fanspeed_t strToFan(const String& f) {
  if (f=="low")    return stdAc::fanspeed_t::kLow;
  if (f=="medium") return stdAc::fanspeed_t::kMedium;
  if (f=="high")   return stdAc::fanspeed_t::kHigh;
  return stdAc::fanspeed_t::kAuto;
}
bool sendACNative(const String& brand, bool power, int temp, const String& mode, const String& fan) {
  auto md = strToMode(mode); auto fs = strToFan(fan);
  // Midea e Elgin — adicionados v5.2.0
  if (brand=="midea" || brand=="elgin") {
    acMidea.begin(); acMidea.setPower(power);
    if(power){acMidea.setTemp(temp);acMidea.setMode(acMidea.convertMode(md));acMidea.setFan(acMidea.convertFan(fs));}
    acMidea.send(); return true;
  }
  if (brand=="samsung") {
    acSamsung.begin(); acSamsung.setPower(power);
    if(power){acSamsung.setTemp(temp);acSamsung.setMode(acSamsung.convertMode(md));acSamsung.setFan(acSamsung.convertFan(fs));}
    acSamsung.send(); return true;
  } else if (brand=="lg") {
    acLG.begin(); acLG.setPower(power);
    if(power){acLG.setTemp(temp);acLG.setMode(acLG.convertMode(md));acLG.setFan(acLG.convertFan(fs));}
    acLG.send(); return true;
  } else if (brand=="daikin") {
    acDaikin.begin(); acDaikin.setPower(power);
    if(power){acDaikin.setTemp(temp);acDaikin.setMode(acDaikin.convertMode(md));acDaikin.setFan(acDaikin.convertFan(fs));}
    acDaikin.send(); return true;
  } else if (brand=="fujitsu") {
    acFujitsu.begin();
    if(power){
      acFujitsu.setPower(true);acFujitsu.setTemp(temp);acFujitsu.setMode(acFujitsu.convertMode(md));
      switch(fs){
        case stdAc::fanspeed_t::kLow:    acFujitsu.setFanSpeed(kFujitsuAcFanLow);  break;
        case stdAc::fanspeed_t::kMedium: acFujitsu.setFanSpeed(kFujitsuAcFanMed);  break;
        case stdAc::fanspeed_t::kHigh:   acFujitsu.setFanSpeed(kFujitsuAcFanHigh); break;
        default:                         acFujitsu.setFanSpeed(kFujitsuAcFanAuto); break;
      }
    } else acFujitsu.off();
    acFujitsu.send(); return true;
  } else if (brand=="mitsubishi") {
    acMitsubishi.begin(); acMitsubishi.setPower(power);
    if(power){acMitsubishi.setTemp(temp);acMitsubishi.setMode(acMitsubishi.convertMode(md));acMitsubishi.setFan(acMitsubishi.convertFan(fs));}
    acMitsubishi.send(); return true;
  } else if (brand=="panasonic") {
    acPanasonic.begin(); acPanasonic.setPower(power);
    if(power){acPanasonic.setTemp(temp);acPanasonic.setMode(acPanasonic.convertMode(md));acPanasonic.setFan(acPanasonic.convertFan(fs));}
    acPanasonic.send(); return true;
  } else if (brand=="carrier") {
    acCarrier.begin(); acCarrier.setPower(power);
    if(power){acCarrier.setTemp(temp);acCarrier.setFan(acCarrier.convertFan(fs));}
    acCarrier.send(); return true;
  } else if (brand=="whirlpool") {
    acWhirlpool.begin(); acWhirlpool.setPowerToggle(power);
    if(power){acWhirlpool.setTemp(temp);acWhirlpool.setMode(acWhirlpool.convertMode(md));acWhirlpool.setFan(acWhirlpool.convertFan(fs));}
    acWhirlpool.send(); return true;
  } else if (brand=="gree"||brand=="tcl") {
    acGree.begin(); acGree.setPower(power);
    if(power){acGree.setTemp(temp);acGree.setMode(acGree.convertMode(md));acGree.setFan(acGree.convertFan(fs));}
    acGree.send(); return true;
  } else if (brand=="hitachi") {
    acHitachi.begin(); acHitachi.setPower(power);
    if(power){acHitachi.setTemp(temp);acHitachi.setMode(acHitachi.convertMode(md));acHitachi.setFan(acHitachi.convertFan(fs));}
    acHitachi.send(); return true;
  }
  return false;
}

// ============================================================
//  TV nativo
// ============================================================
bool sendTVNative(const String& brand, const String& cmd) {
  if (brand=="samsung") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0xE0E040BF},{"mute",0xE0E0F00F},{"volup",0xE0E0E01F},{"voldown",0xE0E0D02F},
      {"chup",0xE0E048B7},{"chdown",0xE0E008F7},{"source",0xE0E0807F},{"menu",0xE0E058A7},
      {"ok",0xE0E016E9},{"back",0xE0E01AE5},{"up",0xE0E006F9},{"down",0xE0E08679},
      {"left",0xE0E0A659},{"right",0xE0E046B9},{"hdmi1",0xE0E0D12E},{"hdmi2",0xE0E0F10E},
      {"exit",0xE0E0B44B},{"info",0xE0E0F807}};
    for(auto&t:T)if(cmd==t.c){irsend.sendSAMSUNG(t.v);return true;}
  } else if (brand=="lg") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x20DF10EF},{"mute",0x20DF906F},{"volup",0x20DF40BF},{"voldown",0x20DFC03F},
      {"chup",0x20DF00FF},{"chdown",0x20DF807F},{"source",0x20DFD02F},{"menu",0x20DFC23D},
      {"ok",0x20DF44BB},{"back",0x20DF14EB},{"up",0x20DF02FD},{"down",0x20DF827D},
      {"left",0x20DFE01F},{"right",0x20DF609F},{"hdmi1",0x20DF738C},{"hdmi2",0x20DF33CC},
      {"exit",0x20DF8877},{"info",0x20DF55AA}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  } else if (brand=="sony") {
    const struct{const char*c;uint16_t v;uint8_t b;}T[]={
      {"power",0xA90,12},{"mute",0x290,12},{"volup",0x490,12},{"voldown",0xC90,12},
      {"chup",0x090,12},{"chdown",0x890,12},{"source",0xA50,12},{"menu",0x070,12},
      {"ok",0xA70,12},{"back",0x6B2,15},{"up",0x2F0,12},{"down",0xAF0,12},
      {"left",0x2D0,12},{"right",0xCD0,12},{"hdmi1",0x650,15},{"hdmi2",0xE50,15},{"info",0x5D4,15}};
    for(auto&t:T)if(cmd==t.c){for(int r=0;r<3;r++){irsend.sendSony(t.v,t.b);delay(45);}return true;}
  } else if (brand=="philips") {
    const struct{const char*c;uint8_t a,f;}T[]={
      {"power",0,12},{"mute",0,13},{"volup",0,16},{"voldown",0,17},{"chup",0,32},{"chdown",0,33},
      {"source",0,56},{"menu",0,18},{"ok",0,87},{"back",0,10},{"up",0,80},{"down",0,81},{"left",0,85},{"right",0,86}};
    for(auto&t:T)if(cmd==t.c){irsend.sendRC5(irsend.encodeRC5(t.a,t.f),12);return true;}
  } else if (brand=="sharp") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x204040F},{"mute",0x2040107},{"volup",0x204016F},{"voldown",0x2040177},{"chup",0x2040137},{"chdown",0x204013F}};
    for(auto&t:T)if(cmd==t.c){irsend.sendSharpRaw(t.v,15);return true;}
  } else if (brand=="panasonic") {
    const struct{const char*c;uint64_t v;}T[]={
      {"power",0x400401000030ULL},{"mute",0x400401004CF0ULL},{"volup",0x400401000420ULL},
      {"voldown",0x400401008020ULL},{"chup",0x400401000034ULL},{"chdown",0x400401000035ULL},
      {"ok",0x400401000390ULL},{"up",0x400401000338ULL},{"down",0x400401000339ULL},
      {"left",0x40040100033AULL},{"right",0x40040100033BULL},{"back",0x400401000270ULL},{"menu",0x400401000360ULL}};
    for(auto&t:T)if(cmd==t.c){irsend.sendPanasonic64(t.v);return true;}
  } else if (brand=="philco") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x807F48B7},{"mute",0x807F08F7},{"volup",0x807F40BF},{"voldown",0x807FC03F},
      {"chup",0x807F00FF},{"chdown",0x807F807F},{"source",0x807FD02F},{"menu",0x807F18E7},
      {"ok",0x807F28D7},{"back",0x807FA05F},{"up",0x807F20DF},{"down",0x807FA05F},{"left",0x807F10EF},{"right",0x807F609F}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  } else if (brand=="cce") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x00FF48B7},{"mute",0x00FF08F7},{"volup",0x00FF40BF},{"voldown",0x00FFC03F},
      {"chup",0x00FF00FF},{"chdown",0x00FF807F},{"source",0x00FFD02F},{"menu",0x00FF18E7},
      {"ok",0x00FF28D7},{"back",0x00FFA05F},{"up",0x00FF20DF},{"down",0x00FFA25D},{"left",0x00FF10EF},{"right",0x00FF609F}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  } else if (brand=="gradiente") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x804048B7},{"mute",0x804008F7},{"volup",0x804040BF},{"voldown",0x8040C03F},
      {"chup",0x804000FF},{"chdown",0x8040807F},{"source",0x8040D02F},{"menu",0x804018E7},
      {"ok",0x804028D7},{"back",0x8040A05F},{"up",0x804020DF},{"down",0x8040A25D},{"left",0x804010EF},{"right",0x8040609F}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  } else if (brand=="semp") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x02FD48B7},{"mute",0x02FD08F7},{"volup",0x02FD40BF},{"voldown",0x02FDC03F},
      {"chup",0x02FD00FF},{"chdown",0x02FD807F},{"source",0x02FDD02F},{"menu",0x02FD18E7},
      {"ok",0x02FD28D7},{"back",0x02FDA05F},{"up",0x02FD20DF},{"down",0x02FDA25D},{"left",0x02FD10EF},{"right",0x02FD609F}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  } else if (brand=="buster") {
    const struct{const char*c;uint32_t v;}T[]={
      {"power",0x4BB640BF},{"mute",0x4BB6A05F},{"volup",0x4BB650AF},{"voldown",0x4BB6D02F},
      {"chup",0x4BB620DF},{"chdown",0x4BB6E01F},{"source",0x4BB6B04F},{"menu",0x4BB6F00F},
      {"ok",0x4BB6E817},{"back",0x4BB638C7},{"up",0x4BB6A25D},{"down",0x4BB622DD},{"left",0x4BB614EB},{"right",0x4BB634CB}};
    for(auto&t:T)if(cmd==t.c){irsend.sendNEC(t.v);return true;}
  }
  return false;
}

// ============================================================
//  Envio unificado
// ============================================================
String doSend(const String& brand, const String& cmd) {
  static uint16_t buf[1200];
  Serial.printf("[SEND] brand='%s' cmd='%s'\n", brand.c_str(), cmd.c_str());
  if (hasNEC(brand, cmd))   { uint32_t v = loadNEC(brand, cmd); irsend.sendNEC(v, 32); Serial.printf("[NEC] 0x%08X OK\n", v); return "NEC"; }
  if (hasMidea(brand, cmd)) { uint64_t v = loadMidea(brand, cmd); irsend.sendMidea(v, 48); return "MIDEA"; }
  if (hasRaw(brand, cmd))   { uint16_t n = loadRaw(brand, cmd, buf, 1200); if (n > 0) { irsend.sendRaw(buf, n, 38); return "RAW"; } }
  if (sendTVNative(brand, cmd)) return "NATIVE";
  return "UNKNOWN";
}

String doSendAC(const String& brand, bool power, int temp, const String& mode, const String& fan) {
  static uint16_t buf[1200];
  String pcmd = power ? "power_on" : "power_off";

  // PRIORIDADE 1: RAW aprendido (envia 3 vezes)
  if (hasRaw(brand, pcmd)) {
    uint16_t n = loadRaw(brand, pcmd, buf, 1200);
    if (n > 0) {
      uint16_t* sendBuf = buf;
      uint16_t  sendLen = n;
      if (sendBuf[0] > 15000) { sendBuf++; sendLen--; Serial.printf("[AC] Gap inicial ignorado: %d µs\n", buf[0]); }
      Serial.printf("[AC] -> RAW aprendido, enviando %d amostras 3x\n", sendLen);
      for(int i = 0; i < 3; i++) { irsend.sendRaw(sendBuf, sendLen, 38); delay(100); }
      return "RAW_LEARNED (3x)";
    }
  }

  // PRIORIDADE 2: MIDEA aprendido (envia 2 vezes)
  if (hasMidea(brand, pcmd)) {
    uint64_t v = loadMidea(brand, pcmd);
    Serial.println("[AC] -> MIDEA aprendido, enviando 2x");
    for(int i = 0; i < 2; i++) { irsend.sendMidea(v, 48); delay(60); }
    return "MIDEA_LEARNED (2x)";
  }

  // PRIORIDADE 3: Nativo
  bool ok = sendACNative(brand, power, temp, mode, fan);
  return ok ? "NATIVE" : "UNKNOWN";
}

// ============================================================
//  CALCULA DESVIO PADRÃO de um buffer RAW
// ============================================================
float calcDesvio(uint16_t* buf, uint16_t len) {
  if (len == 0) return 999999.0f;
  float soma = 0;
  for (uint16_t i = 0; i < len; i++) soma += buf[i];
  float media = soma / len;
  float variancia = 0;
  for (uint16_t i = 0; i < len; i++) {
    float diff = buf[i] - media;
    variancia += diff * diff;
  }
  return sqrt(variancia / len);
}

// ============================================================
//  CAPTURA — suporta 3 rodadas, escolhe menor desvio
// ============================================================
void processCapture() {
  if (!learn.active || learn.captured) return;
  if (!irrecv.decode(&irResult)) return;

  int idx = learn.capturaAtual;

  // Armazena no buffer desta rodada
  learn.len[idx] = min((uint16_t)(irResult.rawlen - 1), (uint16_t)LEARN_BUF_SIZE);
  for (uint16_t i = 0; i < learn.len[idx]; i++)
    learn.buf[idx][i] = (uint16_t)irResult.rawbuf[i + 1];

  // Salva info do protocolo (sempre atualiza — última captura vence, mas só importa para bits)
  learn.proto   = irResult.decode_type;
  learn.value64 = (uint64_t)irResult.value;
  learn.bits    = irResult.bits;

  // Protocolo reconhecido como bits limpos? Encerra já na 1ª captura
  bool isNEC   = (learn.proto == NEC)   && learn.bits == 32 && learn.value64 != 0;
  bool isMidea = (learn.proto == MIDEA || learn.proto == MIDEA24) && learn.value64 != 0;
  if (isNEC || isMidea) {
    Serial.printf("[LEARN] Protocolo reconhecido (%s) — encerra em 1 captura\n",
                  typeToString(learn.proto).c_str());
    learn.melhorIdx    = idx;
    learn.capturaAtual = LEARN_CAPTURES; // força conclusão
    learn.captured     = true;
    irrecv.resume();
    printRawSerial(
      learn.label.length() > 0 ? learn.label : (learn.brand + "/" + learn.cmd),
      learn.buf[learn.melhorIdx], learn.len[learn.melhorIdx],
      typeToString(learn.proto), learn.value64, learn.bits);
    return;
  }

  // RAW puro — calcula desvio desta rodada
  learn.desvio[idx] = calcDesvio(learn.buf[idx], learn.len[idx]);
  Serial.printf("[LEARN] Captura %d/%d — len=%d desvio=%.1fµs\n",
                idx + 1, LEARN_CAPTURES, learn.len[idx], learn.desvio[idx]);

  learn.capturaAtual++;

  if (learn.capturaAtual >= LEARN_CAPTURES) {
    // Escolhe o buffer com menor desvio padrão
    learn.melhorIdx = 0;
    for (int i = 1; i < LEARN_CAPTURES; i++) {
      if (learn.desvio[i] < learn.desvio[learn.melhorIdx])
        learn.melhorIdx = i;
    }
    Serial.printf("[LEARN] Melhor captura: %d (desvio=%.1fµs)\n",
                  learn.melhorIdx + 1, learn.desvio[learn.melhorIdx]);
    learn.captured = true;
    printRawSerial(
      learn.label.length() > 0 ? learn.label : (learn.brand + "/" + learn.cmd),
      learn.buf[learn.melhorIdx], learn.len[learn.melhorIdx],
      typeToString(learn.proto), learn.value64, learn.bits);
  } else {
    // Ainda aguarda próximas capturas
    irrecv.resume();
  }
}

// ============================================================
//  Rooms NVS
// ============================================================
void saveRooms() {
  nvsRooms.begin("rooms", false);
  for(int i=0;i<3;i++){
    nvsRooms.putString(("n"+String(i)).c_str(), rooms[i].name);
    nvsRooms.putString(("a"+String(i)).c_str(), rooms[i].acBrand);
    nvsRooms.putString(("t"+String(i)).c_str(), rooms[i].tvBrand);
  }
  nvsRooms.end();
}
void loadRooms() {
  nvsRooms.begin("rooms", true);
  for(int i=0;i<3;i++){
    rooms[i].name    = nvsRooms.getString(("n"+String(i)).c_str(), rooms[i].name);
    rooms[i].acBrand = nvsRooms.getString(("a"+String(i)).c_str(), rooms[i].acBrand);
    rooms[i].tvBrand = nvsRooms.getString(("t"+String(i)).c_str(), rooms[i].tvBrand);
  }
  nvsRooms.end();
}

// ============================================================
//  CODE PANEL HTML
// ============================================================
String codePanelHTML() {
  return
    "<div id='code-panel' style='display:none;margin-top:16px'>"
    "<div style='display:flex;align-items:center;justify-content:space-between;margin-bottom:10px'>"
    "<span style='font-size:12px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1px'>Código Capturado</span>"
    "<div style='display:flex;gap:6px'>"
    "<button onclick='switchView(\"hex\")' id='vt-hex' style='padding:5px 12px;border-radius:8px;border:none;background:#5B5FFF;color:#fff;font-size:11px;font-weight:800;cursor:pointer;font-family:inherit'>HEX</button>"
    "<button onclick='switchView(\"c\")' id='vt-c' style='padding:5px 12px;border-radius:8px;border:2px solid #E8EAF2;background:transparent;color:#8B8FA8;font-size:11px;font-weight:800;cursor:pointer;font-family:inherit'>C Array</button>"
    "<button onclick='switchView(\"json\")' id='vt-json' style='padding:5px 12px;border-radius:8px;border:2px solid #E8EAF2;background:transparent;color:#8B8FA8;font-size:11px;font-weight:800;cursor:pointer;font-family:inherit'>JSON</button>"
    "</div></div>"
    "<div id='view-hex'>"
    "<div style='background:#F2F3F7;border-radius:12px;padding:14px;margin-bottom:8px'>"
    "<div style='font-size:10px;font-weight:900;color:#8B8FA8;margin-bottom:4px'>PROTOCOLO</div>"
    "<div id='info-proto' style='font-size:15px;font-weight:900;color:#5B5FFF;margin-bottom:10px'></div>"
    "<div style='font-size:10px;font-weight:900;color:#8B8FA8;margin-bottom:4px'>VALOR HEX</div>"
    "<div id='info-hex' style='font-size:17px;font-weight:900;color:#00C48C;font-family:monospace;word-break:break-all;margin-bottom:10px'></div>"
    "<div style='font-size:10px;font-weight:900;color:#8B8FA8;margin-bottom:4px'>AMOSTRAS RAW</div>"
    "<div id='info-len' style='font-size:13px;font-weight:800;color:#1A1D2E'></div>"
    "</div>"
    "<button onclick='copyCode(\"hex\")' style='width:100%;padding:12px;border-radius:10px;border:2px solid #E8EAF2;background:#fff;color:#5B5FFF;font-size:13px;font-weight:800;cursor:pointer;font-family:inherit'>📋 Copiar HEX</button>"
    "</div>"
    "<div id='view-c' style='display:none'>"
    "<div style='background:#F2F3F7;border-radius:12px;padding:14px;margin-bottom:8px;overflow-x:auto'>"
    "<pre id='code-c' style='font-size:11px;color:#1A1D2E;font-family:monospace;white-space:pre-wrap;word-break:break-all;margin:0'></pre>"
    "</div>"
    "<button onclick='copyCode(\"c\")' style='width:100%;padding:12px;border-radius:10px;border:2px solid #E8EAF2;background:#fff;color:#5B5FFF;font-size:13px;font-weight:800;cursor:pointer;font-family:inherit'>📋 Copiar C Array</button>"
    "</div>"
    "<div id='view-json' style='display:none'>"
    "<div style='background:#F2F3F7;border-radius:12px;padding:14px;margin-bottom:8px;overflow-x:auto;max-height:200px;overflow-y:auto'>"
    "<pre id='code-json' style='font-size:11px;color:#1A1D2E;font-family:monospace;white-space:pre-wrap;word-break:break-all;margin:0'></pre>"
    "</div>"
    "<button onclick='copyCode(\"json\")' style='width:100%;padding:12px;border-radius:10px;border:2px solid #E8EAF2;background:#fff;color:#5B5FFF;font-size:13px;font-weight:800;cursor:pointer;font-family:inherit'>📋 Copiar JSON</button>"
    "</div>"
    "</div>";
}

// ============================================================
//  ROTAS API
// ============================================================
void handleTV() {
  String brand = server.arg("brand"), cmd = server.arg("cmd");
  if (brand.length() == 0 || cmd.length() == 0) {
    server.send(400,"application/json","{\"ok\":false,\"msg\":\"brand e cmd são obrigatórios\"}"); return;
  }
  String res = doSend(brand, cmd);
  if (res=="UNKNOWN")
    server.send(200,"application/json","{\"ok\":false,\"status\":\"UNKNOWN\",\"brand\":\""+brand+"\",\"cmd\":\""+cmd+"\"}");
  else
    server.send(200,"application/json","{\"ok\":true,\"status\":\""+res+"\"}");
}

void handleAC() {
  String brand = server.arg("brand");
  if (brand.length() == 0) {
    server.send(400,"application/json","{\"ok\":false,\"msg\":\"brand é obrigatório\"}"); return;
  }
  bool   power = server.arg("power")=="1";
  int    temp  = constrain(server.arg("temp").toInt(),16,30);
  String mode  = server.arg("mode"), fan = server.arg("fan");
  String res   = doSendAC(brand,power,temp,mode,fan);
  if (res=="UNKNOWN")
    server.send(200,"application/json","{\"ok\":false,\"status\":\"UNKNOWN\",\"brand\":\""+brand+"\",\"cmd\":\""+(power?"power_on":"power_off")+"\"}");
  else
    server.send(200,"application/json",
      "{\"ok\":true,\"status\":\""+res+"\""
      ",\"usedNative\":"+(res=="NATIVE"?"true":"false")+"}"
    );
}

void apiDevices() {
  int room = server.arg("room").toInt();
  int total = genericTotal();
  String json = "["; bool first = true;
  for(int i=0;i<total;i++){
    GenericDevice d = genericGet(i);
    if(d.room != room) continue;
    if(!first) json += ",";
    json += "{\"idx\":" + String(i) + ",\"brand\":\"" + jsonEscape(d.brand) + "\",\"cmd\":\"" + jsonEscape(d.cmd) + "\",\"label\":\"" + jsonEscape(d.label) + "\",\"icon\":\"" + jsonEscape(d.icon) + "\"}";
    first = false;
  }
  json += "]";
  server.send(200,"application/json",json);
}

void apiDevicesAll() {
  int total = genericTotal();
  String json = "["; bool first = true;
  for(int i=0;i<total;i++){
    GenericDevice d = genericGet(i);
    String rn = (d.room>=0&&d.room<3) ? rooms[d.room].name : "?";
    if(!first) json += ",";
    json += "{\"idx\":" + String(i) + ",\"brand\":\"" + jsonEscape(d.brand) + "\",\"cmd\":\"" + jsonEscape(d.cmd) + "\",\"label\":\"" + jsonEscape(d.label) + "\",\"icon\":\"" + jsonEscape(d.icon) + "\",\"roomName\":\"" + jsonEscape(rn) + "\"}";
    first = false;
  }
  json += "]";
  server.send(200,"application/json",json);
}

void apiEditDevice() {
  int idx = server.arg("idx").toInt();
  String label = server.arg("label"), icon = server.arg("icon");
  if (idx < 0 || idx >= genericTotal() || label.length() == 0) {
    server.send(400,"application/json","{\"ok\":false,\"msg\":\"Parâmetros inválidos\"}"); return;
  }
  genericEdit(idx, label, icon);
  server.send(200,"application/json","{\"ok\":true}");
}

void apiDeleteDevice() {
  int idx = server.arg("idx").toInt();
  GenericDevice d = genericGet(idx);
  deleteRawNVS(d.brand, d.cmd); deleteNECNVS(d.brand, d.cmd); deleteMideaNVS(d.brand, d.cmd);
  genericDelete(idx);
  server.send(200,"application/json","{\"ok\":true}");
}

// ============================================================
//  API LEARN — com suporte a 3 capturas
// ============================================================
void apiLearnStart() {
  learn.brand = server.arg("brand");
  learn.cmd   = server.arg("cmd");
  learn.label = server.arg("label");
  learn.room  = server.hasArg("room") ? server.arg("room").toInt() : -1;
  // Reseta estado completo
  learn.active       = true;
  learn.captured     = false;
  learn.capturaAtual = 0;
  learn.melhorIdx    = 0;
  learn.proto        = decode_type_t::UNKNOWN;
  learn.value64      = 0;
  learn.bits         = 0;
  for (int i = 0; i < LEARN_CAPTURES; i++) {
    learn.len[i]    = 0;
    learn.desvio[i] = 0;
  }
  irrecv.resume();
  server.send(200,"application/json","{\"ok\":true}");
}

void apiLearnStop() {
  learn.active = false; learn.captured = false; learn.capturaAtual = 0;
  server.send(200,"application/json","{\"ok\":true}");
}

void apiLearnPoll() {
  // Informa progresso ao frontend
  int atual = learn.capturaAtual;
  // Para protocolo reconhecido, capturaAtual é forçado para LEARN_CAPTURES
  // mas o que importa para o frontend é saber quantas rodadas já passaram
  int rodadasFeitas = (atual > LEARN_CAPTURES) ? LEARN_CAPTURES : atual;

  if (learn.captured) {
    server.send(200,"application/json",
      "{\"capturado\":true"
      ",\"len\":" + String(learn.len[learn.melhorIdx]) +
      ",\"capturaAtual\":" + String(LEARN_CAPTURES) +  // total = concluído
      ",\"melhorIdx\":" + String(learn.melhorIdx) +
      ",\"brand\":\"" + learn.brand + "\""
      ",\"cmd\":\"" + learn.cmd + "\"}");
  } else {
    server.send(200,"application/json",
      "{\"capturado\":false"
      ",\"capturaAtual\":" + String(rodadasFeitas) + "}");
  }
}

void apiLearnRaw() {
  if (!learn.captured || learn.len[learn.melhorIdx] == 0) {
    server.send(400,"application/json","{\"ok\":false,\"msg\":\"Nada capturado\"}"); return;
  }
  server.send(200,"application/json",
    buildCodeJson(typeToString(learn.proto), learn.value64, learn.bits,
                  learn.buf[learn.melhorIdx], learn.len[learn.melhorIdx]));
}

void apiLearnSave() {
  if (!learn.captured || learn.len[learn.melhorIdx] == 0) {
    server.send(400,"application/json","{\"ok\":false,\"msg\":\"Nada capturado\"}");
    return;
  }

  bool isNEC   = (learn.proto == NEC)   && learn.bits == 32 && learn.value64 != 0;
  bool isMidea = (learn.proto == MIDEA || learn.proto == MIDEA24) && learn.value64 != 0;

  if (isNEC) {
    saveNECNVS(learn.brand, learn.cmd, (uint32_t)learn.value64);
    Serial.printf("[LEARN] Salvo NEC: 0x%08X\n", (uint32_t)learn.value64);
  } else if (isMidea) {
    saveMideaNVS(learn.brand, learn.cmd, learn.value64);
    Serial.printf("[LEARN] Salvo MIDEA: 0x%012llX\n", (unsigned long long)learn.value64);
  } else {
    // RAW — usa o melhor buffer, remove gap inicial se existir
    uint16_t* cleanBuf = learn.buf[learn.melhorIdx];
    uint16_t  cleanLen = learn.len[learn.melhorIdx];
    if (cleanLen > 2 && cleanBuf[0] > 15000) {
      Serial.printf("[LEARN] Gap inicial removido: %d µs\n", cleanBuf[0]);
      cleanBuf++;
      cleanLen--;
    }
    saveRawNVS(learn.brand, learn.cmd, cleanBuf, cleanLen);
    Serial.printf("[LEARN] Salvo RAW melhor (captura %d, desvio=%.1fµs, len=%d)\n",
                  learn.melhorIdx + 1, learn.desvio[learn.melhorIdx], cleanLen);
  }

  if (learn.room >= 0) {
    String lbl = learn.label.length() > 0 ? learn.label : (learn.brand + "/" + learn.cmd);
    genericSave(learn.brand, learn.cmd, lbl, learn.room);
  }

  uint16_t savedLen = learn.len[learn.melhorIdx];
  learn.active = false; learn.captured = false; learn.capturaAtual = 0; learn.melhorIdx = 0;

  server.send(200,"application/json",
    "{\"ok\":true"
    ",\"brand\":\"" + learn.brand + "\""
    ",\"cmd\":\"" + learn.cmd + "\""
    ",\"len\":" + String(savedLen) +
    ",\"room\":" + String(learn.room) + "}");
}

void apiSignalRaw() {
  String key = server.arg("key");
  if (!key.length()) { server.send(400,"application/json","{\"ok\":false,\"msg\":\"key ausente\"}"); return; }
  nvsRaw.begin("raw", true);
  bool isNEC   = nvsRaw.isKey(key.c_str()) && key.startsWith("N");
  bool isMidea = nvsRaw.isKey(key.c_str()) && key.startsWith("M");
  bool isRaw   = nvsRaw.isKey(key.c_str()) && !isNEC && !isMidea;
  nvsRaw.end();
  if (isNEC) {
    nvsRaw.begin("raw", true); uint32_t v = nvsRaw.getULong(key.c_str(), 0); nvsRaw.end();
    char hexVal[12]; snprintf(hexVal, sizeof(hexVal), "0x%08X", v);
    server.send(200,"application/json","{\"ok\":true,\"proto\":\"NEC\",\"bits\":32,\"len\":0,\"hex\":\""+String(hexVal)+"\",\"raw_c\":\"irsend.sendNEC("+String(hexVal)+", 32);\",\"raw_json\":[]}");
    return;
  }
  if (isMidea) {
    nvsRaw.begin("raw", true); uint64_t v = nvsRaw.getULong64(key.c_str(), 0); nvsRaw.end();
    char hexVal[16]; snprintf(hexVal, sizeof(hexVal), "0x%012llX", (unsigned long long)v);
    server.send(200,"application/json","{\"ok\":true,\"proto\":\"MIDEA\",\"bits\":48,\"len\":0,\"hex\":\""+String(hexVal)+"\",\"raw_c\":\"irsend.sendMidea("+String(hexVal)+", 48);\",\"raw_json\":[]}");
    return;
  }
  if (isRaw) {
    static uint16_t tmpBuf[1200]; uint16_t n = loadRawByKey(key, tmpBuf, 1200);
    if (n == 0) { server.send(404,"application/json","{\"ok\":false,\"msg\":\"Sinal vazio\"}"); return; }
    server.send(200,"application/json", buildCodeJson("RAW_SAVED", 0, 0, tmpBuf, n)); return;
  }
  server.send(404,"application/json","{\"ok\":false,\"msg\":\"Chave não encontrada\"}");
}

void apiSignalRawByDev() {
  String brand = server.arg("brand"), cmd = server.arg("cmd");
  if (!brand.length() || !cmd.length()) { server.send(400,"application/json","{\"ok\":false,\"msg\":\"brand/cmd ausente\"}"); return; }
  if (hasNEC(brand, cmd)) {
    uint32_t v = loadNEC(brand, cmd);
    char hexVal[12]; snprintf(hexVal, sizeof(hexVal), "0x%08X", v);
    server.send(200,"application/json","{\"ok\":true,\"proto\":\"NEC\",\"bits\":32,\"len\":0,\"hex\":\""+String(hexVal)+"\",\"raw_c\":\"irsend.sendNEC("+String(hexVal)+", 32);\",\"raw_json\":[]}");
    return;
  }
  if (hasMidea(brand, cmd)) {
    uint64_t v = loadMidea(brand, cmd);
    char hexVal[16]; snprintf(hexVal, sizeof(hexVal), "0x%012llX", (unsigned long long)v);
    server.send(200,"application/json","{\"ok\":true,\"proto\":\"MIDEA\",\"bits\":48,\"len\":0,\"hex\":\""+String(hexVal)+"\",\"raw_c\":\"irsend.sendMidea("+String(hexVal)+", 48);\",\"raw_json\":[]}");
    return;
  }
  if (hasRaw(brand, cmd)) {
    static uint16_t tmpBuf[1200]; uint16_t n = loadRaw(brand, cmd, tmpBuf, 1200);
    if (n > 0) { server.send(200,"application/json", buildCodeJson("RAW", 0, 0, tmpBuf, n)); return; }
  }
  server.send(404,"application/json","{\"ok\":false,\"msg\":\"Sinal não encontrado\"}");
}

void apiDeleteRaw() {
  if(server.hasArg("brand")&&server.hasArg("cmd")){
    deleteRawNVS(server.arg("brand"),server.arg("cmd"));
    deleteNECNVS(server.arg("brand"),server.arg("cmd"));
    deleteMideaNVS(server.arg("brand"),server.arg("cmd"));
  } else deleteAllRawNVS();
  server.send(200,"application/json","{\"ok\":true}");
}

void handleSave() {
  for(int i=0;i<3;i++){
    if(server.hasArg("n"+String(i))) rooms[i].name    = server.arg("n"+String(i));
    if(server.hasArg("a"+String(i))) rooms[i].acBrand = server.arg("a"+String(i));
    if(server.hasArg("t"+String(i))) rooms[i].tvBrand = server.arg("t"+String(i));
  }
  saveRooms();
  server.sendHeader("Location","/config",true); server.send(302,"text/plain","");
}

// ============================================================
//  API — Redes Wi-Fi salvas
// ============================================================
void apiWiFiList() {
  Preferences p;
  p.begin(WIFI_NAMESPACE, true);
  int count = p.getInt("count", 0);
  String currentSSID = WiFi.SSID();
  String json = "[";
  for (int i = 0; i < count; i++) {
    String ssid = p.getString(("s" + String(i)).c_str(), "");
    if (ssid.isEmpty()) continue;
    if (i > 0) json += ",";
    json += "{\"idx\":" + String(i) + ",\"ssid\":\"" + jsonEscape(ssid) + "\",\"current\":" + (ssid == currentSSID ? "true" : "false") + "}";
  }
  p.end();
  json += "]";
  server.send(200, "application/json", json);
}

void apiWiFiDelete() {
  int idx = server.arg("idx").toInt();
  Preferences p;
  p.begin(WIFI_NAMESPACE, true);
  int count = p.getInt("count", 0);
  String ssid = p.getString(("s" + String(idx)).c_str(), "");
  p.end();
  if (ssid.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Índice inválido\"}"); return; }
  deleteNetwork(ssid);
  server.send(200, "application/json", "{\"ok\":true}");
}

void apiWiFiAdd() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"msg\":\"SSID obrigatório\"}"); return; }
  saveNetwork(ssid, pass);
  bool ok = connectToNetwork(ssid, pass);
  if (ok) {
    MDNS.end(); MDNS.begin("ir-remote");
    server.send(200, "application/json", "{\"ok\":true,\"connected\":true,\"ip\":\"" + WiFi.localIP().toString() + "\"}");
  } else {
    server.send(200, "application/json", "{\"ok\":true,\"connected\":false,\"msg\":\"Rede salva. Tentará na próxima vez.\"}");
  }
}

// ============================================================
//  PÁGINA REDES WI-FI
// ============================================================
void handleRedesPage() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(F("<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Redes Wi-Fi</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Nunito:wght@700;800;900&display=swap' rel='stylesheet'>"
    "<style>"));
  server.sendContent_P(CSS);
  server.sendContent(F(
    ".cfg-wrap{padding:16px 16px 100px}"
    ".card{background:#fff;border-radius:20px;padding:20px;margin-bottom:16px;box-shadow:0 4px 24px rgba(0,0,0,.06)}"
    ".rtitle{font-size:17px;font-weight:900;padding-bottom:14px;margin-bottom:14px;border-bottom:2px solid #F2F3F7}"
    ".net-row{display:flex;align-items:center;gap:12px;padding:14px;background:#F2F3F7;border-radius:14px;margin-bottom:10px}"
    ".net-ico{font-size:22px;flex-shrink:0}"
    ".net-info{flex:1}"
    ".net-name{font-size:14px;font-weight:900;color:#1A1D2E}"
    ".net-badge{display:inline-block;background:#00C48C;color:#fff;font-size:10px;font-weight:900;padding:2px 8px;border-radius:100px;margin-left:6px}"
    ".del-btn{padding:8px 14px;border-radius:10px;border:none;background:#FFE5E5;color:#FF4D4D;font-size:12px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;flex-shrink:0}"
    ".ip-badge{background:#E6FBF5;border-radius:12px;padding:12px 16px;display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}"
    ".ip-label{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1px}"
    ".ip-val{font-size:15px;font-weight:900;color:#00C48C;font-family:monospace}"
    ".empty-msg{text-align:center;color:#8B8FA8;font-size:13px;font-weight:700;padding:20px 0}"
    ".field{margin-bottom:14px}"
    ".lbl{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1.2px;margin-bottom:6px;display:block}"
    "input[type=text],input[type=password]{width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:12px;color:#1A1D2E;padding:12px 14px;font-size:15px;font-family:Nunito,sans-serif;font-weight:700;outline:none}"
    "input:focus{border-color:#5B5FFF}"
    ".add-btn{display:block;width:100%;padding:16px;border-radius:16px;border:none;background:#5B5FFF;color:#fff;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;margin-top:4px}"
    ".add-btn:disabled{opacity:.5}"
    ".ap-info{background:#EEEEFF;border-radius:14px;padding:14px 16px;margin-bottom:14px}"
    ".ap-title{font-size:13px;font-weight:900;color:#5B5FFF;margin-bottom:6px}"
    ".ap-desc{font-size:12px;font-weight:700;color:#5B5FFF;opacity:.8;line-height:1.5}"
    "</style></head><body>"));

  String ipSTA = WiFi.localIP().toString();
  String curSSID = WiFi.SSID();
  bool staOk = WiFi.status() == WL_CONNECTED;

  server.sendContent(F("<div class='hdr'><div><h1>📶 Redes Wi-Fi</h1><p>Gerencie as redes salvas</p></div></div>"));
  server.sendContent("<div class='cfg-wrap'>");
  server.sendContent(F("<div class='card'><div class='rtitle'>📡 Access Point (sempre ativo)</div>"
    "<div class='ap-info'><div class='ap-title'>🔒 Acesso direto sem internet</div>"
    "<div class='ap-desc'>Wi-Fi: <strong>IR-Remote</strong><br>Senha: <strong>12345678</strong><br>IP: <strong>192.168.4.1</strong><br><br>"
    "Conecte neste Wi-Fi e abra http://192.168.4.1 para usar o controle remoto em qualquer lugar, mesmo sem internet."
    "</div></div></div>"));
  server.sendContent(F("<div class='card'><div class='rtitle'>🌐 Rede Local (opcional)</div>"));
  if (staOk) {
    server.sendContent("<div class='ip-badge'><div><div class='ip-label'>Conectado em</div><div class='ip-val' style='color:#5B5FFF'>" + curSSID + "</div></div></div>");
    server.sendContent("<div class='ip-badge'><div><div class='ip-label'>IP na rede local</div><div class='ip-val'>" + ipSTA + "</div></div></div>");
    server.sendContent(F("<div class='ip-badge'><div><div class='ip-label'>Acesso mDNS</div><div class='ip-val' style='color:#5B5FFF'>http://ir-remote.local</div></div></div>"));
  } else {
    server.sendContent(F("<div style='background:#FFF5E6;border-radius:12px;padding:14px;margin-bottom:12px;font-size:13px;font-weight:700;color:#996600'>⚠️ Não conectado a nenhuma rede local.<br>Use o AP acima ou adicione uma rede abaixo.</div>"));
  }
  server.sendContent(F("</div>"));
  server.sendContent(F("<div class='card'><div class='rtitle'>💾 Redes Salvas</div><div id='net-list'><div class='empty-msg'>Carregando...</div></div></div>"));
  server.sendContent(F("<div class='card'><div class='rtitle'>➕ Adicionar Nova Rede</div>"
    "<div class='field'><label class='lbl'>Nome da Rede (SSID)</label>"
    "<input type='text' id='inp-ssid' placeholder='Nome do Wi-Fi' autocomplete='off' autocorrect='off' autocapitalize='none' spellcheck='false'></div>"
    "<div class='field'><label class='lbl'>Senha</label><input type='password' id='inp-pass' placeholder='Senha do Wi-Fi'></div>"
    "<button class='add-btn' id='btn-add' onclick='addNet()'>✅ Salvar e Conectar</button>"
    "<div id='add-result' style='margin-top:12px;display:none'></div></div>"));
  server.sendContent(F("<div class='bnav'>"
    "<a class='bnav-btn' href='/'><span class='bnav-icon'>🎮</span>Controle</a>"
    "<a class='bnav-btn' href='/cadastro'><span class='bnav-icon'>➕</span>Cadastrar</a>"
    "<a class='bnav-btn' href='/codigos'><span class='bnav-icon'>📋</span>Códigos</a>"
    "<a class='bnav-btn active' href='/config'><span class='bnav-icon'>⚙️</span>Config</a>"
    "</div><div id='toast'></div>"));
  server.sendContent(F(R"html(<script>
function loadNets(){
  fetch('/api/wifi/list').then(function(r){return r.json();}).then(function(list){
    var el=document.getElementById('net-list');
    if(!list||!list.length){el.innerHTML='<div class="empty-msg">Nenhuma rede salva ainda.</div>';return;}
    var html='';
    list.forEach(function(n){
      html+='<div class="net-row"><span class="net-ico">📶</span>'
        +'<div class="net-info"><div class="net-name">'+n.ssid+(n.current?'<span class="net-badge">Conectado</span>':'')+'</div></div>'
        +'<button class="del-btn" onclick="delNet('+n.idx+',\''+n.ssid+'\')">🗑️</button></div>';
    });
    el.innerHTML=html;
  }).catch(function(){document.getElementById('net-list').innerHTML='<div class="empty-msg">Erro ao carregar.</div>';});
}
function delNet(idx,ssid){
  if(!confirm('Remover a rede "'+ssid+'"?'))return;
  fetch('/api/wifi/delete?idx='+idx,{method:'DELETE'}).then(function(r){return r.json();})
  .then(function(d){if(d.ok){toast('Rede removida!','ok');loadNets();}else toast('Erro','err');});
}
function addNet(){
  var ssid=document.getElementById('inp-ssid').value.trim();
  var pass=document.getElementById('inp-pass').value;
  if(!ssid){toast('Digite o nome da rede','err');return;}
  var btn=document.getElementById('btn-add');
  var res=document.getElementById('add-result');
  btn.disabled=true;btn.textContent='⏳ Conectando...';res.style.display='none';
  fetch('/api/wifi/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)})
  .then(function(r){return r.json();}).then(function(d){
    btn.disabled=false;btn.textContent='✅ Salvar e Conectar';res.style.display='block';
    if(d.connected){
      res.innerHTML='<div style="background:#E6FBF5;border-radius:12px;padding:14px;font-size:13px;font-weight:700;color:#00804D">'
        +'✅ Conectado! IP: <strong>'+d.ip+'</strong><br>Acesse por <strong>http://ir-remote.local</strong></div>';
      document.getElementById('inp-ssid').value='';document.getElementById('inp-pass').value='';loadNets();
    } else {
      res.innerHTML='<div style="background:#FFF5E6;border-radius:12px;padding:14px;font-size:13px;font-weight:700;color:#996600">⚠️ '+d.msg+'</div>';
      loadNets();
    }
  }).catch(function(){btn.disabled=false;btn.textContent='✅ Salvar e Conectar';toast('Erro de comunicação','err');});
}
function toast(m,t){var el=document.getElementById('toast');el.textContent=m;el.className='show '+(t||'');clearTimeout(el._t);el._t=setTimeout(function(){el.className='';},2200);}
loadNets();
</script>)html"));
  server.sendContent("</body></html>");
  server.sendContent("");
}

// ============================================================
//  OTA
// ============================================================
void handleOTAPage() {
  String h = "<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>OTA</title><style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:sans-serif;background:#0f1117;color:#e2e8f0;display:flex;align-items:center;justify-content:center;min-height:100vh;padding:20px}"
    ".card{background:#1a1f2e;border-radius:24px;padding:32px;width:100%;max-width:420px}"
    "h1{font-size:20px;font-weight:900;text-align:center;margin-bottom:24px}"
    ".drop{border:2px dashed #2d3748;border-radius:16px;padding:28px;text-align:center;cursor:pointer;margin-bottom:16px}"
    ".drop:hover{border-color:#5B5FFF;background:#1e2340}"
    "#fi{display:none}"
    ".btn{width:100%;padding:14px;border-radius:12px;border:none;background:#5B5FFF;color:#fff;font-size:15px;font-weight:900;cursor:pointer}"
    ".btn:disabled{opacity:.4}"
    ".bar-bg{background:#2d3748;border-radius:100px;height:8px;overflow:hidden;margin:16px 0 8px}"
    ".bar{background:#5B5FFF;height:100%;width:0;transition:width .3s}"
    ".msg{text-align:center;font-size:13px;margin-top:12px}"
    ".back{display:block;text-align:center;margin-top:16px;color:#5B5FFF;font-size:13px}"
    "</style></head><body><div class='card'>"
    "<h1>OTA Update — v" FW_VERSION "</h1>"
    "<div class='drop' onclick='document.getElementById(\"fi\").click()'>"
    "<div style='font-size:36px;margin-bottom:8px'>📦</div>"
    "<p style='font-size:13px;color:#718096'>Clique ou arraste o <strong>.bin</strong></p>"
    "<div id='fname' style='font-size:12px;color:#5B5FFF;margin-top:6px;display:none'></div></div>"
    "<input type='file' id='fi' accept='.bin'>"
    "<button class='btn' id='btn' disabled onclick='go()'>Enviar Firmware</button>"
    "<div id='prog' style='display:none'><div class='bar-bg'><div class='bar' id='bar'></div></div>"
    "<div style='text-align:center;font-size:12px;color:#718096' id='pct'>0%</div></div>"
    "<div class='msg' id='msg'></div><a class='back' href='/'>← Voltar</a>"
    "</div><script>"
    "var file=null;"
    "document.getElementById('fi').onchange=function(e){setFile(e.target.files[0]);};"
    "function setFile(f){if(!f||!f.name.endsWith('.bin'))return;file=f;"
    "document.getElementById('fname').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';"
    "document.getElementById('fname').style.display='block';"
    "document.getElementById('btn').disabled=false;}"
    "function go(){if(!file)return;var fd=new FormData();fd.append('firmware',file);"
    "var xhr=new XMLHttpRequest();document.getElementById('prog').style.display='block';"
    "document.getElementById('btn').disabled=true;"
    "xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);"
    "document.getElementById('bar').style.width=p+'%';document.getElementById('pct').textContent=p+'%';}};"
    "xhr.onload=function(){if(xhr.status==200){document.getElementById('msg').innerHTML='<span style=\"color:#00C48C\">Concluído! Reiniciando...</span>';"
    "setTimeout(function(){location.href='/';},5000);}else{document.getElementById('msg').innerHTML='<span style=\"color:#FF4D4D\">Erro: '+xhr.responseText+'</span>';"
    "document.getElementById('btn').disabled=false;}};"
    "xhr.open('POST','/ota/upload');xhr.send(fd);}"
    "</script></body></html>";
  server.send(200,"text/html",h);
}
void handleOTAUpload() {
  HTTPUpload& u = server.upload();
  if(u.status==UPLOAD_FILE_START){ if(!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); }
  else if(u.status==UPLOAD_FILE_WRITE){ if(Update.write(u.buf,u.currentSize)!=u.currentSize) Update.printError(Serial); }
  else if(u.status==UPLOAD_FILE_END){ if(Update.end(true)) Serial.printf("[OTA] OK: %u bytes\n", u.totalSize); else Update.printError(Serial); }
}
void handleOTAResult() {
  if(Update.hasError()) server.send(500,"text/plain","Erro");
  else { server.send(200,"text/plain","OK"); delay(500); ESP.restart(); }
}

// ============================================================
//  sendRoomPanel
// ============================================================
void sendRoomPanel(int i) {
  bool hasAC = rooms[i].acBrand != "nenhum";
  bool hasTV = rooms[i].tvBrand != "nenhum";
  String si = String(i);

  server.sendContent("<div class='rpanel' id='panel-"+si+"'"+(i==0?" style='display:block'":"")+"><div class='device-toggle'>");
  if(hasAC) server.sendContent("<button class='dtog active' id='dsw-ac-"+si+"' onclick='switchDev("+si+",\"ac\")'>❄️ AC</button>");
  if(hasTV) {
    String cls = hasAC ? "dtog" : "dtog active";
    server.sendContent("<button class='"+cls+"' id='dsw-tv-"+si+"' onclick='switchDev("+si+",\"tv\")'>📺 TV</button>");
  }
  server.sendContent("<button class='dtog' id='dsw-gen-"+si+"' onclick='switchDev("+si+",\"gen\")'>📡 Outros</button></div>");

  if(hasAC){
    server.sendContent("<div class='sub' id='sub-ac-"+si+"' style='display:block'>");
    server.sendContent("<div class='ac-main'><div class='ac-brand-tag'>❄️ "+rooms[i].acBrand+"</div>");
    server.sendContent("<div class='temp-display'><span class='temp-num' id='t-val-"+si+"'>22</span><span class='temp-deg'>&nbsp;°C</span></div>");
    server.sendContent("<div class='temp-controls'><button class='tc-btn' onclick='chTemp("+si+",-1)'>−</button><button class='tc-btn' onclick='chTemp("+si+",+1)'>+</button></div>");
    server.sendContent("<div class='ac-actions'><button class='ac-on' onclick='doAC("+si+",1)'>⚡ LIGAR</button><button class='ac-off' onclick='doAC("+si+",0)'>⏻ DESLIGAR</button></div>");
    server.sendContent(
      "<div class='nw-bar' id='nw-"+si+"'>"
      "<span class='nw-txt'>⚠️ O AC não respondeu?</span>"
      "<div style='display:flex;gap:6px;align-items:center'>"
      "<button class='nw-btn' onclick='captureAC("+si+")'>📡 Capturar sinal</button>"
      "<button class='nw-dismiss' onclick='hideNW("+si+")'>✕</button>"
      "</div></div></div>");
    server.sendContent(
      "<div class='section-title'>Modo</div><div class='pills-row'>"
      "<button class='pill sel-mode' data-mode='cool' onclick='selMode("+si+",this)'>❄️ Frio</button>"
      "<button class='pill' data-mode='heat' onclick='selMode("+si+",this)'>🔥 Calor</button>"
      "<button class='pill' data-mode='dry' onclick='selMode("+si+",this)'>💧 Secar</button>"
      "<button class='pill' data-mode='fan' onclick='selMode("+si+",this)'>🌀 Ventil.</button></div>"
      "<div class='section-title'>Ventilador</div><div class='pills-row'>"
      "<button class='pill sel-fan' data-fan='auto' onclick='selFan("+si+",this)'>Auto</button>"
      "<button class='pill' data-fan='low' onclick='selFan("+si+",this)'>Baixo</button>"
      "<button class='pill' data-fan='medium' onclick='selFan("+si+",this)'>Médio</button>"
      "<button class='pill' data-fan='high' onclick='selFan("+si+",this)'>Alto</button></div></div>");
  }

  if(hasTV){
    String tvDisp = hasAC ? "none" : "block";
    server.sendContent("<div class='sub' id='sub-tv-"+si+"' style='display:"+tvDisp+"'><div class='tv-wrap'><div class='tv-brand-row'><div><div class='tv-brand-label'>TV conectada</div><div class='tv-brand-name'>"+rooms[i].tvBrand+"</div></div><span style='font-size:28px'>📺</span></div>");
    server.sendContent("<div class='tv-power-row'><button class='tv-pow' onclick='doTV("+si+",\"power\")'>⏻ POWER</button></div>");
    server.sendContent(
      "<div class='tv-grid-2'>"
      "<button class='tvb' onclick='doTV("+si+",\"voldown\")'><span class='tvb-icon'>🔈</span>Vol -</button>"
      "<button class='tvb' onclick='doTV("+si+",\"volup\")'><span class='tvb-icon'>🔊</span>Vol +</button>"
      "<button class='tvb' onclick='doTV("+si+",\"chdown\")'><span class='tvb-icon'>📻</span>CH -</button>"
      "<button class='tvb' onclick='doTV("+si+",\"chup\")'><span class='tvb-icon'>📻</span>CH +</button></div>");
    server.sendContent(
      "<div class='tv-grid-2'>"
      "<button class='tvb accent-btn' onclick='doTV("+si+",\"mute\")'><span class='tvb-icon'>🔇</span>Mudo</button>"
      "<button class='tvb accent-btn' onclick='doTV("+si+",\"source\")'><span class='tvb-icon'>🔄</span>Fonte</button></div>"
      "<div class='dpad-wrap'>"
      "<div class='dpad-row'><button class='dp-btn' onclick='doTV("+si+",\"up\")'>▲</button></div>"
      "<div class='dpad-row'><button class='dp-btn' onclick='doTV("+si+",\"left\")'>◀</button>"
      "<button class='dp-ok' onclick='doTV("+si+",\"ok\")'>OK</button>"
      "<button class='dp-btn' onclick='doTV("+si+",\"right\")'>▶</button></div>"
      "<div class='dpad-row'><button class='dp-btn' onclick='doTV("+si+",\"down\")'>▼</button></div></div>");
    server.sendContent(
      "<div class='tv-grid-2'>"
      "<button class='tvb' onclick='doTV("+si+",\"menu\")'><span class='tvb-icon'>☰</span>Menu</button>"
      "<button class='tvb' onclick='doTV("+si+",\"back\")'><span class='tvb-icon'>↩</span>Voltar</button>"
      "</div></div></div>");
  }

  server.sendContent(
    "<div class='sub' id='sub-gen-"+si+"' style='display:none'><div class='gen-wrap'>"
    "<div class='tv-brand-row'><div><div class='tv-brand-label'>Dispositivos cadastrados</div>"
    "<div class='tv-brand-name' style='font-size:13px'>"+rooms[i].name+"</div></div><span style='font-size:28px'>📡</span></div>"
    "<div class='gen-grid' id='gen-grid-"+si+"'><div class='gen-empty'>Carregando...</div></div>"
    "<a href='/cadastro?room="+si+"' style='display:block;text-align:center;margin-top:16px;padding:14px;"
    "background:#EEEEFF;border-radius:14px;color:#5B5FFF;font-size:13px;font-weight:800;text-decoration:none'>➕ Cadastrar novo dispositivo</a>"
    "</div></div>");

  server.sendContent("</div>");
}

// ============================================================
//  PÁGINA PRINCIPAL
// ============================================================
void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(F("<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>"
    "<title>IR Remote</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Nunito:wght@700;800;900&display=swap' rel='stylesheet'>"
    "<style>"));
  server.sendContent_P(CSS);
  server.sendContent(F("</style></head><body>"));
  server.sendContent("<div class='hdr'><div><h1>🎮 IR Remote</h1><p id='hrm'>"+rooms[0].name+"</p></div>"
    "<div class='online'><div class='dot'></div>v" FW_VERSION "</div></div>");
  server.sendContent(F("<div class='rtabs'>"));
  for(int i=0;i<3;i++){
    String cls = (i==0) ? "rtab active" : "rtab";
    server.sendContent("<button class='"+cls+"' onclick='selRoom("+String(i)+")'>"+rooms[i].name+"</button>");
  }
  server.sendContent(F("</div>"));
  for(int i=0;i<3;i++) sendRoomPanel(i);

  // Modal de captura — atualizado para mostrar progresso 1/3, 2/3, 3/3
  server.sendContent(F(
    "<div class='modal-overlay' id='learn-modal'><div class='modal'>"
    "<div class='modal-title'>📡 Aprender Sinal</div>"
    "<div class='modal-sub' id='lm-sub'></div>"
    "<div class='ir-ring wait' id='lm-ring'>1/3</div>"
    "<div class='modal-msg' id='lm-msg'>Aponte o controle e pressione o botão...</div>"
    "<div class='modal-btns'>"
    "<button class='mbtn mbtn-save' id='lm-save' onclick='learnSave()' disabled>💾 Salvar</button>"
    "<button class='mbtn mbtn-cancel' onclick='learnCancel()'>Cancelar</button>"
    "</div></div></div>"));

  server.sendContent(F(
    "<div class='bnav'>"
    "<button class='bnav-btn active'><span class='bnav-icon'>🎮</span>Controle</button>"
    "<a class='bnav-btn' href='/cadastro'><span class='bnav-icon'>➕</span>Cadastrar</a>"
    "<a class='bnav-btn' href='/codigos'><span class='bnav-icon'>📋</span>Códigos</a>"
    "<a class='bnav-btn' href='/config'><span class='bnav-icon'>⚙️</span>Config</a>"
    "</div><div id='toast'></div>"));

  server.sendContent("<script>"
    "var T=[22,22,22],M=['cool','cool','cool'],F=['auto','auto','auto'];"
    "var R=[{n:'"+rooms[0].name+"',ac:'"+rooms[0].acBrand+"',tv:'"+rooms[0].tvBrand+"'},"
    "{n:'"+rooms[1].name+"',ac:'"+rooms[1].acBrand+"',tv:'"+rooms[1].tvBrand+"'},"
    "{n:'"+rooms[2].name+"',ac:'"+rooms[2].acBrand+"',tv:'"+rooms[2].tvBrand+"'}];");

  server.sendContent(F(
    "function selRoom(i){"
    "for(var r=0;r<3;r++)hideNW(r);"
    "document.querySelectorAll('.rtab').forEach(function(t,j){t.classList.toggle('active',j===i);});"
    "document.querySelectorAll('.rpanel').forEach(function(p){p.style.display='none';});"
    "var el=document.getElementById('panel-'+i);if(el)el.style.display='block';"
    "document.getElementById('hrm').textContent=R[i].n;}"

    "function switchDev(r,d){['ac','tv','gen'].forEach(function(x){"
    "var s=document.getElementById('sub-'+x+'-'+r);"
    "var b=document.getElementById('dsw-'+x+'-'+r);"
    "if(s)s.style.display=(x===d)?'block':'none';"
    "if(b)b.classList.toggle('active',x===d);});if(d==='gen')loadGeneric(r);}"

    "function chTemp(r,d){T[r]=Math.max(16,Math.min(30,T[r]+d));document.getElementById('t-val-'+r).textContent=T[r];}"
    "function selMode(r,el){el.parentNode.querySelectorAll('.pill').forEach(function(p){p.classList.remove('sel-mode');});el.classList.add('sel-mode');M[r]=el.dataset.mode;}"
    "function selFan(r,el){el.parentNode.querySelectorAll('.pill').forEach(function(p){p.classList.remove('sel-fan');});el.classList.add('sel-fan');F[r]=el.dataset.fan;}"

    "var nwTimer=[null,null,null];"
    "function hideNW(r){var bar=document.getElementById('nw-'+r);if(bar)bar.classList.remove('show');if(nwTimer[r]){clearTimeout(nwTimer[r]);nwTimer[r]=null;}}"
    "function showNW(r){hideNW(r);nwTimer[r]=setTimeout(function(){var bar=document.getElementById('nw-'+r);if(bar)bar.classList.add('show');},4000);}"
    "function captureAC(r){hideNW(r);var b=R[r].ac;var lastCmd=lastACCmd[r]||'power_on';"
    "var label='AC '+b+' / '+(lastCmd==='power_on'?'Ligar':'Desligar');offerLearn(b,lastCmd,label,r);}"
    "var lastACCmd=[null,null,null];"

    "function doAC(r,pw){"
    "var b=R[r].ac;if(!b||b==='nenhum')return;"
    "var cmd=pw?'power_on':'power_off';lastACCmd[r]=cmd;hideNW(r);"
    "fetch('/ac?brand='+encodeURIComponent(b)+'&power='+pw+'&temp='+T[r]+'&mode='+M[r]+'&fan='+F[r])"
    ".then(function(res){return res.json();})"
    ".then(function(d){"
    "if(d.ok){toast(pw?'AC '+T[r]+'°C — '+d.status:'AC desligado — '+d.status,'ok');"
    "if(d.usedNative)showNW(r);"
    "}else if(d.status==='UNKNOWN'){offerLearn(d.brand,d.cmd,'AC '+d.brand+(pw?' Ligar':' Desligar'),r);"
    "}else{toast(d.msg||'Erro','err');}}).catch(function(){toast('Sem resposta','err');});}"

    "function doTV(r,c){var b=R[r].tv;if(!b||b==='nenhum')return;"
    "fetch('/tv?brand='+encodeURIComponent(b)+'&cmd='+encodeURIComponent(c))"
    ".then(function(res){return res.json();})"
    ".then(function(d){if(d.ok)toast(c.toUpperCase(),'ok');"
    "else if(d.status==='UNKNOWN')offerLearn(d.brand,d.cmd,'TV '+d.brand+' / '+c,-1);"
    "else toast(d.msg||'Erro','err');}).catch(function(){toast('Sem resposta','err');});}"

    "function getIcon(brand,apiIcon){if(apiIcon&&apiIcon.length>0)return apiIcon;brand=brand||'';"
    "if(brand.indexOf('vent')===0)return'🌀';if(brand.indexOf('ac')===0)return'❄️';"
    "if(brand.indexOf('tv')===0)return'📺';if(brand.indexOf('luz')===0)return'💡';"
    "if(brand.indexOf('projetor')===0)return'🎥';return'🔌';}"

    "function loadGeneric(r){var g=document.getElementById('gen-grid-'+r);"
    "g.innerHTML='<div class=\"gen-empty\">Carregando...</div>';"
    "fetch('/api/devices?room='+r).then(function(res){return res.json();})"
    ".then(function(list){if(!list||list.length===0){"
    "g.innerHTML='<div class=\"gen-empty\">Nenhum dispositivo aqui.<br>Use Cadastrar para adicionar.</div>';return;}"
    "var html='';for(var i=0;i<list.length;i++){var d=list[i];var ico=getIcon(d.brand,d.icon);"
    "html+='<button class=\"gen-btn\" onclick=\"doGeneric(this)\" data-brand=\"'+d.brand+'\" data-cmd=\"'+d.cmd+'\">"
    "<span class=\"gen-btn-icon\">'+ico+'</span><span>'+d.label+'</span></button>';}g.innerHTML=html;});}"

    "function doGeneric(btn){var brand=btn.getAttribute('data-brand');var cmd=btn.getAttribute('data-cmd');"
    "var label=btn.querySelector('span:last-child').textContent;"
    "fetch('/generic?brand='+encodeURIComponent(brand)+'&cmd='+encodeURIComponent(cmd))"
    ".then(function(res){return res.json();})"
    ".then(function(d){if(d.ok)toast(label+' OK','ok');"
    "else if(d.status==='UNKNOWN')offerLearn(d.brand,d.cmd,label,-1);"
    "else toast(d.msg||'Erro','err');}).catch(function(){toast('Sem resposta','err');});}"

    "function toast(m,t){var el=document.getElementById('toast');el.textContent=m;"
    "el.className='show '+(t||'');clearTimeout(el._t);el._t=setTimeout(function(){el.className='';},2200);}"

    // offerLearn e poll — atualizado para 3 capturas
    "var lPoll=null;"
    "function offerLearn(brand,cmd,label,room){"
    "toast('Abrindo captura de sinal...','warn');"
    "setTimeout(function(){"
    "fetch('/api/learn/start?brand='+encodeURIComponent(brand)+'&cmd='+encodeURIComponent(cmd)+(room>=0?'&room='+room:''),{method:'POST'})"
    ".then(function(){"
    "document.getElementById('lm-sub').textContent=label;"
    "document.getElementById('lm-ring').className='ir-ring wait';"
    "document.getElementById('lm-ring').textContent='1/3';"
    "document.getElementById('lm-msg').textContent='Captura 1/3 — aponte e pressione o botão...';"
    "document.getElementById('lm-save').disabled=true;"
    "document.getElementById('learn-modal').classList.add('open');"
    "if(lPoll){clearInterval(lPoll);lPoll=null;}"
    "lPoll=setInterval(function(){"
    "fetch('/api/learn/poll').then(function(r){return r.json();})"
    ".then(function(d){"
    "var atual=d.capturaAtual||0;"
    "if(!d.capturado){"
    // Ainda capturando — mostra qual captura aguarda
    "document.getElementById('lm-ring').textContent=(atual+1)+'/3';"
    "document.getElementById('lm-msg').textContent='Captura '+(atual+1)+'/3 — aponte e pressione o botão...';"
    "}else{"
    // Todas as capturas concluídas
    "clearInterval(lPoll);lPoll=null;"
    "document.getElementById('lm-ring').className='ir-ring done';"
    "document.getElementById('lm-ring').textContent='✅';"
    "document.getElementById('lm-msg').textContent='Melhor sinal selecionado! ('+d.len+' amostras)';"
    "document.getElementById('lm-save').disabled=false;"
    "}});},600);});},1200);}"

    "function learnSave(){fetch('/api/learn/save',{method:'POST'}).then(function(r){return r.json();})"
    ".then(function(d){document.getElementById('learn-modal').classList.remove('open');"
    "if(lPoll){clearInterval(lPoll);lPoll=null;}"
    "toast(d.ok?'Sinal salvo! Próxima vez usa o capturado.':'Erro: '+d.msg,d.ok?'ok':'err');});}"

    "function learnCancel(){if(lPoll){clearInterval(lPoll);lPoll=null;}"
    "fetch('/api/learn/stop',{method:'POST'});document.getElementById('learn-modal').classList.remove('open');}"
    "</script></body></html>"));

  server.sendContent("");
}

// ============================================================
//  PÁGINA CÓDIGOS
// ============================================================
void handleCodigos() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(F("<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1'>"
    "<title>Códigos Gravados</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Nunito:wght@700;800;900&display=swap' rel='stylesheet'>"
    "<style>"));
  server.sendContent_P(CSS);
  server.sendContent(F(
    ".cod-wrap{padding:16px 16px 100px}"
    ".cod-card{background:#fff;border-radius:24px;padding:20px;margin-bottom:16px;box-shadow:0 4px 24px rgba(0,0,0,.06)}"
    ".cod-title{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1.2px;margin-bottom:14px}"
    ".dev-row{display:flex;align-items:center;gap:12px;padding:12px 14px;background:#F2F3F7;border-radius:14px;margin-bottom:8px;cursor:pointer;transition:.15s}"
    ".dev-row:hover{background:#EEEEFF}"
    ".dev-ico{font-size:24px;flex-shrink:0}"
    ".dev-info{flex:1}"
    ".dev-name{font-size:14px;font-weight:900;color:#1A1D2E}"
    ".dev-key{font-size:11px;color:#8B8FA8;font-weight:700;margin-top:2px;font-family:monospace}"
    ".dev-room{font-size:11px;font-weight:800;color:#5B5FFF;margin-top:2px}"
    ".dev-arrow{font-size:18px;color:#8B8FA8;flex-shrink:0;transition:.2s}"
    ".code-panel{display:none;padding:10px 0 4px}"
    ".code-tabs{display:flex;gap:6px;margin-bottom:10px}"
    ".ctab{padding:6px 14px;border-radius:8px;border:2px solid #E8EAF2;font-size:12px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif;background:transparent;color:#8B8FA8}"
    ".ctab.active{background:#5B5FFF;color:#fff;border-color:#5B5FFF}"
    ".code-box{background:#F2F3F7;border-radius:12px;padding:14px;margin-bottom:8px;overflow-x:auto;max-height:220px;overflow-y:auto}"
    ".code-box pre{font-size:11px;color:#1A1D2E;font-family:monospace;white-space:pre-wrap;word-break:break-all;margin:0}"
    ".code-badges{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}"
    ".badge{padding:4px 10px;border-radius:8px;font-size:11px;font-weight:900}"
    ".badge-proto{background:#EEEEFF;color:#5B5FFF}"
    ".badge-bits{background:#FFF5E6;color:#FF9500}"
    ".badge-len{background:#E6FBF5;color:#00C48C}"
    ".copy-btn{width:100%;padding:11px;border-radius:10px;border:2px solid #E8EAF2;background:#fff;color:#5B5FFF;font-size:13px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif}"
    ".empty-msg{text-align:center;color:#8B8FA8;font-size:13px;font-weight:700;padding:32px 0}"
    ".filter-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:14px}"
    ".filt{padding:8px 14px;border-radius:100px;border:2px solid #E8EAF2;background:#fff;color:#8B8FA8;font-size:12px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif}"
    ".filt.active{background:#5B5FFF;color:#fff;border-color:#5B5FFF}"
    ".search-box{width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:14px;color:#1A1D2E;padding:12px 16px;font-size:14px;font-family:Nunito,sans-serif;font-weight:700;outline:none;margin-bottom:14px}"
    ".search-box:focus{border-color:#5B5FFF}"
    "</style></head><body>"));
  server.sendContent(F("<div class='hdr'><div><h1>📋 Códigos Gravados</h1><p>Veja o sinal IR de qualquer dispositivo</p></div>"
    "<div class='online'><div class='dot'></div>v" FW_VERSION "</div></div>"));
  server.sendContent(F("<div class='cod-wrap'>"
    "<input class='search-box' type='text' id='srch' placeholder='Buscar dispositivo...' oninput='renderDevs()'>"
    "<div class='filter-row'>"
    "<button class='filt active' data-f='all' onclick='setFilt(this)'>Todos</button>"
    "<button class='filt' data-f='vent' onclick='setFilt(this)'>🌀 Ventilador</button>"
    "<button class='filt' data-f='ac' onclick='setFilt(this)'>❄️ AC</button>"
    "<button class='filt' data-f='luz' onclick='setFilt(this)'>💡 Luz</button>"
    "<button class='filt' data-f='tv' onclick='setFilt(this)'>📺 TV</button>"
    "<button class='filt' data-f='outro' onclick='setFilt(this)'>🔌 Outro</button>"
    "</div><div class='cod-card'>"
    "<div class='cod-title'>Dispositivos cadastrados</div>"
    "<div id='dev-list'><div class='empty-msg'>Carregando...</div></div>"
    "</div></div>"));
  server.sendContent(F("<div class='bnav'>"
    "<a class='bnav-btn' href='/'><span class='bnav-icon'>🎮</span>Controle</a>"
    "<a class='bnav-btn' href='/cadastro'><span class='bnav-icon'>➕</span>Cadastrar</a>"
    "<button class='bnav-btn active'><span class='bnav-icon'>📋</span>Códigos</button>"
    "<a class='bnav-btn' href='/config'><span class='bnav-icon'>⚙️</span>Config</a>"
    "</div><div id='toast'></div>"));
  server.sendContent(F(R"html(<script>
var allDevs=[],curFilt='all',loadedData={};
function getIcon(brand,icon){if(icon&&icon.length>0)return icon;brand=brand||'';
  if(brand.indexOf('vent')===0)return'🌀';if(brand.indexOf('ac')===0)return'❄️';
  if(brand.indexOf('tv')===0)return'📺';if(brand.indexOf('luz')===0)return'💡';
  if(brand.indexOf('projetor')===0)return'🎥';return'🔌';}
function getCat(brand){brand=brand||'';
  if(brand.indexOf('vent')===0)return'vent';if(brand.indexOf('ac')===0)return'ac';
  if(brand.indexOf('luz')===0)return'luz';if(brand.indexOf('tv')===0)return'tv';
  if(brand.indexOf('projetor')===0)return'projetor';return'outro';}
function setFilt(btn){curFilt=btn.dataset.f;document.querySelectorAll('.filt').forEach(function(b){b.classList.toggle('active',b===btn);});renderDevs();}
function renderDevs(){
  var q=document.getElementById('srch').value.toLowerCase();
  var list=allDevs.filter(function(d){
    if(curFilt!=='all'&&getCat(d.brand)!==curFilt)return false;
    if(!q)return true;
    return(d.label+' '+d.brand+' '+d.cmd+' '+d.roomName).toLowerCase().indexOf(q)!==-1;
  });
  var el=document.getElementById('dev-list');
  if(!list.length){el.innerHTML='<div class="empty-msg">Nenhum dispositivo encontrado.</div>';return;}
  var html='';
  list.forEach(function(d){
    var ico=getIcon(d.brand,d.icon);
    html+='<div>'
      +'<div class="dev-row" onclick="toggleCode(\''+d.idx+'\',\''+d.brand+'\',\''+d.cmd+'\')">'
      +'<span class="dev-ico">'+ico+'</span>'
      +'<div class="dev-info"><div class="dev-name">'+d.label+'</div>'
      +'<div class="dev-key">'+d.brand+' / '+d.cmd+'</div>'
      +'<div class="dev-room">'+d.roomName+'</div></div>'
      +'<span class="dev-arrow" id="arr-'+d.idx+'">›</span></div>'
      +'<div class="code-panel" id="panel-'+d.idx+'">'
      +'<div class="code-badges" id="badges-'+d.idx+'"></div>'
      +'<div class="code-tabs">'
      +'<button class="ctab active" onclick="switchTab(\''+d.idx+'\',\'hex\',this)">HEX</button>'
      +'<button class="ctab" onclick="switchTab(\''+d.idx+'\',\'c\',this)">C Array</button>'
      +'<button class="ctab" onclick="switchTab(\''+d.idx+'\',\'json\',this)">JSON</button>'
      +'</div>'
      +'<div id="vh-'+d.idx+'"><div class="code-box"><pre id="hex-'+d.idx+'">Carregando...</pre></div>'
      +'<button class="copy-btn" onclick="copyTab(\''+d.idx+'\',\'hex\')">📋 Copiar HEX</button></div>'
      +'<div id="vc-'+d.idx+'" style="display:none"><div class="code-box"><pre id="c-'+d.idx+'"></pre></div>'
      +'<button class="copy-btn" onclick="copyTab(\''+d.idx+'\',\'c\')">📋 Copiar C Array</button></div>'
      +'<div id="vj-'+d.idx+'" style="display:none"><div class="code-box"><pre id="json-'+d.idx+'"></pre></div>'
      +'<button class="copy-btn" onclick="copyTab(\''+d.idx+'\',\'json\')">📋 Copiar JSON</button></div>'
      +'</div></div>';
  });
  el.innerHTML=html;
}
function toggleCode(idx,brand,cmd){
  var panel=document.getElementById('panel-'+idx);
  var arr=document.getElementById('arr-'+idx);
  var open=panel.style.display==='block';
  if(open){panel.style.display='none';arr.textContent='›';return;}
  panel.style.display='block';arr.textContent='⌄';
  if(loadedData[idx]){fillPanel(idx,loadedData[idx]);return;}
  fetch('/api/signal/raw/bydev?brand='+encodeURIComponent(brand)+'&cmd='+encodeURIComponent(cmd))
    .then(function(r){return r.json();})
    .then(function(d){
      if(!d.ok){document.getElementById('hex-'+idx).textContent='Sinal não encontrado.';return;}
      loadedData[idx]=d;fillPanel(idx,d);
    }).catch(function(){document.getElementById('hex-'+idx).textContent='Erro ao carregar.';});
}
function fillPanel(idx,d){
  var bd=document.getElementById('badges-'+idx);
  bd.innerHTML='<span class="badge badge-proto">'+d.proto+'</span>'
    +(d.bits>0?'<span class="badge badge-bits">'+d.bits+' bits</span>':'')
    +(d.len>0?'<span class="badge badge-len">'+d.len+' amostras</span>':'');
  document.getElementById('hex-'+idx).textContent='Protocolo: '+d.proto+'\nValor HEX: '+d.hex+(d.len>0?'\nAmostras RAW: '+d.len:'');
  document.getElementById('c-'+idx).textContent=d.raw_c.replace(/\\n/g,'\n');
  var arr=d.raw_json,lines=[],row=[];
  for(var i=0;i<arr.length;i++){row.push(arr[i]);if(row.length===12){lines.push('  '+row.join(', '));row=[];}}
  if(row.length)lines.push('  '+row.join(', '));
  document.getElementById('json-'+idx).textContent='[\n'+lines.join(',\n')+'\n]';
}
function switchTab(idx,tab,btn){
  ['h','c','j'].forEach(function(t){
    var id={h:'hex',c:'c',j:'json'}[t];
    document.getElementById('v'+t+'-'+idx).style.display=(id===tab)?'block':'none';
  });
  btn.parentNode.querySelectorAll('.ctab').forEach(function(b){b.classList.remove('active');});
  btn.classList.add('active');
}
function copyTab(idx,tab){
  var d=loadedData[idx];if(!d)return;
  var text='';
  if(tab==='hex') text='Protocolo: '+d.proto+'\nValor HEX: '+d.hex+(d.len>0?'\nAmostras RAW: '+d.len:'');
  else if(tab==='c') text=d.raw_c.replace(/\\n/g,'\n');
  else{var arr=d.raw_json,lines=[],row=[];
    for(var i=0;i<arr.length;i++){row.push(arr[i]);if(row.length===12){lines.push('  '+row.join(', '));row=[];}}
    if(row.length)lines.push('  '+row.join(', '));text='[\n'+lines.join(',\n')+'\n]';}
  if(navigator.clipboard)navigator.clipboard.writeText(text).then(function(){toast('Copiado!','ok');});
  else{var ta=document.createElement('textarea');ta.value=text;document.body.appendChild(ta);ta.select();document.execCommand('copy');document.body.removeChild(ta);toast('Copiado!','ok');}
}
function toast(m,t){var el=document.getElementById('toast');el.textContent=m;el.className='show '+(t||'');clearTimeout(el._t);el._t=setTimeout(function(){el.className='';},2200);}
fetch('/api/devices/all').then(function(r){return r.json();}).then(function(list){allDevs=list||[];renderDevs();})
  .catch(function(){document.getElementById('dev-list').innerHTML='<div class="empty-msg">Erro ao carregar.</div>';});
</script>)html"));
  server.sendContent("</body></html>");
  server.sendContent("");
}

// ============================================================
//  TELA CADASTRO
// ============================================================
const char* AC_BRANDS[] = {"nenhum","midea","elgin","samsung","lg","daikin","fujitsu","mitsubishi","panasonic","carrier","whirlpool","gree","tcl","hitachi"};
const char* TV_BRANDS[] = {"nenhum","samsung","lg","sony","philips","sharp","panasonic","philco","cce","gradiente","semp","buster"};

String makeSelect(const String& nm, const char** list, int len, const String& cur){
  String s = "<select name='"+nm+"' style='width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:12px;color:#1A1D2E;padding:12px 14px;font-size:15px;font-family:Nunito,sans-serif;font-weight:700;margin-top:6px;outline:none'>";
  for(int i=0;i<len;i++){
    String b=list[i]; String lbl=b; lbl.setCharAt(0,toupper(lbl.charAt(0)));
    s+="<option value='"+b+"'"+(b==cur?" selected":"")+">"+lbl+"</option>";
  }
  return s+"</select>";
}

void handleCadastro() {
  int preRoom = server.hasArg("room") ? server.arg("room").toInt() : 0;
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(F("<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1'>"
    "<title>Cadastrar</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Nunito:wght@700;800;900&display=swap' rel='stylesheet'>"
    "<style>"));
  server.sendContent_P(CSS);
  server.sendContent(F(
    ".cad-wrap{padding:16px 16px 100px}"
    ".cad-card{background:#fff;border-radius:24px;padding:24px;margin-bottom:16px;box-shadow:0 4px 24px rgba(0,0,0,.06)}"
    ".field{margin-bottom:16px}"
    ".lbl{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1.2px;margin-bottom:8px;display:block}"
    "select,input[type=text]{width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:12px;color:#1A1D2E;padding:12px 14px;font-size:15px;font-family:Nunito,sans-serif;font-weight:700;outline:none}"
    ".dev-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:4px}"
    ".dev-btn{padding:14px 6px;border-radius:16px;border:2px solid #E8EAF2;background:#fff;color:#8B8FA8;font-size:11px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif;display:flex;flex-direction:column;align-items:center;gap:5px;transition:.15s;text-align:center}"
    ".dev-btn.sel{border-color:#5B5FFF;background:#EEEEFF;color:#5B5FFF}"
    ".dev-btn-ico{font-size:26px}"
    ".fn-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}"
    ".fn-btn{padding:12px 8px;border-radius:14px;border:2px solid #E8EAF2;background:#fff;color:#8B8FA8;font-size:12px;font-weight:800;cursor:pointer;font-family:Nunito,sans-serif;display:flex;align-items:center;gap:8px;transition:.15s}"
    ".fn-btn.sel{border-color:#5B5FFF;background:#EEEEFF;color:#5B5FFF}"
    ".fn-btn-ico{font-size:20px;flex-shrink:0}"
    ".cap-btn{width:100%;padding:16px;border-radius:16px;border:none;background:#5B5FFF;color:#fff;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;margin-top:4px}"
    ".ring2{width:80px;height:80px;border-radius:50%;border:3px solid #E8EAF2;margin:0 auto 12px;display:flex;align-items:center;justify-content:center;font-size:20px;font-weight:900;font-family:Nunito,sans-serif}"
    ".ring2.wait{border-color:#5B5FFF;animation:ring-pulse 1.2s infinite}"
    ".ring2.done{border-color:#00C48C;background:#E6FBF5;color:#00C48C}"
    ".cap-msg{text-align:center;font-size:14px;font-weight:700;color:#8B8FA8;margin-bottom:16px}"
    ".save-row{display:flex;gap:10px;margin-top:8px}"
    ".save-btn{flex:1;padding:15px;border-radius:14px;border:none;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}"
    ".save-ok{background:#00C48C;color:#fff}"
    ".save-cancel{background:#F2F3F7;color:#8B8FA8}"
    ".code-btn{width:100%;padding:14px;border-radius:14px;border:2px solid #5B5FFF;background:#EEEEFF;color:#5B5FFF;font-size:14px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif;margin-top:8px}"
    ".list-item{display:flex;align-items:center;gap:10px;padding:12px 14px;background:#fff;border-radius:14px;margin-bottom:8px;box-shadow:0 2px 10px rgba(0,0,0,.05)}"
    ".list-info{flex:1}"
    ".list-name{font-size:14px;font-weight:900}"
    ".list-key{font-size:11px;color:#8B8FA8;font-weight:700;margin-top:2px;font-family:monospace}"
    ".list-actions{display:flex;gap:6px;flex-shrink:0}"
    ".del-btn{padding:8px 12px;border-radius:10px;border:none;background:#FFE5E5;color:#FF4D4D;font-size:12px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}"
    ".view-btn{padding:8px 12px;border-radius:10px;border:none;background:#EEEEFF;color:#5B5FFF;font-size:12px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif}"
    "</style></head><body>"));

  server.sendContent(F("<div class='hdr'><div><h1>➕ Cadastrar Sinal</h1><p>Aponte o controle e capture o sinal</p></div></div>"));

  String roomsJs = "[";
  for(int i=0;i<3;i++){
    String slug="";
    for(char c:rooms[i].name){if(isAlphaNumeric(c))slug+=(char)tolower(c);else if(c==' '&&slug.length()>0)slug+='_';}
    if(i>0) roomsJs += ",";
    roomsJs += "{name:'"+rooms[i].name+"',slug:'"+slug+"'}";
  }
  roomsJs += "]";

  server.sendContent("<div class='cad-wrap'><div class='cad-card'>"
    "<div class='field'><label class='lbl'>🏠 Cômodo</label>"
    "<select id='inp-room' onchange='updateBrand()'>");
  for(int i=0;i<3;i++) server.sendContent("<option value='"+String(i)+"'"+(i==preRoom?" selected":"")+">"+rooms[i].name+"</option>");
  server.sendContent(F("</select></div>"
    "<div class='field'><label class='lbl'>📡 Tipo</label>"
    "<div class='dev-grid'>"
    "<button class='dev-btn sel' data-dev='vent' onclick='selDev(this)'><span class='dev-btn-ico'>🌀</span>Ventilador</button>"
    "<button class='dev-btn' data-dev='ac' onclick='selDev(this)'><span class='dev-btn-ico'>❄️</span>Ar Cond.</button>"
    "<button class='dev-btn' data-dev='tv' onclick='selDev(this)'><span class='dev-btn-ico'>📺</span>TV</button>"
    "<button class='dev-btn' data-dev='luz' onclick='selDev(this)'><span class='dev-btn-ico'>💡</span>Luz</button>"
    "<button class='dev-btn' data-dev='projetor' onclick='selDev(this)'><span class='dev-btn-ico'>🎥</span>Projetor</button>"
    "<button class='dev-btn' data-dev='outro' onclick='selDev(this)'><span class='dev-btn-ico'>🔌</span>Outro</button>"
    "</div></div>"
    "<div class='field'><label class='lbl'>▶ Função</label><div class='fn-grid' id='fn-grid'></div></div>"
    "<div class='field'><label class='lbl'>🏷️ Nome</label>"
    "<input type='text' id='inp-label' placeholder='Ex: Ventilador Ligar' autocomplete='off'></div>"
    "<button class='cap-btn' id='btn-start' onclick='startCap()'>📡 Iniciar Captura</button>"
    "<div id='ir-block' style='display:none;padding-top:20px'>"
    "<div class='ring2 wait' id='ring2'>1/3</div>"
    "<div class='cap-msg' id='cap-msg'>Captura 1/3 — aponte o controle e pressione o botão...</div>"
    "<div id='save-row' style='display:none'>"
    "<div class='save-row'>"
    "<button class='save-btn save-ok' onclick='saveCap()'>💾 Salvar</button>"
    "<button class='save-btn save-cancel' onclick='cancelCap()'>Cancelar</button>"
    "</div>"
    "<button class='code-btn' onclick='verCodigo()'>🔍 Ver Código Capturado</button>"
    "</div>"
    "<button class='save-btn save-cancel' id='btn-cancel' onclick='cancelCap()' style='width:100%;margin-top:8px'>Cancelar captura</button>"
    "</div>"));

  server.sendContent(codePanelHTML());

  server.sendContent(F("</div><div class='cad-card'>"
    "<div style='font-size:13px;font-weight:900;color:#8B8FA8;letter-spacing:1px;text-transform:uppercase;margin-bottom:12px'>📋 Cadastrados</div>"
    "<div id='dev-list'><div style='text-align:center;color:#8B8FA8;padding:20px;font-weight:700'>Carregando...</div></div>"
    "</div></div>"));

  server.sendContent(F("<div class='bnav'>"
    "<a class='bnav-btn' href='/'><span class='bnav-icon'>🎮</span>Controle</a>"
    "<button class='bnav-btn active'><span class='bnav-icon'>➕</span>Cadastrar</button>"
    "<a class='bnav-btn' href='/codigos'><span class='bnav-icon'>📋</span>Códigos</a>"
    "<a class='bnav-btn' href='/config'><span class='bnav-icon'>⚙️</span>Config</a>"
    "</div><div id='toast'></div>"));

  server.sendContent("<script>");
  server.sendContent_P(CODE_VIEWER_JS);
  server.sendContent("var pollT=null;");
  server.sendContent("function getIcon(brand,apiIcon){if(apiIcon&&apiIcon.length>0)return apiIcon;brand=brand||'';"
    "if(brand.indexOf('vent')===0)return'🌀';if(brand.indexOf('ac')===0)return'❄️';"
    "if(brand.indexOf('tv')===0)return'📺';if(brand.indexOf('luz')===0)return'💡';"
    "if(brand.indexOf('projetor')===0)return'🎥';return'🔌';}");
  server.sendContent("var ROOMS="+roomsJs+";");
  server.sendContent(F(
    "var curDev='vent',curCmd='';"
    "var DEVFNS={"
    "vent:[{ico:'⏻',label:'Liga/Desliga',cmd:'power'},{ico:'⬆️',label:'Vel +',cmd:'speed_up'},{ico:'⬇️',label:'Vel -',cmd:'speed_down'},{ico:'🔄',label:'Reverso',cmd:'reverse'},{ico:'🌀',label:'Oscilar',cmd:'swing'},{ico:'😴',label:'Sleep',cmd:'sleep'},{ico:'⏱️',label:'Timer',cmd:'timer'},{ico:'💡',label:'Luz',cmd:'light'}],"
    "ac:[{ico:'⚡',label:'Ligar',cmd:'power_on'},{ico:'⏻',label:'Desligar',cmd:'power_off'},{ico:'⬆️',label:'Temp +',cmd:'temp_up'},{ico:'⬇️',label:'Temp -',cmd:'temp_down'},{ico:'🌀',label:'Oscilar',cmd:'swing'},{ico:'😴',label:'Sleep',cmd:'sleep'},{ico:'⏱️',label:'Timer',cmd:'timer'},{ico:'❄️',label:'Frio',cmd:'mode_cool'},{ico:'🔥',label:'Calor',cmd:'mode_heat'},{ico:'💧',label:'Seco',cmd:'mode_dry'}],"
    "tv:[{ico:'⏻',label:'Power',cmd:'power'},{ico:'🔈',label:'Vol -',cmd:'voldown'},{ico:'🔊',label:'Vol +',cmd:'volup'},{ico:'📻',label:'CH -',cmd:'chdown'},{ico:'📻',label:'CH +',cmd:'chup'},{ico:'🔇',label:'Mudo',cmd:'mute'},{ico:'🔄',label:'Fonte',cmd:'source'},{ico:'☰',label:'Menu',cmd:'menu'}],"
    "luz:[{ico:'💡',label:'Liga/Desliga',cmd:'power'},{ico:'🔆',label:'Brilho +',cmd:'bright_up'},{ico:'🔅',label:'Brilho -',cmd:'bright_down'},{ico:'🌡️',label:'Quente',cmd:'warm'},{ico:'❄️',label:'Frio',cmd:'cool'},{ico:'🌈',label:'Colorido',cmd:'color'}],"
    "projetor:[{ico:'⏻',label:'Power',cmd:'power'},{ico:'🔈',label:'Vol -',cmd:'voldown'},{ico:'🔊',label:'Vol +',cmd:'volup'},{ico:'🔄',label:'Fonte',cmd:'source'},{ico:'☰',label:'Menu',cmd:'menu'},{ico:'🔇',label:'Mudo',cmd:'mute'}],"
    "outro:[{ico:'⏻',label:'Liga/Desliga',cmd:'power'},{ico:'⬆️',label:'Função +',cmd:'func_up'},{ico:'⬇️',label:'Função -',cmd:'func_down'},{ico:'⏱️',label:'Timer',cmd:'timer'},{ico:'😴',label:'Sleep',cmd:'sleep'},{ico:'🔌',label:'Custom',cmd:'_custom'}]};"
    "function genBrand(){var room=parseInt(document.getElementById('inp-room').value);return curDev+'_'+ROOMS[room].slug;}"
    "function updateBrand(){}"
    "function selDev(btn){document.querySelectorAll('.dev-btn').forEach(function(b){b.classList.remove('sel');});btn.classList.add('sel');curDev=btn.dataset.dev;curCmd='';renderFns();}"
    "function renderFns(){var fns=DEVFNS[curDev]||[];var g=document.getElementById('fn-grid');var html='';for(var i=0;i<fns.length;i++){var f=fns[i];html+='<button class=\"fn-btn\" data-cmd=\"'+f.cmd+'\" onclick=\"selFn(this)\"><span class=\"fn-btn-ico\">'+f.ico+'</span><span>'+f.label+'</span></button>';}g.innerHTML=html;}"
    "function selFn(btn){document.querySelectorAll('.fn-btn').forEach(function(b){b.classList.remove('sel');});btn.classList.add('sel');curCmd=btn.dataset.cmd;"
    "if(curCmd==='_custom'){var c=prompt('Nome do comando:');if(!c){curCmd='';return;}curCmd=c.trim().toLowerCase().replace(/\\s+/g,'_').replace(/[^a-z0-9_]/g,'');btn.querySelector('span:last-child').textContent=curCmd;}"
    "var lbl=document.getElementById('inp-label');if(lbl.value===''||lbl._auto){lbl.value=btn.querySelector('span:last-child').textContent;lbl._auto=true;}}"
    "document.getElementById('inp-label').addEventListener('input',function(){this._auto=false;});"
    // startCap — atualizado para 3 capturas
    "function startCap(){"
    "if(!curCmd){toast('Escolha uma função primeiro','err');return;}"
    "var brand=genBrand(),cmd=curCmd,label=document.getElementById('inp-label').value.trim()||cmd;"
    "var room=parseInt(document.getElementById('inp-room').value);"
    "fetch('/api/learn/start?brand='+encodeURIComponent(brand)+'&cmd='+encodeURIComponent(cmd)+'&label='+encodeURIComponent(label)+'&room='+room,{method:'POST'})"
    ".then(function(){"
    "document.getElementById('ir-block').style.display='block';"
    "document.getElementById('btn-start').style.display='none';"
    "document.getElementById('save-row').style.display='none';"
    "document.getElementById('code-panel').style.display='none';"
    "document.getElementById('btn-cancel').style.display='block';"
    "document.getElementById('ring2').className='ring2 wait';"
    "document.getElementById('ring2').textContent='1/3';"
    "document.getElementById('cap-msg').textContent='Captura 1/3 — aponte o controle e pressione o botão...';"
    "if(pollT){clearInterval(pollT);pollT=null;}"
    "pollT=setInterval(pollCap,600);});}"
    // pollCap — atualizado para mostrar progresso das 3 capturas
    "function pollCap(){fetch('/api/learn/poll').then(function(r){return r.json();}).then(function(d){"
    "var atual=d.capturaAtual||0;"
    "if(!d.capturado){"
    "document.getElementById('ring2').textContent=(atual+1)+'/3';"
    "document.getElementById('cap-msg').textContent='Captura '+(atual+1)+'/3 — aponte e pressione o botão...';"
    "}else{"
    "clearInterval(pollT);pollT=null;"
    "document.getElementById('ring2').className='ring2 done';"
    "document.getElementById('ring2').textContent='✅';"
    "document.getElementById('cap-msg').textContent='Melhor sinal selecionado! ('+d.len+' amostras)';"
    "document.getElementById('save-row').style.display='block';"
    "document.getElementById('btn-cancel').style.display='none';"
    "}});}"
    "function verCodigo(){"
    "fetch('/api/learn/raw').then(function(r){return r.json();})"
    ".then(function(d){if(!d.ok){toast('Nada capturado','err');return;}showCode(d);})"
    ".catch(function(){toast('Erro','err');});}"
    "function verCodigoSalvo(brand,cmd){"
    "fetch('/api/signal/raw/bydev?brand='+encodeURIComponent(brand)+'&cmd='+encodeURIComponent(cmd))"
    ".then(function(r){return r.json();})"
    ".then(function(d){if(!d.ok){toast('Sinal não encontrado','err');return;}showCode(d);"
    "document.getElementById('code-panel').scrollIntoView({behavior:'smooth'});})"
    ".catch(function(){toast('Erro ao carregar','err');});}"
    "function saveCap(){fetch('/api/learn/save',{method:'POST'}).then(function(r){return r.json();}).then(function(d){"
    "if(d.ok){toast('Salvo: '+d.brand+'/'+d.cmd,'ok');"
    "document.getElementById('ir-block').style.display='none';"
    "document.getElementById('btn-start').style.display='block';"
    "loadDevList();"
    "}else{toast('Erro: '+d.msg,'err');}});}"
    "function cancelCap(){if(pollT){clearInterval(pollT);pollT=null;}fetch('/api/learn/stop',{method:'POST'});resetForm();}"
    "function resetForm(){"
    "document.getElementById('ir-block').style.display='none';"
    "document.getElementById('btn-start').style.display='block';"
    "document.getElementById('code-panel').style.display='none';}"
    "function loadDevList(){"
    "var el=document.getElementById('dev-list');if(!el)return;"
    "el.innerHTML='<div style=\"text-align:center;color:#8B8FA8;padding:20px;font-weight:700\">Carregando...</div>';"
    "fetch('/api/devices/all').then(function(res){return res.json();})"
    ".then(function(list){"
    "if(!list||!list.length){el.innerHTML='<div style=\"text-align:center;color:#8B8FA8;padding:20px;font-weight:700\">Nenhum dispositivo cadastrado.</div>';return;}"
    "var html='';"
    "for(var i=0;i<list.length;i++){var d=list[i];var ico=getIcon(d.brand,d.icon);var lbl=d.label.replace(/\"/g,'&quot;');"
    "html+='<div class=\"list-item\">'+"
    "'<span style=\"font-size:22px;flex-shrink:0\">'+ico+'</span>'+"
    "'<div class=\"list-info\"><div class=\"list-name\">'+d.label+'</div>'+"
    "'<div class=\"list-key\">'+d.brand+'/'+d.cmd+' · '+d.roomName+'</div></div>'+"
    "'<div class=\"list-actions\">'+"
    "'<button class=\"view-btn\" style=\"background:#FFF5E6;color:#FF9500\" data-idx=\"'+d.idx+'\" data-label=\"'+lbl+'\" data-ico=\"'+ico+'\" onclick=\"openEdit(parseInt(this.dataset.idx),this.dataset.label,this.dataset.ico)\">✏️</button>'+"
    "'<button class=\"view-btn\" data-brand=\"'+d.brand+'\" data-cmd=\"'+d.cmd+'\" onclick=\"verCodigoSalvo(this.dataset.brand,this.dataset.cmd)\">🔍</button>'+"
    "'<button class=\"del-btn\" data-idx=\"'+d.idx+'\" onclick=\"delDev(this)\">🗑️</button>'+"
    "'</div></div>';}"
    "el.innerHTML=html;"
    "}).catch(function(e){el.innerHTML='<div style=\"text-align:center;color:#FF4D4D;padding:20px;font-weight:700\">Erro ao carregar</div>';});}"
    "var editIdx=-1;"
    "function openEdit(idx,label,icon){"
    "editIdx=idx;document.getElementById('edit-label').value=label;"
    "document.querySelectorAll('.edit-ico-btn').forEach(function(b){b.classList.toggle('sel',b.dataset.ico===icon);});"
    "document.getElementById('edit-modal').style.display='flex';}"
    "function closeEdit(){document.getElementById('edit-modal').style.display='none';editIdx=-1;}"
    "function selEditIcon(btn){document.querySelectorAll('.edit-ico-btn').forEach(function(b){b.classList.remove('sel');});btn.classList.add('sel');}"
    "function saveEdit(){var label=document.getElementById('edit-label').value.trim();"
    "if(!label){toast('Digite um nome','err');return;}"
    "var selBtn=document.querySelector('.edit-ico-btn.sel');var icon=selBtn?selBtn.dataset.ico:'';"
    "fetch('/api/devices/edit?idx='+editIdx+'&label='+encodeURIComponent(label)+'&icon='+encodeURIComponent(icon),{method:'POST'})"
    ".then(function(r){return r.json();}).then(function(d){if(d.ok){toast('Atualizado!','ok');closeEdit();loadDevList();}else toast('Erro','err');});}"
    "function delDev(btn){var idx=btn.getAttribute('data-idx');if(!confirm('Apagar?'))return;"
    "fetch('/api/devices?idx='+idx,{method:'DELETE'}).then(function(r){return r.json();}).then(function(d){toast(d.ok?'Removido!':'Erro','ok');loadDevList();});}"
    "function toast(m,t){var el=document.getElementById('toast');el.textContent=m;el.className='show '+(t||'');clearTimeout(el._t);el._t=setTimeout(function(){el.className='';},2500);}"
    "renderFns();loadDevList();"));

  server.sendContent("</script>");

  server.sendContent(F(
    "<div id='edit-modal' style='display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);z-index:400;align-items:flex-end;justify-content:center'>"
    "<div style='background:#fff;border-radius:28px 28px 0 0;padding:28px 20px 40px;width:100%;max-width:430px'>"
    "<div style='font-size:18px;font-weight:900;margin-bottom:4px'>✏️ Editar Botão</div>"
    "<div style='font-size:13px;color:#8B8FA8;font-weight:700;margin-bottom:20px'>Renomear e trocar ícone</div>"
    "<div style='margin-bottom:16px'><label style='font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px;display:block'>Nome</label>"
    "<input id='edit-label' type='text' style='width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:12px;color:#1A1D2E;padding:12px 14px;font-size:15px;font-family:Nunito,sans-serif;font-weight:700;outline:none'></div>"
    "<div style='margin-bottom:20px'><label style='font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px;display:block'>Ícone</label>"
    "<div style='display:grid;grid-template-columns:repeat(5,1fr);gap:8px'>"));

  const char* icones[] = {"🌀","❄️","📺","💡","🎥","🔌","⏻","🔥","💧","⚡","😴","⏱️","🔇","🎛️","🌡️"};
  for(int i=0;i<15;i++){
    server.sendContent("<button class='edit-ico-btn' data-ico='"+String(icones[i])+"' onclick='selEditIcon(this)' "
      "style='padding:10px;border-radius:10px;border:2px solid #E8EAF2;background:#F2F3F7;font-size:22px;cursor:pointer'>"+String(icones[i])+"</button>");
  }

  server.sendContent(F("</div></div>"
    "<div style='display:flex;gap:10px'>"
    "<button onclick='saveEdit()' style='flex:1;padding:15px;border-radius:14px;border:none;background:#5B5FFF;color:#fff;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif'>💾 Salvar</button>"
    "<button onclick='closeEdit()' style='flex:1;padding:15px;border-radius:14px;border:none;background:#F2F3F7;color:#8B8FA8;font-size:15px;font-weight:900;cursor:pointer;font-family:Nunito,sans-serif'>Cancelar</button>"
    "</div></div></div>"
    "<style>.edit-ico-btn.sel{border-color:#5B5FFF!important;background:#EEEEFF!important;}</style>"
    "</body></html>"));

  server.sendContent("");
}

// ============================================================
//  TELA CONFIG
// ============================================================
void handleConfigPage(){
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(F("<!DOCTYPE html><html lang='pt-BR'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Config</title>"
    "<link href='https://fonts.googleapis.com/css2?family=Nunito:wght@700;800;900&display=swap' rel='stylesheet'>"
    "<style>"));
  server.sendContent_P(CSS);
  server.sendContent(F(
    ".cfg-wrap{padding:16px 16px 100px}"
    ".card{background:#fff;border-radius:20px;padding:20px;margin-bottom:16px;box-shadow:0 4px 24px rgba(0,0,0,.06)}"
    ".rtitle{font-size:17px;font-weight:900;padding-bottom:14px;margin-bottom:14px;border-bottom:2px solid #F2F3F7}"
    ".field{margin-bottom:16px}"
    ".lbl{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1.2px;margin-bottom:6px;display:block}"
    "input[type=text]{width:100%;background:#F2F3F7;border:2px solid #E8EAF2;border-radius:12px;color:#1A1D2E;padding:12px 14px;font-size:15px;font-family:Nunito,sans-serif;font-weight:700;outline:none}"
    ".savebtn{display:block;width:100%;padding:18px;border-radius:16px;border:none;background:#5B5FFF;color:#fff;font-size:16px;font-weight:900;font-family:Nunito,sans-serif;cursor:pointer;margin-bottom:12px}"
    ".ip-badge{background:#E6FBF5;border-radius:12px;padding:12px 16px;display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}"
    ".ip-label{font-size:11px;font-weight:900;color:#8B8FA8;text-transform:uppercase;letter-spacing:1px}"
    ".ip-val{font-size:15px;font-weight:900;color:#00C48C;font-family:monospace}"
    ".ap-badge{background:#EEEEFF;border-radius:12px;padding:12px 16px;display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}"
    ".net-count{background:#FFF5E6;border-radius:12px;padding:12px 16px;display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}"
    "</style></head><body>"));

  String ipSTA = WiFi.localIP().toString();
  String curSSID = WiFi.SSID();
  bool staOk = WiFi.status() == WL_CONNECTED;
  Preferences p; p.begin(WIFI_NAMESPACE, true); int netCount = p.getInt("count", 0); p.end();

  server.sendContent(F("<div class='hdr'><div><h1>⚙️ Configurar</h1><p>Cômodos e marcas</p></div></div>"));
  server.sendContent("<div class='cfg-wrap'>");
  server.sendContent(F("<div class='card'><div class='rtitle'>📶 Wi-Fi</div>"));
  server.sendContent(F("<div class='ap-badge'><div><div class='ip-label'>AP fixo (sempre ativo)</div>"
    "<div style='font-size:13px;font-weight:900;color:#5B5FFF'>IR-Remote · 192.168.4.1</div></div><span style='font-size:20px'>📡</span></div>"));
  if (staOk) {
    server.sendContent("<div class='ip-badge'><div><div class='ip-label'>Rede local conectada</div>"
      "<div class='ip-val' style='color:#5B5FFF'>" + curSSID + " · " + ipSTA + "</div></div></div>");
    server.sendContent(F("<div class='ip-badge'><div><div class='ip-label'>mDNS</div><div class='ip-val' style='color:#5B5FFF'>http://ir-remote.local</div></div></div>"));
  } else {
    server.sendContent(F("<div style='background:#FFF5E6;border-radius:12px;padding:12px 16px;margin-bottom:12px;font-size:13px;font-weight:700;color:#996600'>⚠️ Sem rede local. Usando só o AP.</div>"));
  }
  server.sendContent("<div class='net-count'><div><div class='ip-label'>Redes salvas</div>"
    "<div style='font-size:15px;font-weight:900;color:#FF9500'>" + String(netCount) + " / " + String(MAX_NETWORKS) + "</div></div>"
    "<a href='/redes' style='padding:10px 16px;border-radius:12px;background:#FF9500;color:#fff;font-size:13px;font-weight:900;text-decoration:none'>Gerenciar</a></div>");
  server.sendContent(F("</div>"));

  server.sendContent(F("<form action='/save' method='GET'>"));
  for(int i=0;i<3;i++){
    server.sendContent("<div class='card'><div class='rtitle'>🏠 "+rooms[i].name+"</div>");
    server.sendContent("<div class='field'><label class='lbl'>Nome</label><input type='text' name='n"+String(i)+"' value='"+rooms[i].name+"' maxlength='20'></div>");
    server.sendContent("<div class='field'><label class='lbl'>❄️ Marca do AC</label>"+makeSelect("a"+String(i),AC_BRANDS,14,rooms[i].acBrand)+"</div>");
    server.sendContent("<div class='field'><label class='lbl'>📺 Marca da TV</label>"+makeSelect("t"+String(i),TV_BRANDS,12,rooms[i].tvBrand)+"</div>");
    server.sendContent(F("</div>"));
  }
  server.sendContent(F("<button type='submit' class='savebtn'>💾 Salvar</button></form></div>"));
  server.sendContent(F("<div class='bnav'>"
    "<a class='bnav-btn' href='/'><span class='bnav-icon'>🎮</span>Controle</a>"
    "<a class='bnav-btn' href='/cadastro'><span class='bnav-icon'>➕</span>Cadastrar</a>"
    "<a class='bnav-btn' href='/codigos'><span class='bnav-icon'>📋</span>Códigos</a>"
    "<button class='bnav-btn active'><span class='bnav-icon'>⚙️</span>Config</button>"
    "</div></body></html>"));
  server.sendContent("");
}

// ============================================================
//  SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  Serial.println("\n=== CONTROLE UNIVERSAL IR v" FW_VERSION " ===");
  loadRooms();
  irsend.begin();
  irrecv.enableIRIn();

  acMidea.begin();
  acSamsung.begin(); acLG.begin(); acDaikin.begin();
  acFujitsu.begin(); acMitsubishi.begin(); acPanasonic.begin();
  acCarrier.begin(); acWhirlpool.begin(); acGree.begin(); acHitachi.begin();

  WiFi.mode(WIFI_AP_STA);
  startAP();
  tryKnownNetworks();

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("ir-remote")) Serial.println("[mDNS] http://ir-remote.local");
  }

  ArduinoOTA.setHostname("IR-Remote");
  ArduinoOTA.setPassword("ota1234");
  ArduinoOTA.onStart([](){Serial.println("[OTA] Iniciando...");});
  ArduinoOTA.onEnd([](){Serial.println("[OTA] Concluído!");});
  ArduinoOTA.onProgress([](unsigned int p,unsigned int t){Serial.printf("[OTA] %u%%\r",p*100/t);});
  ArduinoOTA.onError([](ota_error_t e){Serial.printf("[OTA] Erro[%u]\n",e);});
  ArduinoOTA.begin();

  server.on("/",                      HTTP_GET,    handleRoot);
  server.on("/tv",                    HTTP_GET,    handleTV);
  server.on("/ac",                    HTTP_GET,    handleAC);
  server.on("/cadastro",              HTTP_GET,    handleCadastro);
  server.on("/codigos",               HTTP_GET,    handleCodigos);
  server.on("/config",                HTTP_GET,    handleConfigPage);
  server.on("/save",                  HTTP_GET,    handleSave);
  server.on("/redes",                 HTTP_GET,    handleRedesPage);
  server.on("/api/wifi/list",         HTTP_GET,    apiWiFiList);
  server.on("/api/wifi/delete",       HTTP_DELETE, apiWiFiDelete);
  server.on("/api/wifi/add",          HTTP_POST,   apiWiFiAdd);
  server.on("/api/learn/start",       HTTP_POST,   apiLearnStart);
  server.on("/api/learn/stop",        HTTP_POST,   apiLearnStop);
  server.on("/api/learn/poll",        HTTP_GET,    apiLearnPoll);
  server.on("/api/learn/raw",         HTTP_GET,    apiLearnRaw);
  server.on("/api/learn/save",        HTTP_POST,   apiLearnSave);
  server.on("/api/signal/raw",        HTTP_GET,    apiSignalRaw);
  server.on("/api/signal/raw/bydev",  HTTP_GET,    apiSignalRawByDev);
  server.on("/api/raw",               HTTP_DELETE, apiDeleteRaw);
  server.on("/api/devices/all",       HTTP_GET,    apiDevicesAll);
  server.on("/api/devices/edit",      HTTP_POST,   apiEditDevice);
  server.on("/ota",                   HTTP_GET,    handleOTAPage);
  server.on("/ota/upload",            HTTP_POST,   handleOTAResult, handleOTAUpload);

  server.on("/generic", HTTP_GET, [](){
    String brand=server.arg("brand"), cmd=server.arg("cmd");
    if(!brand.length()||!cmd.length()){server.send(400,"application/json","{\"ok\":false,\"msg\":\"brand e cmd obrigatórios\"}");return;}
    String res=doSend(brand,cmd);
    if(res=="UNKNOWN")
      server.send(200,"application/json","{\"ok\":false,\"status\":\"UNKNOWN\",\"brand\":\""+brand+"\",\"cmd\":\""+cmd+"\"}");
    else
      server.send(200,"application/json","{\"ok\":true,\"status\":\""+res+"\"}");
  });

  server.on("/api/devices", HTTP_ANY, [](){
    if(server.method()==HTTP_DELETE) apiDeleteDevice();
    else apiDevices();
  });

  server.on("/favicon.ico", HTTP_GET, [](){ server.send(204); });
  server.onNotFound([](){
    server.sendHeader("Location","/",true);
    server.send(302,"text/plain","");
  });

  server.begin();

  Serial.println("========================================");
  Serial.println("  AP fixo:  IR-Remote / 12345678");
  Serial.println("  AP IP:    http://192.168.4.1");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("  STA IP:   http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("  mDNS:     http://ir-remote.local");
  } else {
    Serial.println("  STA:      sem rede local");
  }
  Serial.println("  OTA:      ArduinoOTA ativo");
  Serial.println("  RAM buf:  3x1200x2 = 7.2KB para captura");
  Serial.println("========================================");
}

// ============================================================
//  LOOP
// ============================================================
void loop(){
  ArduinoOTA.handle();
  server.handleClient();
  processCapture();
}
