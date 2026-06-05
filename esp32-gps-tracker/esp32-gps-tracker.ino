// ============================================================
// ESP32 GPS 轨迹记录仪 — ATGM336H + MQTT
// GPS: ATGM336H 接 Serial2 (RX=GPIO16, TX=GPIO17), 9600 baud
// MQTT: broker.emqx.io → map.html 实时显示
// ============================================================

// ==================== 配置 ====================
// 手机热点（改成你自己的）
const char* WIFI_SSID = "WIFI";
const char* WIFI_PASSWORD = "999999999";

// AP 模式（WiFi 连不上时自动创建）
const char* AP_SSID = "ESP32_GPS";
const char* AP_PASSWORD = "12345678";

// MQTT
const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_GPS = "esp32/gps";
const char* MQTT_TOPIC_STATUS = "esp32/status";
const char* MQTT_CLIENT_ID = "esp32-gps-tracker";

// GPS 引脚
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// 记录间隔
const unsigned long RECORD_INTERVAL = 5000;
const unsigned long IDLE_INTERVAL = 30000;
const float IDLE_SPEED = 2.0;

// ==================== 库 ====================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TinyGPS++.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// ==================== 全局变量 ====================
TinyGPSPlus gps;
AsyncWebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// GPS 数据
float currentLat = 0, currentLng = 0;
float currentAlt = 0, currentSpeed = 0;
int currentSats = 0;
bool hasFix = false;

// 轨迹记录
bool isRecording = false;
File trackFile;
String currentTrackName = "";
unsigned long lastRecordTime = 0;
int trackPointCount = 0;

// 定时器
unsigned long lastMqttReconnect = 0;
unsigned long lastStatusPublish = 0;

// ==================== SPIFFS ====================
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS 挂载失败");
  } else {
    Serial.printf("SPIFFS 可用: %d KB\n", SPIFFS.totalBytes() / 1024);
  }
}

// ==================== GPS ====================
void readGPS() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
        hasFix = true;
      }
      if (gps.altitude.isValid()) currentAlt = gps.altitude.meters();
      if (gps.speed.isValid()) currentSpeed = gps.speed.kmph();
      if (gps.satellites.isValid()) currentSats = gps.satellites.value();
    }
  }
}

// ==================== MQTT ====================
void mqttReconnect() {
  if (mqtt.connected()) return;
  if (millis() - lastMqttReconnect < 5000) return;
  lastMqttReconnect = millis();

  Serial.print("MQTT 连接...");
  if (mqtt.connect(MQTT_CLIENT_ID)) {
    Serial.println(" 成功");
  } else {
    Serial.printf(" 失败, rc=%d\n", mqtt.state());
  }
}

void publishGPS() {
  if (!mqtt.connected()) return;
  if (!hasFix) return;

  String json = "{";
  json += "\"lat\":" + String(currentLat, 6) + ",";
  json += "\"lng\":" + String(currentLng, 6) + ",";
  json += "\"alt\":" + String(currentAlt, 1) + ",";
  json += "\"spd\":" + String(currentSpeed, 1) + ",";
  json += "\"sat\":" + String(currentSats) + ",";
  json += "\"fix\":1";
  json += "}";
  mqtt.publish(MQTT_TOPIC_GPS, json.c_str());
}

void publishStatus() {
  if (!mqtt.connected()) return;
  if (millis() - lastStatusPublish < 30000) return;
  lastStatusPublish = millis();

  String json = "{";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
  json += "\"points\":" + String(trackPointCount);
  json += "}";
  mqtt.publish(MQTT_TOPIC_STATUS, json.c_str());
}

// ==================== WiFi ====================
void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("连接 WiFi: %s", WIFI_SSID);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi 已连接, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi 连接失败，仅开启 AP 模式");
    WiFi.disconnect();
  }

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP: %s, IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ==================== 轨迹记录 ====================
String makeTrackFilename() {
  time_t now = time(nullptr);
  if (now < 100000) return "/track_" + String(millis()) + ".json";
  struct tm* t = localtime(&now);
  char buf[32];
  sprintf(buf, "/track_%04d%02d%02d_%02d%02d.json",
    t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min);
  return String(buf);
}

void startRecording() {
  if (isRecording) return;
  currentTrackName = makeTrackFilename();
  trackFile = SPIFFS.open(currentTrackName, "w");
  if (!trackFile) { Serial.println("创建文件失败"); return; }
  isRecording = true;
  trackPointCount = 0;
  lastRecordTime = 0;
  trackFile.println("[");
  Serial.printf("开始记录: %s\n", currentTrackName.c_str());
}

void stopRecording() {
  if (!isRecording) return;
  trackFile.println("\n]");
  long size = trackFile.position();
  trackFile.close();
  isRecording = false;
  Serial.printf("停止记录: %s, %d 点, %ld 字节\n", currentTrackName.c_str(), trackPointCount, size);
}

void recordPoint() {
  if (!isRecording || !hasFix) return;
  unsigned long interval = (currentSpeed < IDLE_SPEED) ? IDLE_INTERVAL : RECORD_INTERVAL;
  if (millis() - lastRecordTime < interval) return;
  lastRecordTime = millis();

  String pt = "{\"la\":";
  pt += String(currentLat, 6) + ",\"lo\":" + String(currentLng, 6);
  pt += ",\"el\":" + String(currentAlt, 1);
  pt += ",\"sp\":" + String(currentSpeed, 1);
  pt += ",\"ts\":" + String(millis()) + "},";
  trackFile.println(pt);
  trackPointCount++;
  if (trackPointCount % 20 == 0) trackFile.flush();
}

// ==================== Web ====================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>ESP32 GPS</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,sans-serif;background:#0b0e14;color:#e2e4e9;min-height:100vh;padding:16px}
.panel{background:#141820;border:1px solid #252a35;border-radius:16px;padding:16px;margin-bottom:12px;max-width:420px;margin:0 auto 12px}
h1{font-size:1.2rem;text-align:center;margin-bottom:12px;color:#4f8fff}
.gps-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.gps-item{text-align:center;background:rgba(255,255,255,.03);border-radius:10px;padding:10px}
.gps-item .v{font-size:1.6rem;font-weight:700;color:#4f8fff}
.gps-item .l{font-size:.65rem;color:#5a5e6a;margin-top:2px}
.btn{display:block;width:100%;padding:14px;border:none;border-radius:12px;font-size:1rem;font-weight:600;cursor:pointer;margin-top:8px}
.btn.start{background:#ff4757;color:#fff}.btn.stop{background:#ff7b42;color:#fff}
.link{text-align:center;margin-top:12px;font-size:.75rem;color:#5a5e6a}
.link a{color:#4f8fff}
</style>
</head>
<body>
<div class="panel">
<h1>ESP32 GPS 轨迹记录</h1>
<div class="gps-grid">
<div class="gps-item"><div class="v" id="lat">--</div><div class="l">纬度</div></div>
<div class="gps-item"><div class="v" id="lng">--</div><div class="l">经度</div></div>
<div class="gps-item"><div class="v" id="spd">--</div><div class="l">速度 km/h</div></div>
<div class="gps-item"><div class="v" id="sat">--</div><div class="l">卫星数</div></div>
</div>
<div style="text-align:center;margin-top:8px">
<span style="color:#5a5e6a;font-size:.7rem" id="status">等待GPS...</span>
</div>
</div>
<div class="panel">
<button class="btn start" id="recBtn" onclick="toggleRecord()">开始记录</button>
</div>
<div class="link">
使用 <a href="map.html" target="_blank">完整地图页面</a> 查看实时定位
</div>
<script>
function toggleRecord(){
fetch('/api/record?a='+(recording?'stop':'start')).then(()=>{recording=!recording;updateBtn()})
}
function updateBtn(){
const b=document.getElementById('recBtn');
if(recording){b.className='btn stop';b.textContent='停止记录'}
else{b.className='btn start';b.textContent='开始记录'}
}
let recording=false;
setInterval(()=>{
fetch('/api/gps').then(r=>r.json()).then(d=>{
document.getElementById('lat').textContent=d.lat.toFixed(6);
document.getElementById('lng').textContent=d.lng.toFixed(6);
document.getElementById('spd').textContent=d.spd.toFixed(1);
document.getElementById('sat').textContent=d.sats;
document.getElementById('status').textContent=d.fix?'已定位 卫星:'+d.sats:'等待定位...';
recording=d.recording;updateBtn();
})
},1000);
</script>
</body>
</html>
)=====";

void setupRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"fix\":" + String(hasFix ? "true" : "false") + ",";
    json += "\"lat\":" + String(currentLat, 6) + ",";
    json += "\"lng\":" + String(currentLng, 6) + ",";
    json += "\"alt\":" + String(currentAlt, 1) + ",";
    json += "\"spd\":" + String(currentSpeed, 1) + ",";
    json += "\"sats\":" + String(currentSats) + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
    json += "\"points\":" + String(trackPointCount) + ",";
    json += "\"track\":\"" + currentTrackName + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/api/record", HTTP_GET, [](AsyncWebServerRequest *request) {
    String action = request->hasParam("a") ? request->getParam("a")->value() : "";
    if (action == "start") startRecording();
    else if (action == "stop") stopRecording();
    String json = "{\"recording\":" + String(isRecording ? "true" : "false") + ",\"points\":" + String(trackPointCount) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/api/tracks", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";
    File root = SPIFFS.open("/");
    File f = root.openNextFile();
    bool first = true;
    while (f) {
      if (String(f.name()).startsWith("/track_")) {
        if (!first) json += ",";
        json += "{\"name\":\"" + String(f.name()) + "\",\"size\":" + String(f.size()) + "}";
        first = false;
      }
      f = root.openNextFile();
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // GPX 下载
  server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("file")) { request->send(400); return; }
    String filename = request->getParam("file")->value();
    File f = SPIFFS.open(filename, "r");
    if (!f) { request->send(404); return; }
    AsyncResponseStream *res = request->beginResponseStream("application/gpx+xml");
    res->print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.1\" creator=\"ESP32-GPS\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n<trk><name>");
    res->print(filename);
    res->print("</name><trkseg>\n");
    String line = f.readStringUntil('\n');
    while (f.available()) {
      line = f.readStringUntil('\n'); line.trim();
      if (line == "]" || line == "]," || line.length() < 5) continue;
      if (line.endsWith(",")) line = line.substring(0, line.length() - 1);
      float la = 0, lo = 0, el = 0, sp = 0;
      int p1 = line.indexOf("\"la\":"), p2 = line.indexOf(",\"lo\":");
      int p3 = line.indexOf(",\"el\":"), p4 = line.indexOf(",\"sp\":");
      int p5 = line.indexOf(",\"ts\":");
      if (p1 >= 0 && p2 >= 0) {
        la = line.substring(p1 + 5, p2).toFloat(); lo = line.substring(p2 + 6, p3).toFloat();
        el = line.substring(p3 + 6, p4).toFloat(); sp = line.substring(p4 + 6, p5).toFloat();
      }
      res->printf("  <trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele><speed>%.2f</speed></trkpt>\n", la, lo, el, sp);
    }
    res->print("</trkseg></trk>\n</gpx>\n");
    f.close();
    request->send(res);
  });

  server.on("/api/delete", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("file")) { SPIFFS.remove(request->getParam("file")->value()); request->send(200, "text/plain", "deleted"); }
    else request->send(400);
  });
}

// ==================== 主程序 ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 GPS Tracker (ATGM336H) ===");

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.printf("GPS: RX=%d TX=%d, %d baud\n", GPS_RX, GPS_TX, GPS_BAUD);

  initSPIFFS();
  initWiFi();

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(30);

  ArduinoOTA.setHostname("esp32-gps");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.begin();
  Serial.println("OTA 就绪");

  setupRoutes();
  server.begin();
  Serial.println("Web 服务器已启动");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("MQTT: %s:%d, topic: %s\n", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_GPS);
  } else {
    Serial.println("无 WiFi，MQTT 不可用");
  }
  Serial.println("就绪\n");
}

void loop() {
  ArduinoOTA.handle();
  readGPS();

  if (WiFi.status() == WL_CONNECTED) {
    mqttReconnect();
    mqtt.loop();
    publishGPS();
    publishStatus();
  }

  recordPoint();
  delay(50);
}
