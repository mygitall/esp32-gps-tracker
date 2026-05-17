// ============================================================
// ESP32 GPS 轨迹记录仪 — 自行车/电动车
// GPS: NEO-6M 接 Serial2 (RX=GPIO16, TX=GPIO17)
// 手机连 WiFi 热点 "ESP32_GPS" → 打开 192.168.4.1
// ============================================================

// ==================== 配置 ====================
const char* AP_SSID = "ESP32_GPS";
const char* AP_PASSWORD = "12345678";

// GPS 串口引脚
#define GPS_RX 16
#define GPS_TX 17

// 记录间隔 (毫秒)
const unsigned long RECORD_INTERVAL = 5000;      // 运动中每 5 秒记录
const unsigned long IDLE_INTERVAL = 30000;        // 静止时每 30 秒记录
const float IDLE_SPEED = 2.0;                     // < 2 km/h 视为静止

// ==================== 库 ====================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TinyGPS++.h>
#include <SPIFFS.h>

// ==================== 全局变量 ====================
TinyGPSPlus gps;
AsyncWebServer server(80);

// 轨迹记录状态
bool isRecording = false;
File trackFile;
String currentTrackName = "";
unsigned long lastRecordTime = 0;
int trackPointCount = 0;

// 最新 GPS 数据
float currentLat = 0, currentLng = 0;
float currentAlt = 0, currentSpeed = 0;
int currentSats = 0;
bool hasFix = false;

// ==================== SPIFFS 初始化 ====================
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS 挂载失败");
  } else {
    Serial.printf("SPIFFS 可用: %d KB\n", SPIFFS.totalBytes() / 1024);
  }
}

// ==================== GPS 读取 ====================
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
  trackFile.println("[");  // JSON 数组开头
  Serial.printf("开始记录: %s\n", currentTrackName.c_str());
}

void stopRecording() {
  if (!isRecording) return;
  // 去掉最后的逗号并闭合数组
  trackFile.println("\n]");
  long size = trackFile.position();
  trackFile.close();
  isRecording = false;
  Serial.printf("停止记录: %s, %d 点, %ld 字节\n",
    currentTrackName.c_str(), trackPointCount, size);
}

void recordPoint() {
  if (!isRecording || !hasFix) return;

  unsigned long interval = (currentSpeed < IDLE_SPEED) ? IDLE_INTERVAL : RECORD_INTERVAL;
  if (millis() - lastRecordTime < interval) return;
  lastRecordTime = millis();

  // 写 JSON 点（带逗号分隔，方便拼接数组）
  String pt = "{\"la\":";
  pt += String(currentLat, 6) + ",\"lo\":" + String(currentLng, 6);
  pt += ",\"el\":" + String(currentAlt, 1);
  pt += ",\"sp\":" + String(currentSpeed, 1);
  pt += ",\"ts\":" + String(millis()) + "},";
  trackFile.println(pt);
  trackPointCount++;

  // 每 20 个点 flush 一次，掉电保护
  if (trackPointCount % 20 == 0) trackFile.flush();
}

// ==================== GPX 生成 ====================
void sendGPX(AsyncWebServerRequest *request, String filename) {
  File f = SPIFFS.open(filename, "r");
  if (!f) { request->send(404); return; }

  AsyncResponseStream *res = request->beginResponseStream("application/gpx+xml");
  res->print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  res->print("<gpx version=\"1.1\" creator=\"ESP32-GPS\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
  res->printf("  <trk><name>%s</name><trkseg>\n", filename.c_str());

  String line = f.readStringUntil('\n');  // skip opening [
  while (f.available()) {
    line = f.readStringUntil('\n');
    line.trim();
    if (line == "]" || line == "]," || line.length() < 5) continue;
    if (line.endsWith(",")) line = line.substring(0, line.length() - 1);

    // 解析 JSON
    float la = 0, lo = 0, el = 0, sp = 0;
    unsigned long ts = 0;
    int p1 = line.indexOf("\"la\":");
    int p2 = line.indexOf(",\"lo\":");
    int p3 = line.indexOf(",\"el\":");
    int p4 = line.indexOf(",\"sp\":");
    int p5 = line.indexOf(",\"ts\":");
    if (p1 >= 0 && p2 >= 0) {
      la = line.substring(p1 + 5, p2).toFloat();
      lo = line.substring(p2 + 6, p3).toFloat();
      el = line.substring(p3 + 6, p4).toFloat();
      sp = line.substring(p4 + 6, p5).toFloat();
      ts = (unsigned long)line.substring(p5 + 6).toInt();
    }

    // 时间戳转 ISO 8601
    char timeBuf[30];
    time_t t = ts / 1000;
    struct tm* tm = gmtime(&t);
    sprintf(timeBuf, "%04d-%02d-%02dT%02d:%02d:%02dZ",
      tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
      tm->tm_hour, tm->tm_min, tm->tm_sec);

    res->printf("    <trkpt lat=\"%.6f\" lon=\"%.6f\">\n", la, lo);
    res->printf("      <ele>%.1f</ele>\n", el);
    res->printf("      <time>%s</time>\n", timeBuf);
    res->printf("      <speed>%.2f</speed>\n", sp);
    res->print("    </trkpt>\n");
  }

  res->print("  </trkseg></trk>\n</gpx>\n");
  f.close();
  request->send(res);
}

// ==================== Web 路由 ====================

// 前向声明（HTML 内容在后面定义）
extern const char INDEX_HTML[] PROGMEM;

void setupRoutes() {
  // 主页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // GPS 实时数据
  server.on("/api/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"fix\":" + String(hasFix ? "true" : "false") + ",";
    json += "\"lat\":" + String(currentLat, 6) + ",";
    json += "\"lng\":" + String(currentLng, 6) + ",";
    json += "\"alt\":" + String(currentAlt, 1) + ",";
    json += "\"speed\":" + String(currentSpeed, 1) + ",";
    json += "\"sats\":" + String(currentSats) + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
    json += "\"points\":" + String(trackPointCount) + ",";
    json += "\"track\":\"" + currentTrackName + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  // 开始/停止记录
  server.on("/api/record", HTTP_GET, [](AsyncWebServerRequest *request) {
    String action = request->hasParam("a") ? request->getParam("a")->value() : "";
    if (action == "start") startRecording();
    else if (action == "stop") stopRecording();
    String json = "{\"recording\":" + String(isRecording ? "true" : "false") + ",\"points\":" + String(trackPointCount) + "}";
    request->send(200, "application/json", json);
  });

  // 轨迹列表
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
    if (request->hasParam("file")) {
      sendGPX(request, request->getParam("file")->value());
    } else {
      request->send(400);
    }
  });

  // 删除轨迹
  server.on("/api/delete", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("file")) {
      SPIFFS.remove(request->getParam("file")->value());
      request->send(200, "text/plain", "deleted");
    } else {
      request->send(400);
    }
  });
}

// ==================== 网页 UI ====================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>ESP32 GPS 轨迹记录</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{font-family:-apple-system,'Segoe UI',sans-serif;background:#0b0e14;color:#e2e4e9;min-height:100vh;display:flex;justify-content:center;padding:16px}
  .app{width:100%;max-width:420px}
  h1{font-size:1.2rem;text-align:center;margin-bottom:16px;color:#4f8fff}

  .panel{background:#141820;border:1px solid #252a35;border-radius:16px;padding:16px;margin-bottom:12px}
  .panel-title{font-size:0.72rem;text-transform:uppercase;letter-spacing:1.5px;color:#5a5e6a;margin-bottom:12px}

  .gps-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .gps-item{text-align:center;background:rgba(255,255,255,0.03);border-radius:10px;padding:10px}
  .gps-item .v{font-size:1.6rem;font-weight:700;color:#4f8fff}
  .gps-item .v.speed{color:#00d4aa}
  .gps-item .v.alt{color:#ff7b42}
  .gps-item .l{font-size:0.65rem;color:#5a5e6a;margin-top:2px}

  .coord{font-size:0.75rem;color:#8b8f9a;text-align:center;padding:8px;word-break:break-all}

  .status-row{display:flex;align-items:center;justify-content:center;gap:16px;margin:8px 0}
  .status-dot{width:10px;height:10px;border-radius:50%}
  .status-dot.fix{background:#00d4aa;box-shadow:0 0 8px rgba(0,212,170,.5)}
  .status-dot.nofix{background:#ff4757}
  .rec-dot{width:10px;height:10px;border-radius:50%;animation:pulse 1s infinite}
  .rec-dot.on{background:#ff4757}
  .rec-dot.off{background:#2a3040}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.3}}

  .btn{display:block;width:100%;padding:14px;border:none;border-radius:12px;font-size:1rem;font-weight:600;cursor:pointer;transition:all .2s;margin-top:8px}
  .btn.rec-start{background:#ff4757;color:#fff}
  .btn.rec-stop{background:#ff7b42;color:#fff}
  .btn.download{background:#4f8fff;color:#fff}
  .btn.sm{padding:8px 14px;font-size:.75rem;width:auto;display:inline-block}

  .track-list{max-height:160px;overflow-y:auto}
  .track-row{display:flex;align-items:center;justify-content:space-between;padding:8px 0;border-bottom:1px solid #252a35;font-size:.78rem}
  .track-row:last-child{border-bottom:none}
  .track-name{color:#8b8f9a}
  .track-actions{display:flex;gap:6px}
  .btn-mini{padding:4px 10px;border-radius:6px;border:1px solid #252a35;background:#1a1f2b;color:#8b8f9a;font-size:.65rem;cursor:pointer}
  .btn-mini.dl{color:#4f8fff;border-color:#4f8fff}

  .footer{text-align:center;font-size:.65rem;color:#5a5e6a;padding:8px}
  .toast{position:fixed;top:20px;left:50%;transform:translateX(-50%) translateY(-120px);background:#1a1f2b;border:1px solid #252a35;border-radius:12px;padding:10px 22px;font-size:.85rem;z-index:99;transition:transform .3s}
  .toast.show{transform:translateX(-50%) translateY(0)}
</style>
</head>
<body>
<div class="toast" id="toast"></div>
<div class="app">
  <h1>ESP32 GPS 轨迹记录</h1>

  <div class="panel">
    <div class="panel-title">实时数据</div>
    <div class="gps-grid">
      <div class="gps-item">
        <div class="v speed" id="speedVal">--</div>
        <div class="l">速度 (km/h)</div>
      </div>
      <div class="gps-item">
        <div class="v alt" id="altVal">--</div>
        <div class="l">海拔 (m)</div>
      </div>
      <div class="gps-item">
        <div class="v" id="satsVal">--</div>
        <div class="l">卫星数</div>
      </div>
      <div class="gps-item">
        <div class="v" id="ptsVal">--</div>
        <div class="l">记录点数</div>
      </div>
    </div>
    <div class="coord" id="coordVal">等待定位...</div>
    <div class="status-row">
      <div class="status-dot nofix" id="fixDot"></div>
      <span id="fixText" style="font-size:.75rem;color:#5a5e6a">无定位</span>
      <div class="rec-dot off" id="recDot"></div>
      <span id="recText" style="font-size:.75rem;color:#5a5e6a">未记录</span>
    </div>
  </div>

  <div class="panel">
    <div class="panel-title">轨迹控制</div>
    <button class="btn rec-start" id="recBtn" onclick="toggleRecord()">开始记录</button>
  </div>

  <div class="panel">
    <div class="panel-title" style="display:flex;justify-content:space-between">
      保存的轨迹 <button class="btn-mini" onclick="loadTracks()">刷新</button>
    </div>
    <div class="track-list" id="trackList">
      <div style="color:#5a5e6a;text-align:center;padding:12px">加载中...</div>
    </div>
  </div>

  <div class="footer" id="updatedAt">等待数据...</div>
</div>

<script>
let recording = false;
function $(id){return document.getElementById(id)}

function toggleRecord(){
  if(recording){
    fetch('/api/record?a=stop').then(()=>{recording=false;updateUI()})
  }else{
    fetch('/api/record?a=start').then(()=>{recording=true;updateUI()})
  }
}

function updateUI(){
  const btn=$('recBtn');
  const dot=$('recDot');
  const txt=$('recText');
  if(recording){
    btn.className='btn rec-stop';btn.textContent='停止记录';
    dot.className='rec-dot on';txt.textContent='记录中';
  }else{
    btn.className='btn rec-start';btn.textContent='开始记录';
    dot.className='rec-dot off';txt.textContent='未记录';
  }
}

function loadTracks(){
  fetch('/api/tracks').then(r=>r.json()).then(data=>{
    const el=$('trackList');
    if(data.length===0){
      el.innerHTML='<div style="color:#5a5e6a;text-align:center;padding:12px">暂无轨迹</div>';
      return;
    }
    let html='';
    data.forEach(t=>{
      const size=(t.size/1024).toFixed(1)+' KB';
      const name=t.name.replace('/track_','').replace('.json','');
      html+=`<div class="track-row">
        <div class="track-name">${name} <span style="color:#5a5e6a;font-size:.65rem">${size}</span></div>
        <div class="track-actions">
          <button class="btn-mini dl" onclick="downloadGPX('${t.name}')">GPX</button>
          <button class="btn-mini" onclick="deleteTrack('${t.name}')">删除</button>
        </div>
      </div>`;
    });
    el.innerHTML=html;
  });
}

function downloadGPX(file){
  window.open('/api/download?file='+encodeURIComponent(file));
  showToast('GPX 下载中');
}

function deleteTrack(file){
  if(!confirm('删除此轨迹?'))return;
  fetch('/api/delete?file='+encodeURIComponent(file)).then(()=>loadTracks());
}

function showToast(msg){
  const el=$('toast');el.textContent=msg;el.classList.add('show');
  setTimeout(()=>el.classList.remove('show'),1800);
}

// 定时刷新
setInterval(()=>{
  fetch('/api/gps').then(r=>r.json()).then(d=>{
    $('speedVal').textContent=d.fix?d.speed:'--';
    $('altVal').textContent=d.fix?d.alt:'--';
    $('satsVal').textContent=d.sats;
    $('ptsVal').textContent=d.points;
    if(d.fix){
      $('coordVal').textContent=d.lat.toFixed(6)+', '+d.lng.toFixed(6);
      $('fixDot').className='status-dot fix';
      $('fixText').textContent='已定位';
    }else{
      $('coordVal').textContent='等待定位...';
      $('fixDot').className='status-dot nofix';
      $('fixText').textContent='无定位';
    }
    recording=d.recording;
    updateUI();
    $('updatedAt').textContent='更新 '+new Date().toLocaleTimeString();
  });
},1000);
setInterval(loadTracks,10000);
loadTracks();
</script>
</body>
</html>
)=====";

// ==================== 主程序 ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // GPS 串口
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS 串口: RX=16 TX=17, 波特率 9600");

  // SPIFFS
  initSPIFFS();

  // WiFi AP 模式
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("WiFi 热点: ");
  Serial.println(AP_SSID);
  Serial.print("IP 地址: ");
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
  Serial.println("Web 服务器已启动");
  Serial.println("手机连 WiFi 后打开 http://192.168.4.1");
}

void loop() {
  readGPS();
  recordPoint();
  delay(50);
}
