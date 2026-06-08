// ESP32 GPS Tracker v7.1 — FLUSH失败保留缓冲重试
#include <TinyGPS++.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#define FW_VER "7.1"

#define AIR_RX 4
#define AIR_TX 5
#define AIR_PWR 27
#define GPS_RX 16
#define GPS_TX 17
#define ALT_N 50

// WiFi（OTA 用）
const char* WIFI_SSID = "WIFI";
const char* WIFI_PASS = "999999999";

// 围栏：家坐标 + 半径（米）
const double HOME_LAT = 30.956900;
const double HOME_LNG = 121.805200;
const float FENCE_RADIUS = 500.0;

TinyGPSPlus gps;
double lat,lng; float alt,spd; int sat; bool fix;
float altBuf[ALT_N]; int altI=0,altC=0;
bool wifiOn=false, fenceAlert=false;
unsigned long lastMoveTime=0;
double lastMoveLat=0,lastMoveLng=0;  // 用于位置变化检测

// GPS 批处理缓冲区
#define BUF_MAX 10
struct GpsPt {double la,lo; float al; int sp,sat,bt,mv,cs;};
GpsPt buf[BUF_MAX]; int bufN=0;

int bat=-1; bool batReady=false;  // -1=待读, batReady=已成功读过
int targetBat=-1,displayBat=-1;    // 目标值 + 平滑显示值
unsigned long lastBatSmooth=0;     // 平滑计时
int battMv=0;                      // 原始电压 mV
int csq=0;                        // 4G 信号质量 (0-31)
char tcpHost[32]=""; int tcpPort=0; // TCP 复用状态
unsigned long tcpConnTime=0;       // TCP 连接建立时间，用于超时重连
bool g4ok=false;                   // 4G 是否已连接
unsigned long last4gRetry=0;       // 上次重连尝试时间

float filterAlt(float raw){
  altBuf[altI]=raw; altI=(altI+1)%ALT_N; if(altC<ALT_N)altC++;
  float s=0; for(int i=0;i<altC;i++)s+=altBuf[i];
  return s/altC;
}

float distKm(double la1,double lo1,double la2,double lo2){
  float dlat=(la2-la1)*111320.0;
  float dlng=(lo2-lo1)*111320.0*cos(la1*PI/180);
  return sqrt(dlat*dlat+dlng*dlng);
}

// ===== AT =====
String atCmd(const char* cmd,unsigned long to=3000){
  while(Serial1.available())Serial1.read();
  Serial1.print(cmd);Serial1.print("\r\n");
  String r; unsigned long t=millis();
  while(millis()-t<to){if(Serial1.available())r+=(char)Serial1.read();delay(1);}
  return r;
}

void airPowerOn(){
  pinMode(AIR_PWR,OUTPUT);digitalWrite(AIR_PWR,LOW);delay(200);
  digitalWrite(AIR_PWR,HIGH);delay(1000);digitalWrite(AIR_PWR,LOW);delay(4000);
}

int lipoPct(int mv){
  // 20mV 一档，更灵敏反映真实放电曲线
  if(mv>=4200)return 100;if(mv>=4180)return 98;if(mv>=4160)return 95;
  if(mv>=4140)return 92;if(mv>=4120)return 89;if(mv>=4100)return 86;
  if(mv>=4080)return 83;if(mv>=4060)return 80;if(mv>=4040)return 77;
  if(mv>=4020)return 74;if(mv>=4000)return 71;if(mv>=3980)return 68;
  if(mv>=3960)return 65;if(mv>=3940)return 62;if(mv>=3920)return 59;
  if(mv>=3900)return 56;if(mv>=3880)return 53;if(mv>=3860)return 50;
  if(mv>=3840)return 47;if(mv>=3820)return 44;if(mv>=3800)return 41;
  if(mv>=3780)return 38;if(mv>=3760)return 35;if(mv>=3740)return 32;
  if(mv>=3720)return 29;if(mv>=3700)return 26;if(mv>=3680)return 23;
  if(mv>=3660)return 20;if(mv>=3640)return 17;if(mv>=3620)return 14;
  if(mv>=3600)return 11;if(mv>=3580)return 8;if(mv>=3560)return 6;
  if(mv>=3540)return 4;if(mv>=3520)return 3;if(mv>=3500)return 2;
  if(mv>=3400)return 1;return 0;
}

// ===== TCP 连接复用 =====
bool tcpConnect(const char* host, int port){
  // 复用检查：相同主机端口且连接未超过 5 分钟
  if(strcmp(tcpHost,host)==0 && tcpPort==port && millis()-tcpConnTime<300000) return true;
  // 关闭旧连接
  if(tcpHost[0]){Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();}
  tcpHost[0]=0; tcpPort=0; tcpConnTime=0;
  Serial1.print("AT+CIPSHUT\r\n");delay(500);while(Serial1.available())Serial1.read();
  char cmd[64];
  snprintf(cmd,sizeof(cmd),"AT+CIPSTART=\"TCP\",\"%s\",%d",host,port);
  String r=atCmd(cmd,10000);
  unsigned long t=millis();while(millis()-t<5000){while(Serial1.available())r+=(char)Serial1.read();if(r.indexOf("CONNECT")>=0)break;delay(50);}
  if(r.indexOf("CONNECT")<0){
    Serial.printf("TCP FAIL %s:%d\n",host,port);
    static int tcpFailCnt=0;
    if(++tcpFailCnt>=3){tcpFailCnt=0;g4ok=false;Serial.println("TCP fails→reset g4ok");}
    return false;
  }
  strncpy(tcpHost,host,31);tcpHost[31]=0;
  tcpPort=port;tcpConnTime=millis();
  return true;
}

// ===== 4G 初始化 =====
bool init4G(){
  // 先探测模块是否已开机，并检查 PDP 是否有效
  Serial1.print("AT\r\n");delay(500);
  String pre=atCmd("AT",2000);
  if(pre.indexOf("OK")>=0){
    Serial.println("4G already ON, check PDP...");
    String ip=atCmd("AT+CIFSR",3000);
    if(ip.indexOf(".")>0){Serial.println("PDP OK");goto pdp;}
    Serial.println("PDP dead, reset module...");
    // PDP 失效，PWRKEY 关机再开机
    airPowerOn();delay(8000);
    for(int i=0;i<10;i++){String r=atCmd("AT",2000);if(r.indexOf("OK")>=0)goto pdp;delay(1000);}
    Serial.println("Reset fail");
    // 继续走 PWRKEY 循环
  }
  // 模块未开机或复位失败，PWRKEY 触发
  for(int cycle=0;cycle<3;cycle++){
    Serial.printf("PWRKEY (%d/3)...\n",cycle+1);
    airPowerOn();
    delay(8000);
    for(int i=0;i<10;i++){
      String r=atCmd("AT",2000);
      if(r.indexOf("OK")>=0){Serial.println("AT OK");goto pdp;}
      Serial.printf("  retry %d/10\n",i+1);
      delay(1000);
    }
  }
  Serial.println("AT FAIL");return false;
  pdp:
  Serial.println("PDP init...");
  String r1=atCmd("AT+CSQ",2000);
  int p=r1.indexOf("+CSQ:");if(p>=0)csq=r1.substring(p+5).toInt();
  Serial.printf("CSQ:%d\n",csq);
  // 先查是否已有 IP（PDP 可能已激活）
  String ip=atCmd("AT+CIFSR",3000);
  if(ip.indexOf(".")>0){Serial.printf("IP reuse:%s\n",ip.c_str());return true;}
  // 无 IP，完整激活 PDP
  atCmd("AT+CGATT=1",5000);
  atCmd("AT+CSTT=\"UNINET\"",2000);
  atCmd("AT+CIICR",8000);
  ip=atCmd("AT+CIFSR",3000);
  delay(500);
  String ip2=atCmd("AT+CIFSR",3000);
  Serial.printf("IP1:%s IP2:%s\n",ip.c_str(),ip2.c_str());
  return ip.indexOf(".")>0 || ip2.indexOf(".")>0;
}

// ===== HTTP 上报 =====
void httpReport(float la,float lo,float al,float sp,int sa,int ba){
  if(!tcpConnect("www.sseeee.com",80))return;
  while(Serial1.available())Serial1.read();
  char url[400];
  snprintf(url,sizeof(url),
    "GET /esp32/mmq/receiver.php?lat=%.6f&lng=%.6f&alt=%.1f&spd=%.1f&sat=%d&fix=1&rssi=%d&mv=%d&csq=%d&ver=%s&uptime=%lu&heap=%u&_=%lu HTTP/1.1\r\nHost: www.sseeee.com\r\nConnection: close\r\nCache-Control: no-cache\r\n\r\n",
    la,lo,al,sp,sa,ba,battMv,csq,FW_VER,millis()/1000,ESP.getFreeHeap(),millis());
  Serial1.print("AT+CIPSEND=");Serial1.print(strlen(url));Serial1.print("\r\n");
  String w;unsigned long t=millis();
  while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")>=0){Serial1.print(url);delay(2000);
    String ok;t=millis();while(millis()-t<3000){while(Serial1.available())ok+=(char)Serial1.read();if(ok.indexOf("SEND OK")>=0||ok.indexOf("ERROR")>=0)break;delay(10);}
    Serial.println(ok.indexOf("SEND OK")>=0?"HTTP OK":"HTTP FAIL");
  }else{Serial.println("HTTP NO >");}
  // HTTP 用 close，发完断开 TCP，避免干扰 MQTT
  Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();
  tcpHost[0]=0;tcpPort=0;
}

// ===== 批量 GET 上报（简单可靠）=====
void httpFlush(){
  if(bufN==0)return;
  if(!tcpConnect("www.sseeee.com",80)){bufN=0;return;}
  while(Serial1.available())Serial1.read();
  // 用 GET URL 参数拼接多条记录
  int cnt=bufN; // 保存计数，失败时恢复
  String url="GET /esp32/mmq/receiver.php?batch=";
  for(int i=0;i<cnt;i++){
    if(i>0)url+="|";
    char pt[96];
    snprintf(pt,sizeof(pt),"%.6f,%.6f,%.1f,%d,%d,%d,%d,%d",
      (float)buf[i].la,(float)buf[i].lo,buf[i].al,buf[i].sp,buf[i].sat,buf[i].bt,buf[i].mv,buf[i].cs);
    url+=pt;
  }
  url+=" HTTP/1.1\r\nHost: www.sseeee.com\r\nConnection: close\r\n\r\n";
  bufN=0;
  Serial1.print("AT+CIPSEND=");Serial1.print(url.length());Serial1.print("\r\n");
  String w;unsigned long t=millis();
  while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")>=0){Serial1.print(url);delay(2000);
    String ok;t=millis();while(millis()-t<3000){while(Serial1.available())ok+=(char)Serial1.read();if(ok.indexOf("SEND OK")>=0||ok.indexOf("ERROR")>=0)break;delay(10);}
    if(ok.indexOf("SEND OK")>=0){Serial.println("FLUSH OK");}
    else{Serial.println("FLUSH FAIL(keep)");bufN=cnt;} // 失败保留数据
  }else{Serial.println("FLUSH NO >");bufN=cnt;}
  Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();
  tcpHost[0]=0;tcpPort=0;
}

// ===== MQTT 发布（含 GPS 坐标 + CSQ）=====
void mqttPublish(const char* topic,const char* payload){
  if(!tcpConnect("broker.emqx.io",1883)){Serial.println("MQTT TCP FAIL");return;}
  int tlen=strlen(topic),plen=strlen(payload);
  uint8_t conn[]={0x10,0x0E,0x00,0x04,'M','Q','T','T',0x04,0x02,0x00,0x1E,0x00,0x02,'g','p'};
  uint8_t pub[256];int pos=0;
  int rem=2+tlen+plen;
  if(pos+rem+4>256){Serial.println("MQTT overflow");return;}
  pub[pos++]=0x30;
  do{uint8_t b=rem%128;rem/=128;if(rem)b|=0x80;pub[pos++]=b;}while(rem);
  pub[pos++]=0x00;pub[pos++]=tlen;
  memcpy(pub+pos,topic,tlen);pos+=tlen;memcpy(pub+pos,payload,plen);pos+=plen;
  // 发 CONNECT
  while(Serial1.available())Serial1.read();
  Serial1.print("AT+CIPSEND=");Serial1.print(sizeof(conn));Serial1.print("\r\n");
  String w;unsigned long t=millis();
  while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")<0){Serial.println("MQTT NO >");return;}
  Serial1.write(conn,sizeof(conn));delay(500);
  // 等 CONNACK 并校验返回码
  String ca;t=millis();bool connOk=false;
  while(millis()-t<5000){while(Serial1.available())ca+=(char)Serial1.read();if(ca.length()>=4){connOk=((uint8_t)ca[0]==0x20&&(uint8_t)ca[1]==0x02&&(uint8_t)ca[3]==0x00);break;}delay(10);}
  while(Serial1.available())Serial1.read();
  if(!connOk){Serial.println("MQTT CONN FAIL");return;}
  // 发 PUBLISH
  Serial1.print("AT+CIPSEND=");Serial1.print(pos);Serial1.print("\r\n");
  w="";t=millis();while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")<0){Serial.println("MQTT PUB NO >");return;}
  Serial1.write(pub,pos);delay(300);
  Serial.println("MQTT OK");
}

// ===== 读电量 =====
void readBattery(){
  Serial1.print("AT\r\n");delay(300);while(Serial1.available())Serial1.read();
  String cbc=atCmd("AT+CBC",5000);
  int p=cbc.indexOf("+CBC:");if(p>=0){int mv=cbc.substring(p+5).toInt();if(mv>0){
    battMv=mv; targetBat=lipoPct(mv);batReady=true;
    if(displayBat<0)displayBat=targetBat;
    Serial.printf("CBC → %dmV target=%d%% display=%d%%\n",mv,targetBat,displayBat);
  }}else{Serial.printf("CBC fail: %s\n",cbc.substring(0,20).c_str());}
  bat=displayBat; // 对外仍用 bat，但值是平滑后的
}

// ===== 读信号 =====
void readCsq(){
  String r=atCmd("AT+CSQ",2000);
  int p=r.indexOf("+CSQ:");if(p>=0){int v=r.substring(p+5).toInt();if(v>=0&&v<=31)csq=v;}
}

// ===== 主程序 =====
void setup(){
  Serial.begin(115200);delay(500);
  Serial.println("\n=== GPS Tracker v7.1 ===");
  pinMode(2,OUTPUT);digitalWrite(2,LOW);
  Serial2.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  Serial1.begin(115200,SERIAL_8N1,AIR_RX,AIR_TX);

  WiFi.begin(WIFI_SSID,WIFI_PASS);
  int w=0;while(WiFi.status()!=WL_CONNECTED&&w++<8){delay(500);Serial.print(".");}
  if(WiFi.status()==WL_CONNECTED){
    wifiOn=true;
    Serial.printf("\nWiFi: %s\n",WiFi.localIP().toString().c_str());
    ArduinoOTA.setHostname("esp32-gps");ArduinoOTA.setPassword("12345678");ArduinoOTA.begin();
    Serial.println("OTA ready");
  }else{Serial.println("\nWiFi fail, 4G only");}

  g4ok = init4G();
  if(g4ok){
    Serial.println("4G OK");
    readBattery();
  }else{Serial.println("4G FAIL");}
}

void loop(){
  if(wifiOn)ArduinoOTA.handle();

  // ==== 4G 自动重连：连续 TCP 失败 3 次或定时重试 ====
  static int tcpFails=0;
  if(!g4ok && millis()-last4gRetry>120000){ // 每2分钟重试
    last4gRetry=millis();tcpFails=0;
    Serial.println("Retry 4G...");
    g4ok=init4G();
    if(g4ok){Serial.println("4G recovered!");readBattery();}
  }

  static unsigned long hb=0;
  if(millis()-hb>1000){hb=millis();digitalWrite(2,HIGH);delay(30);digitalWrite(2,LOW);}

  // ==== 电量平滑：每秒向目标靠近 1% ====
  if(batReady && displayBat!=targetBat && millis()-lastBatSmooth>1000){
    lastBatSmooth=millis();
    displayBat+=(targetBat>displayBat)?1:-1;
    bat=displayBat;
  }

  // GPS
  while(Serial2.available()){
    if(gps.encode(Serial2.read())){
      if(gps.location.isValid()){
        double newLat=gps.location.lat(),newLng=gps.location.lng();
        if(!fix||newLat!=lat||newLng!=lng){ // 位置变化才记
          if(fix&&bufN<BUF_MAX&&batReady){
            GpsPt p={newLat,newLng,alt,(int)spd,sat,bat,battMv,csq};
            buf[bufN++]=p;
          }
          lat=newLat;lng=newLng;fix=true;
        }
      }
      if(gps.altitude.isValid())alt=filterAlt(gps.altitude.meters());
      if(gps.speed.isValid())spd=gps.speed.kmph();
      if(gps.satellites.isValid())sat=gps.satellites.value();
    }
  }

  // ==== 电子围栏 ====
  static unsigned long lastFenceAlert=0;
  if(fix){
    float d=distKm(HOME_LAT,HOME_LNG,lat,lng);
    if(d>FENCE_RADIUS){
      if(!fenceAlert||millis()-lastFenceAlert>300000){ // 首次或每5分钟重告
        fenceAlert=true;lastFenceAlert=millis();
        char fj[96];snprintf(fj,sizeof(fj),"{\"alert\":\"fence\",\"dist\":%.0f,\"lat\":%.6f,\"lng\":%.6f}",d,lat,lng);
        mqttPublish("esp32/gps",fj);
        Serial.printf("FENCE! %.0fm\n",d);
      }
    }
    if(d<FENCE_RADIUS&&fenceAlert){fenceAlert=false;lastFenceAlert=0;}
  }

  // ==== 夜间模式 ====
  bool nightMode=false;
  if(gps.time.isValid()){
    int h=(gps.time.hour()+8)%24;
    nightMode=(h>=22||h<6);
  }

  // ==== 远程指令轮询（每5分钟）====
  static unsigned long lc=0;
  if(millis()-lc>300000){lc=millis();
    if(tcpConnect("www.sseeee.com",80)){
      while(Serial1.available())Serial1.read();
      Serial1.print("AT+CIPSEND=70\r\n");delay(500);
      String w;unsigned long t=millis();
      while(millis()-t<3000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
      if(w.indexOf(">")>=0){
        Serial1.print("GET /esp32/mmq/cmd_api.php?device=esp32 HTTP/1.1\r\nHost: www.sseeee.com\r\nConnection: close\r\n\r\n");
        delay(2000);
        String resp;while(Serial1.available())resp+=(char)Serial1.read();
        if(resp.indexOf("\"reboot\"")>=0){Serial.println("CMD:reboot");ESP.restart();}
        if(resp.indexOf("\"deep\"" )>=0){Serial.println("CMD:deep");nightMode=true;}
      }
      strncpy(tcpHost,"",1);tcpPort=0; // 远程指令用 close，断开复用状态
    }
  }

  // ==== 低功耗：GPS速度 + 位置变化双判 ====
  bool movingBySpd=(spd>=2.0);
  bool movingByPos=false;
  if(fix){
    if(lastMoveLat==0&&lastMoveLng==0){lastMoveLat=lat;lastMoveLng=lng;} // 首次定位初始化参考点
    else{
      float moveDist=distKm(lastMoveLat,lastMoveLng,lat,lng);
      movingByPos=(moveDist>5.0); // 5米以上算移动
    }
  }
  if(movingBySpd||movingByPos){
    lastMoveTime=millis();
    lastMoveLat=lat;lastMoveLng=lng;
  }else{
    if(lastMoveTime==0)lastMoveTime=millis();
  }
  bool idle=!(movingBySpd||movingByPos);
  bool deepSleep=(idle&&millis()-lastMoveTime>300000);
  bool lowPower=(idle&&millis()-lastMoveTime>60000);

  // 上报间隔
  int httpIvl=5; // 5秒批量上报，低延迟

  static unsigned long lg=0;
  if(millis()-lg>(deepSleep?30000:lowPower?15000:5000)){lg=millis();
    if(fix)Serial.printf("GPS: %.6f,%.6f sat=%d alt=%.1f %s\n",lat,lng,sat,alt,deepSleep?"DEEP":lowPower?"LP":"");
    else Serial.println("GPS: wait...");
  }

  // ==== 电量/信号读取（30s）====
  static unsigned long lb=0;
  if(millis()-lb>30000){lb=millis();
    readBattery();readCsq();
  }

  // ==== HTTP 批量上报（每 httpIvl 秒 flush 缓冲区）====
  static unsigned long lp=0;
  if(millis()-lp>httpIvl*1000){lp=millis();
    // 即使位置不变也发当前坐标，保证地图实时更新
    if(bufN==0 && fix && batReady && bufN<BUF_MAX){
      GpsPt p={lat,lng,alt,(int)spd,sat,bat,battMv,csq};
      buf[bufN++]=p;
    }
    Serial.printf("FLUSH %d pts\n",bufN);
    httpFlush();
  }
  delay(50);
}
