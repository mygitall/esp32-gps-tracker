// ESP32 GPS Tracker v3 — 健康监控 + 远程指令 + 夜间降频
#include <TinyGPS++.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#define FW_VER "3.0"

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
const float FENCE_RADIUS = 500.0;  // 500米

TinyGPSPlus gps;
float lat,lng,alt,spd; int sat; bool fix;
float altBuf[ALT_N]; int altI=0,altC=0;
bool wifiOn=false, fenceAlert=false;
unsigned long lastMoveTime=0;

float filterAlt(float raw){
  altBuf[altI]=raw; altI=(altI+1)%ALT_N; if(altC<ALT_N)altC++;
  float s=0; for(int i=0;i<altC;i++)s+=altBuf[i];
  return s/altC;
}

// 距离计算（米）
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

// Li-Ion 18650 真实放电曲线 (mV → %)
int lipoPct(int mv){
  if(mv>=4200)return 100;if(mv>=4100)return 92;if(mv>=4000)return 85;
  if(mv>=3950)return 78;if(mv>=3900)return 70;if(mv>=3850)return 62;
  if(mv>=3800)return 53;if(mv>=3750)return 42;if(mv>=3700)return 30;
  if(mv>=3650)return 20;if(mv>=3600)return 12;if(mv>=3500)return 5;
  if(mv>=3400)return 2;return 0;
}

bool init4G(){
  // 自动 PWRKEY：每次开机固定触发，最多三次
  for(int cycle=0;cycle<3;cycle++){
    Serial.printf("PWRKEY 触发 (%d/3)...\n",cycle+1);
    airPowerOn();  // GPIO27 HIGH → S8050导通 → PWRKEY拉低1秒
    // 等 Air780EX 完全启动
    delay(8000);
    for(int i=0;i<10;i++){
      String r=atCmd("AT",2000);
      if(r.indexOf("OK")>=0){Serial.println("AT OK");goto ok;}
      Serial.printf("  retry %d/10\n",i+1);
      delay(1000);
    }
  }
  Serial.println("AT FAIL");return false;
  ok:
  Serial.println("AT OK, init PDP...");
  String r1=atCmd("AT+CSQ",2000); Serial.printf("CSQ:%s\n",r1.c_str());
  String r2=atCmd("AT+CGATT=1",5000); Serial.printf("ATT:%s\n",r2.c_str());
  String r3=atCmd("AT+CSTT=\"UNINET\"",2000); Serial.printf("STT:%s\n",r3.c_str());
  String r4=atCmd("AT+CIICR",8000); Serial.printf("ICR:%s\n",r4.c_str());
  String r5=atCmd("AT+CIFSR",2000); Serial.printf("IP:%s\n",r5.c_str());
  return r5.indexOf(".")>0;
}

void httpReport(float la,float lo,float al,float sp,int sa,int ba){
  char url[350];
  snprintf(url,sizeof(url),
    "GET /esp32/mmq/receiver.php?lat=%.6f&lng=%.6f&alt=%.1f&spd=%.1f&sat=%d&fix=1&rssi=%d&ver=%s&uptime=%lu&heap=%u HTTP/1.1\r\nHost: www.sseeee.com\r\nConnection: close\r\n\r\n",
    la,lo,al,sp,sa,ba,FW_VER,millis()/1000,ESP.getFreeHeap());
  Serial1.print("AT+CIPSHUT\r\n");delay(500);while(Serial1.available())Serial1.read();
  String r=atCmd("AT+CIPSTART=\"TCP\",\"www.sseeee.com\",80",10000);
  unsigned long t=millis();while(millis()-t<5000){while(Serial1.available())r+=(char)Serial1.read();if(r.indexOf("CONNECT")>=0)break;delay(50);}
  if(r.indexOf("CONNECT")<0){Serial1.print("AT+CIPCLOSE\r\n");delay(200);return;}
  while(Serial1.available())Serial1.read();
  Serial1.print("AT+CIPSEND=");Serial1.print(strlen(url));Serial1.print("\r\n");
  String w;t=millis();while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")>=0){Serial1.print(url);delay(2000);}
  Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();
  Serial.println("HTTP OK");
}

void mqttPublish(const char* topic,const char* payload){
  Serial1.print("AT+CIPSHUT\r\n");delay(500);while(Serial1.available())Serial1.read();
  String r=atCmd("AT+CIPSTART=\"TCP\",\"broker.emqx.io\",1883",8000);
  unsigned long t=millis();while(millis()-t<5000){while(Serial1.available())r+=(char)Serial1.read();if(r.indexOf("CONNECT")>=0)break;delay(50);}
  if(r.indexOf("CONNECT")<0){Serial.println("MQTT TCP FAIL");return;}
  int tlen=strlen(topic),plen=strlen(payload);
  uint8_t conn[]={0x10,0x0E,0x00,0x04,'M','Q','T','T',0x04,0x02,0x00,0x1E,0x00,0x02,'g','p'};
  uint8_t pub[128];int pos=0;
  pub[pos++]=0x30;pub[pos++]=2+tlen+plen;pub[pos++]=0x00;pub[pos++]=tlen;
  memcpy(pub+pos,topic,tlen);pos+=tlen;memcpy(pub+pos,payload,plen);pos+=plen;
  while(Serial1.available())Serial1.read();
  Serial1.print("AT+CIPSEND=");Serial1.print(sizeof(conn));Serial1.print("\r\n");
  String w;t=millis();while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")>=0){Serial1.write(conn,sizeof(conn));delay(300);while(Serial1.available())Serial1.read();}
  Serial1.print("AT+CIPSEND=");Serial1.print(pos);Serial1.print("\r\n");
  w="";t=millis();while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")>=0){Serial1.write(pub,pos);delay(300);}
  Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();
}

// ===== 主程序 =====
void setup(){
  Serial.begin(115200);delay(500);
  Serial.println("\n=== GPS Tracker v2 ===");
  pinMode(2,OUTPUT);digitalWrite(2,LOW);
  Serial2.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  Serial1.begin(115200,SERIAL_8N1,AIR_RX,AIR_TX);

  // WiFi OTA（尝试连接家里WiFi）
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  int w=0;while(WiFi.status()!=WL_CONNECTED&&w++<8){delay(500);Serial.print(".");}
  if(WiFi.status()==WL_CONNECTED){
    wifiOn=true;
    Serial.printf("\nWiFi: %s\n",WiFi.localIP().toString().c_str());
    ArduinoOTA.setHostname("esp32-gps");ArduinoOTA.setPassword("12345678");ArduinoOTA.begin();
    Serial.println("OTA ready");
  }else{Serial.println("\nWiFi fail, 4G only");}

  if(init4G()){
    Serial.println("4G OK");
    atCmd("AT+CSCLK=2");  // Air780EX 空闲自动休眠（省电 ~100mA）
  }else{Serial.println("4G FAIL");}
}

void loop(){
  if(wifiOn)ArduinoOTA.handle();  // OTA 更新检查

  static unsigned long hb=0;
  if(millis()-hb>1000){hb=millis();digitalWrite(2,HIGH);delay(30);digitalWrite(2,LOW);}

  // GPS
  while(Serial2.available()){
    if(gps.encode(Serial2.read())){
      if(gps.location.isValid()){lat=gps.location.lat();lng=gps.location.lng();fix=true;}
      if(gps.altitude.isValid())alt=filterAlt(gps.altitude.meters());
      if(gps.speed.isValid())spd=gps.speed.kmph();
      if(gps.satellites.isValid())sat=gps.satellites.value();
    }
  }

  // ==== 电子围栏 ====
  if(fix){
    float d=distKm(HOME_LAT,HOME_LNG,lat,lng);
    if(d>FENCE_RADIUS&&!fenceAlert){
      fenceAlert=true;
      char fj[64];snprintf(fj,sizeof(fj),"{\"alert\":\"fence\",\"dist\":%.0f,\"lat\":%.6f,\"lng\":%.6f}",d,lat,lng);
      mqttPublish("esp32/gps",fj);
      Serial.printf("FENCE! %.0fm\n",d);
    }
    if(d<FENCE_RADIUS&&fenceAlert)fenceAlert=false;
  }

  // ==== 夜间模式：22:00-6:00 降频 ====
  bool nightMode=false;
  if(gps.time.isValid()){
    int h=(gps.time.hour()+8)%24; // UTC+8 北京时间
    nightMode=(h>=22||h<6);
  }

  // ==== 远程指令轮询（每5分钟）====
  static unsigned long lc=0;
  if(millis()-lc>300000){lc=millis();
    Serial1.print("AT+CIPSHUT\r\n");delay(300);while(Serial1.available())Serial1.read();
    String r=atCmd("AT+CIPSTART=\"TCP\",\"www.sseeee.com\",80",8000);
    if(r.indexOf("CONNECT")>=0){
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
        Serial.printf("CMD: %s\n",resp.substring(0,100).c_str());
      }
      Serial1.print("AT+CIPCLOSE\r\n");delay(200);while(Serial1.available())Serial1.read();
    }
  }

  // ==== 低功耗：三级降频 ====
  bool idle=(spd<2.0);
  if(idle){if(lastMoveTime==0)lastMoveTime=millis();}
  else lastMoveTime=millis();
  bool deepSleep=(idle&&millis()-lastMoveTime>300000); // 静止>5min=深度休眠
  bool lowPower=(idle&&millis()-lastMoveTime>60000);   // 静止>60s=低功耗

  static unsigned long lg=0;
  if(millis()-lg>(deepSleep?30000:lowPower?15000:5000)){lg=millis();
    if(fix)Serial.printf("GPS: %.6f,%.6f sat=%d alt=%.1f %s\n",lat,lng,sat,alt,deepSleep?"DEEP":lowPower?"LP":"");
    else Serial.println("GPS: wait...");
  }

  // 电量（每30秒读一次，避免频繁AT命令）
  static unsigned long bt=0; static int bat=0;
  if(millis()-bt>30000){bt=millis();
    atCmd("AT",1000); // 唤醒模块
    String cbc=atCmd("AT+CBC",5000);
    int p=cbc.indexOf("+CBC:");if(p>=0){int mv=cbc.substring(p+5).toInt();if(mv>0)bat=lipoPct(mv);}
  }
  static bool batAlert=false;if(bat<20&&!batAlert){batAlert=true;}if(bat>25)batAlert=false;

  // MQTT 电量（正常30s，LP 120s，DEEP 600s，夜间×2）
  static unsigned long lb=0;
  int mqttIvl=deepSleep?600:lowPower?120:30; if(nightMode&&!lowPower)mqttIvl*=2;
  if(millis()-lb>mqttIvl*1000){lb=millis();
    char bj[32];snprintf(bj,sizeof(bj),"{\"bat\":%d,\"fix\":%d}",bat,fix?1:0);
    Serial.printf("BAT: %d%% %s%s\n",bat,deepSleep?"DEEP":lowPower?"LP":"",nightMode?" NIGHT":"");
    mqttPublish("esp32/gps",bj);
    Serial.println("MQTT done");
  }

  // HTTP 上报（正常20s，LP 60s，DEEP 300s，夜间×2）
  static unsigned long lp=0;
  int httpIvl=deepSleep?300:lowPower?60:20; if(nightMode&&!lowPower)httpIvl*=2;
  if(fix&&millis()-lp>httpIvl*1000){lp=millis();
    Serial.printf("HTTP>> lat=%.6f lng=%.6f alt=%.1f\n",lat,lng,alt);
    httpReport(lat,lng,alt,spd,sat,bat);
    if(bat<20){char aj[32];snprintf(aj,sizeof(aj),"{\"alert\":\"lowbat\",\"bat\":%d}",bat);mqttPublish("esp32/gps",aj);}
  }
  delay(50);
}
