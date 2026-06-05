// ESP32 + ATGM336H GPS + Air780EX 4G → MQTT
// Air780EX: GPIO4(RX) GPIO5(TX) 115200baud PWRKEY手动
// GPS: GPIO16(RX) GPIO17(TX) 9600baud

#include <TinyGPS++.h>

#define AIR_RX 4
#define AIR_TX 5
#define GPS_RX 16
#define GPS_TX 17

TinyGPSPlus gps;
float lat,lng,alt,spd; int sat; bool fix;

// ===== AT 命令 =====
String atCmd(const char* cmd, unsigned long to=3000) {
  while(Serial1.available()) Serial1.read();
  Serial1.print(cmd); Serial1.print("\r\n");
  String r; unsigned long t=millis();
  while(millis()-t<to){if(Serial1.available())r+=(char)Serial1.read();delay(1);}
  return r;
}

// ===== TCP 发数据 =====
bool tcpSend(const uint8_t* data, int len) {
  while(Serial1.available()) Serial1.read();
  Serial1.print("AT+CIPSEND="); Serial1.print(len); Serial1.print("\r\n");
  String w; unsigned long t=millis();
  while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
  if(w.indexOf(">")<0) return false;
  Serial1.write(data,len);
  delay(500);
  return true;
}

// ===== 4G MQTT 发布 =====
bool mqttPublish(const char* json) {
  // 1. TCP
  Serial1.print("AT+CIPSHUT\r\n"); delay(1000); while(Serial1.available())Serial1.read();

  String r=atCmd("AT+CIPSTART=\"TCP\",\"broker.emqx.io\",1883",15000);
  unsigned long t2=millis();
  while(millis()-t2<8000){while(Serial1.available())r+=(char)Serial1.read();if(r.indexOf("CONNECT")>=0)break;delay(50);}
  if(r.indexOf("CONNECT")<0) return false;

  // 2. MQTT CONNECT
  const char* cid="esp32-4g-gps";
  int cidlen=strlen(cid), rem=2+4+1+1+2+2+cidlen;
  uint8_t conn[64]; int p=0;
  conn[p++]=0x10; conn[p++]=rem;
  conn[p++]=0x00;conn[p++]=0x04; conn[p++]='M';conn[p++]='Q';conn[p++]='T';conn[p++]='T';
  conn[p++]=0x04; conn[p++]=0x02; conn[p++]=0x00;conn[p++]=0x1E;
  conn[p++]=0x00;conn[p++]=cidlen; memcpy(conn+p,cid,cidlen); p+=cidlen;
  if(!tcpSend(conn,p)) return false;

  // 等 CONNACK
  r=""; unsigned long t=millis();
  while(millis()-t<8000){
    while(Serial1.available())r+=(char)Serial1.read();
    bool ok=false;
    for(int i=0;i<(int)r.length()-3;i++){
      if((uint8_t)r[i]==0x20&&(uint8_t)r[i+1]==0x02){ok=true;break;}
    }
    if(ok) break; delay(10);
  }

  // 3. MQTT PUBLISH
  const char* topic="esp32/gps";
  int tlen=strlen(topic),plen=strlen(json),prem=2+tlen+plen;
  uint8_t pub[256]; p=0;
  pub[p++]=0x30; pub[p++]=prem;
  pub[p++]=0x00;pub[p++]=tlen; memcpy(pub+p,topic,tlen);p+=tlen;
  memcpy(pub+p,json,plen);p+=plen;
  if(!tcpSend(pub,p)) return false;

  // 等 SEND OK
  r=""; t=millis();
  while(millis()-t<5000){while(Serial1.available())r+=(char)Serial1.read();delay(10);}
  for(int i=0;i<(int)r.length()-6;i++){
    if(r[i]=='S'&&r[i+1]=='E'&&r[i+2]=='N'&&r[i+3]=='D') return true;
  }

  // 关 TCP
  Serial1.print("AT+CIPCLOSE\r\n"); delay(500);
  while(Serial1.available())Serial1.read();
  return false;
}

// ===== Air780EX PWRKEY 自动开机 =====
#define AIR_PWR 27
void airPowerOn() {
  Serial.println("PWRKEY 开机...");
  pinMode(AIR_PWR, OUTPUT);
  digitalWrite(AIR_PWR, LOW);   // 默认低电平，三极管不导通
  delay(500);
  digitalWrite(AIR_PWR, HIGH);  // 高电平 → 三极管导通 → PWRKEY 拉低
  delay(1500);                   // 保持 1.5 秒
  digitalWrite(AIR_PWR, LOW);   // 释放
  delay(5000);                   // 等 Air780EX 启动
  Serial.println("PWRKEY 释放");
}

// ===== 4G 初始化 =====
bool init4G() {
  airPowerOn();

  // 重试 AT
  for(int i=0;i<5;i++){
    if(atCmd("AT").indexOf("OK")>=0) break;
    Serial.printf("retry %d\n",i+1);
    delay(1000);
  }
  if(atCmd("AT").indexOf("OK")<0){Serial.println("AT FAIL");return false;}
  atCmd("AT+CGATT=1",5000);
  atCmd("AT+CSTT=\"UNINET\"");
  atCmd("AT+CIICR",8000);
  String ip=atCmd("AT+CIFSR");
  Serial.print("4G IP: "); Serial.println(ip);
  return ip.indexOf(".")>0;
}

// ===== 海拔滤波 + 校准 =====
#define ALT_N 50
#define ALT_CAL -30.0   // 校准偏移（GPS原始值 - 真实海拔）
float altBuf[ALT_N]; int altI=0, altC=0;
float filterAlt(float raw) {
  altBuf[altI]=raw; altI=(altI+1)%ALT_N; if(altC<ALT_N)altC++;
  float s=0; for(int i=0;i<altC;i++)s+=altBuf[i];
  return s/altC;
}

// ===== 主程序 =====
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n=== GPS 4G Tracker ===");
  Serial2.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  Serial1.begin(115200,SERIAL_8N1,AIR_RX,AIR_TX);

  if(init4G()) Serial.println("4G OK");
  else Serial.println("4G FAIL");
}

void loop() {
  // GPS
  while(Serial2.available()){
    if(gps.encode(Serial2.read())){
      if(gps.location.isValid()){lat=gps.location.lat();lng=gps.location.lng();fix=true;}
      if(gps.altitude.isValid()){
        float raw=gps.altitude.meters() + ALT_CAL;  // 校准后海拔
        alt=filterAlt(raw);
      }
      if(gps.speed.isValid())spd=gps.speed.kmph();
      if(gps.satellites.isValid())sat=gps.satellites.value();
    }
  }

  static unsigned long lg=0;
  if(millis()-lg>5000){lg=millis();
    if(fix) Serial.printf("GPS: %.6f,%.6f sat=%d\n",lat,lng,sat);
    else Serial.println("GPS: wait...");
  }

  static unsigned long lp=0;
  if(fix && millis()-lp>15000){lp=millis();
    char json[128];
    snprintf(json,sizeof(json),"{\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,\"spd\":%.1f,\"sat\":%d,\"fix\":1}",
             lat,lng,alt,spd,sat);

    // 1. MQTT
    Serial.printf("MQTT>> %s\n",json);
    mqttPublish(json);

    // 2. HTTP POST
    String post="api_key=esp32&data="+String(json);
    Serial1.print("AT+CIPSTART=\"TCP\",\"www.sseeee.com\",80\r\n");
    String r; unsigned long t=millis();
    while(millis()-t<8000){while(Serial1.available())r+=(char)Serial1.read();if(r.indexOf("CONNECT")>=0)break;delay(50);}
    if(r.indexOf("CONNECT")>=0){
      String http="POST /esp32/mmq/api.php HTTP/1.1\r\nHost: www.sseeee.com\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: ";
      http += String(post.length());
      http += "\r\nConnection: close\r\n\r\n";
      http += post;
      while(Serial1.available())Serial1.read();
      Serial1.print("AT+CIPSEND="); Serial1.print(http.length()); Serial1.print("\r\n");
      String w; t=millis();
      while(millis()-t<5000){if(Serial1.available()){w+=(char)Serial1.read();if(w.indexOf(">")>=0)break;}delay(1);}
      if(w.indexOf(">")>=0){Serial1.print(http);delay(2000);}
      Serial1.print("AT+CIPCLOSE\r\n");delay(500);
      while(Serial1.available())Serial1.read();
      Serial.println("HTTP OK");
    }
  }
  delay(50);
}
