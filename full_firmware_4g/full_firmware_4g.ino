// ESP32 GPS Tracker v2 — WiFi OTA + 电子围栏 + 低功耗
#include <TinyGPS++.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

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

bool init4G(){
  if(atCmd("AT",2000).indexOf("OK")>=0){Serial.println("4G already ON");}
  else{airPowerOn();}
  for(int i=0;i<10;i++){if(atCmd("AT",3000).indexOf("OK")>=0)break;delay(2000);}
  if(atCmd("AT").indexOf("OK")<0)return false;
  atCmd("AT+CGATT=1",5000);atCmd("AT+CSTT=\"UNINET\"");atCmd("AT+CIICR",8000);
  return atCmd("AT+CIFSR").indexOf(".")>0;
}

void httpReport(float la,float lo,float al,float sp,int sa){
  char url[250];
  snprintf(url,sizeof(url),"GET /esp32/mmq/receiver.php?lat=%.6f&lng=%.6f&alt=%.1f&spd=%.1f&sat=%d&fix=1 HTTP/1.1\r\nHost: www.sseeee.com\r\nConnection: close\r\n\r\n",la,lo,al,sp,sa);
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
  if(r.indexOf("CONNECT")<0)return;
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

  // 电量
  int bat=0;
  {String cbc=atCmd("AT+CBC",2000);
   int p=cbc.indexOf("+CBC:");if(p>=0){int mv=cbc.substring(p+5).toInt();if(mv>0)bat=constrain(map(mv,3300,4200,0,100),0,100);}}

  // MQTT 电量（正常30s，LP 120s，DEEP 600s）
  static unsigned long lb=0;
  if(millis()-lb>(deepSleep?600000:lowPower?120000:30000)){lb=millis();
    char bj[32];snprintf(bj,sizeof(bj),"{\"bat\":%d,\"fix\":%d}",bat,fix?1:0);
    mqttPublish("esp32/gps",bj);
    Serial.printf("BAT: %d%% %s\n",bat,deepSleep?"DEEP":lowPower?"LP":"");
  }

  // HTTP 上报（正常20s，LP 60s，DEEP 300s）
  static unsigned long lp=0;
  if(fix&&millis()-lp>(deepSleep?300000:lowPower?60000:20000)){lp=millis();
    Serial.printf("HTTP>> lat=%.6f lng=%.6f alt=%.1f\n",lat,lng,alt);
    httpReport(lat,lng,alt,spd,sat);
  }
  delay(50);
}
