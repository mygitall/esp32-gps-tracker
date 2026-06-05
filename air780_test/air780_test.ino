// Air780EX 4G → MQTT 最小测试
#include <TinyGPS++.h>

#define AIR_RX 4
#define AIR_TX 5

TinyGPSPlus gps;
float lat,lng; bool fix;

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n=== 4G MQTT 测试 ===");
  Serial2.begin(9600,SERIAL_8N1,16,17);
  Serial1.begin(115200,SERIAL_8N1,AIR_RX,AIR_TX);

  // 等手动开机
  Serial.println("请手动短接 Air780EX PWR 到 GND...");
  delay(8000);
  Serial.println("开始\n");

  // === 1. AT 测试 ===
  Serial1.print("AT\r\n"); delay(500);
  String r; while(Serial1.available())r+=(char)Serial1.read();
  Serial.print("AT: "); Serial.println(r.indexOf("OK")>=0?"OK":"FAIL");
  if(r.indexOf("OK")<0){Serial.println("Air780EX 无响应");return;}

  // === 2. 4G 网络 ===
  Serial1.print("AT+CGATT=1\r\n"); delay(5000);
  while(Serial1.available())Serial1.read();

  Serial1.print("AT+CSTT=\"UNINET\"\r\n"); delay(2000);
  while(Serial1.available())Serial1.read();

  Serial1.print("AT+CIICR\r\n"); delay(8000);
  while(Serial1.available())Serial1.read();

  Serial1.print("AT+CIFSR\r\n"); delay(1000);
  r=""; while(Serial1.available())r+=(char)Serial1.read();
  Serial.print("IP: "); Serial.println(r);

  // === 3. TCP 连接 MQTT broker ===
  Serial1.print("AT+CIPSHUT\r\n"); delay(2000);
  while(Serial1.available())Serial1.read();

  Serial1.print("AT+CIPSTART=\"TCP\",\"broker.emqx.io\",1883\r\n");
  delay(10000);
  r=""; while(Serial1.available())r+=(char)Serial1.read();
  Serial.print("TCP: "); Serial.println(r.substring(0,150));

  if(r.indexOf("CONNECT")<0){Serial.println("TCP FAIL");return;}

  // === 4. MQTT CONNECT ===
  uint8_t cid[]="esp32-4g-test";
  int cidlen=14;
  int rem=2+4+1+1+2+2+cidlen;
  uint8_t conn[64]; int p=0;
  conn[p++]=0x10; conn[p++]=rem;
  conn[p++]=0x00; conn[p++]=0x04; conn[p++]='M';conn[p++]='Q';conn[p++]='T';conn[p++]='T';
  conn[p++]=0x04; conn[p++]=0x02; conn[p++]=0x00;conn[p++]=0x1E;
  conn[p++]=0x00; conn[p++]=cidlen;
  memcpy(conn+p,cid,cidlen); p+=cidlen;
  int connlen=p;

  Serial.print("CIPSEND="); Serial.println(connlen);
  while(Serial1.available())Serial1.read();
  Serial1.print("AT+CIPSEND="); Serial1.print(connlen); Serial1.print("\r\n");

  // 等 ">"
  r=""; unsigned long t=millis();
  while(millis()-t<5000){if(Serial1.available()){r+=(char)Serial1.read();if(r.indexOf(">")>=0)break;}delay(1);}
  if(r.indexOf(">")<0){Serial.println("NO >");return;}
  Serial.println("GOT >");

  // 发 CONNECT
  Serial1.write(conn,connlen);
  // 等 CONNACK (检查 0x20 0x02 0x00 0x00)
  r=""; t=millis(); bool gotConnack=false;
  while(millis()-t<10000){
    while(Serial1.available()){r+=(char)Serial1.read();}
    // 搜索 CONNACK 序列
    for(int i=0;i<(int)r.length()-3;i++){
      if((uint8_t)r[i]==0x20 && (uint8_t)r[i+1]==0x02 && (uint8_t)r[i+2]==0x00 && (uint8_t)r[i+3]==0x00){
        gotConnack=true; break;
      }
    }
    if(gotConnack || r.indexOf("CLOSED")>=0) break;
    delay(10);
  }
  if(!gotConnack){Serial.print("NO CONNACK: "); Serial.println(r.substring(0,100));return;}
  Serial.println("CONNACK OK!");

  // === 5. MQTT PUBLISH ===
  const char* topic="esp32/gps";
  const char* payload="{\"lat\":30.9568,\"lng\":121.8052,\"fix\":1}";
  int tlen=strlen(topic),plen=strlen(payload);
  int prem=2+tlen+plen;
  uint8_t pub[256]; p=0;
  pub[p++]=0x30; pub[p++]=prem;
  pub[p++]=0x00;pub[p++]=tlen; memcpy(pub+p,topic,tlen);p+=tlen;
  memcpy(pub+p,payload,plen);p+=plen;
  int publen=p;

  while(Serial1.available())Serial1.read();
  Serial1.print("AT+CIPSEND="); Serial1.print(publen); Serial1.print("\r\n");
  r=""; t=millis();
  while(millis()-t<5000){if(Serial1.available()){r+=(char)Serial1.read();if(r.indexOf(">")>=0)break;}delay(1);}
  if(r.indexOf(">")<0){Serial.println("NO > for pub");return;}

  Serial1.write(pub,publen);
  // 等 SEND OK（含 null 字节的响应会截断 String，改用延时+多次读取）
  delay(3000);
  r="";
  t=millis(); while(millis()-t<5000) {while(Serial1.available())r+=(char)Serial1.read(); delay(10);}
  // 检查是否有 SEND OK
  bool ok=false;
  for(int i=0;i<(int)r.length()-6;i++){
    if(r[i]=='S'&&r[i+1]=='E'&&r[i+2]=='N'&&r[i+3]=='D'&&r[i+4]==' '&&r[i+5]=='O'&&r[i+6]=='K'){ok=true;break;}
  }
  if(ok) Serial.println("PUB OK!");
  else {Serial.print("PUB FAIL: "); Serial.println(r.substring(0,100));}

  Serial1.print("AT+CIPCLOSE\r\n"); delay(1000);
  while(Serial1.available())Serial1.read();

  Serial.println("\n=== 测试完成 ===");
}

void loop() {
  while(Serial2.available()) {
    if(gps.encode(Serial2.read()) && gps.location.isValid()) {
      static unsigned long lg=0;
      if(millis()-lg>3000){
        Serial.printf("GPS: %.6f,%.6f\n",gps.location.lat(),gps.location.lng());
        lg=millis();
      }
    }
  }
  delay(100);
}
