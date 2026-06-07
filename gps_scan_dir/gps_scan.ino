// GPS 波特率扫描
#define GPS_RX 16
#define GPS_TX 17

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n=== GPS Baud Scan ===");

  const long bauds[]={9600,38400,115200,57600,4800,19200,230400,460800,921600};
  int n=9;

  for(int i=0;i<n;i++){
    Serial2.begin(bauds[i],SERIAL_8N1,GPS_RX,GPS_TX);
    Serial.printf("=== %d baud ===\n",(int)bauds[i]);
    unsigned long t=millis();
    String line="";
    while(millis()-t<3000){
      while(Serial2.available()){
        char c=Serial2.read();
        if(c=='\n'||c=='\r'){if(line.length()>3){Serial.println(line);line="";}}
        else if(c>=32&&c<=126)line+=c;
      }
      delay(1);
    }
    if(i<n-1){Serial2.end();delay(200);}
  }
  Serial.println("\nDone");
}

void loop(){delay(1000);}
