#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include "mcp3008.h"
#include <Wire.h>

#include "LiquidCrystal_I2C.h" 
LiquidCrystal_I2C lcd(0x27, 16, 2); 

struct CarData{float nowV,minV,maxV;} carData;
struct Joystick{
  float x,y;
} js;
const byte   DNS_PORT  =   53;
WiFiUDP Udp;
DNSServer dnsServer;
const IPAddress local(11, 11, 11, 11);
const IPAddress dns(11, 11, 11, 11);
const IPAddress gateway(11, 11, 11, 1); /* not used */
const IPAddress netmask(255, 255, 255, 0);
const IPAddress broadCast(11, 11, 11, 255);

const unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];
byte jsBuffer[sizeof(js)];
byte cdBuffer[sizeof(carData)];
//=============== [JoyStick] ================
#define CLOCK_PIN 15 //D8
#define MISO_PIN  13 //D7
#define MOSI_PIN  12 //D6
#define CS_PIN    14 //D5

#define joy_stickX 1 //A1
#define joy_stickY 0 //A0

#define buzzer_PIN 0//D3

// Set up MCP3008
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

float jsGap=0.3;
float prevjsx , prevjsy;

void setup()
{
  Serial.begin(9600);   
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(buzzer_PIN, OUTPUT);
  dnsServer.start(DNS_PORT, "*", local);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local, dns, netmask);
  WiFi.softAP("UDP-RECEIVER");
  WiFi.begin();

  Udp.begin(localPort);
  Serial.print("UDP Server Start");

  // 初始化LCD
  lcd.init();
  lcd.backlight();

}

void loop()
{
  // =============== [JoyStick]  ================
  js.x = (adc.readADC(joy_stickX) - 512)/512.0 ;
  js.y= (adc.readADC(joy_stickY) - 522)/512.0 ;  //Y軸歸位時預設522
  Serial.println("X: "+ String(js.x));
  Serial.println("Y: "+ String(js.y));
  Serial.println("======================");
  // =============== [UDP - Send]  ================  
  dnsServer.processNextRequest();
  delay(50);
  if( abs(js.x - prevjsx) >jsGap || abs(js.y - prevjsy) >jsGap ){
    memcpy(jsBuffer , &js , sizeof(js));
    Udp.beginPacket(broadCast, 8888);
    Udp.write(jsBuffer,sizeof(js));
    Udp.endPacket();
    prevjsx = js.x;
    prevjsy = js.y;
    delay(250);  
  }
  // =============== [LCD]  ================
   lcd.setCursor(0, 0); // (colum, row)
   lcd.print("spd:"+String(carData.nowV)); 
   lcd.setCursor(0, 1); // (colum,row)從第二排第三格位置開始顯示
   lcd.print("safe:"+String(carData.minV)+"~"+String(carData.maxV));
  
  
  // =============== [UDP - Recieve]  ================
  int size = Udp.parsePacket();

  if (size > 0)
  {
    //Serial.print((String)"Request from " + Udp.remoteIP().toString() + ", port " + Udp.remotePort());
    // read the packet into packetBufffer
    Udp.read(cdBuffer , sizeof(carData));
    memcpy(&carData,cdBuffer ,sizeof(carData));
    if(carData.nowV < carData.minV || carData.nowV > carData.maxV){
        Serial.print("NOT SAFE!!!!!");
        digitalWrite(buzzer_PIN , HIGH);
    }
    else{
        digitalWrite(buzzer_PIN , LOW);
      }
    /*
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    */
    
   Serial.println("CarData "+String(carData.nowV));
    //Send Back [Test OK]
    /*
    Udp.beginPacket(broadCast, 8888);
    Udp.write("My Data \r\n");
    Udp.endPacket();
    */
  }
  
  // Send JoyStick Data:
  /*
  if(abs(js.x) >0.1 || abs(js.y) > 0.1){
    //Udp.send((byte*)&js , sizeof(js),broadCast ,8888 );
    memcpy(jsBuffer , &js , sizeof(js));
    Udp.beginPacket(broadCast, 8888);
    Udp.write(jsBuffer,sizeof(js));
    Udp.endPacket();
  }
  */
 
  

  
}
