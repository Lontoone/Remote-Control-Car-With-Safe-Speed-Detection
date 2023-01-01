#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "ITG3200.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include "MyVec.h"
int moterRelay_PIN = 13 ;//D7
WiFiUDP Udp;
unsigned int localPort = 8888;

const IPAddress local(11, 11, 11, 30);
const IPAddress server(11, 11, 11, 11);
const IPAddress dns(11, 11, 11, 11);
const IPAddress netmask(255, 255, 255, 0);


float lerp(float a, float b, float f)
{
    return a + f * (b - a);
}
//========== [Structure] ==========
struct Joystick{
  float x,y;
} js;
struct CarData{float nowV,minV,maxV;} carData;
byte ReplyBuffer[sizeof(carData)];
char jsBuffer[sizeof(js)];
//========== [Device Declear] ==========
ITG3200 gyro;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(2345);

//=============== [公式所需參數] ================
float g = 9.81; //重力
float a = 0.08; // 車輪距 8cm
float h =0.05;  //車高 5cm


int16_t gx, gy, gz;
int16_t dgx , dgy , dgz, igx,igy,igz;
int16_t accX,accY,accZ;
Vec3D velocity;
float gyro_lsb = 0.069565; //磁場係數
float deltaTime =0.05;//0.05 ;
Vec3D zeroGap(0.3,0.3,0.3);
float moveThreshold = 0.1;
//============== [Pin] ====================
int led = 1;
int moter1Pin1 =  0; //D3
int moter1Pin2 =  2; //D4
int moter1Pin3 =  14; //D5
int moter1Pin4 =  12; //D6
int LED_PIN = BUILTIN_LED;
bool blinkState = false;

float max(const float &a , const float &b){
  if(a>b && !isnan(a)){return a;}
  else if(b>a && !isnan(b)) return b;  
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Serial.begin(9600);
    WiFi.begin("UDP-RECEIVER"); // NAME of the server!

    WiFi.mode(WIFI_STA);
    WiFi.config(local, dns, netmask);
    WiFi.softAP("udpsender1");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println((String)"\nConnected to " + WiFi.localIP().toString());
    Serial.println("Initializing I2C devices...");
    //========== [Device Init] ==========
    gyro.initialize();
    accel.begin();
    accel.setRange(ADXL345_RANGE_2_G);   

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "ITG3200 connection successful" : "ITG3200 connection failed");

    // configure Arduino LED pin for output
    //pinMode(LED_PIN, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(moterRelay_PIN, OUTPUT);
    pinMode(moter1Pin1 , OUTPUT);
    pinMode(moter1Pin2 , OUTPUT);
    pinMode(moter1Pin3 , OUTPUT);
    pinMode(moter1Pin4 , OUTPUT);
    digitalWrite(moter1Pin1 , LOW);
    digitalWrite(moter1Pin2 , LOW); 
    digitalWrite(moter1Pin3 , LOW);
    digitalWrite(moter1Pin4 , LOW); 
    digitalWrite(moterRelay_PIN , LOW); 
    Udp.begin(localPort);
}

void loop() {
  //======================= [UPD] ==========================
  int size = Udp.parsePacket();

  if (size>0)
  {   
    // read the packet into packetBufffer
    Udp.read(jsBuffer , sizeof(js));
    memcpy(&js,jsBuffer ,sizeof(js));
    Serial.println(String(js.x) +" , "+ String(js.y));

   }else{
      //Serial.print("Lerp");
      js.x=lerp(js.x , 0 ,0.05);
      js.y=lerp(js.y , 0 ,0.05);
      //Serial.println("LERP" +String(js.x) +" , "+ String(js.y));
      digitalWrite(BUILTIN_LED, HIGH);
      
   }
    //Moter
    
    // ===== [Move Wheel] =======
   if(abs(js.y) < moveThreshold && abs(js.x) < moveThreshold){
      //Serial.println("LOW");
      digitalWrite(moterRelay_PIN , LOW);
    }
   else{digitalWrite(moterRelay_PIN , HIGH);}
   if(js.y> moveThreshold){
      //前進 且 左轉
      if(js.x < -moveThreshold){
         digitalWrite(moter1Pin1 , LOW);
         digitalWrite(moter1Pin2 , LOW); 
         digitalWrite(moter1Pin3 , HIGH);
         digitalWrite(moter1Pin4 , LOW);
         Serial.println("Turn Left");
     }
      else if(js.x > moveThreshold ){
         Serial.println("Turn Right");
         digitalWrite(moter1Pin1 , HIGH);
         digitalWrite(moter1Pin2 , LOW); 
         digitalWrite(moter1Pin3 , LOW);
         digitalWrite(moter1Pin4 , LOW);
      }
      else{
        Serial.println("Go Forward");
        digitalWrite(moter1Pin1 , LOW);
        digitalWrite(moter1Pin2 , HIGH);
        digitalWrite(moter1Pin3 , LOW);
        digitalWrite(moter1Pin4 , HIGH);
      }
    }
   else if(js.y< -moveThreshold){
      if( js.x < -moveThreshold){
          Serial.println("Backward Left");
         digitalWrite(moter1Pin1 , LOW);
         digitalWrite(moter1Pin2 , LOW); 
         digitalWrite(moter1Pin3 , LOW);
         digitalWrite(moter1Pin4 , HIGH);
      }
      else if(js.x > moveThreshold){
         Serial.println("Backward Right");
         digitalWrite(moter1Pin1 , LOW);
         digitalWrite(moter1Pin2 , HIGH); 
         digitalWrite(moter1Pin3 , LOW);
         digitalWrite(moter1Pin4 , LOW);
      }
      else{
        Serial.println("Backward");
        digitalWrite(moter1Pin1 , HIGH);
        digitalWrite(moter1Pin2 , LOW);    
        digitalWrite(moter1Pin3 , HIGH);
        digitalWrite(moter1Pin4 , LOW);    
        }
    }
    
   if(abs(js.y) < moveThreshold && abs(js.x) < moveThreshold){
      digitalWrite(moter1Pin1 , LOW);
      digitalWrite(moter1Pin2 , LOW); 
      digitalWrite(moter1Pin3 , LOW);
      digitalWrite(moter1Pin4 , LOW); 
      
      //digitalWrite(BUILTIN_LED, LOW);
    }
  
    //=============== [Accel] ==================
    sensors_event_t event; 
    accel.getEvent(&event);
    accX = (event.acceleration.x);
    accY = (event.acceleration.y);
    accZ = (event.acceleration.z -0.74);
    // 累加加速度
    Vec3D vel(accX , accY , 0);

    //靜止==> TODO:
    //if(vel.IsZero()){
    if(vel < zeroGap){
      velocity = Vec3D(0,0,0);
      //Serial.print("0");
      //return;
    }
    
    //dv = vel * (deltaTime);
    velocity = velocity +  vel * (deltaTime);
    float v = velocity.GetNorm();

    //算roll , pitch , yaw   (與地面傾斜主要用roll)
    float roll = 180 * atan2(accX, sqrt(accY*accY + accZ*accZ))/PI;
    //float roll = 180 * atan2(accY, sqrt(accX*accX + accZ*accZ))/PI;
    float roll_rad = abs(roll) * 0.0174533;
    //R=道路的曲率半徑
    Vec3D R = velocity * velocity / (g * cos(roll) ) ;
    float r = R.GetNorm();

    //計算速度下限    
    float minV = pow((tan(roll_rad) - a/(2*h)) /(1+(a*tan(roll_rad)/(2*h)))*r*g ,0.5);
    minV = isnan(minV)? 0 : minV;
    //計算速度上限
    float maxV = pow((tan(roll_rad) + a/(2*h))/(1-(a*tan(roll_rad)/(2*h)))*r*g ,0.5);

    //比較安全範圍:
    bool isSafe = ( v <maxV) && ( v > minV);
    // =============== [UDP]  ================
    carData.nowV = v;
    carData.minV = minV;
    carData.maxV = maxV;
    Udp.beginPacket(server, 8888);
    memcpy(ReplyBuffer , &carData , sizeof(carData));
    Udp.write(ReplyBuffer,sizeof(carData));
    //Udp.write("acknowledged\r\n");
    Udp.endPacket();
    
    Serial.println("vec " + String(vel.x) +" " +String(vel.y) +" "+String(vel.z) +" " );
    Serial.println("vec " + String(velocity.x) +" " +String(velocity.y) +" "+String(velocity.z) +" " );
    Serial.println("Roll " + String(roll));
    Serial.println("Current V" + String(v));
    Serial.println("Min vec" + String(minV));
    Serial.println("Max vec" + String(maxV));
    Serial.println("=> Is Safe : " + isSafe );
    Serial.println("-------------------------------------------------------------------------");
    
    // read raw gyro measurements from device
    gyro.getRotation(&gx, &gy, &gz);

    //加速度:
    int x = (gx - dgx) / deltaTime;
    int y = (gy - dgy) / deltaTime;
    int z = (gz - dgz) / deltaTime;

    igx+=x;
    igy+=y;
    igz+=z;

    dgx = gx;
    dgy = gy;
    dgz = gz;
    delay(deltaTime*1000);
}
