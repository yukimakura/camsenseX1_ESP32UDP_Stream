#include "BluetoothSerial.h"
#include "HardwareSerial.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#include <vector>
#include <algorithm>

BluetoothSerial SerialBT;
HardwareSerial Serialpin(2);
const char ssid[] = "***"; // SSID
const char pass[] = "***";  // password

static WiFiUDP wifiUdp; 
static const char *kRemoteIpadr = "192.168.0.107";
static const int kRmoteUdpPort = 9000; //送信先のポート

typedef struct{
  bool isFound;
  int begin;
  int end;
}FonudPacketFormat;

static void WiFi_setup()
{
  static const int kLocalPort = 7000;  //自身のポート

  WiFi.begin(ssid, pass);
  while( WiFi.status() != WL_CONNECTED) {
    delay(500);  
    Serial.println("Waiting connection...");
  }  
  Serial.print("Wifi Connected! IP address = ");
  digitalWrite(26,HIGH);
  digitalWrite(27,LOW);
  Serial.println(WiFi.localIP().toString());
  wifiUdp.begin(kLocalPort);
}



void setup() {
  pinMode(26,OUTPUT);
  pinMode(27,OUTPUT);
  digitalWrite(27,HIGH);
  digitalWrite(26,LOW);

  Serialpin.begin(115200);
  Serial.begin(115200);
  WiFi_setup();
}

void loop() {
  static std::vector<char> judgestr;
  static std::vector<char> sendstr;

  String getstr = Serialpin.readStringUntil(0x55);
  if(getstr.length() > 34){
    if(*(getstr.begin())==0xAA && *(getstr.begin()+1)==0x03 && *(getstr.begin()+2)==0x08){
      std::string sendString(getstr.begin()+3, getstr.end());
      wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
      wifiUdp.print(sendString.c_str());
      // Serial.println(sendString.c_str());
      wifiUdp.endPacket();  
      
    }
  }
  delay(1); //これ入れないとすぐリセットかかる(ESPのRTOS？？)

}