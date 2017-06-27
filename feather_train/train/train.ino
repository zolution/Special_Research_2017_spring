#include <SPI.h>
#include <ESP8266WiFi.h>
#include <QueueList.h>
#include <WiFiUdp.h>
#include <string.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


#define MAX_V 900


#define MOTOR_PWMA  12  
#define MOTOR_AIN1  13
#define MOTOR_AIN2  15
#define MOTOR_STBY  0
#define HALL_PIN    14
unsigned long sampleTime=500;
double scale = 0.05;
const char ssid[] = "MVNL_Test";
const char pass[] = "33664888424";
char local_ip[32];
const char host_ip[] = "192.168.1.101";
unsigned int hostUdpPort = 4247;
const char next_ip[] = "192.168.1.205";
unsigned int nextUdpPort = 4210;

/*
struct train_ctrl_cmd {
  char preamble[4]; //TRAN
  uint8_t num;
  uint16_t seq;
  int16_t v;
  uint8_t headlight;
  uint8_t checksum;
};

struct train_ctrl_ack {
  char preamble[4];  //TRAK
  uint16_t seq;
};

struct train_ctrl_cmd tpkt;
struct train_ctrl_ack tpkt_ack;
*/
//boolean power_down=1;

//const int maxRPM = 10000; 
int v = 0;

//RH_NRF24 nrf24(RF24_CE, RF24_CS);

//byte controller_address[6] = "1CNTR";
//byte train_address[6] = "1TRAN";

/*int getRPM()
{
  // sample for sampleTime in millisecs
  int count=0;
  boolean cflag=LOW;
  unsigned long currentTime=0;
  unsigned long startTime=millis();
  while (currentTime<=sampleTime)
  {
    if (digitalRead(HALL_PIN)==HIGH)
    {
      cflag=HIGH;
    }
    if (digitalRead(HALL_PIN)==LOW && cflag==HIGH)
    {
      count++;
      cflag=LOW;
    }
    currentTime=millis()-startTime;
  }
  int count2rpm = int(60000/float(sampleTime))*count/4;
  return count2rpm;
}*/

QueueList <unsigned long> sample;
int getRPM_unblock(){
  // sample for sampleTime in millisecs
  static boolean cflag=LOW;
  unsigned long startTime=millis();
  
  if (digitalRead(HALL_PIN)==HIGH)
  {
    cflag=HIGH;
  }
  if (digitalRead(HALL_PIN)==LOW && cflag==HIGH)
  {
    sample.push(startTime);
    cflag=LOW;
  }
  
  while(!sample.isEmpty()){
    if(startTime - sample.peek() > sampleTime){
      sample.pop();
    }
    else break;
  }
  int count2rpm = int(60000/float(sampleTime))*(sample.count())/4;
  return count2rpm;
}

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets

void setup() {
  Serial.begin(115200); 
  analogWriteFreq(256);
  //queue.setPrinter(Serial);
  pinMode(HALL_PIN, INPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  //digitalWrite(MOTOR_STBY, HIGH);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);

  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
  
  //digitalWrite(MOTOR_STBY, LOW); //turn off motor
  digitalWrite(MOTOR_AIN1, LOW); 
  digitalWrite(MOTOR_AIN2, HIGH); //positive speed*/
  Serial.println();


  
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  int r = 20000;
  //while(sensor.setMeasurementTimingBudget(r) && r>0) r--;
  sensor.setMeasurementTimingBudget(20000);
  Serial.println(r);
  delay(100);
  uint16_t tem = sensor.readRangeContinuousMillimeters();
  while(sensor.readRangeContinuousMillimeters()>10000);
  
  WiFi.begin(ssid,pass);
  Serial.println(WiFi.localIP());
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  delay(1000);
  Udp.begin(localUdpPort);
  //power_down = 1;
  //watchdog_setup();
}

/*void loop(){
  
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
  }
  else{
    Serial.printf("Nothing Got\n");
    delay(500);
  }
}*/
int recv = 0;
int flag = 0;
char replyPacket[256];

void loop() {
  //int v= 128;
  digitalWrite(MOTOR_STBY, HIGH);
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, HIGH);
  analogWrite(MOTOR_PWMA, v);
  int rpm = 0;
  int diff = 0;
  
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    int temp = atoi(incomingPacket);
    if(temp>=10000 && temp < 11000){
      sampleTime = temp - 10000;
    }
    else if(temp>=15000 && temp < 16000){
      scale = (double)(temp - 15000) / 100.0;
    }
    else recv = temp;
    Serial.printf("UDP packet contents: ");
    Serial.println(recv);
  }
  rpm=getRPM_unblock();
  flag++;
 if(flag>=30000) flag = 0;
  Serial.print("rpm: ");
  Serial.println(rpm);
  if(recv >= 0){
      //Serial.println((int)buf[1]); 
    
    
   
    if (recv > rpm && flag % 10 == 0)
    {  
       diff = recv - rpm;
       v = v + diff * scale;
       if (v > MAX_V)
       {
         v = MAX_V;
       }
       
       //analogWrite(MOTOR_PWMA, v);
    }
    else if (recv < rpm && flag % 10 == 0)
    {   
       diff = rpm - recv;
       v = v - diff * scale;
       if (v < 30)
       {
        v = 0;
       }
       //analogWrite(MOTOR_PWMA, v);
    }
  }
  else{
    v = -recv;
  }
    // Send a reply
  analogWrite(MOTOR_PWMA, v);
  Serial.print("diff: ");
  Serial.println(diff);
  Serial.print("v: ");
  Serial.println(v);
  if(flag%8==0){
    sprintf(replyPacket,"%d",rpm);
    Udp.beginPacket(next_ip, nextUdpPort);
    Udp.write(replyPacket);
    Udp.endPacket();
  }
  if(flag%8==4){
    
    sprintf(replyPacket,"%d",rpm);
    Udp.beginPacket(host_ip, hostUdpPort);
    Udp.write(replyPacket);
    Udp.endPacket();
    int rr = millis();
    int temp = (int)(sensor.readRangeContinuousMillimeters());
    int tt = millis();
    Serial.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println(tt-rr);
    Serial.println(temp);
    sprintf(replyPacket,"%d",temp);
    Udp.beginPacket(host_ip, hostUdpPort);
    Udp.write(replyPacket);
    Udp.endPacket();
  }
  

  

    // send back a reply, to the IP address and port we got the packet from

}




