/*

Authors:
	- Matt Almenshad
	- Jacob Lin

*/

#include <MQTTClient.h>
#include <TimeLib.h> 
#include <EthernetUdp.h>


IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
const int timeZone = -8;  // Pacific Daylight Time (USA)
unsigned int localPort = 8888;  // local port to listen for UDP packets

EthernetUDP Udp;
MQTTClient mqttClient;

time_t prevDisplay = 0;
time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
char timeStampedMessage[80];

class Alarm{
    public:
      Alarm();
      ~Alarm();

      virtual void trigger() = 0;
      virtual void stop() = 0;
      virtual void setDuration(int duration) = 0;
      virtual int getDuration() = 0;
      virtual void setTriggerValue(int val) = 0;
      virtual int getTriggerValue() = 0;
      virtual void setPin(int pin) = 0;
      virtual int getPin() = 0;
      

   protected:
     int pin;
     int duration;
     int triggerVal;
};

Alarm::Alarm(){
  digitalWrite(getPin(),HIGH);
    
}
Alarm::~Alarm(){
  digitalWrite(getPin(),HIGH);
    
}

void Alarm::trigger(){}
void Alarm::stop(){}
void Alarm::setDuration(int duration){}
void Alarm::setTriggerValue(int val){}
int Alarm::getTriggerValue(){}
void Alarm::setPin(int pin){}
int Alarm::getPin(){}
int Alarm::getDuration(){}

class BlinkAlarm : public Alarm{
      public:
       BlinkAlarm();
      ~BlinkAlarm();

      void trigger();
      void stop();
      void setDuration(int duration);
      int getDuration();
      void setTriggerValue(int duration);
      void setPin(int pin);
      int getTriggerValue();
      int getPin();
};

BlinkAlarm::BlinkAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
BlinkAlarm::~BlinkAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
void BlinkAlarm::trigger(){
  digitalWrite(getPin(),HIGH);
    
}
void BlinkAlarm::stop(){
  digitalWrite(getPin(),LOW);
}
void BlinkAlarm::setDuration(int duration){
  this->duration = duration;
 }
void BlinkAlarm::setTriggerValue(int val){
  this->triggerVal = val;
 }
int BlinkAlarm::getTriggerValue(){
  return this->triggerVal;
 }
int BlinkAlarm::getDuration(){
  return this->duration;
 }
void BlinkAlarm::setPin(int pin){
  this->pin = pin;
  pinMode(pin, OUTPUT); // built-in light
 }
int BlinkAlarm::getPin(){
 return this->pin;
}

class BuzzerAlarm : public Alarm{
      public:
       BuzzerAlarm();
      ~BuzzerAlarm();

      void trigger();
      void stop();
      void setDuration(int duration);
      int getDuration();
      void setTriggerValue(int duration);
      void setPin(int pin);
      int getTriggerValue();
      int getPin();
  
 };


BuzzerAlarm::BuzzerAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
BuzzerAlarm::~BuzzerAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
void BuzzerAlarm::trigger(){
  digitalWrite(getPin(),HIGH);
    
}
void BuzzerAlarm::stop(){
  digitalWrite(getPin(),LOW);
}
void BuzzerAlarm::setDuration(int duration){
  this->duration = duration;
 }
void BuzzerAlarm::setTriggerValue(int val){
  this->triggerVal = val;
 }
int BuzzerAlarm::getTriggerValue(){
  return this->triggerVal;
 }
int BuzzerAlarm::getDuration(){
  return this->duration;
 }
void BuzzerAlarm::setPin(int pin){
  this->pin = pin;
  pinMode(pin, OUTPUT); // buzzer
 }
int BuzzerAlarm::getPin(){
 return this->pin;
}

class CloudAlarm : public Alarm{
      public:
       CloudAlarm();
      ~CloudAlarm();

      void trigger();
      void stop();
      void setDuration(int duration);
      int getDuration();
      void setTriggerValue(int duration);
      void setPin(int pin);
      int getTriggerValue();
      int getPin();
      char* getIP();
      void setIP(char* tmp);
      void initialize();
      int getPort();
      void setPort(int tmp);

      private:
      int port;
      char* ip;
  
 };


CloudAlarm::CloudAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
CloudAlarm::~CloudAlarm(){
  digitalWrite(getPin(),HIGH);
    
}
char* CloudAlarm::getIP(){
  return this->ip;
  }
void CloudAlarm::setIP(char* tmp){
  this->ip = tmp;
  }
void CloudAlarm::initialize(){
  mqttClient.begin(getIP(),getPort(),OPEN,NULL,"","");
  Serial.print("client initialized with: ");
  Serial.print(getIP());
  Serial.print(" ");
  Serial.println(getPort());
  }

int CloudAlarm::getPort(){
  return this->port;
  }
void CloudAlarm::setPort(int tmp){
  this->port = tmp;
  }
void CloudAlarm::trigger(){
    mqttClient.publish("cis330/MattA_JacobL",timeStampedMessage);
    Serial.println("trigger called");
}
void CloudAlarm::stop(){
  
}
void CloudAlarm::setDuration(int duration){
  
 }
void CloudAlarm::setTriggerValue(int val){
  
 }
int CloudAlarm::getTriggerValue(){
  
 }
int CloudAlarm::getDuration(){
  
 }
void CloudAlarm::setPin(int pin){
  
 }
int CloudAlarm::getPin(){
 
}

int photoResistorPin = 0;
int laserPtrPin = 3;
int builtinLight = 13;
int buzzerPin = 5;
int photocellPin = 14;
int photocellReading;
int lightVal = 0;

BlinkAlarm blinky;
BuzzerAlarm buzzy;
CloudAlarm cloudy;


void setup() {
  pinMode(laserPtrPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("Serial Initialized");

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  Serial.println("synced");
  
  buzzy.setPin(buzzerPin);
  buzzy.setTriggerValue(500);
  buzzy.setDuration(250);
  
  blinky.setPin(builtinLight);
  blinky.setTriggerValue(500);
  blinky.setDuration(250);

  cloudy.setIP("brix.d.cs.uoregon.edu");
  cloudy.setPort(8330);
  cloudy.initialize();
}



void loop() {
  digitalWrite(laserPtrPin, HIGH);
  photocellReading = analogRead(photocellPin); 
  Serial.print("Analog reading = ");
  Serial.println(photocellReading);
  lightVal = photocellReading;
  Serial.println(lightVal);
  if (lightVal < blinky.getTriggerValue() || lightVal < buzzy.getTriggerValue()) {
    blinky.trigger();
    buzzy.trigger();
    cloudy.trigger();
    delay(blinky.getDuration());
    delay(buzzy.getDuration());
    blinky.stop();
    buzzy.stop();
    delay(blinky.getDuration());
    delay(buzzy.getDuration());

    if (timeStatus() != timeNotSet) {
      if (now() != prevDisplay) { //update the message only if time has changed
        prevDisplay = now();
        updateMessage();  
      }
    }

    Serial.println(timeStampedMessage);
       
  }
}

void updateMessage(){
  sprintf(timeStampedMessage,"Alarm triggered at Matt and Jacob's place [%i:%i:%i-%i/%i/%i]",second(), minute(), hour(), month(), day(), year());
}

void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
