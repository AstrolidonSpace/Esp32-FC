#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>


// Wi-Fi AP settings
const char *ssid = "ESP-DRONE_F09E9E21DB55";
const char *password = "12345678";
const IPAddress apIP(192, 168, 43, 42);

// UDP settings
const int udpPort = 2390;
WiFiUDP udp;
byte udpBuffer[31];

// Control variables
float RateRoll, RatePitch, RateYaw, RCR, RCP, RCY, Input_Pitch = 0, Input_Roll = 0, Input_Yaw = 0, DRR, DRP, DRY, PERR, PERP, PERY, PIRR, PIRP, PIRY, PIDReturn[] = {0, 0, 0},Input_Throttle=0,V_max=400.0,V_min=-400.0,ERP,ERR,ERY,IRoll,IPitch,IYaw;
float PRateRoll = 0.6, PRatePitch = 0.6, PRateYaw = 2.0;
float IRateRoll = 3.5, IRatePitch = 3.5, IRateYaw = 12.0;
float DRateRoll = 0.03, DRatePitch = 0.03, DRateYaw = 0.0;
int RCN;
uint16_t thrust = 0;
float M1, M2, M3, M4;
unsigned long previousMillis = 0,lastPacketTime = 0;
const unsigned long interval = 4,disconnectionTimeout = 900;

int controlLED = 2;

// setpoint_t structure
typedef struct {
 float roll;
 float pitch;
 float yaw;
 uint16_t thrust;
} setpoint_t;

// meta command enum
enum metaCommand_e {
 metaNotifySetpointsStop = 0,
 nMetaCommands,
};

// decoder function
void notifySetpointsStopDecoder(const void *data, size_t datalen) {
 Serial.println("Stop command received");
 // Implement stop logic here
}

// meta command decoder array
typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);
const static metaCommandDecoder_t metaCommandDecoders[] = {
 [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};

// setpoint decoding function
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, byte *data) {
 memcpy(&setpoint->roll, data, 4);
 memcpy(&setpoint->pitch, data + 4, 4);
 memcpy(&setpoint->yaw, data + 8, 4);
 memcpy(&setpoint->thrust, data + 12, 2);
 setpoint->pitch *= -1;
}

void setupWiFi() {
 WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
 WiFi.softAP(ssid, password);
 Serial.print("AP IP address: ");
 Serial.println(WiFi.softAPIP());
 udp.begin(udpPort);
}

void gyro_signals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void processUdpData() {
 int packetSize = udp.parsePacket();
 if (packetSize >= 1) {
  udp.read(udpBuffer, packetSize);

  byte header = udpBuffer[0];
  byte port = (header >> 4) & 0x0F;
  byte channel = header & 0x03;

  if (port == 0x03 || (port == 0x07 && channel == 0)) { // CRTP_PORT_SETPOINT or GENERIC setpoint
   setpoint_t setpoint;
   crtpCommanderGenericDecodeSetpoint(&setpoint, &udpBuffer[1]);

   Input_Roll = setpoint.roll;
   Input_Pitch = setpoint.pitch;
   Input_Yaw = setpoint.yaw;
   thrust = setpoint.thrust;

   /*Serial.print("Roll: "); Serial.print(Input_Roll);
   Serial.print(", Pitch: "); Serial.print(Input_Pitch);
   Serial.print(", Yaw: "); Serial.print(Input_Yaw);
   Serial.print(" ");
   Serial.print(udpBuffer[13]);*/
   
   lastPacketTime = millis();

   // Scale thrust to 0-100 range
   float thrustPercentage = (float)thrust / 65535.0 * 100.0;

   // Range check
   if (thrust > 65535) {
    Serial.println("Warning: Thrust value out of range!");
   }

   // Add your drone control logic here, using roll, pitch, yaw, and thrust.
  } else if (port == 0x07 && channel == 1) { // META_COMMAND_CHANNEL
    Serial.println("connection Lost");
   if (packetSize >= 2) {
    uint8_t metaCmd = udpBuffer[1];
    if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
     metaCommandDecoders[metaCmd](&udpBuffer[2], packetSize - 2);
    }
   }
  }
 }
}

void pid_equetion(float Error,float P,float I,float D,float PrevError,float PrevItem){
  float Pterm = P * Error;
  float Iterm = PrevItem + I * (Error + PrevError) * 0.004 / 2;
  Iterm = max(V_min, Iterm);
  Iterm = min(V_max, Iterm);
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain( PIDOutput,V_min,V_max);
  // PIDOutput = min(V_max, PIDOutput);
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void){
  PERR = PERP = PERY = PIRR = PIRP = PIRY = 0;
}

void setup() {
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime) < 5000) {
    delay(10); // Small delay to prevent busy-waiting and allow USB enumeration
  }
 Serial.begin(115200);
 Serial.println("ok");
 setupWiFi();
 pinMode(controlLED, OUTPUT); // Configure GPIO 2 as output

  //Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.print("Calibration Started Wait for Three Seconds...");
  digitalWrite(controlLED,HIGH);
  for(RCN = 0; RCN < 3000;RCN++){
    gyro_signals();
    RCR += RateRoll;
    RCP += RatePitch;
    RCY += RateYaw;
    delay(1);
  }
  RCR /= 3000;
  RCP /= 3000;
  RCY /= 3000;
  Serial.println("Ready");
  digitalWrite(controlLED, LOW);

  ledcAttach(0,250,12);
  ledcAttach(1,250,12);
  ledcAttach(2,250,12);
  ledcAttach(3,250,12);


 /* pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);*/
  unsigned long currentMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  processUdpData();

  float TV;
  if(thrust == 0){
    TV = 0;
  }else if(thrust >= 0){
    TV = 1;
  }

  if(currentMillis - lastPacketTime > disconnectionTimeout){
    M1 = M2 = M3 = M4 = 0;
    digitalWrite(controlLED, HIGH);
    if (currentMillis - lastPacketTime > 2000)
    {
      reset_pid();
    }
    
  }else{
    digitalWrite(controlLED, LOW);
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      gyro_signals();
      RateRoll -= RCR;
      RatePitch -= RCP;
      RateYaw -= RCY;

      DRR = Input_Roll;
      DRP = Input_Pitch;
      DRY = Input_Yaw;
      Input_Throttle = ((float)thrust * 4000.00 / 65535);

      ERR = DRR - RateRoll;
      ERP = DRP - RatePitch;
      ERY = DRY - RateYaw;

      pid_equetion(ERR, PRateRoll, IRateRoll, DRateRoll, PERR, PIRR);
      IRoll = PIDReturn[0];
      PERR = PIDReturn[1];
      PIRR = PIDReturn[2];
      pid_equetion(ERP, PRatePitch, IRatePitch, DRatePitch, PERP, PIRP);
      IPitch = PIDReturn[0];
      PERP = PIDReturn[1];
      PIRP = PIDReturn[2];
      pid_equetion(ERY, PRateYaw, IRateYaw, DRateYaw, PERY, PIRY);
      IYaw = PIDReturn[0];
      PERY = PIDReturn[1];
      PIRY = PIDReturn[2];

      M1 = TV*1.024 * (Input_Throttle - IRoll - IPitch - IYaw);
      M2 = TV*1.024 * (Input_Throttle - IRoll + IPitch + IYaw);
      M3 = TV*1.024 * (Input_Throttle + IRoll + IPitch - IYaw);
      M4 = TV*1.024 * (Input_Throttle + IRoll - IPitch + IYaw);
  }
 }
    /*Reset PID when drone is reset
    ledcWrite(0, M1);
    ledcWrite(1, M2);
    ledcWrite(2, M3);
    ledcWrite(3, M4);*/

    Serial.print("M1: "); Serial.print(M1);
    Serial.print(", M2: "); Serial.print(M2);
    Serial.print(", M3: "); Serial.print(M3);
    Serial.print(", M4: "); Serial.println(M4); 
     
}