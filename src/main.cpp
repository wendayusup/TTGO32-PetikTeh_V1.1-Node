/* NODE MESIN PETIK V1.1
   WENDA YUSUP
   PT.MAKERINDO PRIMA SOLUSI*/

#include <Arduino.h>

//----------------Library & Kebutuhan LoRa
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#define SS 18
#define RST 14
#define DIO0 26
#define SCK 5
#define MISO 19
#define MOSI 27

//-----------------Library & Kebutuhan DHT22
#include <DHT.h>
#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//----------------Library & Kebutuhan MPU9250
#include "sub_module/MPU9250_RPY.h"
#include "sub_module/MPU9250.h"
extern MPU9250 IMU;
int roll_calibrate = 0;
int pitch_calibrate = 0;
int yaw_calibrate = 0;
int roll_c = 0;
int pitch_c = 0;
int yaw_c = 0;

//-----------------Library & Kebutuhan Battery
#include <axp20x.h>
AXP20X_Class axp;
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;

//-----------------Library & Kebutuhan GPS
#include <TinyGPS++.h>
#define RX 34
#define TX 12
HardwareSerial neogps(1);
TinyGPSPlus gps;

//-----------------Definisi LED
#define ledgps 13
#define ledlora 25
#define ledkiri 15
#define ledkanan 2

//-----------------MILLIS
unsigned long int awal = millis();
unsigned long sekarang;
unsigned long int sec = 0;
int interval = 1;
int Secs = 0;

//-----------------Library & Kebutuhan Matematika
#include <math.h>
// Distance
#define R 6371 // Earth Radius
#define torad (3.1415926536 / 180)
double distance(double lat1, double long1, double lat2, double long2)
{
  double dx, dy, dz;
  long1 -= long2;
  long1 *= torad, lat1 *= torad, lat2 *= torad;
  dz = sin(lat1) - sin(lat2);
  dx = cos(long1) * cos(lat1) - cos(lat2);
  dy = sin(long1) * cos(lat1);
  return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R * 1000; // *1000 for metres
}
struct latlong
{
  double latitude;
  double longitude;
};

// Wind Direction
float bearing(float lat, float lon, float lat2, float lon2)
{
  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2 - lat);
  float delta2 = radians(lon2 - lon);

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng); // radians to degrees
  brng = (((int)brng + 360) % 360);

  // Serial.print("Heading GPS: ");
  // Serial.println(brng);

  return brng;
}

//--------------------------------------------------PAYLOAD
String payloadA = "";
String payloadB = "";

// IDENTITAS
String ID = "MPGMBG0823001";
String VBAT = "0";
String Latitude = "0";
String Longitude = "0";
String Altitude = "0";
String Suhu = "0";
String Kelembaban = "0";
extern int roll;
extern int pitch;
extern int yaw;
int strengthrpt = -200;
int strengthnode1 = -200;
int strengthnode3 = -200;
int strengthnode4 = -200;
int strengthnode5 = -200;
int strengthnode6 = -200;
int strengthnode7 = -200;
int strengthnode8 = -200;
int strengthnode9 = -200;
int strengthnode10 = -200;
bool node2 = true;
bool node3 = false;
bool node4 = false;
bool node5 = false;
bool node6 = false;
bool node7 = false;
bool node8 = false;
bool node9 = false;
bool node10 = false;

String message = "";
String outgoing;
byte msgCount = 0;
byte Nodegateway = 0xC0;
byte NoderepeaterA = 0xB0;
byte NoderepeaterB = 0xB1;
// PILIH BYTE SESUAI DEVICE
byte NodeNetral = 0x00;
byte Nodepetik1 = 0xA0;
byte Nodepetik2 = 0xA1;
byte Nodepetik3 = 0xA2;
byte Nodepetik4 = 0xA3;
byte Nodepetik5 = 0xA4;
byte Nodepetik6 = 0xA5;
byte Nodepetik7 = 0xA6;
byte Nodepetik8 = 0xA7;
byte Nodepetik9 = 0xA8;
byte Nodepetik10 = 0xA9;

void setup()
{
  // Serial begin
  Serial.begin(115200);

  // GPS begin
  neogps.begin(9600, SERIAL_8N1, RX, TX);

  // SPI begin & LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  Wire.begin();

  // DHT begin
  dht.begin();

  // Battery Begin
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    ;
  {
    Serial.println("Battery Failed");
  }

  // LoRa Setting
  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa Failed");
    while (true)
      ;
  }
  LoRa.setSpreadingFactor(11);
  LoRa.setCodingRate4(2);
  LoRa.setSignalBandwidth(500E3);

  // MPU Begin
  MPU9250Setup();
  while (millis() < 5000)
  {

    MPU9250Loop();

    roll_calibrate = roll;
    pitch_calibrate = pitch;
    yaw_calibrate = yaw;
  }

  pinMode(ledgps, OUTPUT);
  pinMode(ledlora, OUTPUT);
  pinMode(ledkiri, OUTPUT);
  pinMode(ledkanan, OUTPUT);
}

// SENDING REPEATER A/
void sendMessage(String outgoing, byte NoderepeaterA, byte othernode)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(NoderepeaterA);     // add destination address
  LoRa.write(Nodepetik1);        // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length
  String incoming = "";
  while (LoRa.available())
  {
    incoming = LoRa.readStringUntil('\n');
  }
  if (incomingLength != incoming.length())
  { // check length for error
    // Serial.println("error: message length does not match length");
    ;
    return; // skip rest of function
  }
  // if the recipient isn't this devicerecipient != NodeNetral or broadcast,
  if (recipient != Nodepetik1 && recipient != NoderepeaterA)
  {
    // Serial.println("This message is not for me.");
    ;
    return; // skip rest of function
  }

  if (incoming.startsWith("MPGMBG0823001"))
  {
    // Serial.println(incoming);
    // sendMessage(incoming,NoderepeaterA,Nodepetik2);
  }
  else if (incoming.startsWith("MPGMBG0823002"))
  {
    // Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823003"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823004"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823005"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823006"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823007"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823008"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823009"))
  {
    Serial.println(incoming);
  }
  else if (incoming.startsWith("MPGMBG0823010"))
  {
    Serial.println(incoming);
  }

  int Val = incoming.toInt();
  if (Val == 11)
  {
    strengthrpt = LoRa.packetRssi();
    Serial.println(incoming + "  " + strengthrpt);
    sendMessage(payloadA,NoderepeaterA,Nodepetik1);
  }
  // else if (Val == 21)
  // {
  //   strengthnode1 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode1);
  //   // sendMessage(payloadA,NoderepeaterA,Nodepetik1);
  // }
  // else if (Val == 23)
  // {
  //   strengthnode3 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode3);
  // }
  // else if (Val == 24)
  // {
  //   strengthnode4 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode4);
  // }
  // else if (Val == 25)
  // {
  //   strengthnode5 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode5);
  // }
  // else if (Val == 26)
  // {
  //   strengthnode6 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode6);
  // }
  // else if (Val == 27)
  // {
  //   strengthnode7 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode7);
  // }
  // else if (Val == 28)
  // {
  //   strengthnode8 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode8);
  // }
  // else if (Val == 29)
  // {
  //   strengthnode9 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode9);
  // }
  // else if (Val == 210)
  // {
  //   strengthnode10 = LoRa.packetRssi();
  //   Serial.println(incoming + "  " + strengthnode10);
  // }
}

void gpsdata()
{
  while (neogps.available())
    if (gps.encode(neogps.read()))
      if (gps.location.isValid())
      {
        Latitude = String(gps.location.lat(), 6);
        Longitude = String(gps.location.lng(), 6);
        Altitude = String(gps.altitude.meters() - 25.00);
        digitalWrite(ledgps, HIGH);
        delay(100);
        digitalWrite(ledgps, LOW);
      }
      else
      {
        digitalWrite(ledgps, LOW);
      }
}

void dhtdata()
{
  Suhu = dht.readTemperature();
  Kelembaban = dht.readHumidity();
  // Serial.print(Suhu);
  // Serial.println(Kelembaban);
}

void mpudata()
{
  // Mulai menjalankan MPU9250
  MPU9250Loop();
  roll = roll - roll_calibrate;
  pitch = pitch - pitch_calibrate;
  yaw = yaw - yaw_calibrate;
  Serial.println(pitch);
  if (pitch < -3)
  {
    digitalWrite(ledkiri, HIGH);
  }
  else
  {
    digitalWrite(ledkiri, LOW);
  }
  if (pitch > 3)
  {
    digitalWrite(ledkanan, HIGH);
  }
  else
  {
    digitalWrite(ledkanan, LOW);
  }
}

void batterydata()
{
  VBAT = axp.getBattVoltage();
}

void loop()
{

  
//------DHT
dhtdata();

//------GPS
gpsdata();

//------MPU
// mpudata();

//------Battery
batterydata();

latlong nodepetik = {atof(Latitude.c_str()), atof(Longitude.c_str())};
// Distance & Bearing Node to RepeaterA/
latlong repeaterA = {-7.143875, 107.515526}; // Koordinat Repeater
int JarakRA = static_cast<int>(distance(nodepetik.latitude, nodepetik.longitude, repeaterA.latitude, repeaterA.longitude));
int ArahRA = static_cast<int>(bearing(nodepetik.latitude, nodepetik.longitude, repeaterA.latitude, repeaterA.longitude));
// // /Distance & Bearing Node to RepeaterB/
// String LatitudeRB;
// String LongitudeRB;
// latlong repeaterB = {-6.967658, 107.659067}; // Koordinat Alun-alun Cimahi
// int JarakRB = static_cast<int>(distance(nodepetik.latitude, nodepetik.longitude, repeaterB.latitude, repeaterB.longitude));
// int ArahRB = static_cast<int>(bearing(nodepetik.latitude, nodepetik.longitude, repeaterB.latitude, repeaterB.longitude));

// Distance & Bearing Node to Gateway/
latlong gateway = {-7.143831, 107.515966}; // Koordinat Gateway
int JarakG = static_cast<int>(distance(nodepetik.latitude, nodepetik.longitude, gateway.latitude, gateway.longitude));
int ArahG = static_cast<int>(bearing(nodepetik.latitude, nodepetik.longitude, gateway.latitude, gateway.longitude));

if (nodepetik.latitude == 0 && nodepetik.longitude == 0)
{
  JarakG = 0;
  ArahG = 0;

  JarakRA = 0;
  ArahRA = 0;

  // JarakRB = 0;
  // ArahRB = 0;
}

payloadA = String() +
           ID +
           "," +
           Latitude +
           "," +
           Longitude +
           "," +
           Altitude +
           "," +
           Suhu +
           "," +
           Kelembaban +
           "," +
           roll +
           "," +
           pitch +
           "," +
           yaw +
           "," +
           VBAT +
           ",*";

onReceive(LoRa.parsePacket());
}