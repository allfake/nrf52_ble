/*
 * Serial Port over BLE
 * Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.
 *
 * BLESerial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
 * replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make the
 * service discoverable all UUIDs are NUS (Nordic UART Service) compatible.
 *
 * Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
 * that the data will make it to the other end. However, under normal circumstances and reasonable signal
 * strengths everything works well.
 */
// Import libraries (BLEPeripheral depends on SPI)

#define CBC 1
#include <SPI.h>
#include <BLEPeripheral.h>
#include "BLESerial.h"
#include "aes.hpp"
#include <Uart.h>
#include <CMMC_NB_IoT.h> 

//custom boards may override default pin definitions with BLESerial(PIN_REQ, PIN_RDY, PIN_RST)
BLESerial bleSerial;

bool startReadPassword = false;
int paswordTimeout = 20000;
long passwordTimeCount = 0;
int maxPassword = 64;
int passwordCount = 0;
uint8_t password[64]; 

bool isAuth = false;
bool isUnlock = false;

// idle
int led1 = 6;
// command ready
int led2 = 7;
// unlock state
int led3 = 8;

// force lock button
int button1 = 2;

int POWER_HIGH = LOW;
int POWER_LOW = HIGH;

uint8_t devicePassword[] = { 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61 };
// {0xDD,0xAE,0x2D,0x95,0x4F,0xE3,0xDC,0x48,0x98,0x15,0x7C,0x44,0xC8,0x0D,0x6E,0xD2,0x4D,0xF8,0xD6,0xEC,0x0D,0xF8,0xA9,0x70,0x65,0xCA,0xC8,0xF3,0x9D,0xC0,0x26,0xAD,0xC6,0x36,0x14,0xA9,0x2B,0x92,0xCE,0x72,0xA0,0x7F,0xA0,0x42,0xE0,0x0D,0xAE,0xEA,0x40,0x9B,0x02,0x32,0x22,0x08,0x30,0xC5,0xA8,0x8E,0x8E,0xD9,0x14,0x52,0xD0,0x92}

// test
uint8_t key[] = { 0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe, 0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
                      0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7, 0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4 };
uint8_t iv[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };

#define  Rxpin  8          // Serial RxD pin (can be specified arbitrarily) 
#define  Txpin  9          // Serial TxD pin (can be specified arbitrarily) 
#define  baudRate 9600  // Communication speed (1200 to 921600 bps)

Uart Serial2 (NRF_UART0, UART0_IRQn, Rxpin, Txpin);

CMMC_NB_IoT nb(&Serial2);

void setup() {

//  Serial2.setPins(Rxpin, Txpin);
  Serial2.begin(baudRate);
  Serial.begin(115200);

  pinMode(button1, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  digitalWrite(led1, POWER_LOW);
  digitalWrite(led2, POWER_LOW);
  digitalWrite(led3, POWER_LOW);

  // custom services and characteristics can be added as well
  bleSerial.setLocalName("CMMC");
  bleSerial.setDeviceName("bike");
  bleSerial.begin();

  SetLock();

  /////////////////////
  //// TODO add gsm and gps

//  nb.setDebugStream(&Serial);
//  nb.onDeviceReboot([]() {
//    Serial.println(F("[user] Device rebooted."));
//    delay(2000);
//  });
//
//  nb.onDeviceReady([]() {
//    Serial.println("[user] Device Ready!");
//  });
//
//  nb.onDeviceInfo([](CMMC_NB_IoT::DeviceInfo device) {
//    Serial.print(F("# Module IMEI-->  "));
//    Serial.println(device.imei);
//    Serial.print(F("# Firmware ver-->  "));
//    Serial.println(device.firmware);
//    Serial.print(F("# IMSI SIM-->  "));
//    Serial.println(device.imsi);
//  });
//
//  nb.onMessageArrived([](char *text, size_t len, uint8_t socketId, char *ip, uint16_t port) {
//    char buffer[100];
//    sprintf(buffer, "++ [recv:] socketId=%u, ip=%s, port=%u, len=%d bytes (%lums)", socketId, ip, port, len, millis());
//    Serial.println(buffer);
//  });
//
//  nb.onConnecting([]() {
//    delay(300);
//    Serial.println("Connecting to NB-IoT...");
//    delay(300);
//  });
//
//  nb.onConnected([]() {
//
//    Serial.print("[user] NB-IoT Network connected at (");
//    Serial.print(millis());
//    Serial.println("ms)");
//    nb.createUdpSocket("103.20.205.85", 5683, UDPConfig::ENABLE_RECV);
//    delay(1000);
//  });
//
//  Serial.println("WAIT... 2s");
//  delay(2000);
//  Serial.println("Rebooting module");
//
//  nb.hello();
  // nb.rebootModule();
  
}

void loop() {
  bleSerial.poll();

  forward();
  // loopback();
  // spam();
  
  if (isUnlock == true)
  {
    if (digitalRead(button1) == POWER_HIGH)
    {
      SetLock();
    }
  }

  // read password timout
  if (startReadPassword == true && passwordTimeCount < millis())
  {
    startReadPassword = false;
    SetWaitMode();
  }
}

void SetUnLock()
{
  isUnlock = true;
  digitalWrite(led1, POWER_LOW);
  digitalWrite(led2, POWER_LOW);
  digitalWrite(led3, POWER_HIGH);

  // send OK
  bleSerial.write(79);
  bleSerial.write(75);
}

void SetLock()
{
  SetWaitMode();
  digitalWrite(led2, POWER_LOW);
  digitalWrite(led3, POWER_LOW);
  isAuth = false;
  startReadPassword = false;
}

void SetWaitMode()
{
  digitalWrite(led1, POWER_HIGH);
  digitalWrite(led2, POWER_LOW);
}

void SetInputMode()
{
  digitalWrite(led1, POWER_LOW);
  digitalWrite(led2, POWER_HIGH);
}

// forward received from Serial to BLESerial and vice versa
void forward() {
  if (bleSerial && Serial) {
    int byte;
    while ((byte = bleSerial.read()) > 0)
    {
      if (isAuth == true)
      {
        if ((char)byte == 'u')
        {
          SetUnLock();
        }
         else if ((char)byte == 'l')
        {
          SetLock();
        }
      }
    }
    while ((byte = bleSerial.read2()) > 0)
    {
      // enter input password mode
      if (startReadPassword == true)
      {
        int sizeOfdevicePassword = sizeof(devicePassword);
        if (passwordCount < maxPassword)
        {
          password[passwordCount++] = byte;
//          Serial.write((char)byte);
//          Serial.println(passwordCount);
//          Serial.println(byte);
        }

        if (passwordCount >= maxPassword)
        {
          // force end read password after get 64 bytes
          startReadPassword = false;
          passwordTimeCount = 0;

          //decrypt password;
          struct AES_ctx ctx2;
          AES_init_ctx_iv(&ctx2, key, iv);
          AES_CBC_decrypt_buffer(&ctx2, password, 64);

          if (sizeof(password) == sizeOfdevicePassword && memcmp(password, devicePassword, sizeOfdevicePassword) == 0)
          {
            //Serial.println("Password corrent");
            isAuth = true;
            SetInputMode();
          }
        }
      }

      // command | to enter password 
      if (startReadPassword == false && (char)byte == '|')
      {
        startReadPassword = true;
        memset(password, 0, 64);
        passwordCount = 0;
        passwordTimeCount = paswordTimeout + millis();
      }

    }
    while ((byte = Serial.read()) > 0) {
      bleSerial.write((char)byte);
    }
  }
}

// echo all received data back
void loopback() {
  nb.loop();
//  if (bleSerial) {
//    int byte;
//    while ((byte = bleSerial.read()) > 0) {
//      bleSerial.write(byte);
//    }
//  }
}

// periodically sent time stamps
void spam() {
  if (bleSerial) {
    bleSerial.print(millis());
    bleSerial.println(" tick-tacks!");
    delay(1000);
  }
}
