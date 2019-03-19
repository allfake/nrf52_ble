#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

#include <Arduino.h>
#include <BLEPeripheral.h>

class BLESerial : public BLEPeripheral, public Stream
{
  public:
    BLESerial(unsigned char req = BLE_DEFAULT_REQ, unsigned char rdy = BLE_DEFAULT_RDY, unsigned char rst = BLE_DEFAULT_RST);

    void begin(...);
    void poll();
    void end();

    virtual int available(void);
    virtual int available2(void);
    virtual int peek(void);
    virtual int read(void);
    virtual int read2(void);
    virtual void flush(void);
    virtual size_t write(uint8_t byte);
    using Print::write;
    virtual operator bool();

  private:
    unsigned long _flushed;
    static BLESerial* _instance;

    size_t _rxHead;
    size_t _rxTail;
    size_t _rxCount() const;
    uint8_t _rxBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];

    size_t _rx2Head;
    size_t _rx2Tail;
    size_t _rx2Count() const;
    uint8_t _rx2Buffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];
    
    size_t _txCount;
    uint8_t _txBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];

    BLEService _uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor _uartNameDescriptor = BLEDescriptor("2901", "UART");
    BLECharacteristic _rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Write)");

    BLECharacteristic _rx2Characteristic = BLECharacteristic("80BF2A15-73BD-465E-A80F-C8C910821495", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _rx2NameDescriptor = BLEDescriptor("2901", "RX2 - Receive Data (Write)");
    
    BLECharacteristic _txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Notify)");

    void _received(const uint8_t* data, size_t size);
    static void _received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic);

    void _received2(const uint8_t* data, size_t size);
    static void _received2(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic);
};

#endif
