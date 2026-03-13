#ifndef PN5180_H
#define PN5180_H

#include <SPI.h>

// PN5180 Registers
#define SYSTEM_CONFIG       (0x00)
#define IRQ_ENABLE          (0x01)
#define IRQ_STATUS          (0x02)
#define IRQ_CLEAR           (0x03)
#define TRANSCEIVE_CONTROL  (0x04)
#define TIMER1_RELOAD       (0x0c)
#define TIMER1_CONFIG       (0x0f)
#define RX_WAIT_CONFIG      (0x11)
#define CRC_RX_CONFIG       (0x12)
#define RX_STATUS           (0x13)
#define TX_WAIT_CONFIG      (0x17)
#define TX_CONFIG           (0x18)
#define CRC_TX_CONFIG       (0x19)
#define RF_STATUS           (0x1d)
#define SYSTEM_STATUS       (0x24)
#define TEMP_CONTROL        (0x25)
#define AGC_REF_CONFIG      (0x26)

// PN5180 EEPROM Addresses
#define DIE_IDENTIFIER      (0x00)
#define PRODUCT_VERSION     (0x10)
#define FIRMWARE_VERSION    (0x12)
#define EEPROM_VERSION      (0x14)
#define IRQ_PIN_CONFIG      (0x1A)

// PN5180 EEPROM Addresses - LPCD
#define DPC_XI              (0x5C)

#define LPCD_REFERENCE_VALUE            (0x34)
#define LPCD_FIELD_ON_TIME              (0x36)
#define LPCD_THRESHOLD                  (0x37)
#define LPCD_REFVAL_GPO_CONTROL         (0x38)
#define LPCD_GPO_TOGGLE_BEFORE_FIELD_ON (0x39)
#define LPCD_GPO_TOGGLE_AFTER_FIELD_ON  (0x3A)

enum PN5180TransceiveStat {
  PN5180_TS_Idle = 0,
  PN5180_TS_WaitTransmit = 1,
  PN5180_TS_Transmitting = 2,
  PN5180_TS_WaitReceive = 3,
  PN5180_TS_WaitForData = 4,
  PN5180_TS_Receiving = 5,
  PN5180_TS_LoopBack = 6,
  PN5180_TS_RESERVED = 7
};

// PN5180 IRQ_STATUS bits
#define RX_IRQ_STAT               (1 << 0)
#define TX_IRQ_STAT               (1 << 1)
#define IDLE_IRQ_STAT             (1 << 2)
#define RFOFF_DET_IRQ_STAT        (1 << 6)
#define RFON_DET_IRQ_STAT         (1 << 7)
#define TX_RFOFF_IRQ_STAT         (1 << 8)
#define TX_RFON_IRQ_STAT          (1 << 9)
#define RX_SOF_DET_IRQ_STAT       (1 << 14)
#define GENERAL_ERROR_IRQ_STAT    (1 << 17)
#define LPCD_IRQ_STAT             (1 << 19)

#define MIFARE_CLASSIC_KEYA 0x60
#define MIFARE_CLASSIC_KEYB 0x61

class PN5180 {
private:
  uint8_t PN5180_NSS;
  uint8_t PN5180_BUSY;
  uint8_t PN5180_RST;
  SPIClass& PN5180_SPI;
  int8_t PN5180_SCK;
  int8_t PN5180_MISO;
  int8_t PN5180_MOSI;

  SPISettings SPI_SETTINGS;
  static uint8_t readBufferStatic16[16];
  uint8_t* readBufferDynamic508 = NULL;

public:
  PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi = SPI);
  ~PN5180();

  void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t SSpin = -1);
  void end();
  void setSPISettingsFrecuency(uint32_t frecuency);

  // direct commands
  bool writeRegister(uint8_t reg, uint32_t value);
  bool writeRegisterWithOrMask(uint8_t addr, uint32_t mask);
  bool writeRegisterWithAndMask(uint8_t addr, uint32_t mask);
  bool readRegister(uint8_t reg, uint32_t *value);

  bool writeEEprom(uint8_t addr, const uint8_t *buffer, uint8_t len);
  bool readEEprom(uint8_t addr, uint8_t *buffer, int len);

  bool sendData(const uint8_t *data, int len, uint8_t validBits = 0);
  uint8_t * readData(int len);
  bool readData(int len, uint8_t *buffer);

  bool prepareLPCD();
  bool switchToLPCD(uint16_t wakeupCounterInMs);
  int16_t mifareAuthenticate(uint8_t blockno, const uint8_t *key, uint8_t keyType, const uint8_t *uid);
  bool loadRFConfig(uint8_t txConf, uint8_t rxConf);

  bool setRF_on();
  bool setRF_off();

  bool sendCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);

  // helpers
  void reset();

  uint16_t commandTimeout = 1500;   // tăng timeout host-interface cho ESP32-S3
  uint32_t getIRQStatus();
  bool clearIRQStatus(uint32_t irqMask);

  PN5180TransceiveStat getTransceiveState();

private:
  bool transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen,
                         uint8_t *recvBuffer = 0, size_t recvBufferLen = 0);
};

#endif /* PN5180_H */