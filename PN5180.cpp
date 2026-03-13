#include <Arduino.h>
#include "PN5180.h"
#include "Debug.h"

//#define DEBUG 1

// PN5180 1-Byte Direct Commands
#define PN5180_WRITE_REGISTER           (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK   (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK  (0x02)
#define PN5180_READ_REGISTER            (0x04)
#define PN5180_WRITE_EEPROM             (0x06)
#define PN5180_READ_EEPROM              (0x07)
#define PN5180_SEND_DATA                (0x09)
#define PN5180_READ_DATA                (0x0A)
#define PN5180_SWITCH_MODE              (0x0B)
#define PN5180_MIFARE_AUTHENTICATE      (0x0C)
#define PN5180_LOAD_RF_CONFIG           (0x11)
#define PN5180_RF_ON                    (0x16)
#define PN5180_RF_OFF                   (0x17)

uint8_t PN5180::readBufferStatic16[16];

namespace {
  static void spiWriteFrameBytewise(SPIClass& spi, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      spi.transfer(data[i]);
    }
  }

  static void spiReadFrameBytewise(SPIClass& spi, uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      data[i] = spi.transfer(0xFF);
    }
  }
}

PN5180::PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi) :
  PN5180_NSS(SSpin),
  PN5180_BUSY(BUSYpin),
  PN5180_RST(RSTpin),
  PN5180_SPI(spi),
  PN5180_SCK(-1),
  PN5180_MISO(-1),
  PN5180_MOSI(-1)
{
  // Giảm từ 7MHz xuống 2MHz cho ổn định trên ESP32-S3 + PN5180
  SPI_SETTINGS = SPISettings(2000000, MSBFIRST, SPI_MODE0);
}

PN5180::~PN5180() {
  if (readBufferDynamic508) {
    free(readBufferDynamic508);
  }
}

void PN5180::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
  PN5180_SCK  = sck;
  PN5180_MISO = miso;
  PN5180_MOSI = mosi;
  if (ss >= 0) PN5180_NSS = ss;

  pinMode(PN5180_NSS, OUTPUT);
  pinMode(PN5180_BUSY, INPUT);
  pinMode(PN5180_RST, OUTPUT);

  digitalWrite(PN5180_NSS, HIGH);
  digitalWrite(PN5180_RST, HIGH);

  if ((PN5180_SCK >= 0) && (PN5180_MISO >= 0) && (PN5180_MOSI >= 0)) {
    PN5180_SPI.begin(PN5180_SCK, PN5180_MISO, PN5180_MOSI, PN5180_NSS);
  } else {
    PN5180_SPI.begin();
  }
}

void PN5180::end() {
  digitalWrite(PN5180_NSS, HIGH);
  PN5180_SPI.end();
}

void PN5180::setSPISettingsFrecuency(uint32_t frecuency) {
  SPI_SETTINGS = SPISettings(frecuency, MSBFIRST, SPI_MODE0);
}

bool PN5180::writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;
  uint8_t cmd[] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(cmd, sizeof(cmd));
}

bool PN5180::writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t cmd[] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(cmd, sizeof(cmd));
}

bool PN5180::writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t cmd[] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(cmd, sizeof(cmd));
}

bool PN5180::readRegister(uint8_t reg, uint32_t *value) {
  uint8_t cmd[] = { PN5180_READ_REGISTER, reg };
  return transceiveCommand(cmd, sizeof(cmd), (uint8_t*)value, 4);
}

bool PN5180::writeEEprom(uint8_t addr, const uint8_t *buffer, uint8_t len) {
  uint8_t cmd[len + 2];
  cmd[0] = PN5180_WRITE_EEPROM;
  cmd[1] = addr;
  for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
  return transceiveCommand(cmd, len + 2);
}

bool PN5180::readEEprom(uint8_t addr, uint8_t *buffer, int len) {
  if ((addr > 254) || ((addr + len) > 254)) {
    return false;
  }
  uint8_t cmd[] = { PN5180_READ_EEPROM, addr, uint8_t(len) };
  return transceiveCommand(cmd, sizeof(cmd), buffer, len);
}

bool PN5180::sendData(const uint8_t *data, int len, uint8_t validBits) {
  if (len > 260 || len < 0) {
    return false;
  }

  uint8_t buffer[len + 2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits;
  for (int i = 0; i < len; i++) {
    buffer[2 + i] = data[i];
  }

  // clear IRQ cũ trước khi phát
  clearIRQStatus(0xFFFFFFFF);

  // set Idle/StopCom
  if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFF8)) {
    return false;
  }

  // set Transceive
  if (!writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003)) {
    return false;
  }

  PN5180TransceiveStat transceiveState = getTransceiveState();
  if (PN5180_TS_WaitTransmit != transceiveState) {
    return false;
  }

  bool ret = transceiveCommand(buffer, len + 2);
  if (!ret) return false;

  // check nhanh general error ngay sau SEND_DATA host command
  uint32_t irq = getIRQStatus();
  if (irq & GENERAL_ERROR_IRQ_STAT) {
    return false;
  }

  return true;
}

uint8_t * PN5180::readData(int len) {
  if (len < 0 || len > 508) {
    return 0L;
  }

  uint8_t cmd[] = { PN5180_READ_DATA, 0x00 };

  uint8_t *readBuffer;
  if (len <= 16) {
    readBuffer = readBufferStatic16;
  } else {
    if (!readBufferDynamic508) {
      readBufferDynamic508 = (uint8_t *) malloc(508);
      if (!readBufferDynamic508) {
        return 0;
      }
    }
    readBuffer = readBufferDynamic508;
  }

  if (!transceiveCommand(cmd, sizeof(cmd), readBuffer, len)) {
    return 0;
  }
  return readBuffer;
}

bool PN5180::readData(int len, uint8_t *buffer) {
  if (len < 0 || len > 508) {
    return false;
  }
  uint8_t cmd[] = { PN5180_READ_DATA, 0x00 };
  return transceiveCommand(cmd, sizeof(cmd), buffer, len);
}

bool PN5180::prepareLPCD() {
  uint8_t data[255];
  uint8_t response[256];

  uint8_t fieldOn = 0xF0;
  data[0] = fieldOn;
  writeEEprom(0x36, data, 1);
  readEEprom(0x36, response, 1);

  uint8_t threshold = 0x03;
  data[0] = threshold;
  writeEEprom(0x37, data, 1);
  readEEprom(0x37, response, 1);

  uint8_t lpcdMode = 0x01;
  data[0] = lpcdMode;
  writeEEprom(0x38, data, 1);
  readEEprom(0x38, response, 1);

  uint8_t beforeFieldOn = 0xF0;
  data[0] = beforeFieldOn;
  writeEEprom(0x39, data, 1);
  readEEprom(0x39, response, 1);

  uint8_t afterFieldOn = 0xF0;
  data[0] = afterFieldOn;
  writeEEprom(0x3A, data, 1);
  readEEprom(0x3A, response, 1);

  delay(100);
  return true;
}

bool PN5180::switchToLPCD(uint16_t wakeupCounterInMs) {
  clearIRQStatus(0xFFFFFFFF);
  writeRegister(IRQ_ENABLE, LPCD_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);
  uint8_t cmd[] = {
    PN5180_SWITCH_MODE,
    0x01,
    (uint8_t)(wakeupCounterInMs & 0xFF),
    (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF)
  };
  return transceiveCommand(cmd, sizeof(cmd));
}

int16_t PN5180::mifareAuthenticate(uint8_t blockNo, const uint8_t *key, uint8_t keyType, const uint8_t *uid) {
  if (keyType != 0x60 && keyType != 0x61) {
    return -2;
  }

  uint8_t cmdBuffer[13];
  uint8_t rcvBuffer[1] = {0x02};

  cmdBuffer[0] = PN5180_MIFARE_AUTHENTICATE;
  for (int i = 0; i < 6; i++) cmdBuffer[i + 1] = key[i];
  cmdBuffer[7] = keyType;
  cmdBuffer[8] = blockNo;
  for (int i = 0; i < 4; i++) cmdBuffer[9 + i] = uid[i];

  bool retval = transceiveCommand(cmdBuffer, 13, rcvBuffer, 1);
  if (!retval) return -3;

  return rcvBuffer[0];
}

bool PN5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };
  return transceiveCommand(cmd, sizeof(cmd));
}

bool PN5180::setRF_on() {
  uint8_t cmd[] = { PN5180_RF_ON, 0x00 };
  if (!transceiveCommand(cmd, sizeof(cmd))) return false;

  unsigned long startedWaiting = millis();
  while (0 == (TX_RFON_IRQ_STAT & getIRQStatus())) {
    delay(1);
    if (millis() - startedWaiting > 500) {
      return false;
    }
  }

  clearIRQStatus(TX_RFON_IRQ_STAT);
  return true;
}

bool PN5180::setRF_off() {
  uint8_t cmd[] = { PN5180_RF_OFF, 0x00 };
  if (!transceiveCommand(cmd, sizeof(cmd))) return false;

  unsigned long startedWaiting = millis();
  while (0 == (TX_RFOFF_IRQ_STAT & getIRQStatus())) {
    delay(1);
    if (millis() - startedWaiting > 500) {
      return false;
    }
  }

  clearIRQStatus(TX_RFOFF_IRQ_STAT);
  return true;
}

bool PN5180::sendCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  return transceiveCommand(sendBuffer, sendBufferLen, recvBuffer, recvBufferLen);
}

bool PN5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  PN5180_SPI.beginTransaction(SPI_SETTINGS);

  // ---------- WRITE FRAME ----------
  unsigned long startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      PN5180_SPI.endTransaction();
      digitalWrite(PN5180_NSS, HIGH);
      return false;
    }
  }

  digitalWrite(PN5180_NSS, LOW);
  delayMicroseconds(5);

  spiWriteFrameBytewise(PN5180_SPI, sendBuffer, sendBufferLen);

  startedWaiting = millis();
  while (HIGH != digitalRead(PN5180_BUSY)) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      PN5180_SPI.endTransaction();
      digitalWrite(PN5180_NSS, HIGH);
      return false;
    }
  }

  digitalWrite(PN5180_NSS, HIGH);
  delayMicroseconds(5);

  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      PN5180_SPI.endTransaction();
      digitalWrite(PN5180_NSS, HIGH);
      return false;
    }
  }

  // write-only
  if ((recvBuffer == 0) || (recvBufferLen == 0)) {
    PN5180_SPI.endTransaction();
    digitalWrite(PN5180_NSS, HIGH);
    return true;
  }

  // ---------- READ FRAME ----------
  digitalWrite(PN5180_NSS, LOW);
  delayMicroseconds(5);

  memset(recvBuffer, 0x00, recvBufferLen);
  spiReadFrameBytewise(PN5180_SPI, recvBuffer, recvBufferLen);

  startedWaiting = millis();
  while (HIGH != digitalRead(PN5180_BUSY)) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      PN5180_SPI.endTransaction();
      digitalWrite(PN5180_NSS, HIGH);
      return false;
    }
  }

  digitalWrite(PN5180_NSS, HIGH);
  delayMicroseconds(5);

  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      PN5180_SPI.endTransaction();
      digitalWrite(PN5180_NSS, HIGH);
      return false;
    }
  }

  PN5180_SPI.endTransaction();
  return true;
}

void PN5180::reset() {
  digitalWrite(PN5180_RST, LOW);
  delay(1);
  digitalWrite(PN5180_RST, HIGH);
  delay(5);

  unsigned long startedWaiting = millis();
  while (0 == (IDLE_IRQ_STAT & getIRQStatus())) {
    delay(1);
    if (millis() - startedWaiting > commandTimeout) {
      digitalWrite(PN5180_RST, LOW);
      delay(10);
      digitalWrite(PN5180_RST, HIGH);
      delay(50);
      return;
    }
  }
}

uint32_t PN5180::getIRQStatus() {
  uint32_t irqStatus = 0;
  readRegister(IRQ_STATUS, &irqStatus);
  return irqStatus;
}

bool PN5180::clearIRQStatus(uint32_t irqMask) {
  return writeRegister(IRQ_CLEAR, irqMask);
}

PN5180TransceiveStat PN5180::getTransceiveState() {
  uint32_t rfStatus = 0;
  if (!readRegister(RF_STATUS, &rfStatus)) {
    return PN5180_TS_Idle;
  }

  uint8_t state = ((rfStatus >> 24) & 0x07);
  return PN5180TransceiveStat(state);
}