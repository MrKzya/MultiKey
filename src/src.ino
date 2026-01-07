#include <U8g2lib.h>
#include <Wire.h>
#include "OneWireHub.h"
#include "DS2401.h" 
#include <OneWire.h> 
#include <avr/pgmspace.h> 
#include <EEPROM.h> 

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define ANTENNA 7        
#define FreqGen 11       
#define PIN_RFID_RX 2    
#define PIN_EMUL_IB 10   
#define PIN_READ_IB 12   

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC_VAL  0x55 
#define EEPROM_IBTN_CNT_ADDR 2
#define EEPROM_IBTN_START    10
#define EEPROM_RFID_CNT_ADDR 3
#define EEPROM_RFID_START    420 

auto hub = OneWireHub(PIN_EMUL_IB);
enum emRWType {rwUnknown, TM01, RW1990_1, RW1990_2, TM2004};

char txtBuf[32]; 

struct KeyData {
  const char* name; 
  uint64_t id;
};

const char n_Uni[] PROGMEM = "Universal";
const char n_FFF[] PROGMEM = "FFFF";

const KeyData rfidKeys[] = {
  {n_Uni, 0xFFFFFFFFFF}
};
const uint8_t rfidCount = sizeof(rfidKeys) / sizeof(rfidKeys[0]);

const KeyData ibuttonKeys[] = {
  {n_FFF, 0x01FFFFFFFFFFFF2F}
};
const uint8_t ibtnCount = sizeof(ibuttonKeys) / sizeof(ibuttonKeys[0]);

const char m_Main0[] PROGMEM = "RFID 125kHz";
const char m_Main1[] PROGMEM = "iButton";
const char m_Main2[] PROGMEM = "Utils";
const char m_Back[]  PROGMEM = "Back";

const char m_Sub0[] PROGMEM = "Read";
const char m_Sub1[] PROGMEM = "Emulate";
const char m_Sub2[] PROGMEM = "Write";

const char m_Util0[] PROGMEM = "[Clear EEPROM]";
const char m_Util1[] PROGMEM = "[Clear RFID keys]";
const char m_Util2[] PROGMEM = "[Clear iBut keys]";

const char* const mainMenu[] PROGMEM = {m_Main0, m_Main1, m_Main2, m_Back};
const char* const subMenu[]  PROGMEM = {m_Sub0, m_Sub1, m_Sub2, m_Back};
const char* const utilMenu[] PROGMEM = {m_Util0, m_Util1, m_Util2, m_Back};

const uint8_t btnUp = 4;    
const uint8_t btnOk = 3;    
const uint8_t btnDown = 8;  

uint8_t screenState = 0; 
uint8_t cursorPosition = 0; 
uint8_t menuOffset = 0; 
bool needUpdate = true; 
unsigned long lastPressTime = 0; 
bool okWasPressed = false; 
uint8_t savedIbtnCount = 0; 
uint8_t savedRfidCount = 0;

volatile uint8_t bit_counter = 0;
volatile uint8_t byte_counter = 0;
volatile uint8_t half = 0;
volatile uint8_t rfidData[8];
volatile bool isEmulating = false; 

#define RFID_THRESHOLD 380 
volatile unsigned long lastChange = 0;
volatile uint16_t pulses[150]; 
volatile int pulseCount = 0;
volatile bool captured = false;

uint64_t lastReadID_u64 = 0; 

void runiButtonEmulation(uint8_t keyIndex);
void runRFIDEmulation(uint8_t keyIndex);
void runiButtonRead();
void runiButtonWrite(uint8_t keyIndex);
void runRFIDWrite(uint8_t keyIndex);
void runRFIDRead(); 
void runClearEEPROM();
void runClearRFIDKeys();
void runCleariButtonKeys();
void writeEM4100(byte keyID[5]);

void drawCentered(int y, const char* str) {
  int w = u8g2.getStrWidth(str);
  u8g2.drawStr((128 - w) / 2, y, str);
}

void printHexByte(byte b, int x, int y) {
  const char* hexChars = "0123456789ABCDEF";
  txtBuf[0] = hexChars[(b >> 4) & 0xF];
  txtBuf[1] = hexChars[b & 0xF];
  txtBuf[2] = '\0';
  u8g2.drawStr(x, y, txtBuf);
}

void drawIDRow(uint64_t id, int bytesCount, int y) {
    int xStart = (bytesCount == 5) ? 20 : 28; 
    int spacing = 18;
    if (bytesCount <= 5) {
        for (int i = bytesCount - 1; i >= 0; i--) {
            byte b = (byte)(id >> (i * 8));
            printHexByte(b, xStart + (bytesCount - 1 - i) * spacing, y);
        }
    }
}

void fetchKeyData(bool isRFID, int index, uint64_t &id, char* nameBuffer) {
    int builtInCount = isRFID ? rfidCount : ibtnCount;
    if (index < builtInCount) {
        const KeyData* k = isRFID ? &rfidKeys[index] : &ibuttonKeys[index];
        id = k->id;
        strcpy_P(nameBuffer, k->name);
    } else {
        int eepromIdx = index - builtInCount;
        int startAddr = isRFID ? EEPROM_RFID_START : EEPROM_IBTN_START;
        int addr = startAddr + (eepromIdx * 8);
        byte data[8];
        for (int i=0; i<8; i++) EEPROM.get(addr + i, data[i]);
        id = 0;
        for(int i=0; i<8; i++) id = (id << 8) | data[i];
        sprintf(nameBuffer, isRFID ? "Saved RFID %d" : "Saved Key %d", eepromIdx + 1);
    }
}

uint8_t onewire_crc8(const uint8_t *addr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

void initEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    EEPROM.write(EEPROM_IBTN_CNT_ADDR, 0); 
    EEPROM.write(EEPROM_RFID_CNT_ADDR, 0); 
  }
  savedIbtnCount = EEPROM.read(EEPROM_IBTN_CNT_ADDR);
  savedRfidCount = EEPROM.read(EEPROM_RFID_CNT_ADDR);
}

void saveToEEPROM(bool isRFID, byte* data8, uint64_t id64) {
    int cntAddr = isRFID ? EEPROM_RFID_CNT_ADDR : EEPROM_IBTN_CNT_ADDR;
    int startAddr = isRFID ? EEPROM_RFID_START : EEPROM_IBTN_START;
    int idx = EEPROM.read(cntAddr);
    if (idx >= 50) return;
    int addr = startAddr + (idx * 8);
    if (isRFID) {
        for(int i=0; i<8; i++) {
             EEPROM.put(addr + i, (byte)((id64 >> ((7-i)*8)) & 0xFF));
        }
    } else {
        for (int i=0; i<8; i++) EEPROM.put(addr + i, data8[i]);
    }
    EEPROM.write(cntAddr, idx + 1);
    if (isRFID) savedRfidCount = idx + 1; else savedIbtnCount = idx + 1;
}

void rfidACsetOn(){
  pinMode(FreqGen, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(WGM21); TCCR2B = _BV(CS20); OCR2A = 63; 
}

void rfidACsetOff(){
  TCCR2A &= 0b00111111; 
  digitalWrite(FreqGen, LOW); 
}

void rfidGap(unsigned int tm){
  TCCR2A &= 0b00111111; delayMicroseconds(tm); TCCR2A |= _BV(COM2A0);
}

void startRFID() { 
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0; OCR1A = 4095; 
  TCCR1B |= (1 << CS10); TIMSK1 |= (1 << OCIE1A);
  isEmulating = true; interrupts();
}

void stopRFID() { 
  noInterrupts(); TIMSK1 &= ~(1 << OCIE1A); TCCR1B = 0;
  isEmulating = false; digitalWrite(ANTENNA, LOW); interrupts();
}

void data_card_ul(uint64_t id_to_encode) {
  uint64_t data_card = (uint64_t)0x1FFF; 
  int32_t i;
  uint8_t tmp_nybble, column_parity_bits = 0;
  for (i = 9; i >= 0; i--) { 
    tmp_nybble = (uint8_t) (0x0f & (id_to_encode >> i*4));
    data_card = (data_card << 4) | tmp_nybble;
    data_card = (data_card << 1) | ((tmp_nybble >> 3 & 0x01) ^ (tmp_nybble >> 2 & 0x01) ^\
      (tmp_nybble >> 1 & 0x01) ^ (tmp_nybble  & 0x01));
    column_parity_bits ^= tmp_nybble;
  }
  data_card = (data_card << 4) | column_parity_bits;
  data_card = (data_card << 1); 
  for (i = 0; i < 8; i++) rfidData[i] = (uint8_t)(0xFF & (data_card >> (7 - i) * 8));
}

ISR(TIMER1_COMPA_vect) {
  if (!isEmulating) return;
  TCNT1 = 0;
  bool signalHigh = (((rfidData[byte_counter] << bit_counter) & 0x80) == 0x00) ? (half == 1) : (half == 0);
  if (signalHigh) PORTD |= B10000000; else PORTD &= ~B10000000; 
  half++;
  if (half == 2) {
      half = 0; bit_counter++;
      if (bit_counter == 8) { bit_counter = 0; byte_counter = (byte_counter + 1) % 8; }
  }
}

void rxISR() {
  unsigned long now = micros();
  unsigned long duration = now - lastChange;
  lastChange = now;
  if (duration < 60) return;
  if (pulseCount < 150) {
    pulses[pulseCount] = (duration > 65535) ? 65535 : (uint16_t)duration; 
    pulseCount++;
  } else {
    detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));
    captured = true;
  }
}

bool attemptDecode(uint64_t &outID) {
    int startIdx = -1, shortCount = 0;
    for (int i = 0; i < 100; i++) { 
       if (pulses[i] > 100 && pulses[i] <= RFID_THRESHOLD) shortCount++;
       else { if (shortCount > 14) { startIdx = i; break; } shortCount = 0; }
    }
    if (startIdx != -1) {
        int bits[64], bitCount = 0, currentLogic = 1; 
        for (int i = startIdx; i < pulseCount && bitCount < 64; i++) {
          if (pulses[i] > RFID_THRESHOLD) { currentLogic = !currentLogic; bits[bitCount++] = currentLogic; } 
          else { if (i + 1 < pulseCount && pulses[i+1] <= RFID_THRESHOLD) { bits[bitCount++] = currentLogic; i++; } }
        }
        uint64_t currentU64 = 0; bool valid = true;
        for (int row = 0; row < 10; row++) {
          int pos = row * 5; if (pos + 4 >= bitCount) { valid = false; break; }
          int val = (bits[pos] << 3) | (bits[pos+1] << 2) | (bits[pos+2] << 1) | bits[pos+3];
          if ((bits[pos] ^ bits[pos+1] ^ bits[pos+2] ^ bits[pos+3]) != bits[pos+4]) { valid = false; break; } 
          currentU64 = (currentU64 << 4) | val;
        }
        if (valid) {
            if (currentU64 == 0x7777777770) currentU64 = 0xFFFFFFFFFF;
            outID = currentU64;
            return true;
        }
    }
    return false;
}

void drawMainScreenContent(); 
void drawMenuContent(const char* title, const char* const items[], int count); 
void drawListWithOffsetContent(const char* title, const char* const items[], int count, int type);

void refreshDisplay() {
    u8g2.firstPage();
    do {
        if (screenState == 0) drawMainScreenContent();
        else if (screenState == 1) drawMenuContent("Main Menu", mainMenu, 4);
        else if (screenState == 2) drawMenuContent("RFID Menu", subMenu, 4);
        else if (screenState == 3) drawMenuContent("iButton Menu", subMenu, 4);
        else if (screenState == 4) drawListWithOffsetContent("Emulate RFID", NULL, rfidCount + savedRfidCount + 1, 1);
        else if (screenState == 5) drawListWithOffsetContent("Emulate iButton", NULL, ibtnCount + savedIbtnCount + 1, 2);
        else if (screenState == 6) drawListWithOffsetContent("Write RFID", NULL, rfidCount + savedRfidCount + 1, 3);
        else if (screenState == 7) drawListWithOffsetContent("Write iButton", NULL, ibtnCount + savedIbtnCount + 1, 4);
        else if (screenState == 8) drawMenuContent("Utilities", utilMenu, 4);
    } while (u8g2.nextPage());
}

void drawMainScreenContent() {
  u8g2.drawStr(0, 4, "v0.52"); 
  drawCentered(4, "MultiKey");
  u8g2.drawLine(0, 15, 127, 15); 
  drawCentered(30, "Idle...");
  drawCentered(45, "Press OK btn");
}

void drawListWithOffsetContent(const char* title, const char* const items[], int count, int type) {
  drawCentered(4, title);
  u8g2.drawLine(0, 15, 127, 15); 
  int visibleItems = 4;
  if (cursorPosition >= menuOffset + visibleItems) menuOffset = cursorPosition - visibleItems + 1;
  if (cursorPosition < menuOffset) menuOffset = cursorPosition;

  for (int i = 0; i < visibleItems; i++) {
    int itemIndex = menuOffset + i; 
    if (itemIndex >= count) break; 
    int y = 20 + (i * 11); 
    bool isBack = false;
    if (type != 0) {
       if (itemIndex == count - 1) isBack = true;
    }
    if (type == 0) strcpy_P(txtBuf, (char*)pgm_read_word(&(items[itemIndex])));
    else { 
       if (isBack) strcpy_P(txtBuf, m_Back);
       else {
          uint64_t _id;
          fetchKeyData((type==1 || type==3), itemIndex, _id, txtBuf);
       }
    }
    if (itemIndex == cursorPosition) {
      u8g2.drawStr(0, y, ">"); u8g2.drawStr(12, y, txtBuf); u8g2.drawFrame(10, y-2, 106, 13); 
    } else u8g2.drawStr(12, y, txtBuf); 
  }
  u8g2.drawLine(122, 18, 122, 62); 
  if (count > 1) {
     int barY = 18 + (cursorPosition * 35) / (count - 1);
     u8g2.drawFrame(119, barY, 7, 10);
  }
}

void drawMenuContent(const char* title, const char* const items[], int count) {
  drawListWithOffsetContent(title, items, count, 0);
}

void handleEnter();

void setup() {
  u8g2.begin(); u8g2.setFontPosTop(); u8g2.setFont(u8g2_font_6x10_tr); 
  pinMode(ANTENNA, OUTPUT); digitalWrite(ANTENNA, LOW);
  pinMode(FreqGen, OUTPUT); digitalWrite(FreqGen, LOW);
  pinMode(PIN_RFID_RX, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP); pinMode(btnOk, INPUT_PULLUP); pinMode(btnDown, INPUT_PULLUP);
  initEEPROM(); 
}

void loop() {
  bool up = !digitalRead(btnUp), ok = !digitalRead(btnOk), down = !digitalRead(btnDown);
  uint8_t maxPos = 0;
  if (screenState==1) maxPos=3; 
  else if(screenState==2 || screenState==3 || screenState==8) maxPos=3;
  else if(screenState==4 || screenState==6) maxPos = rfidCount + savedRfidCount;
  else if(screenState==5 || screenState==7) maxPos = ibtnCount + savedIbtnCount;
  
  if (millis() - lastPressTime > 150) { 
    if (up) { cursorPosition = (cursorPosition > 0) ? cursorPosition - 1 : maxPos; needUpdate=true; lastPressTime=millis(); }
    else if (down) { cursorPosition = (cursorPosition < maxPos) ? cursorPosition + 1 : 0; needUpdate=true; lastPressTime=millis(); }
  }
  if (ok) {
     if (!okWasPressed) { handleEnter(); needUpdate=true; okWasPressed=true; lastPressTime=millis() + 200; }
  } else okWasPressed = false; 

  if (needUpdate) { refreshDisplay(); needUpdate = false; }
}

void changeScreen(uint8_t newState) {
  screenState = newState; cursorPosition = 0; menuOffset = 0;
}

void handleEnter() {
  switch (screenState) {
    case 0: changeScreen(1); break;
    case 1: changeScreen((cursorPosition==0)?2 : (cursorPosition==1)?3 : (cursorPosition==2)?8 : 0); break;
    case 2: 
      if (cursorPosition==0) runRFIDRead(); 
      else if (cursorPosition==1) changeScreen(4); 
      else if (cursorPosition==2) changeScreen(6); 
      else changeScreen(1); break;
    case 3: 
      if (cursorPosition==0) runiButtonRead(); 
      else if (cursorPosition==1) changeScreen(5); 
      else if (cursorPosition==2) changeScreen(7); 
      else changeScreen(1); break;
    case 4: 
      if (cursorPosition == rfidCount + savedRfidCount) changeScreen(2); else runRFIDEmulation(cursorPosition); break;
    case 6: 
      if (cursorPosition == rfidCount + savedRfidCount) changeScreen(2); else runRFIDWrite(cursorPosition); break;
    case 5: 
      if (cursorPosition == ibtnCount + savedIbtnCount) changeScreen(3); else runiButtonEmulation(cursorPosition); break;
    case 7: 
      if (cursorPosition == ibtnCount + savedIbtnCount) changeScreen(3); 
      else runiButtonWrite(cursorPosition); break;
    case 8:
      if (cursorPosition == 0) runClearEEPROM();
      else if (cursorPosition == 1) runClearRFIDKeys();
      else if (cursorPosition == 2) runCleariButtonKeys();
      else changeScreen(1); break;
  }
}

void uiWaitOK(const char* line1, const char* line2) {
    u8g2.firstPage(); do {
        if(line1) { strcpy_P(txtBuf, line1); drawCentered(25, txtBuf); }
        if(line2) { strcpy_P(txtBuf, line2); drawCentered(45, txtBuf); }
    } while (u8g2.nextPage());
    while(!digitalRead(btnOk)) delay(10);
    while(digitalRead(btnOk)) delay(10);
}

void runRFIDRead() {
  while(!digitalRead(btnOk)); delay(100);
  rfidACsetOn();
  lastReadID_u64 = 0; pulseCount = 0; captured = false; lastChange = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);

  bool loopRunning = true, keyFound = false, saved = false;
  bool uiNeedsUpdate = true; 

  while(loopRunning) { 
     if (!digitalRead(btnOk)) { loopRunning = false; continue; }
     
     if (!digitalRead(btnUp)) {
         keyFound = false; saved = false; lastReadID_u64 = 0; pulseCount = 0; captured = false;
         attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);
         uiNeedsUpdate = true; delay(200); 
     }

     if (keyFound && !saved && !digitalRead(btnDown)) {
         saveToEEPROM(true, NULL, lastReadID_u64);
         saved = true; uiNeedsUpdate = true; delay(200); 
     }

     if (uiNeedsUpdate) {
       u8g2.firstPage(); do {
          drawCentered(4, "Read RFID"); u8g2.drawLine(0, 15, 127, 15);
          if (keyFound) {
             drawCentered(22, "Key Found!");
             drawIDRow(lastReadID_u64, 5, 36);
             u8g2.drawFrame(10, 34, 108, 12);
             u8g2.drawStr(0, 54, "< Retry");
             u8g2.drawStr(80, 54, saved ? "[Saved]" : "Save >");
          } else {
             drawCentered(30, "Bring Key..."); drawCentered(54, "Press OK to exit");
          }
       } while (u8g2.nextPage());
       uiNeedsUpdate = false;
     }

     if (!keyFound && captured) {
        uint64_t tempID;
        if (attemptDecode(tempID)) {
             lastReadID_u64 = tempID; 
             keyFound = true; saved = false; uiNeedsUpdate = true;
             detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));
        } else {
             pulseCount = 0; captured = false; attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);
        }
     }
  }
  detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));
  rfidACsetOff();
  while(!digitalRead(btnOk)) delay(10); 
}

void runRFIDEmulation(uint8_t keyIndex) {
  uint64_t currentID;
  fetchKeyData(true, keyIndex, currentID, txtBuf);
  data_card_ul(currentID); 
  char tmpName[20]; strcpy(tmpName, txtBuf);

  u8g2.firstPage(); do {
      drawCentered(4, "RFID Emulation"); u8g2.drawLine(0, 15, 127, 15);
      drawCentered(18, tmpName);
      drawIDRow(currentID, 5, 35);
      drawCentered(54, "Press OK to stop"); u8g2.drawFrame(16, 33, 96, 14); 
  } while (u8g2.nextPage());

  while(!digitalRead(btnOk)) delay(10); 
  startRFID(); while(digitalRead(btnOk)) delay(10); stopRFID();
  while(!digitalRead(btnOk)) delay(10); 
}

void runRFIDWrite(uint8_t keyIndex) {
  uint64_t targetID;
  fetchKeyData(true, keyIndex, targetID, txtBuf);
  byte keyBytes[5]; for(int i=0; i<5; i++) keyBytes[i] = (byte)(targetID >> ((4-i)*8)) & 0xFF;
  char tmpName[20]; strcpy(tmpName, txtBuf);

  while(true) {
      rfidACsetOn(); 
      pulseCount = 0; captured = false; lastChange = micros();
      attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);

      bool tagDetected = false;
      while(!tagDetected) {
          u8g2.firstPage(); do {
              drawCentered(4, "Ready to Write"); u8g2.drawLine(0, 15, 127, 15);
              drawCentered(18, tmpName);
              drawIDRow(targetID, 5, 35);
              drawCentered(54, "Hold Tag...");
          } while (u8g2.nextPage());

          if (!digitalRead(btnOk)) {
              detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));
              rfidACsetOff();
              delay(300); 
              return; 
          }

          if (captured) {
              uint64_t dummy;
              if (attemptDecode(dummy)) tagDetected = true; 
              else {
                  pulseCount = 0; captured = false; 
                  attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);
              }
          }
      }
      detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));

      u8g2.firstPage(); do { drawCentered(25, "Tag Found!"); drawCentered(40, "Writing..."); } while (u8g2.nextPage());
      
      writeEM4100(keyBytes); delay(30); writeEM4100(keyBytes); 

      u8g2.firstPage(); do { drawCentered(30, "Verifying..."); } while (u8g2.nextPage());
      
      rfidACsetOff(); delay(500); rfidACsetOn(); delay(300);     

      pulseCount = 0; captured = false; lastChange = micros();
      attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);
      
      unsigned long startVerify = millis();
      bool verified = false;
      uint64_t readID = 0;

      while (millis() - startVerify < 2500) { 
          if (captured) {
              if (attemptDecode(readID)) { if (readID == targetID) { verified = true; break; } }
              pulseCount = 0; captured = false; attachInterrupt(digitalPinToInterrupt(PIN_RFID_RX), rxISR, CHANGE);
          }
          delay(10);
      }
      detachInterrupt(digitalPinToInterrupt(PIN_RFID_RX));
      rfidACsetOff();

      if (verified) {
          u8g2.firstPage(); do {
             drawCentered(4, "Write SUCCESS!"); drawIDRow(readID, 5, 30);
             drawCentered(50, "[ OK ]"); u8g2.drawLine(0, 15, 127, 15);
          } while (u8g2.nextPage());
          while(!digitalRead(btnOk)) delay(10); while(digitalRead(btnOk)) delay(10);
          return; 
      } else {
          u8g2.firstPage(); do {
             drawCentered(4, "Write FAILED!");
             if(readID != 0) drawIDRow(readID, 5, 30); else drawCentered(30, "No Data");
             drawCentered(48, "Retrying..."); u8g2.drawLine(0, 15, 127, 15);
          } while (u8g2.nextPage());
          delay(1500); 
      }
  }
}

void TxBitRfid(byte data){
  if (data & 1) delayMicroseconds(432); else delayMicroseconds(192); rfidGap(152); 
}
void sendOpT5557(byte opCode, unsigned long data, byte blokAddr){
  TxBitRfid(opCode >> 1); TxBitRfid(opCode & 1); TxBitRfid(0);
  for (byte i = 0; i<32; i++) { TxBitRfid((data>>(31-i)) & 1); }
  TxBitRfid(blokAddr>>2); TxBitRfid(blokAddr>>1); TxBitRfid(blokAddr & 1);
}
void writeEM4100(byte keyID[5]) {
  uint64_t data64 = 0x1FF; 
  for (int i = 0; i < 5; i++) {
    byte b = keyID[i];
    byte n1 = (b >> 4) & 0xF; byte p1 = (n1 ^ (n1 >> 1) ^ (n1 >> 2) ^ (n1 >> 3)) & 1;
    data64 = (data64 << 5) | (n1 << 1) | p1;
    byte n2 = b & 0xF; byte p2 = (n2 ^ (n2 >> 1) ^ (n2 >> 2) ^ (n2 >> 3)) & 1;
    data64 = (data64 << 5) | (n2 << 1) | p2;
  }
  byte col_p = 0;
  for(int bit=0; bit<4; bit++) {
     int count = 0;
     for(int i=0; i<10; i++) { 
        byte full_byte = keyID[i/2];
        if ( (( (i%2==0)?(full_byte>>4):(full_byte&0xF) ) >> (3-bit)) & 1 ) count++;
     }
     if (count % 2 != 0) col_p |= (1 << (3-bit));
  }
  data64 = (data64 << 5) | col_p << 1; 
  rfidACsetOn(); delay(300); 
  rfidGap(240); sendOpT5557(0b10, (unsigned long)(data64 >> 32), 1); delay(15);
  rfidGap(240); sendOpT5557(0b10, (unsigned long)(data64 & 0xFFFFFFFF), 2); delay(15);
  rfidGap(240); sendOpT5557(0b10, 0x00148040, 0); delay(15);
  TCCR2A &= 0b00111111; digitalWrite(FreqGen, LOW);
}

void runiButtonEmulation(uint8_t keyIndex) {
  uint64_t id; fetchKeyData(false, keyIndex, id, txtBuf);
  byte kb[8]; for(int i=0; i<8; i++) kb[7-i] = (byte)((id >> (i*8)) & 0xFF);
  kb[7] = onewire_crc8(kb, 7);
  auto ds = DS2401(kb[0], kb[1], kb[2], kb[3], kb[4], kb[5], kb[6]); hub.attach(ds);
  char tmpName[20]; strcpy(tmpName, txtBuf);
  u8g2.firstPage(); do {
      drawCentered(4, "iButton Emul"); u8g2.drawLine(0, 15, 127, 15); drawCentered(18, tmpName); 
      for(int i=0; i<4; i++) printHexByte(kb[i], 28 + i*18, 30);
      for(int i=0; i<4; i++) printHexByte(kb[i+4], 28 + i*18, 40); drawCentered(54, "Press OK to stop");
  } while (u8g2.nextPage());
  while(!digitalRead(btnOk)) delay(10); while(digitalRead(btnOk)) hub.poll(); while(!digitalRead(btnOk)) delay(10);
}

void runiButtonRead() {
  OneWire ib(PIN_READ_IB); byte addr[8]; while(!digitalRead(btnOk)) delay(10); bool nR = true;
  while(digitalRead(btnOk)) { 
    if (nR) { u8g2.firstPage(); do { drawCentered(4, "Read iButton"); u8g2.drawLine(0, 15, 127, 15); drawCentered(30, "Attach Key..."); } while (u8g2.nextPage()); nR = false; }
    if (!ib.search(addr)) { ib.reset_search(); delay(200); continue; }
    if (OneWire::crc8(addr, 7) != addr[7]) { nR = true; continue; }
    bool sV = false, uN = true, rT = false;
    while(digitalRead(btnOk)) { 
      if (!digitalRead(btnUp)) { rT = true; delay(200); break; }
      if (!digitalRead(btnDown) && !sV) { saveToEEPROM(false, addr, 0); sV = true; uN = true; delay(200); }
      if (uN) { u8g2.firstPage(); do { drawCentered(4, "Read iButton"); u8g2.drawLine(0, 15, 127, 15); for(int i=0; i<4; i++) printHexByte(addr[i], 28 + i*18, 25); for(int i=0; i<4; i++) printHexByte(addr[i+4], 28 + i*18, 38); u8g2.drawStr(0, 54, "< Retry"); u8g2.drawStr(80, 54, sV ? "[Saved]" : "Save >"); } while (u8g2.nextPage()); uN = false; }
      delay(20);
    }
    if (rT) { ib.reset_search(); delay(300); nR = true; continue; }
    if (!digitalRead(btnOk)) break;
  }
  while(!digitalRead(btnOk)) delay(10); 
}

void runClearEEPROM() {
  u8g2.firstPage(); do {
      drawCentered(4, "Clear EEPROM"); u8g2.drawLine(0, 15, 127, 15);
      drawCentered(30, "Down -> CONFIRM"); drawCentered(50, "OK -> CANCEL");
  } while (u8g2.nextPage());
  while(true) {
    if (!digitalRead(btnOk)) { delay(200); return; }
    if (!digitalRead(btnDown)) {
      EEPROM.write(EEPROM_IBTN_CNT_ADDR, 0); EEPROM.write(EEPROM_RFID_CNT_ADDR, 0);
      savedIbtnCount = 0; savedRfidCount = 0;
      uiWaitOK(PSTR("ALL CLEARED!"), NULL);
      return;
    }
  }
}

void runClearRFIDKeys() {
  u8g2.firstPage(); do {
      drawCentered(4, "Clear RFID keys"); u8g2.drawLine(0, 15, 127, 15);
      drawCentered(30, "Down -> CONFIRM"); drawCentered(50, "OK -> CANCEL");
  } while (u8g2.nextPage());
  while(true) {
    if (!digitalRead(btnOk)) { delay(200); return; }
    if (!digitalRead(btnDown)) {
      EEPROM.write(EEPROM_RFID_CNT_ADDR, 0);
      savedRfidCount = 0;
      uiWaitOK(PSTR("RFID CLEARED!"), NULL);
      return;
    }
  }
}

void runCleariButtonKeys() {
  u8g2.firstPage(); do {
      drawCentered(4, "Clear iBut keys"); u8g2.drawLine(0, 15, 127, 15);
      drawCentered(30, "Down -> CONFIRM"); drawCentered(50, "OK -> CANCEL");
  } while (u8g2.nextPage());
  while(true) {
    if (!digitalRead(btnOk)) { delay(200); return; }
    if (!digitalRead(btnDown)) {
      EEPROM.write(EEPROM_IBTN_CNT_ADDR, 0);
      savedIbtnCount = 0;
      uiWaitOK(PSTR("iButton CLEARED!"), NULL);
      return;
    }
  }
}

void BurnByte(OneWire& ib, byte data){
  for(byte n=0; n<8; n++){ ib.write_bit(data & 1); delay(5); data >>= 1; }
  pinMode(PIN_READ_IB, INPUT);
}
bool checkWrite(OneWire& ib, byte* tID){
  byte b[8]; if (!ib.reset()) return false;
  ib.write(0x33); ib.read_bytes(b, 8);
  for (byte i = 0; i < 8; i++) if (tID[i] != b[i]) return false;
  return true;
}
bool write2iBtn(OneWire& ib, emRWType rt, byte* kID){
  ib.reset();
  if (rt == TM2004) {
      ib.write(0x3C); ib.write(0x00); ib.write(0x00);
      for (byte i = 0; i<8; i++){ ib.write(kID[i]); delayMicroseconds(600); ib.write_bit(1); delay(50); pinMode(PIN_READ_IB, INPUT); }
  } else {
      byte cmd = (rt == TM01) ? 0xC1 : (rt == RW1990_2) ? 0x1D : 0xD1;
      byte flag = (rt == RW1990_1) ? 0 : 1;
      ib.write(cmd); ib.write_bit(flag); delay(5); pinMode(PIN_READ_IB, INPUT);
      ib.reset(); ib.write((rt == TM01)? 0xC5 : 0xD5); 
      for (byte i = 0; i< 8; i++){ if (rt == RW1990_1) BurnByte(ib, ~kID[i]); else BurnByte(ib, kID[i]); }
      if (rt != TM01) { ib.write(cmd); ib.write_bit(!flag); delay(5); pinMode(PIN_READ_IB, INPUT); }
  }
  return checkWrite(ib, kID);
}
emRWType getRWtype(OneWire& ib){
  ib.reset(); ib.write(0xD1); ib.write_bit(1); delay(10); pinMode(PIN_READ_IB, INPUT); ib.reset(); ib.write(0xB5);
  if (ib.read() == 0xFE) return RW1990_1;
  ib.reset(); ib.write(0x1D); ib.write_bit(1); delay(10); pinMode(PIN_READ_IB, INPUT); ib.reset(); ib.write(0x1E); 
  if (ib.read() == 0xFE){ ib.reset(); ib.write(0x1D); ib.write_bit(0); return RW1990_2; }
  ib.reset(); ib.write(0x33); for (byte i=0; i<8; i++) ib.read(); 
  ib.write(0xAA); ib.write(0x00); ib.write(0x00); byte answer = ib.read();
  if (OneWire::crc8((uint8_t*)"\xAA\x00\x00", 3) == answer) return TM2004;
  return TM01; 
}

void runiButtonWrite(uint8_t keyIndex) {
  uint64_t id; fetchKeyData(false, keyIndex, id, txtBuf);
  byte kb[8]; for(int i=0; i<8; i++) kb[7-i] = (byte)((id >> (i*8)) & 0xFF);
  kb[7] = onewire_crc8(kb, 7);
  OneWire ib(PIN_READ_IB); byte addr[8]; char tmpName[20]; strcpy(tmpName, txtBuf);
  u8g2.firstPage(); do { drawCentered(4, "Auto Write..."); u8g2.drawLine(0, 15, 127, 15); drawCentered(18, tmpName); drawCentered(35, "Attach BLANK key"); drawCentered(54, "OK to Cancel"); } while (u8g2.nextPage());
  while(!digitalRead(btnOk)) delay(10); 
  while(digitalRead(btnOk)) { 
    if ( !ib.search(addr)) { ib.reset_search(); delay(50); continue; }
    u8g2.firstPage(); do { drawCentered(20, "Key Detected!"); drawCentered(35, "Writing..."); } while (u8g2.nextPage());
    emRWType rt = getRWtype(ib);
    bool res = (rt != rwUnknown) ? write2iBtn(ib, rt, kb) : false;
    u8g2.firstPage(); do {
      if (res) { drawCentered(4, "SUCCESS!"); u8g2.drawLine(0, 15, 127, 15); for(int i=0; i<4; i++) printHexByte(kb[i], 28 + i*18, 25); for(int i=0; i<4; i++) printHexByte(kb[i+4], 28 + i*18, 38); drawCentered(54, "Press OK to exit"); } else { drawCentered(25, "WRITE FAILED"); drawCentered(45, "Retrying..."); }
    } while (u8g2.nextPage());
    if (res) { while(digitalRead(btnOk)) delay(10); while(!digitalRead(btnOk)) delay(10); return; } else { delay(1000); }
  }
}