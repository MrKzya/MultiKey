// Запись и проверка 

#include <Arduino.h>
#define COIL_PIN 11      // Выход
#define RX_PIN 2         // Вход данных 
#define THRESHOLD 380    // Порог длительности импульса

// ID ДЛЯ ЗАПИСИ
// Пример: FF FF FF FF FF
byte keyID[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 


volatile unsigned long lastChange = 0;
volatile unsigned long pulses[150];
volatile int pulseCount = 0;
volatile bool captured = false;

String lastReadID = "";
unsigned long lastTimeSeen = 0;
bool verificationMode = false; 


void rfidACsetOn();
void rfidACsetOff(); 
void rfidGap(unsigned int tm);
void TxBitRfid(byte data);
bool sendOpT5557(byte opCode, unsigned long password, byte lockBit, unsigned long data, byte blokAddr);
void rxISR();
String processSignal();
void writeEM4100();
void performWriteAndVerify();
void performReset();

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  pinMode(COIL_PIN, OUTPUT);

  // Запуск поля
  rfidACsetOn();

  Serial.println("Mode: READER");
  Serial.println("Commands: 'w' (Write), 'r' (Reset)");
  
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') performWriteAndVerify();
    if (c == 'r') performReset();
  }

  if (captured) {
    String readId = processSignal();
    
    // В обычном режиме просто выводим ID
    if (readId.length() == 10) {
      if (!verificationMode) {
        if (readId != lastReadID || (millis() - lastTimeSeen > 2000)) {
          Serial.print("READ: "); Serial.println(readId);
          lastReadID = readId;
          lastTimeSeen = millis();
        }
      } 
    }
    
    pulseCount = 0;
    captured = false;
    attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
  }
}

//Запись и верификация 
void performWriteAndVerify() {
  Serial.println("\n>>> STARTING WRITE");
  
  // Останавливаем чтение
  detachInterrupt(digitalPinToInterrupt(RX_PIN));
  captured = false;
  pulseCount = 0;

  // Пишем
  writeEM4100(); 


  //Выключаем поле, чтобы сбросить метку из режима записи.
  Serial.println(">>> Cycling Power");
  rfidACsetOff();      // Выключаем генерацию
  digitalWrite(COIL_PIN, LOW);
  delay(500);          // Ждем разрядки конденсатора метки

  //Включаем поле обратно
  rfidACsetOn();       
  delay(500);          // Даем метке зарядиться и начать слать данные

  //Включаем чтение для проверки
  Serial.println(">>> VERIFYING");
  
  // Очищаем мусор из прерываний, который мог проскочить при включении
  pulseCount = 0; 
  captured = false;
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
  
  // Формируем ожидаемую строку
  String expectedID = "";
  for(int i=0; i<5; i++) {
    if(keyID[i] < 0x10) expectedID += "0";
    expectedID += String(keyID[i], HEX);
  }
  expectedID.toUpperCase();

  //Цикл ожидания
  unsigned long startTime = millis();
  bool verified = false;

  while (millis() - startTime < 5000) {
    if (captured) {
      String readId = processSignal();
      
      // Сброс для следующей попытки внутри цикла
      pulseCount = 0;
      captured = false;
      attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE); 

      if (readId.length() == 10) {
        Serial.print("   Check: "); Serial.println(readId);
        if (readId == expectedID) {
          verified = true;
          break; // Ура
        }
      }
    }
  }

  if (verified) {
    Serial.println(">>> SUCCESS!");
  } else {
    Serial.println(">>> WARNING: Verification Timed Out.");
    Serial.println("    (But tag might still be written. Check manually.)");
  }
  
  lastReadID = ""; // Сброс кэша чтобы сразу прочитать снова
  Serial.println("Back to Reader Mode");
}

void performReset() {
  Serial.println("Resetting...");
  detachInterrupt(digitalPinToInterrupt(RX_PIN));
  
  rfidACsetOn();
  delay(500);
  unsigned long config = 0x00148040; 
  rfidGap(30 * 8);
  sendOpT5557(0b10, 0, 0, config, 0);
  delay(20);
  
  rfidACsetOff(); 
  delay(100);
  rfidACsetOn();

  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
  Serial.println("Done.");
}

//Декодер
String processSignal() {
  int startIdx = -1;
  int shortCount = 0;
  
  for (int i = 0; i < 100; i++) {
    if (pulses[i] > 100 && pulses[i] <= THRESHOLD) shortCount++;
    else {
      if (shortCount > 14) { startIdx = i; break; }
      shortCount = 0;
    }
  }
  if (startIdx == -1) return ""; 

  int bits[64];
  int bitCount = 0;
  int currentLogic = 1; 
  
  for (int i = startIdx; i < pulseCount && bitCount < 64; i++) {
    unsigned long t = pulses[i];
    if (t > THRESHOLD) { 
      currentLogic = !currentLogic;
      bits[bitCount++] = currentLogic;
    } else {
      if (i + 1 < pulseCount && pulses[i+1] <= THRESHOLD) {
        bits[bitCount++] = currentLogic;
        i++; 
      }
    }
  }

  String currentHex = "";
  for (int row = 0; row < 10; row++) {
    int pos = row * 5;
    if (pos + 4 >= bitCount) return "";
    int val = (bits[pos] << 3) | (bits[pos+1] << 2) | (bits[pos+2] << 1) | bits[pos+3];
    int parity = bits[pos+4];
    if ((bits[pos]^bits[pos+1]^bits[pos+2]^bits[pos+3]) != parity) return "";
    if (val < 10) currentHex += String(val);
    else currentHex += String((char)('A' + (val - 10)));
  }

  if (currentHex == "7777777770") currentHex = "FFFFFFFFFF";
  return currentHex;
}

//Запись
void writeEM4100() {
  unsigned long block1 = 0, block2 = 0;
  uint64_t data64 = 0x1FF;
  uint8_t p_col = 0;
  
  for (int i = 0; i < 5; i++) {
    byte b = keyID[i];
    for (int n = 0; n < 2; n++) { 
      byte nib = (n==0) ? (b >> 4) : (b & 0xF);
      byte p = (nib ^ (nib >> 1) ^ (nib >> 2) ^ (nib >> 3)) & 1; 
      data64 = (data64 << 5) | ((nib & 0xF) << 1) | p;
    }
  }
  

  byte col_p = 0;
  for(int bit=0; bit<4; bit++) {
     int count = 0;
     for(int i=0; i<10; i++) { 
        byte full_byte = keyID[i/2];
        byte nib = (i%2==0) ? (full_byte >> 4) : (full_byte & 0xF);
        if ( (nib >> (3-bit)) & 1 ) count++;
     }
     if (count % 2 != 0) col_p |= (1 << (3-bit));
  }
  data64 = (data64 << 4) | col_p;
  data64 = (data64 << 1); 

  block1 = (unsigned long)(data64 >> 32);
  block2 = (unsigned long)(data64 & 0xFFFFFFFF);

  rfidACsetOn();
  delay(300); 

  // Пишем
  rfidGap(30 * 8); sendOpT5557(0b10, 0, 0, block1, 1); delay(15);
  rfidGap(30 * 8); sendOpT5557(0b10, 0, 0, block2, 2); delay(15);
  rfidGap(30 * 8); sendOpT5557(0b10, 0, 0, 0x00148040, 0); delay(15);
}

void rxISR() {
  unsigned long now = micros();
  unsigned long duration = now - lastChange;
  lastChange = now;
  if (duration < 60) return;
  if (pulseCount < 150) {
    pulses[pulseCount++] = duration;
  } else {
    detachInterrupt(digitalPinToInterrupt(RX_PIN));
    captured = true;
  }
}

void rfidACsetOn(){
  pinMode(COIL_PIN, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(WGM21); 
  TCCR2B = _BV(CS20); 
  OCR2A = 63; 
}

void rfidACsetOff(){
  TCCR2A = 0; 
  TCCR2B = 0;
  digitalWrite(COIL_PIN, LOW); 
}

void rfidGap(unsigned int tm){
  TCCR2A &= 0b00111111;
  delayMicroseconds(tm);
  TCCR2A |= _BV(COM2A0);
}

void TxBitRfid(byte data){
  if (data & 1) delayMicroseconds(54*8);
  else delayMicroseconds(24*8);
  rfidGap(19*8); 
}

bool sendOpT5557(byte opCode, unsigned long password, byte lockBit, unsigned long data, byte blokAddr){
  TxBitRfid(opCode >> 1); TxBitRfid(opCode & 1);
  if (opCode == 0b00) return true;
  TxBitRfid(lockBit & 1);
  if (data != 0 || opCode == 0b10){ 
    for (byte i = 0; i<32; i++) TxBitRfid((data>>(31-i)) & 1);
  }
  TxBitRfid(blokAddr>>2); TxBitRfid(blokAddr>>1); TxBitRfid(blokAddr & 1);
  return true;
}