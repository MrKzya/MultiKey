// Запись на RW1990 

#include <OneWire.h>

#define pinIB 12

OneWire ibutton(pinIB);

// ID Ключа для записи (8 байт). Последний байт - CRC.
// Пример: Family Code 01 (Dallas), ID, CRC
byte keyID[8] = {0x01, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x3D}; 

// Типы болванок
enum emRWType {rwUnknown, TM01, RW1990_1, RW1990_2, TM2004};

void setup() {
  Serial.begin(115200);
  Serial.println("Touch the RW1990 to the reader.");
  Serial.println("Send 'w' to write.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      Serial.println("Checking key type...");
      
      // Сначала ищем ключ на шине
      byte addr[8];
      if (!ibutton.search(addr)) {
        ibutton.reset_search();
        Serial.println("No key found! Hold the key firmly.");
        return;
      }
      
      // Определяем тип болванки
      emRWType type = getRWtype();
      
      if (type == rwUnknown) {
        Serial.println("Unknown key type. Cannot write.");
      } else {
        Serial.print("Detected type: ");
        if(type == RW1990_1) Serial.println("RW1990.1");
        if(type == RW1990_2) Serial.println("RW1990.2");
        if(type == TM01) Serial.println("TM-01");
        if(type == TM2004) Serial.println("TM2004");
        
        Serial.println("Writing...");
        
        bool result = false;
        if (type == TM2004) result = write2iBtnTM2004();
        else result = write2iBtnRW1990_1_2_TM01(type);

        if (result) Serial.println("SUCCESS! Write Complete.");
        else Serial.println("ERROR. Write Failed.");
      }
    }
  }
}

// Логика определения типа болванки
emRWType getRWtype(){
  byte answer;
  
  // RW-1990.1
  ibutton.reset(); ibutton.write(0xD1); ibutton.write_bit(1); delay(10);
  pinMode(pinIB, INPUT); ibutton.reset(); ibutton.write(0xB5);
  answer = ibutton.read();
  if (answer == 0xFE) return RW1990_1;

  //RW-1990.2
  ibutton.reset(); ibutton.write(0x1D); ibutton.write_bit(1); delay(10);
  pinMode(pinIB, INPUT); ibutton.reset(); ibutton.write(0x1E); 
  answer = ibutton.read();
  if (answer == 0xFE){
    ibutton.reset(); ibutton.write(0x1D); ibutton.write_bit(0); // Выключаем запись
    return RW1990_2;
  }

  // TM-2004
  ibutton.reset(); ibutton.write(0x33); 
  for (byte i=0; i<8; i++) ibutton.read(); // skip rom
  ibutton.write(0xAA); ibutton.write(0x00); ibutton.write(0x00);
  answer = ibutton.read();
  byte m1[3] = {0xAA, 0,0};
  if (OneWire::crc8(m1, 3) == answer) return TM2004;

  // Иначе считаем, что это TM-01
  return TM01; 
}

// --- Функция записи для RW1990 и TM-01 ---
bool write2iBtnRW1990_1_2_TM01(emRWType rwType){
  byte rwCmd, rwFlag = 1;
  switch (rwType){
    case TM01: rwCmd = 0xC1; break; 
    case RW1990_1: rwCmd = 0xD1; rwFlag = 0; break; // Инвертированный флаг
    case RW1990_2: rwCmd = 0x1D; break; 
    default: return false;
  }

  // Разрешаем запись
  ibutton.reset(); ibutton.write(rwCmd); ibutton.write_bit(rwFlag); 
  delay(5); pinMode(pinIB, INPUT);

  // Команда записи
  ibutton.reset();
  if (rwType == TM01) ibutton.write(0xC5);
  else ibutton.write(0xD5); 

  // Пишем байты
  for (byte i = 0; i< 8; i++){
    if (rwType == RW1990_1) BurnByte(~keyID[i]); // Инверсная запись
    else BurnByte(keyID[i]);
  }

  // Запрещаем запись
  if (rwType != TM01) {
      ibutton.write(rwCmd); ibutton.write_bit(!rwFlag); 
      delay(5); pinMode(pinIB, INPUT);
  }

  return checkWrite(); // Проверка
}

// Функция записи для TM2004
bool write2iBtnTM2004(){
  ibutton.reset();
  ibutton.write(0x3C);
  ibutton.write(0x00); ibutton.write(0x00); // Адрес
  
  for (byte i = 0; i<8; i++){
    ibutton.write(keyID[i]);
    delayMicroseconds(600); ibutton.write_bit(1); delay(50);
    pinMode(pinIB, INPUT);
  }
  return checkWrite();
}

void BurnByte(byte data){
  for(byte n_bit = 0; n_bit < 8; n_bit++){
    ibutton.write_bit(data & 1);
    delay(5); 
    data = data >> 1; 
  }
  pinMode(pinIB, INPUT);
}

// Проверка записанного
bool checkWrite(){
  byte buff[8];
  if (!ibutton.reset()) return false;
  ibutton.write(0x33);
  ibutton.read_bytes(buff, 8);
  
  for (byte i = 0; i < 8; i++){
    if (keyID[i] != buff[i]) return false;
  }
  return true;
}