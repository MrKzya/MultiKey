// Скрипт для чтения iButton Dallas
#include <OneWire.h>

#define pinIB 12

OneWire ibutton(pinIB);

void setup() {
  Serial.begin(115200);
  Serial.println("Touch the key to the reader");
}

void loop() {
  // ID Ключа для записи (8 байт). Последний байт - CRC.
  // Пример: Family Code 01 (Dallas), ID, CRC
  byte addr[8]; // Массив для хранения ID

  // Ищем ключи на шине
  if ( !ibutton.search(addr)) {
    ibutton.reset_search();
    delay(250);
    return;
  }


  Serial.println("\nKey Detected!");
  
  // Выводим сырой HEX 
  Serial.print("ID: { ");
  for(int i = 0; i < 8; i++) {
    Serial.print("0x");
    // Добавляем ноль, если число меньше 16 (чтобы было 0A, а не A)
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println(" };");

  // Проверяем контрольную сумму (CRC)
  // CRC гарантирует, что ключ считался без ошибок
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("Warning: CRC is invalid! Read error.");
  } else {
      Serial.println("CRC: OK");
      
      // Определяем тип (Family Code - первый байт)
      Serial.print("Type: ");
      switch (addr[0]) {
        case 0x01: Serial.println("DS1990"); break;
        case 0x36: Serial.println("DS1994"); break;
        case 0x0C: Serial.println("DS1996"); break;
        case 0x81: Serial.println("DS1420"); break;
        case 0x14: Serial.println("DS1971"); break;
        case 0x08: Serial.println("DS1992"); break;
        default:   Serial.println("Unknown / Other"); break;
      }
  }

  Serial.println("---------------------------------");
  delay(1000); // Не спамим в порт
}