//Чтение RFID

#define COIL_PIN 11
#define RX_PIN 2

#define THRESHOLD 380 

volatile unsigned long lastChange = 0;
volatile unsigned long pulses[150];
volatile int pulseCount = 0;
volatile bool captured = false;

String lastReadID = "";
unsigned long lastTimeSeen = 0;

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  pinMode(COIL_PIN, OUTPUT);

  TCCR2A = _BV(COM2A0) | _BV(WGM21);
  TCCR2B = _BV(CS20);
  OCR2A = 63; 

  Serial.println("--- READY ---");
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
}

void loop() {
  if (captured) {
    processSignal();
    pulseCount = 0;
    captured = false;
    attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
  }

  if (millis() - lastTimeSeen > 2000) {
    lastReadID = "";
  }
}

void rxISR() {
  unsigned long now = micros();
  unsigned long duration = now - lastChange;
  lastChange = now;

  if (duration < 60) return;

  if (pulseCount < 150) {
    pulses[pulseCount] = duration;
    pulseCount++;
  } else {
    detachInterrupt(digitalPinToInterrupt(RX_PIN));
    captured = true;
  }
}

void processSignal() {
  int startIdx = -1;
  int shortCount = 0;
  
  // Поиск заголовка
  for (int i = 0; i < 100; i++) {
    if (pulses[i] > 100 && pulses[i] <= THRESHOLD) shortCount++;
    else {
      if (shortCount > 14) { startIdx = i; break; }
      shortCount = 0;
    }
  }
  if (startIdx == -1) return; 

  // Декодирование
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

  //Проверка четности
  String currentHex = "";
  for (int row = 0; row < 10; row++) {
    int pos = row * 5;
    if (pos + 4 >= bitCount) return;

    int b3 = bits[pos+0];
    int b2 = bits[pos+1];
    int b1 = bits[pos+2];
    int b0 = bits[pos+3];
    int parityBit = bits[pos+4];

    int calcParity = b3 ^ b2 ^ b1 ^ b0;
    if (calcParity != parityBit) return; // Ошибка четности - мусор

    int val = (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
    if (val < 10) currentHex += String(val);
    else currentHex += String((char)('A' + (val - 10)));
  }

  if (currentHex.length() == 10) {
    
    // Из-за инверсии сигнала FFFFFFFFFF часто читается как 7777777770
    if (currentHex == "7777777770") {
      currentHex = "FFFFFFFFFF";
    }

    if (currentHex != lastReadID) {
      Serial.print("Card ID: ");
      Serial.println(currentHex);
      lastReadID = currentHex;
    }
    lastTimeSeen = millis();
  }
}