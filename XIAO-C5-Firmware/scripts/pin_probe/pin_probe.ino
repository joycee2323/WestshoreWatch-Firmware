/*
 * XIAO ESP32-C5 pin-map probe  —  TEMPORARY diagnostic, NOT firmware.
 *
 * Purpose: print the real GPIO number behind every Dx header label, as
 * defined by the *installed* Seeed XIAO ESP32-C5 Arduino core
 * (pins_arduino.h). This is ground truth for the cellular UART pin fix —
 * it replaces unverified community/forum pinout tables.
 *
 * Flash (Arduino IDE):
 *   Board:  "XIAO_ESP32C5"  (Seeed esp32 core)
 *   Tools → USB CDC On Boot: Enabled
 *   Upload, then open Serial Monitor @ 115200.
 *
 * Flash (arduino-cli), replace <PORT>:
 *   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C5 scripts/pin_probe
 *   arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32C5 -p <PORT> scripts/pin_probe
 *   arduino-cli monitor -p <PORT> -c baudrate=115200
 *
 * Paste the full output back. We care most about: which GPIO is D4
 * (modem R / C5-TX) and which is D5 (modem T / C5-RX).
 *
 * This folder is under scripts/ and is NOT part of the idf.py build
 * (main/CMakeLists.txt lists sources explicitly). Delete after use.
 */

void setup() {
  Serial.begin(115200);
  delay(3000);                       // allow USB-CDC to enumerate

  Serial.println();
  Serial.println("=== XIAO ESP32-C5  Dx -> GPIO  (from installed core) ===");

  const int dx[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10 };
  for (int i = 0; i < 11; i++) {
    Serial.printf("D%-2d = GPIO%d\n", i, dx[i]);
  }

  Serial.println("--- named peripheral aliases (core defaults) ---");
#ifdef TX
  Serial.printf("TX  = GPIO%d    RX  = GPIO%d   (Arduino Serial1 default)\n", TX, RX);
#endif
#ifdef SDA
  Serial.printf("SDA = GPIO%d    SCL = GPIO%d   (D4/D5 I2C default)\n", SDA, SCL);
#endif
#ifdef SS
  Serial.printf("SS=GPIO%d  MOSI=GPIO%d  MISO=GPIO%d  SCK=GPIO%d\n", SS, MOSI, MISO, SCK);
#endif
#ifdef LED_BUILTIN
  Serial.printf("LED_BUILTIN = GPIO%d\n", LED_BUILTIN);
#endif

  Serial.println("=========================================================");
  Serial.println("KEY QUESTION: D4 = GPIO? (modem R, C5 TX)   D5 = GPIO? (modem T, C5 RX)");
}

void loop() {
  delay(5000);
  Serial.println("(probe idle — the table above is the ground truth)");
}
