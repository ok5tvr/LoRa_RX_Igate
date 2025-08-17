// board_pins.h
#pragma once

// Vyber desku při překladu: -DBOARD_HELTEC nebo -DBOARD_TBEAM nebo -DBOARD_GENERIC
#if defined(BOARD_HELTEC)
// Heltec WiFi LoRa 32 (V2)
  #define OLED_SDA 4
  #define OLED_SCL 15
  #define OLED_RST 16
  #define LORA_SS   18
  #define LORA_RST  14
  #define LORA_DIO0 26
  #define LORA_DIO1 33
  #define LORA_DIO2 32
#elif defined(BOARD_TBEAM)
// TTGO T-Beam (bez GPS části zde)
  #define OLED_SDA 21
  #define OLED_SCL 22
  #define OLED_RST 16
  #define LORA_SS   18
  #define LORA_RST  23
  #define LORA_DIO0 26
  #define LORA_DIO1 33
  #define LORA_DIO2 32
#else
// GENERIC ESP32 + SX127x na breadboardu
  #define OLED_SDA 21
  #define OLED_SCL 22
  #define OLED_RST 16
  #define LORA_SS   18
  #define LORA_RST  23
  #define LORA_DIO0 26
  #define LORA_DIO1 33
  #define LORA_DIO2 32
#endif
// TTGO board v1
#if defined(BOARD_TTGO_V1)
  #define OLED_SDA 21
  #define OLED_SCL 22
  #define OLED_RST 16
  #define LORA_SS   18
  #define LORA_RST  23
  #define LORA_DIO0 26
  #define LORA_DIO1 33
  #define LORA_DIO2 32
#endif