#pragma once

#include "driver/gpio.h"

// Placa: ESP32-S3 WROOM CAM (N16R8: 16 MB flash, 8 MB PSRAM octal)
// Pines ocupados:
//   Cámara : 4 (SIOD), 5 (SIOC), 6 (VSYNC), 7 (HREF), 8, 9, 10, 11, 12, 13 (PCLK),
//            15 (XCLK), 16, 17, 18
//   PSRAM  : 33-37 (octal)
//   SD     : 38, 39, 40
//   LED    : 2 (integrado; no usar para I2C, deja la línea a medio voltaje)
//   Evitar : 0, 3, 45, 46 (strapping), 19, 20 (USB), 43, 44 (UART consola)

// MLX90614 (I2C/SMBus, máx. 100 kHz)
#define PIN_I2C_SDA     GPIO_NUM_41
#define PIN_I2C_SCL     GPIO_NUM_42

// Radar HLK-LD2410B (UART 256000 baudios, alimentación 5V)
#define PIN_RADAR_TX    GPIO_NUM_14     // ESP32 TX -> RX del radar
#define PIN_RADAR_RX    GPIO_NUM_21     // ESP32 RX <- TX del radar
#define PIN_RADAR_OUT   GPIO_NUM_47     // Salida digital de presencia (respaldo)
#define UART_RADAR      UART_NUM_1

// Cámara OV3660 (mapa de la placa ESP32-S3 WROOM CAM)
#define PIN_CAM_XCLK    15
#define PIN_CAM_SIOD    4
#define PIN_CAM_SIOC    5
#define PIN_CAM_VSYNC   6
#define PIN_CAM_HREF    7
#define PIN_CAM_PCLK    13
#define PINES_CAM_DATOS { 11, 9, 8, 10, 12, 18, 17, 16 }   // D0..D7 = Y2..Y9

// Tarjeta SD (SDMMC de 1 línea)
#define PIN_SD_CMD      GPIO_NUM_38
#define PIN_SD_CLK      GPIO_NUM_39
#define PIN_SD_D0       GPIO_NUM_40

// Válvula de gas: optoacoplador (p. ej. PC817) sobre el botón de la válvula, que ALTERNA abrir/cerrar.
// GPIO 48 -> resistencia de 330 Ω -> ánodo del LED; cátodo a GND. Una resistencia de 10 kΩ de GPIO 48
// a GND evita pulsos al arrancar. El transistor del optoacoplador va en paralelo con el botón.
#define PIN_VALVULA_PULSO       GPIO_NUM_48
#define PIN_VALVULA_POSICION    GPIO_NUM_NC     // Final de carrera opcional (contacto a GND = cerrada)
#define DURACION_PULSO_VALVULA_MS   500

// Botón BOOT de la placa (solo es strapping al arrancar; después se usa como entrada)
#define PIN_BOTON_CALIBRACION   GPIO_NUM_0
