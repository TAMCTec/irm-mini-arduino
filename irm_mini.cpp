/*!
 * @file IRM_Mini.cpp
 *
 * @mainpage GFX-compatible layer for NeoPixel matrices.
 *
 * @section intro_sec Introduction
 *
 * Arduino library to control single and tiled matrices of WS2811- and
 * WS2812-based RGB LED devices such as the Adafruit NeoPixel Shield or
 * displays assembled from NeoPixel strips, making them compatible with
 * the Adafruit_GFX graphics library.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_NeoPixel"> Adafruit_NeoPixel</a>
 * and <a
 * href="https://github.com/adafruit/Adafruit-GFX-Library"> Adafruit_GFX</a>
 * being present on your system. Please make sure you have installed the
 * latest versions before using this library.
 *
 * @section author Author
 *
 * Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
 *
 * @section license License
 *
 * This file is part of the Adafruit NeoMatrix library.
 *
 * NeoMatrix is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * NeoMatrix is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with NeoMatrix.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "gamma.h"
#include <irm_mini.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#else
#ifndef pgm_read_byte
#define pgm_read_byte(addr)                                                    \
  (*(const unsigned char *)(addr)) ///< PROGMEM concept doesn't apply on ESP8266
#endif
#endif

#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b)                                                   \
  {                                                                            \
    uint16_t t = a;                                                            \
    a = b;                                                                     \
    b = t;                                                                     \
  } ///< Swap contents of two uint16_t variables
#endif

// Normally IRM mini doesn't need to flip for simplifies wiring
#define NEO_TILE_ZIGZAG_NOFLIP

static const uint8_t PROGMEM FONT7_BITMAT[][8] = {
  {4,  B1111,  B1001,  B1001,  B1001,  B1001,  B1111,  B0000}, // ERROR_CHAR
  {1,     B0,     B0,     B0,     B0,     B0,     B0,     B0}, // " "
  {1,     B1,     B1,     B1,     B1,     B0,     B1,     B0}, // !
  {3,   B101,   B101,   B000,   B000,   B000,   B000,   B000}, // "
  {5, B01010, B11111, B01010, B11111, B01010, B01010, B00000}, // #
  {4,  B0010,  B0111,  B1000,  B1111,  B0001,  B1110,  B0100}, // $
  {5, B11000, B11001, B00010, B00100, B01000, B10011, B00011}, // % TODO
  {5, B01000, B10100, B01000, B10101, B10010, B10010, B00000}, // &
  {1,     B1,     B1,     B0,     B0,     B0,     B0,     B0}, // '
  {2,    B01,    B10,    B10,    B10,    B10,    B01,    B00}, // (
  {2,    B10,    B01,    B01,    B01,    B01,    B10,    B00}, // )
  {5, B00000, B10101, B01110, B00100, B01110, B10101, B00000}, // *
  {5, B00000, B00100, B00100, B11111, B00100, B00100, B00000}, // +
  {2,    B00,    B00,    B00,    B00,    B00,    B01,    B10}, // ,
  {5, B00000, B00000, B00000, B11111, B00000, B00000, B00000}, // -
  {1,     B0,     B0,     B0,     B0,     B0,     B1,     B0}, // .
  {5, B00000, B00001, B00010, B00100, B01000, B10000, B00000}, // /
  {5, B01110, B10001, B10011, B10101, B11001, B10001, B01110}, // 0
  {5, B00100, B01100, B00100, B00100, B00100, B00100, B11111}, // 1
  {5, B01110, B10001, B00001, B00110, B01000, B10000, B11111}, // 2
  {5, B01110, B10001, B00001, B00110, B00001, B10001, B01110}, // 3
  {5, B00011, B00101, B01001, B10001, B11111, B00001, B00001}, // 4
  {5, B11111, B10000, B11110, B00001, B00001, B10001, B01110}, // 5
  {5, B00110, B01000, B10000, B11110, B10001, B10001, B01110}, // 6
  {5, B11111, B10001, B00001, B00010, B00100, B00100, B00100}, // 7
  {5, B01110, B10001, B10001, B01110, B10001, B10001, B01110}, // 8
  {5, B01110, B10001, B10001, B01111, B00001, B00010, B01100}, // 9
  {1,     B0,     B1,     B0,     B0,     B1,     B0,     B0}, // :
  {2,    B00,    B01,    B00,    B00,    B01,    B10,    B00}, // ;
  {3,   B000,   B001,   B010,   B100,   B010,   B001,   B000}, // <
  {5, B00000, B00000, B11111, B00000, B11111, B00000, B00000}, // =
  {3,   B000,   B100,   B010,   B001,   B010,   B100,   B000}, // >
  {3,   B110,   B001,   B010,   B010,   B000,   B010,   B000}, // ?
  {5, B01110, B10001, B10101, B10110, B10000, B01111, B00000}, // @
  {4,  B0110,  B1001,  B1001,  B1111,  B1001,  B1001,  B0000}, // A
  {4,  B1110,  B1001,  B1110,  B1001,  B1001,  B1110,  B0000}, // B
  {4,  B0110,  B1001,  B1000,  B1000,  B1001,  B0110,  B0000}, // C
  {4,  B1110,  B1001,  B1001,  B1001,  B1001,  B1110,  B0000}, // D
  {4,  B1111,  B1000,  B1110,  B1000,  B1000,  B1111,  B0000}, // E
  {4,  B1111,  B1000,  B1110,  B1000,  B1000,  B1000,  B0000}, // F
  {4,  B0110,  B1001,  B1000,  B1011,  B1001,  B0110,  B0000}, // G
  {4,  B1001,  B1001,  B1111,  B1001,  B1001,  B1001,  B0000}, // H
  {3,   B111,   B010,   B010,   B010,   B010,   B111,   B000}, // I
  {4,  B0001,  B0001,  B0001,  B1001,  B1001,  B0110,  B0000}, // J
  {4,  B1001,  B1010,  B1010,  B1100,  B1010,  B1001,  B0000}, // K
  {4,  B1000,  B1000,  B1000,  B1000,  B1000,  B1111,  B0000}, // L
  {5, B10001, B11011, B10101, B10001, B10001, B10001, B00000}, // M
  {4,  B1001,  B1001,  B1101,  B1011,  B1001,  B1001,  B0000}, // N
  {4,  B0110,  B1001,  B1001,  B1001,  B1001,  B0110,  B0000}, // O
  {4,  B1110,  B1001,  B1001,  B1110,  B1000,  B1000,  B0000}, // P
  {5, B01110, B10001, B10001, B10001, B10101, B01110, B00001}, // Q
  {4,  B1110,  B1001,  B1001,  B1110,  B1010,  B1001,  B0000}, // R
  {4,  B0110,  B1001,  B0100,  B0010,  B1001,  B0110,  B0000}, // S
  {5, B11111, B00100, B00100, B00100, B00100, B00100, B00000}, // T
  {4,  B1001,  B1001,  B1001,  B1001,  B1001,  B0110,  B0000}, // U
  {4,  B1001,  B1001,  B1001,  B1001,  B1010,  B0100,  B0000}, // V
  {5, B10001, B10001, B10001, B10101, B10101, B01010, B00000}, // W
  {4,  B1001,  B1001,  B0110,  B1001,  B1001,  B1001,  B0000}, // X
  {5, B10001, B10001, B01010, B00100, B00100, B00100, B00000}, // Y
  {4,  B1111,  B0001,  B0010,  B0100,  B1000,  B1111,  B0000}, // Z
  {2,    B11,    B10,    B10,    B10,    B10,    B11,    B00}, // [
  {5, B00000, B10000, B01000, B00100, B00010, B00001, B00000}, // \ 
  {2,    B11,    B01,    B01,    B01,    B01,    B11,    B00}, // ]
  {3,   B010,   B101,   B000,   B000,   B000,   B000,   B000}, // ^
  {4,  B0000,  B0000,  B0000,  B0000,  B0000,  B1111,  B0000}, // _
  {2,    B10,    B01,    B00,    B00,    B00,    B00,    B00}, // `
  {4,  B0000,  B0000,  B0111,  B1001,  B1001,  B0111,  B0000}, // a
  {4,  B0000,  B1000,  B1110,  B1001,  B1001,  B1110,  B0000}, // b
  {3,   B000,   B000,   B011,   B100,   B100,   B011,   B000}, // c
  {4,  B0000,  B0001,  B0111,  B1001,  B1001,  B0111,  B0000}, // d
  {4,  B0000,  B0000,  B0110,  B1011,  B1100,  B0110,  B0000}, // e
  {3,   B000,   B001,   B010,   B111,   B010,   B010,   B000}, // f
  {4,  B0000,  B0111,  B1001,  B1001,  B0111,  B0001,  B0110}, // g
  {4,  B0000,  B1000,  B1110,  B1001,  B1001,  B1001,  B0000}, // h
  {1,     B0,     B1,     B0,     B1,     B1,     B1,     B0}, // i
  {2,    B01,    B00,    B01,    B01,    B01,    B10,    B00}, // j
  {4,  B0000,  B1000,  B1001,  B1010,  B1110,  B1001,  B0000}, // k
  {1,     B0,     B1,     B1,     B1,     B1,     B1,     B0}, // l
  {5, B00000, B00000, B11110, B10101, B10101, B10101, B00000}, // m
  {4,  B0000,  B0000,  B1110,  B1001,  B1001,  B1001,  B0000}, // n
  {4,  B0000,  B0000,  B0110,  B1001,  B1001,  B0110,  B0000}, // o
  {4,  B0000,  B0000,  B0110,  B1001,  B1001,  B1110,  B1000}, // p
  {4,  B0000,  B0000,  B0110,  B1001,  B1001,  B0111,  B0001}, // q
  {3,   B000,   B000,   B101,   B110,   B100,   B100,   B000}, // r
  {4,  B0000,  B0000,  B0111,  B1100,  B0011,  B1110,  B0000}, // s
  {3,   B000,   B010,   B111,   B010,   B010,   B001,   B000}, // t
  {4,  B0000,  B0000,  B1001,  B1001,  B1001,  B0111,  B0000}, // u
  {4,  B0000,  B0000,  B1001,  B1001,  B1010,  B0100,  B0000}, // v
  {5, B00000, B00000, B10101, B10101, B01010, B01010, B00000}, // w
  {3,   B000,   B000,   B101,   B010,   B010,   B101,   B000}, // x
  {4,  B0000,  B1001,  B1001,  B0111,  B0001,  B0110,  B0000}, // y
  {4,  B0000,  B0000,  B1111,  B0010,  B0100,  B1111,  B0000}, // z
  {3,   B001,   B010,   B010,   B100,   B010,   B010,   B001}, // {
  {1,     B1,     B1,     B1,     B1,     B1,     B1,     B1}, // |
  {3,   B100,   B010,   B010,   B001,   B010,   B010,   B100}, // }
  {5, B00000, B00000, B01000, B10101, B00010, B00000, B00000}, // ~
};

static const uint8_t PROGMEM FONT5_BITMAT[][6] = {
  {4,  B1111,  B1001,  B1001,  B1001,  B1111}, // ERROR_CHAR
  {1,     B0,     B0,     B0,     B0,     B0}, // " "
  {1,     B1,     B1,     B1,     B0,     B1}, // !
  {3,   B101,   B101,   B000,   B000,   B000}, // "
  {5, B01010, B11111, B01010, B11111, B01010}, // #
  {4,  B1111,  B1001,  B1001,  B1001,  B1111}, // $ TODO
  {4,  B0000,  B1001,  B0010,  B0100,  B1001}, // % TODO
  {4,  B1111,  B1001,  B1001,  B1001,  B1111}, // & TODO
  {1,     B1,     B1,     B0,     B0,     B0}, // '
  {2,    B01,    B10,    B10,    B10,    B01}, // (
  {2,    B10,    B01,    B01,    B01,    B10}, // )
  {5, B10101, B01110, B00100, B01110, B10101}, // *
  {3,   B000,   B010,   B111,   B010,   B000}, // +
  {2,    B00,    B00,    B00,    B01,    B10}, // ,
  {3,   B000,   B000,   B111,   B000,   B000}, // -
  {1,     B0,     B0,     B0,     B1,     B0}, // .
  {4,  B0000,  B0001,  B0010,  B0100,  B1000}, // /
  {4,  B0111,  B1001,  B1001,  B1001,  B1111}, // 0
  {2,    B01,    B11,    B01,    B01,    B01}, // 1
  {4,  B1110,  B0001,  B0111,  B1000,  B1111}, // 2
  {4,  B1110,  B0001,  B0110,  B0001,  B1111}, // 3
  {4,  B0011,  B0101,  B1001,  B1111,  B0001}, // 4
  {4,  B1110,  B1000,  B1111,  B0001,  B1111}, // 5
  {4,  B0110,  B1000,  B1111,  B1001,  B1111}, // 6
  {4,  B1111,  B0001,  B0001,  B0010,  B0100}, // 7
  {4,  B0111,  B1001,  B1111,  B1001,  B1111}, // 8
  {4,  B1111,  B1001,  B1111,  B0001,  B0110}, // 9
  {1,     B0,     B1,     B0,     B1,     B0}, // :
  {2,    B00,    B01,    B00,    B01,    B10}, // ;
  {3,   B001,   B010,   B100,   B010,   B001}, // <
  {4,  B0000,  B1111,  B0000,  B1111,  B0000}, // =
  {3,   B100,   B010,   B001,   B010,   B100}, // >
  {3,   B110,   B001,   B111,   B000,   B010}, // ?
  {3,   B100,   B010,   B001,   B010,   B100}, // @ TODO
  {4,  B0111,  B1001,  B1001,  B1111,  B1001}, // A
  {4,  B1110,  B1001,  B1111,  B1001,  B1111}, // B
  {4,  B0110,  B1001,  B1000,  B1001,  B0110}, // C
  {4,  B1110,  B1001,  B1001,  B1001,  B1110}, // D
  {4,  B0111,  B1000,  B1110,  B1000,  B1111}, // E
  {4,  B1111,  B1000,  B1110,  B1000,  B1000}, // F
  {4,  B0110,  B1000,  B1011,  B1001,  B0111}, // G
  {4,  B1001,  B1001,  B1111,  B1001,  B1001}, // H
  {3,   B111,   B010,   B010,   B010,   B111}, // I
  {4,  B0001,  B0001,  B0001,  B1001,  B0110}, // J
  {4,  B1001,  B1001,  B1110,  B1001,  B1001}, // K
  {4,  B1000,  B1000,  B1000,  B1000,  B1111}, // L
  {5, B10001, B11011, B10101, B10101, B10001}, // M
  {4,  B1001,  B1101,  B1011,  B1001,  B1001}, // N
  {4,  B0110,  B1001,  B1001,  B1001,  B0110}, // O
  {4,  B1110,  B1001,  B1001,  B1110,  B1000}, // P
  {4,  B0110,  B1001,  B1001,  B1010,  B0101}, // Q
  {4,  B0110,  B1001,  B1001,  B1110,  B1001}, // R
  {4,  B0111,  B1000,  B1111,  B0001,  B1111}, // S
  {5, B11111, B00100, B00100, B00100, B00100}, // T
  {4,  B1001,  B1001,  B1001,  B1001,  B0110}, // U
  {4,  B1001,  B1001,  B1001,  B1010,  B0100}, // V
  {5, B10001, B10101, B10101, B10101, B01111}, // W
  {4,  B1001,  B1001,  B0110,  B1001,  B1001}, // X
  {4,  B1001,  B1001,  B1001,  B0111,  B0001}, // Y
  {4,  B1111,  B0001,  B0110,  B1000,  B1111}, // Z
  {2,    B11,    B10,    B10,    B10,    B11}, // [
  {4,  B0000,  B1000,  B0100,  B0010,  B0001}, // \ 
  {2,    B11,    B01,    B01,    B01,    B11}, // ]
  {3,   B010,   B101,   B000,   B000,   B000}, // ^
  {3,   B000,   B000,   B000,   B000,   B111}, // _
  {2,    B10,    B01,    B00,    B00,    B00}, // `
  {4,  B0111,  B1001,  B1001,  B1111,  B1001}, // A
  {4,  B1110,  B1001,  B1111,  B1001,  B1111}, // B
  {4,  B0110,  B1001,  B1000,  B1001,  B0110}, // C
  {4,  B1110,  B1001,  B1001,  B1001,  B1110}, // D
  {4,  B0111,  B1000,  B1110,  B1000,  B1111}, // E
  {4,  B1111,  B1000,  B1110,  B1000,  B1000}, // F
  {4,  B0110,  B1000,  B1011,  B1001,  B0111}, // G
  {4,  B1001,  B1001,  B1111,  B1001,  B1001}, // H
  {3,   B111,   B010,   B010,   B010,   B111}, // I
  {4,  B0001,  B0001,  B0001,  B1001,  B0110}, // J
  {4,  B1001,  B1001,  B1110,  B1001,  B1001}, // K
  {4,  B1000,  B1000,  B1000,  B1000,  B1111}, // L
  {5, B10001, B11011, B10101, B10101, B10001}, // M
  {4,  B1001,  B1101,  B1011,  B1001,  B1001}, // N
  {4,  B0110,  B1001,  B1001,  B1001,  B0110}, // O
  {4,  B1110,  B1001,  B1001,  B1110,  B1000}, // P
  {4,  B0110,  B1001,  B1001,  B1010,  B0101}, // Q
  {4,  B0110,  B1001,  B1001,  B1110,  B1001}, // R
  {4,  B0111,  B1000,  B1111,  B0001,  B1111}, // S
  {5, B11111, B00100, B00100, B00100, B00100}, // T
  {4,  B1001,  B1001,  B1001,  B1001,  B0110}, // U
  {4,  B1001,  B1001,  B1001,  B1010,  B0100}, // V
  {5, B10001, B10101, B10101, B10101, B01111}, // W
  {4,  B1001,  B1001,  B0110,  B1001,  B1001}, // X
  {4,  B1001,  B1001,  B1001,  B0111,  B0001}, // Y
  {4,  B1111,  B0001,  B0110,  B1000,  B1111}, // Z
  {3,   B001,   B010,   B110,   B010,   B001}, // {
  {1,     B1,     B1,     B1,     B1,     B1}, // |
  {3,   B100,   B010,   B011,   B010,   B100}, // }
  {4,  B0000,  B0101,  B1010,  B0000,  B0000}, // ~
};

// Constructor for single matrix:
IRM_Mini::IRM_Mini(int w, int h, uint8_t pin,
                                       uint8_t matrixType, neoPixelType ledType)
    : Adafruit_GFX(w, h), Adafruit_NeoPixel(w * h, pin, ledType),
      type(matrixType), matrixWidth(w), matrixHeight(h), tilesX(0), tilesY(0),
      remapFn(NULL) {}

// Constructor for tiled matrices:
IRM_Mini::IRM_Mini(uint8_t mW, uint8_t mH, uint8_t tX,
                                       uint8_t tY, uint8_t pin,
                                       uint8_t matrixType, neoPixelType ledType)
    : Adafruit_GFX(mW * tX, mH * tY),
      Adafruit_NeoPixel(mW * mH * tX * tY, pin, ledType), type(matrixType),
      matrixWidth(mW), matrixHeight(mH), tilesX(tX), tilesY(tY), remapFn(NULL) {
}

// Expand 16-bit input color (Adafruit_GFX colorspace) to 24-bit (NeoPixel)
// (w/gamma adjustment)
static uint32_t expandColor(uint16_t color) {
  return ((uint32_t)pgm_read_byte(&gamma5[color >> 11]) << 16) |
         ((uint32_t)pgm_read_byte(&gamma6[(color >> 5) & 0x3F]) << 8) |
         pgm_read_byte(&gamma5[color & 0x1F]);
}

uint16_t IRM_Mini::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}

// Pass raw color value to set/enable passthrough
void IRM_Mini::setPassThruColor(uint32_t c) {
  passThruColor = c;
  passThruFlag = true;
}

// Call without a value to reset (disable passthrough)
void IRM_Mini::setPassThruColor(void) { passThruFlag = false; }

void IRM_Mini::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    return;

  int16_t t;
  switch (rotation) {
  case 1:
    t = x;
    x = WIDTH - 1 - y;
    y = t;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    t = x;
    x = y;
    y = HEIGHT - 1 - t;
    break;
  }

  int tileOffset = 0, pixelOffset;

  if (remapFn) { // Custom X/Y remapping function
    pixelOffset = (*remapFn)(x, y);
  } else { // Standard single matrix or tiled matrices

    uint8_t corner = type & NEO_MATRIX_CORNER;
    uint16_t minor, major, majorScale;

    if (tilesX) { // Tiled display, multiple matrices
      uint16_t tile;

      minor = x / matrixWidth;           // Tile # X/Y; presume row major to
      major = y / matrixHeight,          // start (will swap later if needed)
          x = x - (minor * matrixWidth); // Pixel X/Y within tile
      y = y - (major * matrixHeight);    // (-* is less math than modulo)

      // Determine corner of entry, flip axes if needed
      if (type & NEO_TILE_RIGHT)
        minor = tilesX - 1 - minor;
      if (type & NEO_TILE_BOTTOM)
        major = tilesY - 1 - major;

      // Determine actual major axis of tiling
      if ((type & NEO_TILE_AXIS) == NEO_TILE_ROWS) {
        majorScale = tilesX;
      } else {
        _swap_uint16_t(major, minor);
        majorScale = tilesY;
      }

      // Determine tile number
      if ((type & NEO_TILE_SEQUENCE) == NEO_TILE_PROGRESSIVE) {
        // All tiles in same order
        tile = major * majorScale + minor;
      } else {
        // Zigzag; alternate rows change direction.  On these rows,
        // this also flips the starting corner of the matrix for the
        // pixel math later.
        if (major & 1) {
          #ifndef NEO_TILE_ZIGZAG_NOFLIP
          corner ^= NEO_MATRIX_CORNER;
          #endif
          tile = (major + 1) * majorScale - 1 - minor;
        } else {
          tile = major * majorScale + minor;
        }

      }

      // Index of first pixel in tile
      tileOffset = tile * matrixWidth * matrixHeight;

    } // else no tiling (handle as single tile)

    // Find pixel number within tile
    minor = x; // Presume row major to start (will swap later if needed)
    major = y;

    // Determine corner of entry, flip axes if needed
    if (corner & NEO_MATRIX_RIGHT)
      minor = matrixWidth - 1 - minor;
    if (corner & NEO_MATRIX_BOTTOM)
      major = matrixHeight - 1 - major;

    // Determine actual major axis of matrix
    if ((type & NEO_MATRIX_AXIS) == NEO_MATRIX_ROWS) {
      majorScale = matrixWidth;
    } else {
      _swap_uint16_t(major, minor);
      majorScale = matrixHeight;
    }

    // Determine pixel number within tile/matrix
    if ((type & NEO_MATRIX_SEQUENCE) == NEO_MATRIX_PROGRESSIVE) {
      // All lines in same order
      pixelOffset = major * majorScale + minor;
    } else {
      // Zigzag; alternate rows change direction.
      if (major & 1)
        pixelOffset = (major + 1) * majorScale - 1 - minor;
      else
        pixelOffset = major * majorScale + minor;
    }
  }

  setPixelColor(tileOffset + pixelOffset,
                passThruFlag ? passThruColor : expandColor(color));
}

void IRM_Mini::fillScreen(uint16_t color) {
  uint16_t i, n;
  uint32_t c;

  c = passThruFlag ? passThruColor : expandColor(color);
  n = numPixels();
  for (i = 0; i < n; i++)
    setPixelColor(i, c);
}

void IRM_Mini::setRemapFunction(uint16_t (*fn)(uint16_t, uint16_t)) {
  remapFn = fn;
}

void IRM_Mini::drawAscii(uint16_t x, uint16_t y, char text, uint16_t color, uint8_t fontSize) {
  this->drawAscii(x, y, String(text), color, fontSize);
}
void IRM_Mini::drawAscii(uint16_t x, uint16_t y, char* text, uint16_t color, uint8_t fontSize) {
  this->drawAscii(x, y, String(text), color, fontSize);
}
void IRM_Mini::drawAscii(uint16_t x, uint16_t y, const char* text, uint16_t color, uint8_t fontSize) {
  this->drawAscii(x, y, String(text), color, fontSize);
}

void IRM_Mini::drawAscii(uint16_t x, uint16_t y, String text, uint16_t color, uint8_t fontSize) {
  uint8_t* bitmap;
  for (uint8_t i=0; i<text.length(); i++) {
    uint8_t asciiValue = text[i] - 32 + 1;
    if (asciiValue < 0) asciiValue = 0;
    switch (fontSize) {
      case FONT7: bitmap = (uint8_t*)FONT7_BITMAT[asciiValue]; break;
      case FONT5: bitmap = (uint8_t*)FONT5_BITMAT[asciiValue]; break;
      default: Serial.println("Unknown font size"); while(1); break;
    }
    uint8_t width = bitmap[0];
    this->fillRect(x, y, x+width, y+fontSize, 0);
    uint8_t fontSizeOffset = fontSize - fontSize;
    for (uint8_t j=1; j<=fontSize; j++) {
      uint8_t line = bitmap[j];
      for (uint8_t k=0; k<width; k++) {
        if (line & (1 << k)) {
          this->drawPixel(x+width-k-1, y+j+fontSizeOffset-1, color);
        }
      }
    }
    x += width;
    this->fillRect(x, y, x+1, y+fontSize, 0);
    x += 1;
  }
}

// Draw a RGB 8bit bitmap
void IRM_Mini::drawRGBBitmap(int16_t startx, int16_t starty, const uint32_t *bitmap, int16_t w, int16_t h, bool cover) {
  // work around "a15 cannot be used in asm here" compiler bug when using an array on ESP8266
  // uint16_t RGB_bmp_fixed[w * h];
  for (uint16_t x=0; x<w; x++) {
    for (uint16_t y=0; y<h; y++) {
      uint32_t color = bitmap[x + (y * w)];
      if (color == 0 && !cover) continue;
      uint8_t r = color >> 16 & 0xFF;
      uint8_t g = color >> 8 & 0xFF;
      uint8_t b = color & 0xFF;
      this->drawPixel(startx + x, starty + y, this->Color(r, g, b));
    }
  }
}
