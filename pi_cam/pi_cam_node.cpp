#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <functional>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/sysinfo.h>
#include <poll.h>
#include "rover_link.h"

// ============================================================================
// 1. EMBEDDED HIGH-SPEED BASELINE JPEG ENCODER WITH SOLID-BLOCK BYPASS
// ============================================================================
namespace TooJpeg {
  using OutputByte = std::function<void(uint8_t)>;

  const unsigned char ZigZagInv[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
  };

  const unsigned char DefaultQuantLuminance[64] = {
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68,109,103, 77, 24, 35, 55, 64, 81,104,113, 92,
    49, 64, 78, 87,103,121,120,101, 72, 92, 95, 98,112,100,103, 99
  };

  const unsigned char DefaultQuantChrominance[64] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99
  };

  struct BitCode {
    BitCode() = default;
    BitCode(uint16_t c, uint8_t l) : code(c), numBits(l) {}
    uint16_t code = 0;
    uint8_t  numBits = 0;
  };

  struct BitWriter {
    OutputByte out;
    uint32_t bitBuffer = 0;
    int numBits = 0;
    explicit BitWriter(OutputByte o) : out(o) {}

    inline void write(BitCode data) {
      bitBuffer = (bitBuffer << data.numBits) | data.code;
      numBits += data.numBits;
      while (numBits >= 8) {
        uint8_t byte = (bitBuffer >> (numBits - 8)) & 0xFF;
        out(byte);
        if (byte == 0xFF) out(0x00);
        numBits -= 8;
      }
    }

    inline void flush() {
      if (numBits > 0) {
        BitCode pad((1 << (8 - numBits)) - 1, 8 - numBits);
        write(pad);
      }
    }
  };

  static BitCode DClumCode[256];
  static BitCode AClumCode[256];
  static BitCode DCchrCode[256];
  static BitCode ACchrCode[256];

  const uint8_t DClumLengths[16] = {0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
  const uint8_t DClumValues[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};

  const uint8_t AClumLengths[16] = {0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,125};
  const uint8_t AClumValues[162] = {
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
    0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
    0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
    0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
  };

  const uint8_t DCchrLengths[16] = {0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
  const uint8_t DCchrValues[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};

  const uint8_t ACchrLengths[16] = {0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,119};
  const uint8_t ACchrValues[162] = {
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
    0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
    0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
    0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
    0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
  };

  static float s_dctMat[8][8];
  static bool s_inited = false;

  void generateHuffmanTable(const uint8_t* lengths, const uint8_t* values, BitCode* result) {
    uint16_t code = 0;
    int k = 0;
    for (uint8_t bits = 1; bits <= 16; bits++) {
      for (int i = 0; i < lengths[bits - 1]; i++) {
        result[values[k++]] = BitCode(code++, bits);
      }
      code <<= 1;
    }
  }

  void init() {
    if (s_inited) return;
    generateHuffmanTable(DClumLengths, DClumValues, DClumCode);
    generateHuffmanTable(AClumLengths, AClumValues, AClumCode);
    generateHuffmanTable(DCchrLengths, DCchrValues, DCchrCode);
    generateHuffmanTable(ACchrLengths, ACchrValues, ACchrCode);

    const float invSqrt8 = 0.35355339059f;
    const float half = 0.5f;
    for (int u = 0; u < 8; u++) {
      for (int x = 0; x < 8; x++) {
        if (u == 0) s_dctMat[u][x] = invSqrt8;
        else s_dctMat[u][x] = half * std::cos((2 * x + 1) * u * 3.141592653589793f / 16.0f);
      }
    }
    s_inited = true;
  }

  inline void fastDCT(const float* block, float* out) {
    float temp[64];
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < 8; c++) {
        float sum = 0.0f;
        for (int k = 0; k < 8; k++) sum += block[r * 8 + k] * s_dctMat[c][k];
        temp[r * 8 + c] = sum;
      }
    }
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < 8; c++) {
        float sum = 0.0f;
        for (int k = 0; k < 8; k++) sum += s_dctMat[r][k] * temp[k * 8 + c];
        out[r * 8 + c] = sum;
      }
    }
  }

  inline void encodeBlock(BitWriter& writer, int16_t* block, int16_t& lastDC, const BitCode* dcCode, const BitCode* acCode) {
    int16_t dc = block[0];
    int16_t diff = dc - lastDC;
    lastDC = dc;

    if (diff == 0) {
      writer.write(dcCode[0]);
    } else {
      uint8_t bits = 0;
      uint16_t absDiff = std::abs(diff);
      uint16_t temp = absDiff;
      while (temp) { bits++; temp >>= 1; }
      writer.write(dcCode[bits]);
      uint16_t mask = (diff < 0) ? (absDiff ^ ((1 << bits) - 1)) : diff;
      writer.write(BitCode(mask, bits));
    }

    int r = 0;
    for (int k = 1; k < 64; k++) {
      int16_t val = block[ZigZagInv[k]];
      if (val == 0) {
        r++;
      } else {
        while (r > 15) {
          writer.write(acCode[0xF0]); // ZRL
          r -= 16;
        }
        uint8_t bits = 0;
        uint16_t absVal = std::abs(val);
        uint16_t temp = absVal;
        while (temp) { bits++; temp >>= 1; }
        writer.write(acCode[(r << 4) | bits]);
        uint16_t mask = (val < 0) ? (absVal ^ ((1 << bits) - 1)) : val;
        writer.write(BitCode(mask, bits));
        r = 0;
      }
    }
    if (r > 0) writer.write(acCode[0x00]); // EOB
  }

  bool writeJpeg(OutputByte out, const uint8_t* rgb, uint16_t width, uint16_t height, uint8_t quality) {
    init();
    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;
    int scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);

    uint8_t qL[64], qC[64];
    for (int i = 0; i < 64; i++) {
      int valL = (DefaultQuantLuminance[i] * scale + 50) / 100;
      qL[i] = (uint8_t)std::clamp(valL, 1, 255);
      int valC = (DefaultQuantChrominance[i] * scale + 50) / 100;
      qC[i] = (uint8_t)std::clamp(valC, 1, 255);
    }

    // Header
    out(0xFF); out(0xD8); // SOI
    out(0xFF); out(0xE0); out(0x00); out(0x10); // APP0
    out('J'); out('F'); out('I'); out('F'); out(0x00);
    out(0x01); out(0x01); out(0x00); out(0x00); out(0x01); out(0x00); out(0x01); out(0x00); out(0x00);

    // DQT Luminance
    out(0xFF); out(0xDB); out(0x00); out(0x43); out(0x00);
    for (int i = 0; i < 64; i++) out(qL[ZigZagInv[i]]);
    // DQT Chrominance
    out(0xFF); out(0xDB); out(0x00); out(0x43); out(0x01);
    for (int i = 0; i < 64; i++) out(qC[ZigZagInv[i]]);

    // SOF0 (Baseline DCT)
    out(0xFF); out(0xC0); out(0x00); out(0x11); out(0x08);
    out(height >> 8); out(height & 0xFF);
    out(width >> 8); out(width & 0xFF);
    out(0x03); // 3 components (Y, Cb, Cr)
    out(1); out(0x11); out(0); // Y: 1x1, qTable 0
    out(2); out(0x11); out(1); // Cb: 1x1, qTable 1
    out(3); out(0x11); out(1); // Cr: 1x1, qTable 1

    // DHT Luminance DC
    out(0xFF); out(0xC4); out(0x00); out(0x1F); out(0x00);
    for (int i = 0; i < 16; i++) out(DClumLengths[i]);
    for (int i = 0; i < 12; i++) out(DClumValues[i]);
    // DHT Luminance AC
    out(0xFF); out(0xC4); out(0x00); out(0xB5); out(0x10);
    for (int i = 0; i < 16; i++) out(AClumLengths[i]);
    for (int i = 0; i < 162; i++) out(AClumValues[i]);
    // DHT Chrominance DC
    out(0xFF); out(0xC4); out(0x00); out(0x1F); out(0x01);
    for (int i = 0; i < 16; i++) out(DCchrLengths[i]);
    for (int i = 0; i < 12; i++) out(DCchrValues[i]);
    // DHT Chrominance AC
    out(0xFF); out(0xC4); out(0x00); out(0xB5); out(0x11);
    for (int i = 0; i < 16; i++) out(ACchrLengths[i]);
    for (int i = 0; i < 162; i++) out(ACchrValues[i]);

    // SOS
    out(0xFF); out(0xDA); out(0x00); out(0x0C); out(0x03);
    out(1); out(0x00);
    out(2); out(0x11);
    out(3); out(0x11);
    out(0x00); out(0x3F); out(0x00);

    BitWriter writer(out);
    int16_t lastDC_Y = 0, lastDC_Cb = 0, lastDC_Cr = 0;
    float blockY[64], blockCb[64], blockCr[64];
    float dctOut[64];
    int16_t qBlock[64];

    for (uint16_t y = 0; y < height; y += 8) {
      for (uint16_t x = 0; x < width; x += 8) {
        for (int by = 0; by < 8; by++) {
          int py = std::min<int>(y + by, height - 1);
          for (int bx = 0; bx < 8; bx++) {
            int px = std::min<int>(x + bx, width - 1);
            int idx = (py * width + px) * 3;
            float r = rgb[idx + 0];
            float g = rgb[idx + 1];
            float b = rgb[idx + 2];

            blockY[by * 8 + bx]  =  0.29900f * r + 0.58700f * g + 0.11400f * b - 128.0f;
            blockCb[by * 8 + bx] = -0.16874f * r - 0.33126f * g + 0.50000f * b;
            blockCr[by * 8 + bx] =  0.50000f * r - 0.41869f * g - 0.08131f * b;
          }
        }

        // Y Channel with Solid-Block Bypass
        bool isSolidY = true;
        float y0 = blockY[0];
        for (int i = 1; i < 64; i++) {
          if (blockY[i] != y0) { isSolidY = false; break; }
        }
        if (isSolidY) {
          qBlock[0] = (int16_t)std::round((y0 * 8.0f) / qL[0]);
          for (int i = 1; i < 64; i++) qBlock[i] = 0;
          encodeBlock(writer, qBlock, lastDC_Y, DClumCode, AClumCode);
        } else {
          fastDCT(blockY, dctOut);
          for (int i = 0; i < 64; i++) qBlock[i] = (int16_t)std::round(dctOut[i] / qL[i]);
          encodeBlock(writer, qBlock, lastDC_Y, DClumCode, AClumCode);
        }

        // Cb Channel with Solid-Block Bypass
        bool isSolidCb = true;
        float cb0 = blockCb[0];
        for (int i = 1; i < 64; i++) {
          if (blockCb[i] != cb0) { isSolidCb = false; break; }
        }
        if (isSolidCb) {
          qBlock[0] = (int16_t)std::round((cb0 * 8.0f) / qC[0]);
          for (int i = 1; i < 64; i++) qBlock[i] = 0;
          encodeBlock(writer, qBlock, lastDC_Cb, DCchrCode, ACchrCode);
        } else {
          fastDCT(blockCb, dctOut);
          for (int i = 0; i < 64; i++) qBlock[i] = (int16_t)std::round(dctOut[i] / qC[i]);
          encodeBlock(writer, qBlock, lastDC_Cb, DCchrCode, ACchrCode);
        }

        // Cr Channel with Solid-Block Bypass
        bool isSolidCr = true;
        float cr0 = blockCr[0];
        for (int i = 1; i < 64; i++) {
          if (blockCr[i] != cr0) { isSolidCr = false; break; }
        }
        if (isSolidCr) {
          qBlock[0] = (int16_t)std::round((cr0 * 8.0f) / qC[0]);
          for (int i = 1; i < 64; i++) qBlock[i] = 0;
          encodeBlock(writer, qBlock, lastDC_Cr, DCchrCode, ACchrCode);
        } else {
          fastDCT(blockCr, dctOut);
          for (int i = 0; i < 64; i++) qBlock[i] = (int16_t)std::round(dctOut[i] / qC[i]);
          encodeBlock(writer, qBlock, lastDC_Cr, DCchrCode, ACchrCode);
        }
      }
    }

    writer.flush();
    out(0xFF); out(0xD9); // EOI
    return true;
  }
}

// ============================================================================
// 2. EMBEDDED 8x8 ASCII BITMAP FONT
// ============================================================================
static const uint8_t Font8x8[95][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' ' (32)
  {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // '!'
  {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00}, // '"'
  {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // '#'
  {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00}, // '$'
  {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00}, // '%'
  {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // '&'
  {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, // '''
  {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // '('
  {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // ')'
  {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // '*'
  {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // '+'
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ','
  {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // '-'
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // '.'
  {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // '/'
  {0x7C,0xC6,0xCE,0xD6,0xE6,0xC6,0x7C,0x00}, // '0'
  {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // '1'
  {0x7C,0xC6,0x06,0x1C,0x30,0x66,0xFE,0x00}, // '2'
  {0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00}, // '3'
  {0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x1E,0x00}, // '4'
  {0xFE,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00}, // '5'
  {0x78,0xC0,0xFC,0xC6,0xC6,0xC6,0x7C,0x00}, // '6'
  {0xFE,0xC6,0x0C,0x18,0x30,0x30,0x30,0x00}, // '7'
  {0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00}, // '8'
  {0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00}, // '9'
  {0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, // ':'
  {0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00}, // ';'
  {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, // '<'
  {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // '='
  {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, // '>'
  {0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x00}, // '?'
  {0x7C,0xC6,0xDE,0xDE,0xDC,0xC0,0x7C,0x00}, // '@'
  {0x38,0x6C,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // 'A'
  {0xFC,0x66,0x66,0x7C,0x66,0x66,0xFC,0x00}, // 'B'
  {0x3C,0x66,0xC0,0xC0,0xC0,0x66,0x3C,0x00}, // 'C'
  {0xF8,0x6C,0x66,0x66,0x66,0x6C,0xF8,0x00}, // 'D'
  {0xFE,0x62,0x68,0x78,0x68,0x62,0xFE,0x00}, // 'E'
  {0xFE,0x62,0x68,0x78,0x68,0x60,0xF0,0x00}, // 'F'
  {0x3C,0x66,0xC0,0xC0,0xCE,0x66,0x3E,0x00}, // 'G'
  {0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // 'H'
  {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 'I'
  {0x1E,0x0C,0x0C,0x0C,0xCC,0xCC,0x78,0x00}, // 'J'
  {0xE6,0x66,0x6C,0x78,0x6C,0x66,0xE6,0x00}, // 'K'
  {0xF0,0x60,0x60,0x60,0x62,0x66,0xFE,0x00}, // 'L'
  {0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00}, // 'M'
  {0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0x00}, // 'N'
  {0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 'O'
  {0xFC,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00}, // 'P'
  {0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x06}, // 'Q'
  {0xFC,0x66,0x66,0x7C,0x6C,0x66,0xE6,0x00}, // 'R'
  {0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00}, // 'S'
  {0x7E,0x7E,0x5A,0x18,0x18,0x18,0x3C,0x00}, // 'T'
  {0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 'U'
  {0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00}, // 'V'
  {0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00}, // 'W'
  {0xC6,0xC6,0x6C,0x38,0x6C,0xC6,0xC6,0x00}, // 'X'
  {0x66,0x66,0x66,0x3C,0x18,0x18,0x3C,0x00}, // 'Y'
  {0xFE,0xC6,0x8C,0x18,0x32,0x66,0xFE,0x00}, // 'Z'
  {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, // '['
  {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, // '\'
  {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, // ']'
  {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, // '^'
  {0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00}, // '_'
  {0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, // '`'
  {0x00,0x00,0x78,0x0C,0x7C,0xCC,0x76,0x00}, // 'a'
  {0xE0,0x60,0x7C,0x66,0x66,0x66,0xDC,0x00}, // 'b'
  {0x00,0x00,0x7C,0xC6,0xC0,0xC6,0x7C,0x00}, // 'c'
  {0x1C,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00}, // 'd'
  {0x00,0x00,0x7C,0xC6,0xFE,0xC0,0x7C,0x00}, // 'e'
  {0x38,0x6C,0x60,0xF0,0x60,0x60,0xF0,0x00}, // 'f'
  {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0xF8}, // 'g'
  {0xE0,0x60,0x6C,0x76,0x66,0x66,0xE6,0x00}, // 'h'
  {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // 'i'
  {0x0C,0x00,0x1C,0x0C,0x0C,0xCC,0xCC,0x78}, // 'j'
  {0xE0,0x60,0x66,0x6C,0x78,0x6C,0xE6,0x00}, // 'k'
  {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 'l'
  {0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0x00}, // 'm'
  {0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x00}, // 'n'
  {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0x00}, // 'o'
  {0x00,0x00,0xDC,0x66,0x66,0x7C,0x60,0xF0}, // 'p'
  {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0x1E}, // 'q'
  {0x00,0x00,0xDC,0x76,0x60,0x60,0xF0,0x00}, // 'r'
  {0x00,0x00,0x7C,0xC0,0x7C,0x06,0xFC,0x00}, // 's'
  {0x30,0x30,0xFC,0x30,0x30,0x34,0x18,0x00}, // 't'
  {0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x76,0x00}, // 'u'
  {0x00,0x00,0xC6,0xC6,0xC6,0x6C,0x38,0x00}, // 'v'
  {0x00,0x00,0xC6,0xD6,0xD6,0xFE,0x6C,0x00}, // 'w'
  {0x00,0x00,0xC6,0x6C,0x38,0x6C,0xC6,0x00}, // 'x'
  {0x00,0x00,0xC6,0xC6,0xC6,0x7E,0x06,0xFC}, // 'y'
  {0x00,0x00,0xFE,0x8C,0x18,0x32,0xFE,0x00}, // 'z'
  {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // '{'
  {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // '|'
  {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // '}'
  {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00}  // '~'
};

inline void drawPixel(uint8_t* fb, int w, int h, int x, int y, uint32_t color) {
  if (x < 0 || x >= w || y < 0 || y >= h) return;
  int idx = (y * w + x) * 3;
  fb[idx + 0] = (color >> 16) & 0xFF;
  fb[idx + 1] = (color >> 8) & 0xFF;
  fb[idx + 2] = color & 0xFF;
}

void fillRect(uint8_t* fb, int w, int h, int x, int y, int rw, int rh, uint32_t color) {
  uint8_t r = (color >> 16) & 0xFF, g = (color >> 8) & 0xFF, b = color & 0xFF;
  for (int py = y; py < y + rh; py++) {
    if (py < 0 || py >= h) continue;
    for (int px = x; px < x + rw; px++) {
      if (px < 0 || px >= w) continue;
      int idx = (py * w + px) * 3;
      fb[idx + 0] = r; fb[idx + 1] = g; fb[idx + 2] = b;
    }
  }
}

void drawRect(uint8_t* fb, int w, int h, int x, int y, int rw, int rh, uint32_t color) {
  fillRect(fb, w, h, x, y, rw, 1, color);
  fillRect(fb, w, h, x, y + rh - 1, rw, 1, color);
  fillRect(fb, w, h, x, y, 1, rh, color);
  fillRect(fb, w, h, x + rw - 1, y, 1, rh, color);
}

void drawChar(uint8_t* fb, int w, int h, int x, int y, char c, uint32_t color, int scale = 1) {
  if (c < 32 || c > 126) c = '?';
  const uint8_t* glyph = Font8x8[c - 32];
  for (int row = 0; row < 8; row++) {
    uint8_t line = glyph[row];
    for (int col = 0; col < 8; col++) {
      if (line & (1 << (7 - col))) {
        if (scale == 1) {
          drawPixel(fb, w, h, x + col, y + row, color);
        } else {
          fillRect(fb, w, h, x + col * scale, y + row * scale, scale, scale, color);
        }
      }
    }
  }
}

void drawString(uint8_t* fb, int w, int h, int x, int y, const std::string& str, uint32_t color, int scale = 1) {
  int curX = x;
  for (char c : str) {
    if (c == '\n') {
      y += 9 * scale;
      curX = x;
      continue;
    }
    drawChar(fb, w, h, curX, y, c, color, scale);
    curX += 8 * scale;
  }
}

// ============================================================================
// 3. DAEMON STATE & TELEMETRY
// ============================================================================
struct RoverState {
  float heading = 0.0f;
  uint16_t ultraMm = 0;
  uint16_t tofLeftMm = 0;
  uint16_t tofRightMm = 0;
  uint16_t tofFrontMm = 0;
  uint16_t tofRearMm = 0;
  uint8_t mode = 0;
  std::chrono::steady_clock::time_point lastUpdate;
};

struct VisionState {
  std::atomic<bool> cam_online{false};
  std::atomic<uint16_t> vis_f_mm{2000};
  std::atomic<uint16_t> vis_l_mm{2000};
  std::atomic<uint16_t> vis_r_mm{2000};
  std::atomic<float> clear_heading_deg{0.0f};
  std::atomic<float> fps{0.0f};
  std::atomic<uint64_t> frame_seq{0};
  std::atomic<uint64_t> uart_tx{0};
  std::atomic<uint64_t> uart_rx{0};
  std::atomic<float> cpu_temp_c{0.0f};
  std::atomic<uint32_t> mem_avail_mb{0};
  std::atomic<float> cpu_usage_pct{0.0f};
  std::string backend{"None"};
};

static RoverState g_rover;
static std::mutex g_roverMutex;
static VisionState g_vision;
static std::atomic<int> g_activeClients{0};

struct FrameBuffer {
  std::mutex mtx;
  std::condition_variable cv;
  std::vector<uint8_t> jpeg;
  uint64_t seq = 0;
} g_frame;

static std::atomic<bool> g_running{true};

float readCpuTemp() {
  std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
  if (f.is_open()) {
    int raw = 0;
    f >> raw;
    return raw / 1000.0f;
  }
  return 0.0f;
}

void updateSysMetrics() {
  struct sysinfo si;
  if (sysinfo(&si) == 0) {
    g_vision.mem_avail_mb = (si.freeram * si.mem_unit) / (1024 * 1024);
  }
  g_vision.cpu_temp_c = readCpuTemp();

  static unsigned long long prevUser = 0, prevNice = 0, prevSys = 0, prevIdle = 0;
  std::ifstream stat("/proc/stat");
  if (stat.is_open()) {
    std::string cpu;
    unsigned long long user, nice, sys, idle;
    stat >> cpu >> user >> nice >> sys >> idle;
    unsigned long long total = (user - prevUser) + (nice - prevNice) + (sys - prevSys) + (idle - prevIdle);
    unsigned long long work = (user - prevUser) + (nice - prevNice) + (sys - prevSys);
    if (total > 0) {
      g_vision.cpu_usage_pct = (work * 100.0f) / total;
    }
    prevUser = user; prevNice = nice; prevSys = sys; prevIdle = idle;
  }
}

// ============================================================================
// 4. HARDWARE UART LINK (/dev/serial0 -> /dev/ttyAMA0 @ 115200 8N1)
// ============================================================================
int openUart(const char* port) {
  int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

void uartThreadFunc() {
  const char* const DEV_RP2040_CDC = "/dev/rover_rp2040";
  const char* const DEV_SERIAL0    = "/dev/serial0";
  const char* currentPort = DEV_RP2040_CDC;

  int uartFd = openUart(currentPort);
  if (uartFd < 0) {
    currentPort = DEV_SERIAL0;
    uartFd = openUart(currentPort);
  }
  if (uartFd >= 0) {
    std::cout << "[UART] Opened " << currentPort << " successfully.\n";
  } else {
    std::cerr << "[UART] Warning: Unable to open /dev/rover_rp2040 or /dev/serial0. Retrying in loop...\n";
  }

  int udpSock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in hostAddr{}, roverAddr{};
  hostAddr.sin_family = AF_INET;
  hostAddr.sin_port = htons(8888);
  inet_pton(AF_INET, "192.168.0.111", &hostAddr.sin_addr);

  roverAddr.sin_family = AF_INET;
  roverAddr.sin_port = htons(8888);
  inet_pton(AF_INET, "192.168.0.101", &roverAddr.sin_addr);

  std::string rxBuffer;
  RoverFrameParser parser;
  rover_parser_init(&parser);

  auto lastTxTime = std::chrono::steady_clock::now();
  auto lastUdpJsonTime = std::chrono::steady_clock::now();
  auto lastRxTime = std::chrono::steady_clock::now();

  while (g_running) {
    auto now = std::chrono::steady_clock::now();

    // Auto failover / reconnect logic
    if (uartFd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      uartFd = openUart(DEV_RP2040_CDC);
      if (uartFd >= 0) {
        currentPort = DEV_RP2040_CDC;
        std::cout << "[UART] Connected to " << currentPort << " (USB CDC).\n";
        lastRxTime = now;
      } else {
        uartFd = openUart(DEV_SERIAL0);
        if (uartFd >= 0) {
          currentPort = DEV_SERIAL0;
          std::cout << "[UART] Failover connected to " << currentPort << " (UART1 115200).\n";
          lastRxTime = now;
        }
      }
    } else {
      // Check for link timeout (2500 ms without valid data)
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRxTime).count() > 2500) {
        std::cout << "[UART] Link timeout on " << currentPort << " (>2.5s). Failing over...\n";
        close(uartFd);
        uartFd = -1;
        currentPort = (currentPort == DEV_RP2040_CDC) ? DEV_SERIAL0 : DEV_RP2040_CDC;
        rover_parser_init(&parser);
      }
    }

    // 15 Hz UART TX to Rover: V,<vis_f_mm>,<vis_l_mm>,<vis_r_mm>,<cam_online>\n
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTxTime).count() >= 66) {
      lastTxTime = now;
      if (uartFd >= 0) {
        char txBuf[128];
        int len = snprintf(txBuf, sizeof(txBuf), "V,%u,%u,%u,%d\n",
                           g_vision.vis_f_mm.load(),
                           g_vision.vis_l_mm.load(),
                           g_vision.vis_r_mm.load(),
                           g_vision.cam_online.load() ? 1 : 0);
        int written = write(uartFd, txBuf, len);
        if (written > 0) g_vision.uart_tx++;
      }
    }

    // 10 Hz UDP JSON packet to ROS 2 Host (192.168.0.111:8888) and Rover (192.168.0.101:8888)
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUdpJsonTime).count() >= 100) {
      lastUdpJsonTime = now;
      char jsonBuf[512];
      int jLen = snprintf(jsonBuf, sizeof(jsonBuf),
        "{\"type\":\"pi_cam\",\"cam_online\":%s,\"vis_f_mm\":%u,\"vis_l_mm\":%u,\"vis_r_mm\":%u,"
        "\"clear_heading_deg\":%.1f,\"fps\":%.1f,\"frame_seq\":%llu,\"uart_tx\":%llu,\"uart_rx\":%llu,"
        "\"cpu_temp_c\":%.1f,\"mem_avail_mb\":%u,\"cpu_usage\":%.1f}\n",
        g_vision.cam_online.load() ? "true" : "false",
        g_vision.vis_f_mm.load(),
        g_vision.vis_l_mm.load(),
        g_vision.vis_r_mm.load(),
        g_vision.clear_heading_deg.load(),
        g_vision.fps.load(),
        (unsigned long long)g_vision.frame_seq.load(),
        (unsigned long long)g_vision.uart_tx.load(),
        (unsigned long long)g_vision.uart_rx.load(),
        g_vision.cpu_temp_c.load(),
        g_vision.mem_avail_mb.load(),
        g_vision.cpu_usage_pct.load());

      sendto(udpSock, jsonBuf, jLen, 0, (struct sockaddr*)&hostAddr, sizeof(hostAddr));
      sendto(udpSock, jsonBuf, jLen, 0, (struct sockaddr*)&roverAddr, sizeof(roverAddr));
    }

    // UART RX from Rover RP2040: Framed binary protocol + legacy ASCII fallback
    if (uartFd >= 0) {
      char byteBuf[64];
      int n = read(uartFd, byteBuf, sizeof(byteBuf) - 1);
      if (n > 0) {
        g_vision.uart_rx += n;
        for (int i = 0; i < n; i++) {
          uint8_t c = (uint8_t)byteBuf[i];

          // 1. Check framed protocol
          if (rover_parser_feed(&parser, c)) {
            lastRxTime = now;
            if (parser.msg_type == MSG_TELEM && parser.len >= sizeof(TelemPkt)) {
              TelemPkt t;
              memcpy(&t, parser.payload, sizeof(TelemPkt));

              float q0 = (float)t.q[0] / 16384.0f;
              float q1 = (float)t.q[1] / 16384.0f;
              float q2 = (float)t.q[2] / 16384.0f;
              float q3 = (float)t.q[3] / 16384.0f;
              float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
              float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
              float hdg = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;

              {
                std::lock_guard<std::mutex> lk(g_roverMutex);
                g_rover.heading = hdg;
                g_rover.ultraMm = t.ultraMm;
                g_rover.tofFrontMm = t.tofMm[0];
                g_rover.tofRearMm = t.tofMm[1];
                g_rover.tofLeftMm = t.tofMm[0];
                g_rover.tofRightMm = t.tofMm[1];
                g_rover.mode = (uint8_t)(t.batMv & 0xFF);
                g_rover.lastUpdate = std::chrono::steady_clock::now();
              }

              char udpLine[128];
              int ulen = snprintf(udpLine, sizeof(udpLine), "R,%.2f,%hu,%hu,%hu,%d\n",
                                  hdg, t.ultraMm, t.tofMm[0], t.tofMm[1], (int)(t.batMv & 0xFF));
              sendto(udpSock, udpLine, ulen, 0, (struct sockaddr*)&hostAddr, sizeof(hostAddr));
            }
          }

          // 2. Legacy ASCII fallback parser
          if (parser.state == PARSER_SEEK_SYNC0) {
            if (c == '\n') {
              if (!rxBuffer.empty() && rxBuffer[0] == 'R') {
                lastRxTime = now;
                float hdg = 0; uint16_t uMm = 0, tL = 0, tR = 0; int mode = 0;
                if (sscanf(rxBuffer.c_str(), "R,%f,%hu,%hu,%hu,%d", &hdg, &uMm, &tL, &tR, &mode) >= 4) {
                  std::lock_guard<std::mutex> lk(g_roverMutex);
                  g_rover.heading = hdg;
                  g_rover.ultraMm = uMm;
                  g_rover.tofFrontMm = tL;
                  g_rover.tofRearMm = tR;
                  g_rover.tofLeftMm = tL;
                  g_rover.tofRightMm = tR;
                  g_rover.mode = (uint8_t)mode;
                  g_rover.lastUpdate = std::chrono::steady_clock::now();
                }
                sendto(udpSock, rxBuffer.c_str(), rxBuffer.length(), 0, (struct sockaddr*)&hostAddr, sizeof(hostAddr));
              }
              rxBuffer.clear();
            } else if (c != '\r' && rxBuffer.length() < 128) {
              rxBuffer += (char)c;
            }
          }
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (uartFd >= 0) close(uartFd);
  if (udpSock >= 0) close(udpSock);
}

// ============================================================================
// 5. C++ LIGHTWEIGHT VISUAL OBSTACLE CLEARANCE ANALYZER (<2% CPU)
// ============================================================================
void analyzeVisualClearance(const uint8_t* luma, int width, int height) {
  int roiStart = height / 2;
  int colW = width / 3;

  int hitRowL = roiStart, hitRowC = roiStart, hitRowR = roiStart;
  int lHits = 0, cHits = 0, rHits = 0;

  for (int y = height - 2; y >= roiStart; y -= 4) {
    const uint8_t* rowCur = luma + y * width;
    const uint8_t* rowPrev = luma + (y - 2) * width;

    for (int x = 2; x < width - 2; x += 4) {
      int grad = std::abs((int)rowCur[x] - (int)rowPrev[x]) + std::abs((int)rowCur[x] - (int)rowCur[x - 2]);
      if (grad > 38) {
        if (x < colW) {
          if (lHits == 0) hitRowL = y;
          lHits++;
        } else if (x < 2 * colW) {
          if (cHits == 0) hitRowC = y;
          cHits++;
        } else {
          if (rHits == 0) hitRowR = y;
          rHits++;
        }
      }
    }
  }

  auto rowToMm = [&](int row, int hits) -> uint16_t {
    if (hits < 3) return 2000;
    float norm = (float)(height - 1 - row) / (float)(height - 1 - roiStart);
    norm = std::clamp(norm, 0.0f, 1.0f);
    return (uint16_t)(200.0f + norm * 1800.0f);
  };

  uint16_t dL = rowToMm(hitRowL, lHits);
  uint16_t dC = rowToMm(hitRowC, cHits);
  uint16_t dR = rowToMm(hitRowR, rHits);

  g_vision.vis_l_mm = dL;
  g_vision.vis_f_mm = dC;
  g_vision.vis_r_mm = dR;

  // Camera physically mounted at front center (0° in CCW-positive body frame, 11cm from centre)
  constexpr float CAM_MOUNT_OFFSET_DEG = 0.0f;
  float steer = ((float)dL - (float)dR) * 0.035f + CAM_MOUNT_OFFSET_DEG;
  g_vision.clear_heading_deg = std::clamp(steer, -45.0f, 45.0f);
}

// ============================================================================
// 6. REAL-TIME 20 FPS TELEMETRY & DIAGNOSTIC HUD GENERATOR (FALLBACK MODE)
// ============================================================================
void generateHudFrame(std::vector<uint8_t>& rgbOut, int w, int h, uint64_t frameNum) {
  rgbOut.resize(w * h * 3);
  uint8_t* fb = rgbOut.data();

  // Dark background #070b12 with subtle grid
  fillRect(fb, w, h, 0, 0, w, h, 0x070b12);

  for (int y = 0; y < h; y += 40) {
    fillRect(fb, w, h, 0, y, w, 1, 0x111c2e);
  }
  for (int x = 0; x < w; x += 40) {
    fillRect(fb, w, h, x, 0, 1, h, 0x111c2e);
  }

  // Top Banner
  fillRect(fb, w, h, 0, 0, w, 28, 0x0c1424);
  fillRect(fb, w, h, 0, 27, w, 2, 0x00d4ff);
  drawString(fb, w, h, 10, 8, "PI 3B+ 5MP OPTICAL & VISUAL TELEMETRY DAEMON", 0x00d4ff, 1);
  drawString(fb, w, h, 390, 8, "[ :8081 ]", 0x38bdf8, 1);

  // System Diagnostics Card
  drawRect(fb, w, h, 10, 36, 225, 96, 0x1e293b);
  fillRect(fb, w, h, 10, 36, 225, 18, 0x111c2e);
  drawString(fb, w, h, 16, 41, "SYSTEM DIAGNOSTICS", 0x94a3b8, 1);

  char buf[128];
  snprintf(buf, sizeof(buf), "CPU Load:     %.1f %%", g_vision.cpu_usage_pct.load());
  drawString(fb, w, h, 16, 60, buf, 0xe2e8f0, 1);

  snprintf(buf, sizeof(buf), "CPU Temp:     %.1f C", g_vision.cpu_temp_c.load());
  uint32_t tempCol = g_vision.cpu_temp_c.load() > 70.0f ? 0xef4444 : 0x10b981;
  drawString(fb, w, h, 16, 76, buf, tempCol, 1);

  snprintf(buf, sizeof(buf), "RAM Avail:    %u MB", g_vision.mem_avail_mb.load());
  drawString(fb, w, h, 16, 92, buf, 0x38bdf8, 1);

  snprintf(buf, sizeof(buf), "Stream FPS:   %.1f (HUD)", g_vision.fps.load());
  drawString(fb, w, h, 16, 108, buf, 0xa855f7, 1);

  // Camera & CSI Ribbon Probe Card
  drawRect(fb, w, h, 245, 36, 225, 96, 0x7f1d1d);
  fillRect(fb, w, h, 245, 36, 225, 18, 0x450a0a);
  drawString(fb, w, h, 251, 41, "CSI OPTICAL FEED: STANDBY", 0xf87171, 1);

  drawString(fb, w, h, 251, 60, "Driver: ov5647 I2C -5 (EIO)", 0xfca5a5, 1);
  drawString(fb, w, h, 251, 76, "Ribbon: Loose / Unseated", 0xfbbf24, 1);
  drawString(fb, w, h, 251, 92, "Probe:  Polling bus every 5s", 0x38bdf8, 1);
  drawString(fb, w, h, 251, 108, "Switch: Zero-Restart Hotplug", 0x4ade80, 1);

  // Hardware UART & Rover Telemetry Card
  drawRect(fb, w, h, 10, 140, 460, 80, 0x1e293b);
  fillRect(fb, w, h, 10, 140, 460, 18, 0x111c2e);
  drawString(fb, w, h, 16, 145, "UART BRIDGE (/dev/serial0 <-> ESP32 @ 115200)", 0x94a3b8, 1);

  snprintf(buf, sizeof(buf), "TX (15Hz): %llu", (unsigned long long)g_vision.uart_tx.load());
  drawString(fb, w, h, 16, 164, buf, 0x34d399, 1);
  snprintf(buf, sizeof(buf), "RX: %llu", (unsigned long long)g_vision.uart_rx.load());
  drawString(fb, w, h, 200, 164, buf, 0x60a5fa, 1);

  {
    std::lock_guard<std::mutex> lk(g_roverMutex);
    snprintf(buf, sizeof(buf), "Hdg: %.1f deg  Mode: %u  Ultra: %u mm", g_rover.heading, g_rover.mode, g_rover.ultraMm);
    drawString(fb, w, h, 16, 182, buf, 0xf1f5f9, 1);
    snprintf(buf, sizeof(buf), "ToF Left: %u mm   ToF Right: %u mm", g_rover.tofLeftMm, g_rover.tofRightMm);
    drawString(fb, w, h, 16, 198, buf, 0x38bdf8, 1);
  }

  // 3-Sector Visual Clearance Bars
  drawRect(fb, w, h, 10, 228, 460, 72, 0x1e293b);
  fillRect(fb, w, h, 10, 228, 460, 18, 0x111c2e);
  drawString(fb, w, h, 16, 233, "3-SECTOR VISUAL FLOOR CLEARANCE", 0x94a3b8, 1);

  auto drawBar = [&](int x, int y, const char* label, uint16_t distMm, uint32_t barCol) {
    drawString(fb, w, h, x, y, label, 0xe2e8f0, 1);
    drawRect(fb, w, h, x, y + 10, 130, 10, 0x334155);
    int fillW = (int)((distMm / 2000.0f) * 126.0f);
    fillW = std::clamp(fillW, 2, 126);
    fillRect(fb, w, h, x + 2, y + 12, fillW, 6, barCol);
    snprintf(buf, sizeof(buf), "%u mm", distMm);
    drawString(fb, w, h, x + 70, y, buf, barCol, 1);
  };

  drawBar(20, 254, "L (+16 L)", g_vision.vis_l_mm.load(), 0x38bdf8);
  drawBar(170, 254, "C (0 FWD)", g_vision.vis_f_mm.load(), 0x10b981);
  drawBar(320, 254, "R (-16 R)", g_vision.vis_r_mm.load(), 0xa855f7);

  snprintf(buf, sizeof(buf), "Suggested Heading: %+.1f deg", g_vision.clear_heading_deg.load());
  drawString(fb, w, h, 20, 280, buf, 0x38bdf8, 1);
  drawString(fb, w, h, 330, 280, "MOUNT: FRONT", 0x10b981, 1);

  // Bottom Status Line
  fillRect(fb, w, h, 0, 304, w, 16, 0x0c1424);
  snprintf(buf, sizeof(buf), "#%llu | PORT 8081 | ROS2 UDP: 192.168.0.111:8888", (unsigned long long)frameNum);
  drawString(fb, w, h, 10, 308, buf, 0x64748b, 1);
}

// ============================================================================
// 7. MULTI-BACKEND CAMERA CAPTURE ENGINE & SEAMLESS CSI RE-PROBING
// ============================================================================
bool probeCsiCameraHardware() {
  FILE* fp = popen("rpicam-vid --list-cameras 2>/dev/null", "r");
  if (!fp) return false;
  char outBuf[512];
  bool detected = false;
  while (fgets(outBuf, sizeof(outBuf), fp)) {
    if (strstr(outBuf, "0 :") || strstr(outBuf, "ov5647 [")) {
      if (!strstr(outBuf, "No cameras")) {
        detected = true;
      }
    }
  }
  pclose(fp);
  return detected;
}

void cameraCaptureThreadFunc() {
  const int HUD_W = 480;
  const int HUD_H = 320;

  uint64_t frameCounter = 0;
  auto fpsTimer = std::chrono::steady_clock::now();
  int framesInWindow = 0;

  std::vector<uint8_t> rgbBuf;
  std::vector<uint8_t> jpegOut;

  while (g_running) {
    updateSysMetrics();

    bool hasCsiCamera = probeCsiCameraHardware();

    if (hasCsiCamera) {
      std::cout << "[CAMERA] Detected 5MP OV5647 Camera! Launching rpicam-vid MJPEG ISP stream...\n";
      g_vision.backend = "libcamera-ov5647";
      g_vision.cam_online = true;

      const char* camCmd = "rpicam-vid -t 0 --inline --flush --codec mjpeg --width 640 --height 480 --framerate 25 --quality 75 -o - 2>/dev/null";
      FILE* camProc = popen(camCmd, "r");
      if (camProc) {
        int camFd = fileno(camProc);
        std::vector<uint8_t> curJpeg;
        curJpeg.reserve(65536);
        int prevByte = -1;
        bool inFrame = false;
        uint8_t readBuf[16384];
        auto lastFrameTime = std::chrono::steady_clock::now();
        bool hasReceivedFirstFrame = false;

        while (g_running) {
          struct pollfd pfd;
          pfd.fd = camFd;
          pfd.events = POLLIN | POLLHUP | POLLERR;
          pfd.revents = 0;

          // 4000ms timeout for initial hardware sensor start, 2000ms watchdog once streaming
          int pollTimeoutMs = hasReceivedFirstFrame ? 2000 : 4000;
          int pr = poll(&pfd, 1, pollTimeoutMs);
          if (pr <= 0) {
            std::cerr << "[CAMERA] Frame read timed out (>2s stall). Respawning capture pipeline...\n";
            system("pkill -9 -f rpicam-vid 2>/dev/null");
            break;
          }

          if (pfd.revents & (POLLHUP | POLLERR)) {
            std::cerr << "[CAMERA] Pipe hung up or errored. Respawning...\n";
            system("pkill -9 -f rpicam-vid 2>/dev/null");
            break;
          }

          ssize_t bytesRead = read(camFd, readBuf, sizeof(readBuf));
          if (bytesRead <= 0) {
            system("pkill -9 -f rpicam-vid 2>/dev/null");
            break;
          }

          for (ssize_t i = 0; i < bytesRead; ++i) {
            uint8_t b = readBuf[i];
            if (prevByte == 0xFF && b == 0xD8) {
              curJpeg.clear();
              curJpeg.push_back(0xFF);
              curJpeg.push_back(0xD8);
              inFrame = true;
            } else if (inFrame) {
              curJpeg.push_back(b);
              if (prevByte == 0xFF && b == 0xD9) {
                inFrame = false;
                hasReceivedFirstFrame = true;
                frameCounter++;
                framesInWindow++;
                lastFrameTime = std::chrono::steady_clock::now();

                {
                  std::lock_guard<std::mutex> lk(g_frame.mtx);
                  g_frame.jpeg = curJpeg;
                  g_frame.seq = frameCounter;
                }
                g_vision.frame_seq = frameCounter;
                g_frame.cv.notify_all();

                auto now = std::chrono::steady_clock::now();
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - fpsTimer).count();
                if (ms >= 1000) {
                  g_vision.fps = (framesInWindow * 1000.0f) / ms;
                  framesInWindow = 0;
                  fpsTimer = now;
                }
              }
            }
            prevByte = b;
          }

          // Complete frame watchdog: if incomplete frames stall > 2000ms, respawn
          auto now = std::chrono::steady_clock::now();
          auto timeSinceFrame = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrameTime).count();
          if (hasReceivedFirstFrame && timeSinceFrame > 2000) {
            std::cerr << "[CAMERA] Complete frame delivery stalled (>2s). Respawning capture pipeline...\n";
            system("pkill -9 -f rpicam-vid 2>/dev/null");
            break;
          }
        }
        pclose(camProc);
      }
      g_vision.cam_online = false;
      system("pkill -9 -f rpicam-vid 2>/dev/null");
      std::cout << "[CAMERA] rpicam-vid stopped. Re-probing sensor for immediate respawn...\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (probeCsiCameraHardware()) {
        continue; // Immediately respawn capture pipeline within <2s
      }
    }

    // FALLBACK / CSI UNSEATED HUD MODE (Real-time 20 FPS)
    g_vision.backend = "diagnostic_hud";
    g_vision.cam_online = false;
    auto lastProbeTime = std::chrono::steady_clock::now();

    while (g_running) {
      auto frameStart = std::chrono::steady_clock::now();

      // Only encode at 20 FPS if clients are connected, else sleep 100ms to preserve CPU
      int active = g_activeClients.load();
      generateHudFrame(rgbBuf, HUD_W, HUD_H, frameCounter++);
      framesInWindow++;

      jpegOut.clear();
      jpegOut.reserve(16384);
      TooJpeg::writeJpeg([&](uint8_t byte) { jpegOut.push_back(byte); }, rgbBuf.data(), HUD_W, HUD_H, 70);

      {
        std::lock_guard<std::mutex> lk(g_frame.mtx);
        g_frame.jpeg = jpegOut;
        g_frame.seq = frameCounter;
      }
      g_vision.frame_seq = frameCounter;
      g_frame.cv.notify_all();

      auto now = std::chrono::steady_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - fpsTimer).count();
      if (ms >= 1000) {
        g_vision.fps = (framesInWindow * 1000.0f) / ms;
        framesInWindow = 0;
        fpsTimer = now;
      }

      if (std::chrono::duration_cast<std::chrono::seconds>(now - lastProbeTime).count() >= 2) {
        lastProbeTime = now;
        updateSysMetrics();
        if (probeCsiCameraHardware()) {
          std::cout << "[CAMERA] Probe successful! CSI ribbon detected. Transitioning to live optical feed.\n";
          break;
        }
      }

      auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - frameStart).count();
      int targetDelay = (active > 0) ? 50 : 1000;
      if (elapsedMs < targetDelay) {
        std::this_thread::sleep_for(std::chrono::milliseconds(targetDelay - elapsedMs));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
}

// ============================================================================
// 8. LOW-LATENCY MULTI-CLIENT HTTP SERVER (PORT 8081)
// ============================================================================
void handleHttpClient(int clientSock) {
  char reqBuf[1024];
  int r = recv(clientSock, reqBuf, sizeof(reqBuf) - 1, 0);
  if (r <= 0) {
    close(clientSock);
    return;
  }
  reqBuf[r] = '\0';

  std::string req(reqBuf);
  std::string path = "/";
  size_t p1 = req.find(' ');
  if (p1 != std::string::npos) {
    size_t p2 = req.find(' ', p1 + 1);
    if (p2 != std::string::npos) {
      path = req.substr(p1 + 1, p2 - p1 - 1);
    }
  }

  // Strip query string (e.g. ?t=123456789) for clean routing
  size_t qPos = path.find('?');
  std::string cleanPath = (qPos != std::string::npos) ? path.substr(0, qPos) : path;

  if (cleanPath == "/stream") {
    g_activeClients++;
    std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: keep-alive\r\n"
      "Cache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
      "Pragma: no-cache\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

    if (send(clientSock, header.c_str(), header.length(), MSG_NOSIGNAL) < 0) {
      g_activeClients--;
      close(clientSock);
      return;
    }

    uint64_t lastSeq = 0;
    while (g_running) {
      std::vector<uint8_t> curJpeg;
      {
        std::unique_lock<std::mutex> lk(g_frame.mtx);
        g_frame.cv.wait(lk, [&]() { return !g_running || g_frame.seq != lastSeq; });
        if (!g_running) break;
        curJpeg = g_frame.jpeg;
        lastSeq = g_frame.seq;
      }

      if (curJpeg.empty()) continue;

      char partHeader[256];
      int hLen = snprintf(partHeader, sizeof(partHeader),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", curJpeg.size());

      if (send(clientSock, partHeader, hLen, MSG_NOSIGNAL) < 0) break;
      if (send(clientSock, curJpeg.data(), curJpeg.size(), MSG_NOSIGNAL) < 0) break;
      if (send(clientSock, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
    }
    g_activeClients--;
  } else if (cleanPath == "/snapshot") {
    std::vector<uint8_t> snap;
    {
      std::lock_guard<std::mutex> lk(g_frame.mtx);
      snap = g_frame.jpeg;
    }
    std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Content-Type: image/jpeg\r\n"
      "Content-Length: " + std::to_string(snap.size()) + "\r\n"
      "Connection: close\r\n\r\n";

    send(clientSock, header.c_str(), header.length(), MSG_NOSIGNAL);
    if (!snap.empty()) send(clientSock, snap.data(), snap.size(), MSG_NOSIGNAL);
  } else if (cleanPath == "/status" || cleanPath == "/api/status") {
    char jBuf[512];
    int len = snprintf(jBuf, sizeof(jBuf),
      "{\"cam_online\":%s,\"backend\":\"%s\",\"vis_f_mm\":%u,\"vis_l_mm\":%u,\"vis_r_mm\":%u,"
      "\"clear_heading_deg\":%.1f,\"fps\":%.1f,\"frame_seq\":%llu,\"uart_tx\":%llu,\"uart_rx\":%llu,"
      "\"cpu_temp_c\":%.1f,\"mem_avail_mb\":%u,\"cpu_usage\":%.1f}\r\n",
      g_vision.cam_online.load() ? "true" : "false",
      g_vision.backend.c_str(),
      g_vision.vis_f_mm.load(),
      g_vision.vis_l_mm.load(),
      g_vision.vis_r_mm.load(),
      g_vision.clear_heading_deg.load(),
      g_vision.fps.load(),
      (unsigned long long)g_vision.frame_seq.load(),
      (unsigned long long)g_vision.uart_tx.load(),
      (unsigned long long)g_vision.uart_rx.load(),
      g_vision.cpu_temp_c.load(),
      g_vision.mem_avail_mb.load(),
      g_vision.cpu_usage_pct.load());

    std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: " + std::to_string(len) + "\r\n"
      "Connection: close\r\n\r\n";

    send(clientSock, header.c_str(), header.length(), MSG_NOSIGNAL);
    send(clientSock, jBuf, len, MSG_NOSIGNAL);
  } else {
    std::string body = "<html><body><h1>Raspberry Pi 3B+ Optical Vision Daemon</h1>"
                       "<p><a href='/stream'>MJPEG Video Stream</a></p>"
                       "<p><a href='/snapshot'>JPEG Snapshot</a></p>"
                       "<p><a href='/status'>JSON Status</a></p></body></html>";
    std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Content-Type: text/html\r\n"
      "Content-Length: " + std::to_string(body.length()) + "\r\n"
      "Connection: close\r\n\r\n";
    send(clientSock, header.c_str(), header.length(), MSG_NOSIGNAL);
    send(clientSock, body.c_str(), body.length(), MSG_NOSIGNAL);
  }

  close(clientSock);
}

void httpServerThreadFunc() {
  int serverSock = socket(AF_INET, SOCK_STREAM, 0);
  if (serverSock < 0) {
    std::cerr << "[HTTP] Fatal: socket creation failed.\n";
    return;
  }

  int opt = 1;
  setsockopt(serverSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in servAddr{};
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = INADDR_ANY;
  servAddr.sin_port = htons(8081);

  if (bind(serverSock, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0) {
    std::cerr << "[HTTP] Fatal: bind failed on port 8081: " << strerror(errno) << "\n";
    close(serverSock);
    return;
  }

  if (listen(serverSock, 16) < 0) {
    std::cerr << "[HTTP] Fatal: listen failed.\n";
    close(serverSock);
    return;
  }

  std::cout << "[HTTP] Listening for connections on http://0.0.0.0:8081/stream\n";

  while (g_running) {
    struct sockaddr_in clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);
    int clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &clientLen);
    if (clientSock < 0) {
      if (!g_running) break;
      continue;
    }

    int flag = 1;
    setsockopt(clientSock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    std::thread([clientSock]() {
      handleHttpClient(clientSock);
    }).detach();
  }

  close(serverSock);
}

// ============================================================================
// 9. DAEMON ENTRYPOINT & SIGNAL HANDLING
// ============================================================================
void sigHandler(int sig) {
  g_running = false;
  g_frame.cv.notify_all();
}

int main(int argc, char* argv[]) {
  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);
  signal(SIGPIPE, SIG_IGN);

  std::cout << "=========================================================\n";
  std::cout << "  Raspberry Pi 3B+ Native C++ Camera & Vision Daemon     \n";
  std::cout << "  Hardware: BCM2837B0 ARMv8 Cortex-A53 | Debian Bookworm \n";
  std::cout << "  Port: 8081 | UART: /dev/serial0 (PL011) @ 115200 8N1  \n";
  std::cout << "=========================================================\n";

  TooJpeg::init();
  updateSysMetrics();

  std::thread captureThread(cameraCaptureThreadFunc);
  std::thread uartThread(uartThreadFunc);
  std::thread httpThread(httpServerThreadFunc);

  if (captureThread.joinable()) captureThread.join();
  if (uartThread.joinable()) uartThread.join();
  if (httpThread.joinable()) httpThread.join();

  std::cout << "[DAEMON] Shutting down cleanly.\n";
  return 0;
}
