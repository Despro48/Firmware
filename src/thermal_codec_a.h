#pragma once

#include <stddef.h>
#include <stdint.h>

// Profile A: lossless 32x24 frames as three independently decodable 32x8
// strips (2D MED predictor + adaptive Golomb-Rice). If a strip exceeds the
// 255-byte LoRa packet, it is split into four 32x2 parts (raw fallback per
// part), keeping the nominal 3-packet format for normal scenes.
// Pure C++, no Arduino dependencies, so the codec can be unit-tested on the host.
namespace thermal_a {

constexpr int kFrameWidth = 32;
constexpr int kFrameHeight = 24;
constexpr int kStripHeight = 8;
constexpr int kStripCount = 3;
constexpr int kStripPixels = kFrameWidth * kStripHeight;
constexpr int kSplitHeight = 2;
constexpr int kSplitParts = kStripHeight / kSplitHeight;
constexpr int kMaxPackets = kStripCount * kSplitParts;
constexpr int kHeaderBytes = 4;
constexpr int kMaxPacketBytes = 255;
constexpr int kMaxPayloadBytes = kMaxPacketBytes - kHeaderBytes;
constexpr int kSplitRawBytes = kFrameWidth * kSplitHeight * 2;
constexpr int kRiceLimit = 15;
constexpr uint8_t kSplitFlag = 0x40;
constexpr int kPartShift = 4;

struct Packet {
  uint8_t data[kMaxPacketBytes];
  uint16_t length;
  uint8_t stripId;
  uint8_t riceK;
  uint8_t part;
  bool split;
  int16_t reference;
};

// 0.05 C LSB, round half away from zero, clamped to int16.
int16_t quantizeTemperature(float celsius);

// Encodes one 32x8 int16 strip. Returns false if it cannot fit one packet;
// use encodeStripPackets for the split fallback.
bool encodeStrip(const int16_t *pixels, uint8_t frameId, uint8_t stripId, Packet *out);

// Encodes a strip into one packet, or four split packets if the nominal packet
// would exceed 255 bytes. Returns the packet count, or 0 on invalid input.
int encodeStripPackets(const int16_t *pixels, uint8_t frameId, uint8_t stripId,
                       Packet out[kMaxPackets]);

// Quantizes and encodes a full frame. Returns 3 packets for normal scenes, or
// up to 12 when strips must be split; 0 on invalid input.
int encodeFrame(const float *temperaturesC, uint8_t frameId, Packet out[kMaxPackets]);
int encodeQuantizedFrame(const int16_t *quantized, uint8_t frameId, Packet out[kMaxPackets]);

// Decodes one nominal 32x8 packet. Returns false for malformed or split packets.
bool decodeStrip(const uint8_t *data, size_t length, int16_t out[kStripPixels]);

// Decodes a complete frame from any mix of nominal and split packets.
bool decodeFrame(const Packet *packets, int count, int16_t out[kFrameWidth * kFrameHeight]);

}  // namespace thermal_a
