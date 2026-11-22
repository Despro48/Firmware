#pragma once

#include <stddef.h>
#include <stdint.h>

// Profile B: lossy 32x24 frame in a single 222-byte LoRa packet.
// 4x4 integer DCT + uniform quantization + DPCM/Exp-Golomb + RS(222,190).
// Pure C++ (no Arduino dependencies) so the encoder can be tested on the host.
namespace thermal_b {

constexpr int kFrameWidth = 32;
constexpr int kFrameHeight = 24;
constexpr int kPixels = kFrameWidth * kFrameHeight;
constexpr int kBlock = 4;
constexpr int kBlocksX = kFrameWidth / kBlock;
constexpr int kBlocksY = kFrameHeight / kBlock;
constexpr int kBlockCount = kBlocksX * kBlocksY;
constexpr int kHeaderBytes = 4;
constexpr int kPayloadBytes = 186;
constexpr int kDataBytes = kHeaderBytes + kPayloadBytes;  // 190
constexpr int kParityBytes = 32;
constexpr int kPacketBytes = kDataBytes + kParityBytes;   // 222

// Quantizes a Celsius frame, rate-controls to <=186 payload bytes, and emits a
// systematic RS(222,190) codeword. Returns false only for invalid arguments.
bool encodeFrame(const float *temperaturesC, uint8_t frameId, uint8_t out[kPacketBytes]);

// Same as encodeFrame but takes an already quantized (0.05 C LSB) frame.
bool encodeQuantized(const int16_t *quantized, uint8_t frameId, uint8_t out[kPacketBytes]);

// Exposed for host tests: verifies that all RS syndromes are zero.
bool rsCodewordIsValid(const uint8_t packet[kPacketBytes]);

}  // namespace thermal_b
