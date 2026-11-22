#include "thermal_codec_b.h"

#include <math.h>
#include <string.h>

namespace thermal_b {
namespace {

// HEVC-style 4x4 integer DCT basis.
const int64_t kM[4][4] = {
    {64, 64, 64, 64},
    {83, 36, -36, -83},
    {64, -64, -64, 64},
    {36, -83, 83, -36},
};

// 4x4 zigzag order as row-major indices.
const uint8_t kZigzag[16] = {0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15};

constexpr int kNsym = kParityBytes;

int16_t gQuantized[kPixels];
uint16_t gResidual[kPixels];
int32_t gCoefficients[kPixels];

struct RsState {
  bool ready = false;
  uint8_t exp[512];
  uint8_t log[256];
  uint8_t generator[kNsym + 1];
};

RsState gRs;

class BitWriter {
 public:
  BitWriter(uint8_t *buffer, size_t capacity)
      : buffer_(buffer), capacity_(capacity), bitPos_(0), ok_(true) {}

  void bit(uint8_t value) {
    const size_t byte = bitPos_ >> 3;
    if (byte >= capacity_) {
      ok_ = false;
      return;
    }
    if (value) buffer_[byte] |= static_cast<uint8_t>(0x80u >> (bitPos_ & 7));
    ++bitPos_;
  }

  void bits(uint32_t value, int count) {
    for (int shift = count - 1; shift >= 0; --shift) bit((value >> shift) & 1u);
  }

  size_t size() const { return (bitPos_ + 7) >> 3; }
  bool ok() const { return ok_; }

 private:
  uint8_t *buffer_;
  size_t capacity_;
  size_t bitPos_;
  bool ok_;
};

int32_t quantizeCoefficient(int32_t coefficient, int step) {
  const int64_t magnitude = coefficient < 0 ? -static_cast<int64_t>(coefficient) : coefficient;
  const int64_t quantized = (magnitude + step / 2) / step;
  return coefficient < 0 ? static_cast<int32_t>(-quantized) : static_cast<int32_t>(quantized);
}

uint32_t foldSigned(int32_t value) {
  return value >= 0 ? static_cast<uint32_t>(value) * 2u
                    : static_cast<uint32_t>(-static_cast<int64_t>(value)) * 2u - 1u;
}

void writeExpGolomb(BitWriter &writer, uint32_t value) {
  const uint32_t magnitude = value + 1u;
  int bitLength = 0;
  for (uint32_t v = magnitude; v; v >>= 1) ++bitLength;
  for (int i = 0; i < bitLength - 1; ++i) writer.bit(0);
  writer.bits(magnitude, bitLength);
}

void forwardDct(const uint16_t block[16], int32_t out[16]) {
  int64_t temporary[16];
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      int64_t sum = 0;
      for (int i = 0; i < 4; ++i) sum += kM[y][i] * static_cast<int64_t>(block[i * 4 + x]);
      temporary[y * 4 + x] = sum;
    }
  }
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      int64_t sum = 0;
      for (int j = 0; j < 4; ++j) sum += temporary[y * 4 + j] * kM[x][j];
      out[y * 4 + x] = static_cast<int32_t>((sum + (1 << 15)) >> 16);
    }
  }
}

int encodePayload(int step, uint8_t *payload) {
  memset(payload, 0, kPayloadBytes);
  BitWriter writer(payload, kPayloadBytes);
  int32_t previousDc = 0;

  for (int block = 0; block < kBlockCount; ++block) {
    const int32_t dc = quantizeCoefficient(gCoefficients[block * 16], step);
    writeExpGolomb(writer, foldSigned(dc - previousDc));
    previousDc = dc;

    uint16_t mask = 0;
    int32_t levels[15];
    for (int position = 1; position < 16; ++position) {
      const int32_t level =
          quantizeCoefficient(gCoefficients[block * 16 + kZigzag[position]], step);
      levels[position - 1] = level;
      if (level) mask |= static_cast<uint16_t>(1u << (position - 1));
    }

    if (mask == 0) {
      writer.bit(0);
    } else {
      writer.bit(1);
      writer.bits(mask, 15);
      for (int position = 1; position < 16; ++position) {
        const int32_t level = levels[position - 1];
        if (!level) continue;
        writer.bit(level < 0 ? 1 : 0);
        writeExpGolomb(writer, static_cast<uint32_t>(level < 0 ? -level : level) - 1u);
      }
    }
    if (!writer.ok()) return -1;
  }
  if (!writer.ok()) return -1;
  return static_cast<int>(writer.size());
}

inline uint8_t gfMul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  return gRs.exp[gRs.log[a] + gRs.log[b]];
}

void initRs() {
  if (gRs.ready) return;
  uint16_t x = 1;
  for (int i = 0; i < 255; ++i) {
    gRs.exp[i] = static_cast<uint8_t>(x);
    gRs.log[x] = static_cast<uint8_t>(i);
    x <<= 1;
    if (x & 0x100u) x ^= 0x11Du;
  }
  for (int i = 255; i < 512; ++i) gRs.exp[i] = gRs.exp[i - 255];

  // generator(x) = product (x + alpha^i), coefficients high-order first.
  memset(gRs.generator, 0, sizeof(gRs.generator));
  gRs.generator[0] = 1;
  int degree = 0;
  for (int i = 0; i < kNsym; ++i) {
    const uint8_t root = gRs.exp[i];
    const uint8_t previousLeading = gRs.generator[degree];
    gRs.generator[degree + 1] = gfMul(previousLeading, root);
    for (int j = degree; j >= 1; --j) {
      gRs.generator[j] ^= gfMul(gRs.generator[j - 1], root);
    }
    ++degree;
  }
  gRs.ready = true;
}

void rsEncode(const uint8_t *data, uint8_t *out) {
  initRs();
  uint8_t work[kPacketBytes];
  memcpy(work, data, kDataBytes);
  memset(work + kDataBytes, 0, kParityBytes);
  for (int i = 0; i < kDataBytes; ++i) {
    const uint8_t coefficient = work[i];
    if (!coefficient) continue;
    for (int j = 1; j <= kNsym; ++j) {
      work[i + j] ^= gfMul(gRs.generator[j], coefficient);
    }
  }
  memcpy(out, data, kDataBytes);
  memcpy(out + kDataBytes, work + kDataBytes, kParityBytes);
}

}  // namespace

bool encodeQuantized(const int16_t *quantized, uint8_t frameId, uint8_t out[kPacketBytes]) {
  if (!quantized || !out) return false;
  memcpy(gQuantized, quantized, sizeof(gQuantized));

  int16_t minimum = gQuantized[0];
  for (int i = 1; i < kPixels; ++i) {
    if (gQuantized[i] < minimum) minimum = gQuantized[i];
  }
  for (int i = 0; i < kPixels; ++i) {
    gResidual[i] = static_cast<uint16_t>(static_cast<int32_t>(gQuantized[i]) - minimum);
  }

  for (int by = 0; by < kBlocksY; ++by) {
    for (int bx = 0; bx < kBlocksX; ++bx) {
      uint16_t block[16];
      for (int dy = 0; dy < kBlock; ++dy) {
        for (int dx = 0; dx < kBlock; ++dx) {
          block[dy * kBlock + dx] = gResidual[(by * kBlock + dy) * kFrameWidth + bx * kBlock + dx];
        }
      }
      forwardDct(block, gCoefficients + (by * kBlocksX + bx) * 16);
    }
  }

  uint8_t bestPayload[kPayloadBytes];
  memset(bestPayload, 0, sizeof(bestPayload));
  uint8_t candidate[kPayloadBytes];
  int bestStep = 0;
  int low = 1;
  int high = 255;
  while (low <= high) {
    const int step = (low + high) / 2;
    const int length = encodePayload(step, candidate);
    if (length > 0 && length <= kPayloadBytes) {
      memcpy(bestPayload, candidate, sizeof(bestPayload));
      bestStep = step;
      high = step - 1;
    } else {
      low = step + 1;
    }
  }

  if (bestStep == 0) {
    // DC-only fallback: one rounded block mean per 4x4 block.
    memset(bestPayload, 0, sizeof(bestPayload));
    for (int by = 0; by < kBlocksY; ++by) {
      for (int bx = 0; bx < kBlocksX; ++bx) {
        uint32_t sum = 0;
        for (int dy = 0; dy < kBlock; ++dy) {
          for (int dx = 0; dx < kBlock; ++dx) {
            sum += gResidual[(by * kBlock + dy) * kFrameWidth + bx * kBlock + dx];
          }
        }
        const uint16_t mean = static_cast<uint16_t>((sum + 8) >> 4);
        const int blockIndex = by * kBlocksX + bx;
        bestPayload[blockIndex * 2] = static_cast<uint8_t>(mean & 0xFF);
        bestPayload[blockIndex * 2 + 1] = static_cast<uint8_t>(mean >> 8);
      }
    }
    bestStep = 0;
  }

  uint8_t data[kDataBytes];
  data[0] = frameId;
  data[1] = static_cast<uint8_t>(minimum & 0xFF);
  data[2] = static_cast<uint8_t>((minimum >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(bestStep);
  memcpy(data + kHeaderBytes, bestPayload, kPayloadBytes);
  rsEncode(data, out);
  return true;
}

bool encodeFrame(const float *temperaturesC, uint8_t frameId, uint8_t out[kPacketBytes]) {
  if (!temperaturesC || !out) return false;
  for (int i = 0; i < kPixels; ++i) {
    if (!isfinite(temperaturesC[i])) {
      gQuantized[i] = 0;
      continue;
    }
    const double scaled = static_cast<double>(temperaturesC[i]) * 20.0;
    double rounded = scaled >= 0.0 ? floor(scaled + 0.5) : ceil(scaled - 0.5);
    if (rounded < -32768.0) rounded = -32768.0;
    if (rounded > 32767.0) rounded = 32767.0;
    gQuantized[i] = static_cast<int16_t>(rounded);
  }
  return encodeQuantized(gQuantized, frameId, out);
}

bool rsCodewordIsValid(const uint8_t packet[kPacketBytes]) {
  if (!packet) return false;
  initRs();
  for (int i = 0; i < kNsym; ++i) {
    const uint8_t root = gRs.exp[i];
    uint8_t value = 0;
    for (int j = 0; j < kPacketBytes; ++j) value = gfMul(value, root) ^ packet[j];
    if (value != 0) return false;
  }
  return true;
}

}  // namespace thermal_b
