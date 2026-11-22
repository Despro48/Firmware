#include "thermal_codec_a.h"

#include <math.h>
#include <string.h>

namespace thermal_a {
namespace {

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

class BitReader {
 public:
  BitReader(const uint8_t *data, size_t length) : data_(data), bitLength_(length * 8) {}

  int bit() {
    if (pos_ >= bitLength_) return -1;
    const uint8_t value = (data_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1u;
    ++pos_;
    return value;
  }

  int bits(int count) {
    uint32_t value = 0;
    for (int i = 0; i < count; ++i) {
      const int b = bit();
      if (b < 0) return -1;
      value = (value << 1) | static_cast<uint32_t>(b);
    }
    return static_cast<int>(value);
  }

 private:
  const uint8_t *data_;
  size_t bitLength_;
  size_t pos_ = 0;
};

inline int clampInt16(int value) {
  if (value < -32768) return -32768;
  if (value > 32767) return 32767;
  return value;
}

inline int medianOfThree(int a, int b, int c) {
  const int lo = a < b ? a : b;
  const int hi = a < b ? b : a;
  if (c <= lo) return lo;
  if (c >= hi) return hi;
  return c;
}

inline int predict(const int16_t *pixels, int x, int y, int reference) {
  if (y == 0) return x > 0 ? pixels[x - 1] : reference;
  if (x == 0) return pixels[(y - 1) * kFrameWidth];
  const int a = pixels[y * kFrameWidth + x - 1];
  const int b = pixels[(y - 1) * kFrameWidth + x];
  const int c = pixels[(y - 1) * kFrameWidth + x - 1];
  return clampInt16(medianOfThree(a, b, a + b - c));
}

inline uint32_t zigzag(int32_t residual) {
  return residual >= 0 ? static_cast<uint32_t>(residual) * 2u
                       : static_cast<uint32_t>(-residual) * 2u - 1u;
}

uint32_t riceCost(const int16_t *pixels, int height, int reference, int k) {
  uint32_t cost = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < kFrameWidth; ++x) {
      if (x == 0 && y == 0) continue;
      const int32_t residual =
          static_cast<int32_t>(pixels[y * kFrameWidth + x]) - predict(pixels, x, y, reference);
      const uint32_t quotient = zigzag(residual) >> k;
      cost += quotient >= kRiceLimit ? kRiceLimit + 1 + 16 : quotient + 1 + static_cast<uint32_t>(k);
    }
  }
  return cost;
}

void setHeader(Packet *out, uint8_t frameId, uint8_t stripId, int k, int part, bool split,
               int reference) {
  out->data[0] = frameId;
  out->data[1] = static_cast<uint8_t>(stripId | (k << 2) | ((part & 0x03) << kPartShift) |
                                      (split ? kSplitFlag : 0));
  out->data[2] = static_cast<uint8_t>((reference >> 8) & 0xFF);
  out->data[3] = static_cast<uint8_t>(reference & 0xFF);
  out->stripId = stripId;
  out->riceK = static_cast<uint8_t>(k);
  out->part = static_cast<uint8_t>(part);
  out->split = split;
  out->reference = static_cast<int16_t>(reference);
}

bool encodeCodedBlock(const int16_t *pixels, int height, uint8_t frameId, uint8_t stripId,
                      uint8_t part, bool split, Packet *out) {
  const int reference = pixels[0];
  uint32_t bestCost = 0xFFFFFFFFu;
  int bestK = 1;
  for (int k = 1; k <= 3; ++k) {
    const uint32_t cost = riceCost(pixels, height, reference, k);
    if (cost < bestCost) {
      bestCost = cost;
      bestK = k;
    }
  }

  memset(out->data, 0, sizeof(out->data));
  BitWriter writer(out->data + kHeaderBytes, kMaxPayloadBytes);
  const uint32_t mask = (1u << bestK) - 1u;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < kFrameWidth; ++x) {
      if (x == 0 && y == 0) continue;
      const int index = y * kFrameWidth + x;
      const int32_t residual = static_cast<int32_t>(pixels[index]) - predict(pixels, x, y, reference);
      const uint32_t folded = zigzag(residual);
      const uint32_t quotient = folded >> bestK;
      if (quotient >= kRiceLimit) {
        for (int i = 0; i < kRiceLimit; ++i) writer.bit(1);
        writer.bit(0);
        writer.bits(static_cast<uint16_t>(pixels[index]), 16);
      } else {
        for (uint32_t i = 0; i < quotient; ++i) writer.bit(1);
        writer.bit(0);
        writer.bits(folded & mask, bestK);
      }
    }
  }

  if (!writer.ok()) return false;
  const size_t payloadBytes = writer.size();
  if (payloadBytes > static_cast<size_t>(kMaxPayloadBytes)) return false;

  setHeader(out, frameId, stripId, bestK, part, split, reference);
  out->length = static_cast<uint16_t>(kHeaderBytes + payloadBytes);
  return true;
}

void encodeRawPart(const int16_t *pixels, uint8_t frameId, uint8_t stripId, uint8_t part,
                   Packet *out) {
  memset(out->data, 0, sizeof(out->data));
  setHeader(out, frameId, stripId, 0, part, true, 0);
  for (int i = 0; i < kFrameWidth * kSplitHeight; ++i) {
    const uint16_t value = static_cast<uint16_t>(pixels[i]);
    out->data[kHeaderBytes + i * 2] = static_cast<uint8_t>(value & 0xFF);
    out->data[kHeaderBytes + i * 2 + 1] = static_cast<uint8_t>(value >> 8);
  }
  out->length = static_cast<uint16_t>(kHeaderBytes + kSplitRawBytes);
}

bool decodePacketInternal(const uint8_t *data, size_t length, int16_t *out, int *stripId,
                          int *part, bool *split) {
  if (!data || !out || length < kHeaderBytes) return false;
  const uint8_t flags = data[1];
  if (flags & 0x80) return false;
  const int sid = flags & 0x03;
  const int k = (flags >> 2) & 0x03;
  const int packetPart = (flags >> kPartShift) & 0x03;
  const bool isSplit = (flags & kSplitFlag) != 0;
  if (sid >= kStripCount) return false;
  if (!isSplit) {
    if (packetPart != 0 || k < 1 || k > 3) return false;
  } else if (k > 3) {
    return false;
  }

  if (isSplit && k == 0) {
    if (length - kHeaderBytes < static_cast<size_t>(kSplitRawBytes)) return false;
    for (int i = 0; i < kFrameWidth * kSplitHeight; ++i) {
      out[i] = static_cast<int16_t>(static_cast<uint16_t>(data[kHeaderBytes + i * 2]) |
                                    (static_cast<uint16_t>(data[kHeaderBytes + i * 2 + 1]) << 8));
    }
  } else {
    const int reference = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                                               static_cast<uint16_t>(data[3]));
    const int height = isSplit ? kSplitHeight : kStripHeight;
    out[0] = static_cast<int16_t>(reference);
    BitReader reader(data + kHeaderBytes, length - kHeaderBytes);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < kFrameWidth; ++x) {
        if (x == 0 && y == 0) continue;
        int ones = 0;
        int bit;
        while ((bit = reader.bit()) == 1) {
          if (++ones > kRiceLimit) return false;
        }
        if (bit < 0) return false;
        int value;
        if (ones == kRiceLimit) {
          const int raw = reader.bits(16);
          if (raw < 0) return false;
          value = raw >= 32768 ? raw - 65536 : raw;
        } else {
          const int remainder = reader.bits(k);
          if (remainder < 0) return false;
          const uint32_t folded =
              (static_cast<uint32_t>(ones) << k) | static_cast<uint32_t>(remainder);
          const int32_t residual =
              static_cast<int32_t>(folded >> 1) ^ -static_cast<int32_t>(folded & 1u);
          value = predict(out, x, y, reference) + static_cast<int>(residual);
        }
        out[y * kFrameWidth + x] = static_cast<int16_t>(value);
      }
    }
  }

  *stripId = sid;
  *part = packetPart;
  *split = isSplit;
  return true;
}

}  // namespace

int16_t quantizeTemperature(float celsius) {
  if (!isfinite(celsius)) return 0;
  const double scaled = static_cast<double>(celsius) * 20.0;
  double rounded = scaled >= 0.0 ? floor(scaled + 0.5) : ceil(scaled - 0.5);
  if (rounded < -32768.0) return -32768;
  if (rounded > 32767.0) return 32767;
  return static_cast<int16_t>(rounded);
}

bool encodeStrip(const int16_t *pixels, uint8_t frameId, uint8_t stripId, Packet *out) {
  if (!pixels || !out || stripId >= kStripCount) return false;
  return encodeCodedBlock(pixels, kStripHeight, frameId, stripId, 0, false, out);
}

int encodeStripPackets(const int16_t *pixels, uint8_t frameId, uint8_t stripId,
                       Packet out[kMaxPackets]) {
  if (!pixels || !out || stripId >= kStripCount) return 0;
  if (encodeStrip(pixels, frameId, stripId, &out[0])) return 1;

  for (int part = 0; part < kSplitParts; ++part) {
    const int16_t *sub = pixels + part * kSplitHeight * kFrameWidth;
    if (!encodeCodedBlock(sub, kSplitHeight, frameId, stripId, static_cast<uint8_t>(part), true,
                          &out[part])) {
      encodeRawPart(sub, frameId, stripId, static_cast<uint8_t>(part), &out[part]);
    }
  }
  return kSplitParts;
}

int encodeQuantizedFrame(const int16_t *quantized, uint8_t frameId, Packet out[kMaxPackets]) {
  if (!quantized || !out) return 0;
  int count = 0;
  for (int strip = 0; strip < kStripCount; ++strip) {
    const int written =
        encodeStripPackets(quantized + strip * kStripPixels, frameId, static_cast<uint8_t>(strip),
                           out + count);
    if (written == 0) return 0;
    count += written;
  }
  return count;
}

int encodeFrame(const float *temperaturesC, uint8_t frameId, Packet out[kMaxPackets]) {
  if (!temperaturesC || !out) return 0;
  // Static buffer keeps the LoRa task stack small; encoding is single-threaded.
  static int16_t quantized[kFrameWidth * kFrameHeight];
  for (int i = 0; i < kFrameWidth * kFrameHeight; ++i) {
    quantized[i] = quantizeTemperature(temperaturesC[i]);
  }
  return encodeQuantizedFrame(quantized, frameId, out);
}

bool decodeStrip(const uint8_t *data, size_t length, int16_t out[kStripPixels]) {
  int stripId = 0;
  int part = 0;
  bool split = false;
  int16_t decoded[kStripPixels];
  if (!decodePacketInternal(data, length, decoded, &stripId, &part, &split) || split) return false;
  memcpy(out, decoded, sizeof(decoded));
  return true;
}

bool decodeFrame(const Packet *packets, int count, int16_t out[kFrameWidth * kFrameHeight]) {
  if (!packets || !out || count < kStripCount || count > kMaxPackets) return false;

  static int16_t full[kStripCount][kStripPixels];
  static int16_t parts[kStripCount][kSplitParts][kFrameWidth * kSplitHeight];
  bool fullSeen[kStripCount] = {false, false, false};
  bool partSeen[kStripCount][kSplitParts] = {};
  int frameId = -1;

  for (int i = 0; i < count; ++i) {
    const Packet &packet = packets[i];
    if (frameId < 0) frameId = packet.data[0];
    if (packet.data[0] != frameId) return false;

    int16_t decoded[kStripPixels];
    int stripId = 0;
    int part = 0;
    bool split = false;
    if (!decodePacketInternal(packet.data, packet.length, decoded, &stripId, &part, &split)) {
      return false;
    }
    if (split) {
      if (fullSeen[stripId] || partSeen[stripId][part]) return false;
      partSeen[stripId][part] = true;
      memcpy(parts[stripId][part], decoded, sizeof(parts[stripId][part]));
    } else {
      bool anyPart = false;
      for (int p = 0; p < kSplitParts; ++p) anyPart = anyPart || partSeen[stripId][p];
      if (fullSeen[stripId] || anyPart) return false;
      fullSeen[stripId] = true;
      memcpy(full[stripId], decoded, sizeof(full[stripId]));
    }
  }

  for (int strip = 0; strip < kStripCount; ++strip) {
    int16_t *destination = out + strip * kStripPixels;
    if (fullSeen[strip]) {
      for (int part = 0; part < kSplitParts; ++part) {
        if (partSeen[strip][part]) return false;
      }
      memcpy(destination, full[strip], sizeof(full[strip]));
      continue;
    }
    for (int part = 0; part < kSplitParts; ++part) {
      if (!partSeen[strip][part]) return false;
      memcpy(destination + part * kFrameWidth * kSplitHeight, parts[strip][part],
             sizeof(parts[strip][part]));
    }
  }
  return true;
}

}  // namespace thermal_a
