// Host-side test for the profile-A firmware codec. No Arduino toolchain needed:
//   g++ -std=c++17 -O2 -Wall -Wextra src/thermal_codec_a.cpp test/host/test_option_a.cpp \
//       -o /tmp/test_option_a && /tmp/test_option_a
//
// Optional cross-language check:
//   /tmp/test_option_a --fixture /tmp/frame_q.bin --packets /tmp/packets_a.bin

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "../../src/thermal_codec_a.h"

namespace {

constexpr int kPixels = thermal_a::kFrameWidth * thermal_a::kFrameHeight;

float blob(float x, float y, float cx, float cy, float amplitude, float sigma) {
  const float dx = x - cx;
  const float dy = y - cy;
  return amplitude * std::exp(-(dx * dx + dy * dy) / (2.0f * sigma * sigma));
}

std::vector<float> syntheticFrame(uint32_t seed, float noise = 0.08f) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> dist(0.0f, noise);
  std::vector<float> frame(kPixels);
  for (int y = 0; y < thermal_a::kFrameHeight; ++y) {
    for (int x = 0; x < thermal_a::kFrameWidth; ++x) {
      float value = 25.0f + 0.04f * x - 0.06f * y;
      value += blob(x, y, 10, 8, 7.0f, 3.0f);
      value += blob(x, y, 22, 16, 4.0f, 4.0f);
      value += blob(x, y, 5, 18, -5.0f, 5.0f);
      value += blob(x, y, 27, 5, 2.0f, 2.5f);
      frame[y * thermal_a::kFrameWidth + x] = value + dist(rng);
    }
  }
  return frame;
}

std::vector<float> hotFrame() {
  std::vector<float> frame(kPixels);
  for (int y = 0; y < thermal_a::kFrameHeight; ++y) {
    for (int x = 0; x < thermal_a::kFrameWidth; ++x) {
      const float dx = x - 16.0f;
      const float dy = y - 12.0f;
      frame[y * thermal_a::kFrameWidth + x] =
          22.0f + 0.02f * x - 0.03f * y + 150.0f * std::exp(-(dx * dx + dy * dy) / 18.0f);
    }
  }
  return frame;
}

bool readFixture(const std::string &path, std::vector<int16_t> &out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  out.resize(kPixels);
  in.read(reinterpret_cast<char *>(out.data()), kPixels * sizeof(int16_t));
  return in.gcount() == static_cast<std::streamsize>(kPixels * sizeof(int16_t));
}

bool readFloatFixture(const std::string &path, std::vector<float> &out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  out.resize(kPixels);
  in.read(reinterpret_cast<char *>(out.data()), kPixels * sizeof(float));
  return in.gcount() == static_cast<std::streamsize>(kPixels * sizeof(float));
}

bool writePackets(const std::string &path, const thermal_a::Packet *packets, int count) {
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  const uint8_t packetCount = static_cast<uint8_t>(count);
  out.write(reinterpret_cast<const char *>(&packetCount), 1);
  for (int i = 0; i < count; ++i) {
    const uint16_t length = packets[i].length;
    out.write(reinterpret_cast<const char *>(&length), sizeof(length));
    out.write(reinterpret_cast<const char *>(packets[i].data), length);
  }
  return static_cast<bool>(out);
}

bool readPackets(const std::string &path, std::vector<std::vector<uint8_t>> &packets) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  uint8_t count = 0;
  in.read(reinterpret_cast<char *>(&count), 1);
  if (in.gcount() != 1) return false;
  packets.clear();
  for (int i = 0; i < count; ++i) {
    uint16_t length = 0;
    in.read(reinterpret_cast<char *>(&length), sizeof(length));
    if (in.gcount() != sizeof(length) || length > thermal_a::kMaxPacketBytes) return false;
    std::vector<uint8_t> packet(length);
    in.read(reinterpret_cast<char *>(packet.data()), length);
    if (in.gcount() != length) return false;
    packets.push_back(std::move(packet));
  }
  return packets.size() == count && count >= thermal_a::kStripCount &&
         count <= thermal_a::kMaxPackets;
}

int runSynthetic() {
  std::printf("== synthetic round trips (profile A) ==\n");
  size_t worst = 0;
  for (uint32_t seed = 0; seed < 20; ++seed) {
    const std::vector<float> frame = syntheticFrame(seed);
    thermal_a::Packet packets[thermal_a::kMaxPackets];
    const int count = thermal_a::encodeFrame(frame.data(), static_cast<uint8_t>(seed), packets);
    if (count < thermal_a::kStripCount || count > thermal_a::kMaxPackets) {
      std::printf("seed %u: bad packet count %d\n", seed, count);
      return 1;
    }
    int16_t decoded[kPixels];
    if (!thermal_a::decodeFrame(packets, count, decoded)) {
      std::printf("seed %u: decode failed\n", seed);
      return 1;
    }
    size_t total = 0;
    for (int i = 0; i < count; ++i) {
      total += packets[i].length;
      if (packets[i].length > thermal_a::kMaxPacketBytes) {
        std::printf("seed %u: packet too large\n", seed);
        return 1;
      }
    }
    for (int i = 0; i < kPixels; ++i) {
      if (decoded[i] != thermal_a::quantizeTemperature(frame[i])) {
        std::printf("seed %u pixel %d mismatch\n", seed, i);
        return 1;
      }
    }
    worst = std::max(worst, total);
    std::printf("seed %2u packets=%d total=%3zu B sizes=%u..%u\n", seed, count, total,
                packets[0].length, packets[count - 1].length);
  }

  // A lost strip must not affect the other two.
  const std::vector<float> frame = syntheticFrame(7);
  thermal_a::Packet packets[thermal_a::kMaxPackets];
  const int count = thermal_a::encodeFrame(frame.data(), 7, packets);
  const thermal_a::Packet *strip0 = nullptr;
  const thermal_a::Packet *strip2 = nullptr;
  for (int i = 0; i < count; ++i) {
    if (packets[i].stripId == 0 && !packets[i].split) strip0 = &packets[i];
    if (packets[i].stripId == 2 && !packets[i].split) strip2 = &packets[i];
  }
  if (!strip0 || !strip2) {
    std::printf("packet-loss test expected nominal strips\n");
    return 1;
  }
  int16_t decoded0[thermal_a::kStripPixels];
  int16_t decoded2[thermal_a::kStripPixels];
  if (!thermal_a::decodeStrip(strip0->data, strip0->length, decoded0) ||
      !thermal_a::decodeStrip(strip2->data, strip2->length, decoded2)) {
    return 1;
  }
  for (int i = 0; i < thermal_a::kStripPixels; ++i) {
    if (decoded0[i] != thermal_a::quantizeTemperature(frame[i])) return 1;
    if (decoded2[i] != thermal_a::quantizeTemperature(frame[2 * thermal_a::kStripPixels + i]))
      return 1;
  }
  std::printf("packet-loss test: strips 0 and 2 decode exactly\n");

  // Large hot objects must use the split fallback but stay lossless.
  const std::vector<float> hot = hotFrame();
  thermal_a::Packet hotPackets[thermal_a::kMaxPackets];
  const int hotCount = thermal_a::encodeFrame(hot.data(), 9, hotPackets);
  int16_t hotDecoded[kPixels];
  bool sawSplit = false;
  for (int i = 0; i < hotCount; ++i) sawSplit = sawSplit || hotPackets[i].split;
  if (hotCount < thermal_a::kStripCount || !sawSplit ||
      !thermal_a::decodeFrame(hotPackets, hotCount, hotDecoded)) {
    std::printf("hot-scene split test failed (count=%d)\n", hotCount);
    return 1;
  }
  for (int i = 0; i < kPixels; ++i) {
    if (hotDecoded[i] != thermal_a::quantizeTemperature(hot[i])) return 1;
  }
  std::printf("hot-scene test: %d packets, split fallback, lossless\n", hotCount);
  std::printf("worst nominal frame: %zu B, all <= %d B\n", worst, thermal_a::kMaxPacketBytes);
  return 0;
}

int runFixture(const std::string &fixturePath, const std::string &packetPath) {
  std::vector<int16_t> frame;
  if (!readFixture(fixturePath, frame)) {
    std::fprintf(stderr, "cannot read fixture %s\n", fixturePath.c_str());
    return 1;
  }
  thermal_a::Packet packets[thermal_a::kMaxPackets];
  const int count = thermal_a::encodeQuantizedFrame(frame.data(), 0, packets);
  if (count == 0 || !writePackets(packetPath, packets, count)) {
    std::fprintf(stderr, "fixture encode/write failed\n");
    return 1;
  }
  int16_t decoded[kPixels];
  if (!thermal_a::decodeFrame(packets, count, decoded) ||
      std::memcmp(decoded, frame.data(), sizeof(decoded)) != 0) {
    std::fprintf(stderr, "fixture round trip mismatch\n");
    return 1;
  }
  std::printf("fixture: %s -> %s (%d packets), C++ round trip exact\n", fixturePath.c_str(),
              packetPath.c_str(), count);
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  std::string fixturePath;
  std::string packetPath;
  std::string floatFixturePath;
  std::string floatPacketPath;
  std::string pythonPacketPath;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) {
      fixturePath = argv[++i];
    } else if (std::strcmp(argv[i], "--packets") == 0 && i + 1 < argc) {
      packetPath = argv[++i];
    } else if (std::strcmp(argv[i], "--float-fixture") == 0 && i + 1 < argc) {
      floatFixturePath = argv[++i];
    } else if (std::strcmp(argv[i], "--float-packets") == 0 && i + 1 < argc) {
      floatPacketPath = argv[++i];
    } else if (std::strcmp(argv[i], "--py-packets") == 0 && i + 1 < argc) {
      pythonPacketPath = argv[++i];
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 2;
    }
  }

  if (runSynthetic() != 0) return 1;
  if (!fixturePath.empty()) {
    if (packetPath.empty()) {
      std::fprintf(stderr, "--fixture requires --packets\n");
      return 2;
    }
    if (runFixture(fixturePath, packetPath) != 0) return 1;
  }
  if (!floatFixturePath.empty()) {
    std::vector<float> frame;
    if (!readFloatFixture(floatFixturePath, frame)) {
      std::fprintf(stderr, "cannot read float fixture %s\n", floatFixturePath.c_str());
      return 1;
    }
    thermal_a::Packet packets[thermal_a::kMaxPackets];
    const int count = thermal_a::encodeFrame(frame.data(), 0, packets);
    int16_t decoded[kPixels];
    if (count == 0 || !thermal_a::decodeFrame(packets, count, decoded)) return 1;
    for (int i = 0; i < kPixels; ++i) {
      if (decoded[i] != thermal_a::quantizeTemperature(frame[i])) {
        std::fprintf(stderr, "float fixture round trip mismatch at %d\n", i);
        return 1;
      }
    }
    std::printf("float fixture: %d packets, round trip exact\n", count);
    if (!floatPacketPath.empty()) {
      if (!writePackets(floatPacketPath, packets, count)) {
        std::fprintf(stderr, "cannot write %s\n", floatPacketPath.c_str());
        return 1;
      }
      std::printf("wrote %s\n", floatPacketPath.c_str());
    }
  }
  if (!pythonPacketPath.empty()) {
    if (fixturePath.empty()) {
      std::fprintf(stderr, "--py-packets requires --fixture\n");
      return 2;
    }
    std::vector<int16_t> frame;
    std::vector<std::vector<uint8_t>> blobs;
    if (!readFixture(fixturePath, frame) || !readPackets(pythonPacketPath, blobs)) {
      std::fprintf(stderr, "cannot read Python packets/fixture\n");
      return 1;
    }
    thermal_a::Packet packets[thermal_a::kMaxPackets];
    for (size_t i = 0; i < blobs.size(); ++i) {
      if (blobs[i].size() > thermal_a::kMaxPacketBytes) return 1;
      std::memcpy(packets[i].data, blobs[i].data(), blobs[i].size());
      packets[i].length = static_cast<uint16_t>(blobs[i].size());
    }
    int16_t decoded[kPixels];
    if (!thermal_a::decodeFrame(packets, static_cast<int>(blobs.size()), decoded) ||
        std::memcmp(decoded, frame.data(), sizeof(decoded)) != 0) {
      std::fprintf(stderr, "Python -> C++ decode mismatch\n");
      return 1;
    }
    std::printf("Python encoder -> C++ decoder: bit-exact\n");
  }
  std::printf("ALL PROFILE-A HOST TESTS PASSED\n");
  return 0;
}
