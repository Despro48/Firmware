// Host-side test for the profile-B firmware encoder. No Arduino toolchain:
//   g++ -std=c++17 -O2 -Wall -Wextra src/thermal_codec_b.cpp test/host/test_option_b.cpp \
//       -o /tmp/test_option_b && /tmp/test_option_b
//
// Optional cross-language check:
//   /tmp/test_option_b --fixture /tmp/frame_q.bin --packet /tmp/packet_b.bin

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "../../src/thermal_codec_b.h"

namespace {

constexpr int kPixels = thermal_b::kPixels;

float blob(float x, float y, float cx, float cy, float amplitude, float sigma) {
  const float dx = x - cx;
  const float dy = y - cy;
  return amplitude * std::exp(-(dx * dx + dy * dy) / (2.0f * sigma * sigma));
}

std::vector<float> syntheticFrame(uint32_t seed, float noise = 0.08f) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> dist(0.0f, noise);
  std::vector<float> frame(kPixels);
  for (int y = 0; y < thermal_b::kFrameHeight; ++y) {
    for (int x = 0; x < thermal_b::kFrameWidth; ++x) {
      float value = 25.0f + 0.04f * x - 0.06f * y;
      value += blob(x, y, 10, 8, 7.0f, 3.0f);
      value += blob(x, y, 22, 16, 4.0f, 4.0f);
      value += blob(x, y, 5, 18, -5.0f, 5.0f);
      value += blob(x, y, 27, 5, 2.0f, 2.5f);
      frame[y * thermal_b::kFrameWidth + x] = value + dist(rng);
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

bool writePacket(const std::string &path, const uint8_t *packet) {
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  const uint16_t length = thermal_b::kPacketBytes;
  out.write(reinterpret_cast<const char *>(&length), sizeof(length));
  out.write(reinterpret_cast<const char *>(packet), length);
  return static_cast<bool>(out);
}

void report(const char *label, const uint8_t *packet) {
  const int minimum = static_cast<int16_t>((static_cast<uint16_t>(packet[2]) << 8) |
                                           static_cast<uint16_t>(packet[1]));
  std::printf("%s packet=%d B q_scale=%u min_q=%d rs=%s\n", label, thermal_b::kPacketBytes,
              packet[3], minimum, thermal_b::rsCodewordIsValid(packet) ? "valid" : "INVALID");
}

}  // namespace

int main(int argc, char **argv) {
  std::string fixturePath;
  std::string packetPath;
  std::string floatFixturePath;
  std::string floatPacketPath;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) {
      fixturePath = argv[++i];
    } else if (std::strcmp(argv[i], "--packet") == 0 && i + 1 < argc) {
      packetPath = argv[++i];
    } else if (std::strcmp(argv[i], "--float-fixture") == 0 && i + 1 < argc) {
      floatFixturePath = argv[++i];
    } else if (std::strcmp(argv[i], "--float-packet") == 0 && i + 1 < argc) {
      floatPacketPath = argv[++i];
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 2;
    }
  }

  std::printf("== synthetic encodes (profile B) ==\n");
  for (uint32_t seed = 0; seed < 20; ++seed) {
    const std::vector<float> frame = syntheticFrame(seed);
    uint8_t packet[thermal_b::kPacketBytes];
    if (!thermal_b::encodeFrame(frame.data(), static_cast<uint8_t>(seed), packet)) {
      std::printf("seed %u: encode failed\n", seed);
      return 1;
    }
    if (!thermal_b::rsCodewordIsValid(packet)) {
      std::printf("seed %u: bad RS codeword\n", seed);
      return 1;
    }
    char label[32];
    std::snprintf(label, sizeof(label), "seed %2u", seed);
    report(label, packet);
  }

  // Adversarial full-range input must still fit and produce a valid codeword.
  std::mt19937 rng(1234);
  std::vector<float> noise(kPixels);
  for (float &value : noise) value = static_cast<float>(rng() % 16000) * 0.01f - 80.0f;
  uint8_t noisy[thermal_b::kPacketBytes];
  if (!thermal_b::encodeFrame(noise.data(), 250, noisy) ||
      !thermal_b::rsCodewordIsValid(noisy)) {
    std::printf("adversarial frame failed\n");
    return 1;
  }
  report("noise", noisy);

  if (!fixturePath.empty()) {
    std::vector<int16_t> frame;
    if (!readFixture(fixturePath, frame)) {
      std::fprintf(stderr, "cannot read fixture %s\n", fixturePath.c_str());
      return 1;
    }
    uint8_t packet[thermal_b::kPacketBytes];
    if (!thermal_b::encodeQuantized(frame.data(), 0, packet)) {
      std::fprintf(stderr, "fixture encode failed\n");
      return 1;
    }
    if (!thermal_b::rsCodewordIsValid(packet)) {
      std::fprintf(stderr, "fixture RS codeword invalid\n");
      return 1;
    }
    report("fixture", packet);
    if (!packetPath.empty() && !writePacket(packetPath, packet)) {
      std::fprintf(stderr, "cannot write %s\n", packetPath.c_str());
      return 1;
    }
    if (!packetPath.empty()) {
      std::printf("wrote %s\n", packetPath.c_str());
    }
  }

  if (!floatFixturePath.empty()) {
    std::vector<float> frame;
    if (!readFloatFixture(floatFixturePath, frame)) {
      std::fprintf(stderr, "cannot read float fixture %s\n", floatFixturePath.c_str());
      return 1;
    }
    uint8_t packet[thermal_b::kPacketBytes];
    if (!thermal_b::encodeFrame(frame.data(), 0, packet) ||
        !thermal_b::rsCodewordIsValid(packet)) {
      std::fprintf(stderr, "float fixture encode failed\n");
      return 1;
    }
    report("float", packet);
    if (!floatPacketPath.empty() && !writePacket(floatPacketPath, packet)) {
      std::fprintf(stderr, "cannot write %s\n", floatPacketPath.c_str());
      return 1;
    }
    if (!floatPacketPath.empty()) std::printf("wrote %s\n", floatPacketPath.c_str());
  }

  std::printf("ALL PROFILE-B ENCODER HOST TESTS PASSED\n");
  return 0;
}
