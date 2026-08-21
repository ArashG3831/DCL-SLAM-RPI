#include "my_epuck_project_cpp/d500_parser.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace my_epuck_project_cpp;

static std::array<std::uint8_t, kD500PacketLength> make_packet(
  std::uint16_t start, std::uint16_t end, std::uint16_t distance = 1000)
{
  std::array<std::uint8_t, kD500PacketLength> p{};
  p[0] = kD500Header0; p[1] = kD500VerLen;
  p[2] = 0xB0; p[3] = 0x04;  // 1200 dps -> 200 rpm
  p[4] = static_cast<std::uint8_t>(start & 0xff); p[5] = start >> 8;
  for (std::size_t i = 0; i < kD500Points; ++i) {
    const auto off = 6 + i * 3;
    p[off] = static_cast<std::uint8_t>(distance & 0xff); p[off + 1] = distance >> 8; p[off + 2] = 10;
  }
  p[42] = static_cast<std::uint8_t>(end & 0xff); p[43] = end >> 8;
  p[44] = 1; p[45] = 0;
  p[46] = d500_crc8(p.data(), 46);
  return p;
}

TEST(D500Parser, PacketAndCrc)
{
  auto p = make_packet(0, 3300);
  D500Packet parsed;
  ASSERT_TRUE(parse_d500_packet(p, parsed));
  EXPECT_NEAR(parsed.rpm, 200.0, 1e-12);
  EXPECT_NEAR(parsed.angles_deg.front(), 0.0, 1e-12);
  EXPECT_NEAR(parsed.angles_deg.back(), 33.0, 1e-12);
  p[20] ^= 0x01;
  EXPECT_FALSE(parse_d500_packet(p, parsed));
}

TEST(D500Parser, ChunkBoundariesAndResynchronization)
{
  D500StreamParser parser;
  std::vector<std::uint8_t> bytes{0x00, 0x01, 0x02};
  const auto a = make_packet(35000, 35900);
  const auto b = make_packet(100, 1000);
  bytes.insert(bytes.end(), a.begin(), a.end()); bytes.insert(bytes.end(), b.begin(), b.end());
  for (auto byte : bytes) parser.feed(&byte, 1, 1.0);
  EXPECT_EQ(parser.packet_count(), 2U);
}

TEST(D500Parser, BadCrcDoesNotBecomePacket)
{
  D500StreamParser parser;
  auto p = make_packet(0, 1000); p.back() ^= 0xff;
  parser.feed(p.data(), p.size(), 1.0);
  EXPECT_EQ(parser.packet_count(), 0U);
  EXPECT_EQ(parser.bad_crc_count(), 1U);
}
