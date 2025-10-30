#include <gtest/gtest.h>
#include "frame_parser.h"
using namespace cn105_protocol;

TEST(BuildFrame, ConnectAckMatchesCapture) {
    uint8_t out[16]; const uint8_t ok = 0x00;
    int n = build_frame(0x7A, &ok, 1, out);
    const uint8_t expect[] = {0xFC, 0x7A, 0x01, 0x30, 0x01, 0x00, 0x54};
    ASSERT_EQ(n, 7);
    EXPECT_EQ(memcmp(out, expect, 7), 0);
}

TEST(BuildFrame, RoundTripsThroughFrameParser) {
    uint8_t payload[16] = {0x02, 0, 0, 1, 1, 0x1A};
    uint8_t out[32];
    int n = build_frame(0x62, payload, 16, out);
    FrameParser p;
    for (int i = 0; i < n; i++) p.feed(out[i]);
    ASSERT_TRUE(p.frame_complete());
    EXPECT_TRUE(p.checksum_valid());
    EXPECT_EQ(p.command(), 0x62);
    EXPECT_EQ(p.data()[5], 0x1A);
}
