/// test_build_frame.cpp — Tests for cn105_protocol::build_frame().
/// Deps: frame_parser.h (production code, no mocks needed)
///
/// build_frame() assembles the frames the ESP sends to the upstream controller
/// (PAC UART): FC cmd 01 30 len payload... checksum. These tests check it against
/// a real captured frame and round-trip it through FrameParser.
#include <gtest/gtest.h>
#include <cstring>
#include "frame_parser.h"

using namespace cn105_protocol;

TEST(BuildFrame, ConnectAckMatchesCapture) {
    uint8_t frame_buffer[16];
    const uint8_t success_status = 0x00;
    const uint8_t expected_frame[] = {0xFC, 0x7A, 0x01, 0x30, 0x01, 0x00, 0x54};

    const int frame_size = build_frame(0x7A, &success_status, 1, frame_buffer);

    ASSERT_EQ(frame_size, 7);
    EXPECT_EQ(memcmp(frame_buffer, expected_frame, 7), 0);
}

TEST(BuildFrame, EmptyPayloadHasHeaderAndChecksumOnly) {
    uint8_t frame_buffer[16];
    const uint8_t expected_frame[] = {0xFC, 0x7A, 0x01, 0x30, 0x00, 0x55};

    const int frame_size = build_frame(0x7A, nullptr, 0, frame_buffer);

    ASSERT_EQ(frame_size, 6);
    EXPECT_EQ(memcmp(frame_buffer, expected_frame, 6), 0);
}

TEST(BuildFrame, RoundTripsThroughFrameParser) {
    uint8_t payload[16] = {0x02, 0, 0, 1, 1, 0x1A};
    uint8_t frame_buffer[32];

    const int frame_size = build_frame(0x62, payload, 16, frame_buffer);

    FrameParser parser;
    for (int i = 0; i < frame_size; i++) {
        parser.feed(frame_buffer[i]);
    }
    ASSERT_TRUE(parser.frame_complete());
    EXPECT_TRUE(parser.checksum_valid());
    EXPECT_EQ(parser.command(), 0x62);
    EXPECT_EQ(parser.data_length(), 16);
    EXPECT_EQ(parser.data()[5], 0x1A);
}
