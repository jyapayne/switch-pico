#include <assert.h>
#include <stdint.h>
#include "parser/uni_switch2_haptics.h"

static uint64_t unpack(const uint8_t sample[5]) {
    uint64_t result = 0;
    for (unsigned byte = 0; byte < 5; ++byte) result |= (uint64_t)sample[byte] << (byte * 8);
    return result;
}

int main(void) {
    uni_switch2_haptics_frame_t frame;
    uni_switch2_haptics_silence(&frame);
    assert(uni_switch2_haptics_valid(&frame));
    assert(uni_switch2_haptics_is_stop(&frame));
    uint64_t sample = unpack(frame.sides[0].samples[0]);
    assert((sample & 1023) == 385); // Measured160Hz anchor.
    assert(((sample >> 20) & 1023) == 481); // Measured320Hz anchor.
    assert(((sample >> 10) & 1023) == 0 && ((sample >> 30) & 1023) == 0);

    uni_switch2_haptics_encode_sample(frame.sides[0].samples[0], 96, 96, 32767, 16384);
    sample = unpack(frame.sides[0].samples[0]);
    assert((sample & 1023) == 481);
    assert(((sample >> 20) & 1023) == 577); // Upper frequency bit is frequency, not tone mode.
    assert(((sample >> 10) & 1023) == 453 && ((sample >> 30) & 1023) == 226);
    assert(!uni_switch2_haptics_is_stop(&frame));

    frame.sides[0].count = 3;
    uni_switch2_haptics_encode_sample(frame.sides[0].samples[1], 32, 64, 8192, 0);
    uni_switch2_haptics_encode_sample(frame.sides[0].samples[2], 64, 127, 0, 32767);
    uint8_t block[16];
    assert(uni_switch2_haptics_write_block(block, &frame.sides[0], 0xab));
    assert(block[0] == 0x7b);
    assert((unpack(block + 1) & 1023) == 481);
    assert((unpack(block + 6) & 1023) == 289);
    assert(((unpack(block + 6) >> 10) & 1023) == 113);
    assert(((unpack(block + 11) >> 20) & 1023) == 670);
    assert(((unpack(block + 11) >> 10) & 1023) == 0);
    assert(((unpack(block + 11) >> 30) & 1023) == 453);

    frame.sides[0].count = 2;
    assert(uni_switch2_haptics_write_block(block, &frame.sides[0], 15));
    assert(block[0] == 0x6f);
    for (unsigned i = 11; i < 16; ++i) assert(block[i] == 0);
    frame.sides[0].count = 1;
    assert(uni_switch2_haptics_write_block(block, &frame.sides[0], 16));
    assert(block[0] == 0x50);
    for (unsigned i = 6; i < 16; ++i) assert(block[i] == 0);

    uni_switch2_haptics_encode_sample(frame.sides[0].samples[0], 0, 255, 65535, 65535);
    sample = unpack(frame.sides[0].samples[0]);
    assert((sample & 1023) == 196 && ((sample >> 20) & 1023) == 670);
    assert(((sample >> 10) & 1023) == 453 && ((sample >> 30) & 1023) == 453);
    assert(uni_switch2_haptics_valid(&frame));
    // A malformed internal native frame cannot bypass the amplitude envelope.
    frame.sides[0].samples[0][2] |= 0x0f;
    assert(!uni_switch2_haptics_valid(&frame));

    uni_switch2_haptics_silence(&frame);
    frame.sides[1].count = 0;
    assert(uni_switch2_haptics_valid(&frame));
    assert(!uni_switch2_haptics_is_stop(&frame)); // A side-only mute must not clear its partner.
    frame.sides[0].count = 0;
    assert(!uni_switch2_haptics_valid(&frame));
    assert(!uni_switch2_haptics_write_block(block, &frame.sides[0], 0));
    frame.sides[0].count = 4;
    assert(!uni_switch2_haptics_valid(&frame));
    assert(!uni_switch2_haptics_write_block(block, &frame.sides[0], 0));
    return 0;
}
