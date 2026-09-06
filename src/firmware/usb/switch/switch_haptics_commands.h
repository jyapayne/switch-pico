#pragma once

#include <stdint.h>

// Internal command semantics shared by the host decoder and native encoder.
// Compressed forms follow the existing decoder, not the older public absolute
// rumble tables. Their physical acceptance/repeated-word behavior still needs
// qualification on each enabled controller model/firmware.
namespace SwitchHapticsCommands {
enum class CommandAction : uint8_t { Ignore, Default, Substitute, Sum };
struct HapticCommand {
    CommandAction amplitude_action;
    CommandAction frequency_action;
    int16_t amplitude_offset;
    int16_t frequency_offset;
};
constexpr HapticCommand kCommands[32] = {
    {CommandAction::Default, CommandAction::Default, 0, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 0, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 240, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 224, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 208, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 192, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 176, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 160, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 144, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 128, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 112, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 96, 0},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 5},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 5},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 0},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 7},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 7},
    {CommandAction::Sum, CommandAction::Sum, 4, 1},
    {CommandAction::Sum, CommandAction::Ignore, 4, 0},
    {CommandAction::Sum, CommandAction::Sum, 4, -1},
    {CommandAction::Sum, CommandAction::Sum, 1, 1},
    {CommandAction::Sum, CommandAction::Ignore, 1, 0},
    {CommandAction::Sum, CommandAction::Sum, 1, -1},
    {CommandAction::Ignore, CommandAction::Sum, 0, 1},
    {CommandAction::Ignore, CommandAction::Ignore, 0, 0},
    {CommandAction::Ignore, CommandAction::Sum, 0, -1},
    {CommandAction::Sum, CommandAction::Sum, -1, 1},
    {CommandAction::Sum, CommandAction::Ignore, -1, 0},
    {CommandAction::Sum, CommandAction::Sum, -1, -1},
    {CommandAction::Sum, CommandAction::Sum, -4, 1},
    {CommandAction::Sum, CommandAction::Ignore, -4, 0},
    {CommandAction::Sum, CommandAction::Sum, -4, -1},
};
inline uint8_t apply_command(CommandAction action, int16_t offset, uint8_t current,
                             uint8_t default_value, uint8_t maximum) {
    switch (action) {
    case CommandAction::Ignore: return current;
    case CommandAction::Default: return default_value;
    case CommandAction::Substitute: return static_cast<uint8_t>(offset);
    case CommandAction::Sum: {
        const int value = static_cast<int>(current) + offset;
        return static_cast<uint8_t>(value < 0 ? 0 : value > maximum ? maximum : value);
    }
    }
    return default_value;
}
constexpr uint8_t host_amplitude_to_lut_index(uint8_t host_index) {
    const unsigned index = host_index & 0x7fu;
    return static_cast<uint8_t>(index == 0 ? 0 : index < 16 ? 7u + 8u * index
                                   : index < 32 ? 97u + 2u * index : 128u + index);
}
}  // namespace SwitchHapticsCommands
