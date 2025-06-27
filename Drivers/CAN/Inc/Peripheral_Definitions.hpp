#pragma once

namespace LogSource {
    /**
    * The maximum number of letters in a string representation of a subsystem's name.
    */
    inline constexpr uint8_t MaximumLettersInSubsystemName = 8;

    /**
     * The maximum number of Logging Sources
     */
    inline constexpr uint8_t NumberOfLogSources = 5;

    /**
     * The subsystem to be used, if no other source is defined using the stream operator.
     */
    extern String<MaximumLettersInSubsystemName> currentSubsystem;
} // namespace LogSource


namespace CAN {

    enum NodeIDs : uint8_t {
        OBC = 0x5,
        TTC = 0x6,
        ADCS = 0x2,
    };
    /**
     * The ID for the current node as described in DDJF_OBDH
     */
    inline const NodeIDs NodeID = TTC;

    /**
     * The maximum of the length of the queue for incoming/outgoing CAN frames.
     */
    inline const uint8_t FrameQueueSize = 20;

    /**
     * The maximum size for the data field of a CAN-TP message.
     */
    inline const uint16_t TPMessageMaximumSize = 256;

    /**
     * The maximum numbers of parameters, function arguments etc. inside a single CAN-TP Message.
     */
    inline const uint8_t TPMessageMaximumArguments = 30;

} // namespace CAN

/**
 * Used to control COBS Encoding for Log Messages in the UART Gatekeeper task.
 */
inline const bool LogsAreCOBSEncoded = false;