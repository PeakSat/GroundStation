#pragma once
#include "CANDriver.hpp"
#include "etl/map.h"
#include "etl/array.h"
#include "etl/String.hpp"
#include "Message.hpp"
#include "Peripheral_Definitions.hpp"
#include "TPMessage.hpp"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

namespace CAN::Application {


    uint16_t constexpr MAX_REQUEST_SEND_PARAMETERS = 32;
    /**
     * Entity that maps subsystem names to strings for use in logging functions.
     */
    inline etl::map<NodeIDs, String<LogSource::MaximumLettersInSubsystemName>, LogSource::NumberOfLogSources> nodeIdToString = {
        {OBC, "OBC"},
        {TTC, "TTC"},
        {ADCS, "ADCS"}};

    /**
     * CAN-TP message IDs, as specified in DDJF_OBDH.
     */
    enum MessageIDs : uint8_t {
        InvalidMessageID = 0,                    // 0
        SendParameters = 0x01,                   // 1
        RequestParameters = 0x02,                // 2
        PerformFunction = 0x03,                  // 3
        UpdateTime = 0x04,                       // 4
        OperationalMode = 0x05,                  // 5
        ACK = 0x06,                              // 6
        GNSSData = 0x07,                         // 7
        EPSRelay_I2CwriteRegister = 0x08,        // 8
        EPSRelay_I2CreadRegister = 0x09,         // 9
        Response = 0x0A,                         // 10
        EventReport = 0x0B,                      // 11
        PingSubsystem = 0x0C,                    // 12
        NACK = 0x15,                             // 21
        TMPacket = 0x20,                         // 32
        TCPacket = 0x23,                         // 35
        CCSDSPacket = 0x22,                     // 34
        Ping = 0x30,                             // 48
        Pong = 0x31,                             // 49
        LogMessage = 0x40,                       // 64
        UTCTime = 0x50,                          // 80
        BusSwitchover = 0x51,                    // 81
        Heartbeat = 0x52                         // 82
    };


    /**
     * Toggles the active CAN Bus.
     * @param bus A default argument that uses the currentBus member variable if a value is not provided.
     * @return The ID of the bus to be switched to.
     */
    ActiveBus switchBus(CAN::ActiveBus newBus);

    /**
     * The available Event Report Types, for an Event Report CAN-TP Message.
     */
    enum EventReportType : uint8_t {
        Informative = 0x0,
        LowSeverity = 0x1,
        MediumSeverity = 0x2,
        HighSeverity = 0x3
    };

    /**
     * The size in bytes for the function ID required in Perform Function Messages.
     */
    static constexpr uint8_t FunctionIdSize = 6;

    /**
     * Milliseconds per day.
     */
    inline constexpr uint64_t millisecondsPerDay = 24 * 60 * 60 * 1000;


    /**
     * Removes the ID of the sender in an incoming CAN Message.
     * @param id The ID to be filtered.
     * @return The filtered ID.
     */
    inline uint32_t filterMessageID(uint32_t id) {
        return id & 0x700;
    }

    /**
     * Adds a Ping message to the outgoing queue, according to DDJF_OBDH.
     */
    uint16_t sendPingMessage(NodeIDs destinationAddress, uint8_t retries);

    /**
     * Adds a Pong message to the outgoing queue, to be sent in response to a Ping message, according to DDJF_OBDH.
     */
    uint16_t sendPongMessage(uint8_t retries);

    /**
     * Handle the EPS communication relay.
     */
    uint16_t EPSWriteRegisterThroughCAN(TPMessage& message, uint16_t ms_i2c_timeout);
    uint16_t EPSReadRegisterThroughCAN(TPMessage& message, uint8_t retries, uint16_t ms_i2c_timeout);
    uint16_t pingCOMMSSubsystem(TPMessage& message, uint8_t retries);
    // uint16_t pingOBCSubsystem(pingIDs ID, uint8_t retries);
    uint16_t parseOperationalModeMessage(const TPMessage& message);

    /**
     * Adds a Heartbeat message to the outgoing queue, to be called periodically, according to DDJF_OBDH.
     */
    uint16_t sendHeartbeatMessage(uint8_t retries);

    /**
     * Sends a Send Parameters CAN-TP Message as described in DDJF_OBDH.
     * @param destinationAddress The ID of the destination node.
     * @param parameterIDs The IDs of the parameters to be sent.
     */
    uint16_t createSendParametersMessage(NodeIDs destinationAddress, const etl::array<uint16_t, TPMessageMaximumArguments>& parameterIDs, uint8_t retries);
    uint16_t createSendParametersMessageMap(NodeIDs destinationAddress, const etl::array<uint16_t, TPMessageMaximumArguments>& parameterIDs, uint8_t retries);

    /**
     * Sends a Request Parameters CAN-TP Message as described in DDJF_OBDH.
     * @param destinationAddress The ID of the destination node.
     * @param parameterIDs The IDs of the parameters to be requested.
     */
    uint16_t createRequestParametersMessage(NodeIDs destinationAddress, const etl::array<uint16_t, TPMessageMaximumArguments>& parameterIDs, uint8_t retries);

    /**
     * Creates an ECSS-E-ST-70-41C Services TM/TC packet to be sent. After creation the packet is split into CAN-TP
     * Protocol frames to be transmitted.
     * @param destinationAddress The ID of the destination node.
     */
    uint16_t createPacketMessage(NodeIDs destinationAddress, const etl::string<ECSSMaxStringSize>& incomingMessage, Message::PacketType packetType, uint8_t retries);

    uint16_t createTCPacket(NodeIDs destinationAddress, Message& message, uint8_t retries);

    /**
     * Creates a CCSDS packet to be sent. After creation the packet is split into CAN-TP
     * Protocol frames to be transmitted.
     * @param destinationAddress The ID of the destination node.
     * @param message An ECSS Message.
     */
    uint16_t createCCSDSPacketMessage(NodeIDs destinationAddress, const Message& message, uint8_t retries);

    /**
     * Parses an incoming Send Parameters Message and updates the according parameters
     * @param message An incoming TPMessage
     */
    uint16_t parseSendParametersMessage(TPMessage& message);

    /**
     * Parses an incoming Request Parameters Message and sends the according parameters back, if they exist.
     * @param message An incoming TPMessage
     */
    uint16_t parseRequestParametersMessage(TPMessage& message, uint8_t retries);

    /**
     * Parses an incoming TM Packet and logs it.
     * @param message An incoming TMPacket
     */
    void parseTMMessage(TPMessage& message);

    /**
     * Parses an incoming TC Packet and executes it.
     * @param message An incoming TCPacket
     */
    void parseTCMessage(TPMessage& message);
} // namespace CAN::Application
