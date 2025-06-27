#include "ApplicationLayer.hpp"
#include "TPMessage.hpp"
#include "TPProtocol.hpp"
#include "Message.hpp"

namespace CAN::Application {

    uint16_t sendPingMessage(NodeIDs destinationAddress, uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, destinationAddress, false}};
        tp_message.appendUint8(Ping);
        Message default_message{};
        return TPProtocol::createCANTPMessage(tp_message, nullptr, default_message, retries);
    }

    uint16_t sendPongMessage(uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, OBC, false}};
        tp_message.appendUint8(Pong);
        Message default_message{};
        return TPProtocol::createCANTPMessage(tp_message, nullptr, default_message, retries);
    }

    uint16_t sendHeartbeatMessage(uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, OBC, false}};
        tp_message.appendUint8(Heartbeat);
        Message default_message{};
        return TPProtocol::createCANTPMessage(tp_message, nullptr, default_message, retries);
    }

    uint16_t EPSWriteRegisterThroughCAN(TPMessage& message, uint16_t ms_i2c_timeout) {
        uint16_t size = message.data[1];
        size = size << 8;
        size = size | message.data[2];
        auto status = HAL_I2C_Master_Transmit(&hi2c2, 0x20 << 1, &message.data[3], size, ms_i2c_timeout);
        if (status != HAL_OK) {
            if (status == HAL_ERROR)
                return 0;
            if (status == HAL_BUSY)
                return 0;
            return 0;
        }
        return 1;
    }

    uint16_t createRequestParametersMessage(NodeIDs destinationAddress,
                                                       const etl::array<uint16_t, TPMessageMaximumArguments>& parameterIDs,
                                                       uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, destinationAddress, false}};

        tp_message.appendUint8(RequestParameters);
        tp_message.appendUint16(parameterIDs.size());

        if constexpr (1) {
            // String<128> logString = "Requesting parameters with ID: ";
            for (auto parameterID: parameterIDs) {
                if (parameterID == 0) {
                    continue;
                }
                // etl::to_string(parameterID, logString, true);
                // tp_message.append("");
                tp_message.append(parameterID);
            }
            // LOG_DEBUG << logString.c_str();
        } else {
            for (auto parameterID: parameterIDs) {
                tp_message.append(parameterID);
            }
        }
        Message default_message{};
        return TPProtocol::createCANTPMessage(tp_message, nullptr, default_message, retries); // todo: maybe expect response
    }


    uint16_t createPacketMessage(NodeIDs destinationAddress, const etl::string<1024>& incomingMessage, Message::PacketType packetType, uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, destinationAddress, false}};

        if (packetType == Message::TM) {
            tp_message.appendUint8(TMPacket);
        } else {
            tp_message.appendUint8(TCPacket);
        }
        tp_message.appendString(incomingMessage);
        Message default_message{};
        return TPProtocol::createCANTPMessage(tp_message, nullptr, default_message, retries);
    }

    uint16_t createTCPacket(NodeIDs destinationAddress, Message& message, uint8_t retries) {
        TPMessage tp_message = {{0, 0, NodeID, destinationAddress, false}};
        tp_message.appendUint8(TCPacket);
        tp_message.appendMessage(message, message.total_size_ecss_);
        //
        tp_message.serviceType = message.serviceType;
        tp_message.messageType = message.messageType;
        tp_message.total_size_ecss_ = message.total_size_ecss_;
        tp_message.data_size_ecss_ = message.data_size_ecss_;
        tp_message.packet_type_ = message.packet_type_;
        //
        return TPProtocol::createCANTPMessage(tp_message, nullptr, message, retries);
    }



    void parseTMMessage(TPMessage& message) {
        // String<ECSSMaxMessageSize> logString = message.data.data() + 1;
        // LOG_DEBUG << logString.c_str();
    }

    void parseTCMessage(TPMessage& message) {
        // Message teleCommand = MessageParser::parseECSSTC(message.data.data() + 1);
        // MessageParser::execute(teleCommand);
    }

} // namespace CAN::Application