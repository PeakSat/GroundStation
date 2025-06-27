#include "TPProtocol.hpp"
#include "CANGatekeeperTask.hpp"
#include "ApplicationLayer.hpp"
#include "GlobalVariables.hpp"
#include "Logger.hpp"

using namespace CAN;

extern FDCAN_HandleTypeDef hfdcan1;


uint16_t TPProtocol::parseMessage(TPMessage& tp_message, Message& message, uint16_t length, uint8_t retries) {
    uint16_t spacecraft_error_code = 1;
    switch (uint8_t messageType = static_cast<Application::MessageIDs>(tp_message.data[0])) {
        case Application::OperationalMode:
            // Application::parseOperationalModeMessage(tp_message);
            break;
        case Application::SendParameters:
            // spacecraft_error_code = Application::parseSendParametersMessage(tp_message);
            break;
        case Application::RequestParameters:
            // spacecraft_error_code = Application::parseRequestParametersMessage(tp_message, retries);
            break;
        case Application::TMPacket:
            Application::parseTMMessage(tp_message);
            break;
        case Application::Ping: {
            spacecraft_error_code = Application::sendPongMessage(1);
            break;
        }
        case Application::Pong: {
            auto senderID = tp_message.idInfo.sourceAddress;
            auto senderName = Application::nodeIdToString.at(senderID);
            break;
        }
        case Application::Heartbeat: {
            heartbeatReceived = true;
            break;
        }
        case Application::PingSubsystem: {
            // spacecraft_error_code = Application::pingCOMMSSubsystem(tp_message, retries);
            break;
        }
        case Application::LogMessage: {
            auto logData = String<1024>(message.data.data() + 1, message.data_size_ecss_ - 1);
            LOG_DEBUG << logData.c_str();
            break;
        }
        default:
            spacecraft_error_code = 0;
            break;
    }
    return spacecraft_error_code;
}

uint16_t TPProtocol::CANBreaker(TPMessage& tp_message, localPacketHandler* response, bool& recovered) {
    // if (COMMSParameters::COMMS_CAN_STATUS == COMMSParameters::CANNominal) {
    //     return GENERIC_ERROR_NONE;
    // }
    // Called by the ParameterMonitoringTask
    if (tp_message.data[0] == Application::Heartbeat) {
        auto error = createCANTPMessageNoRetransmit(tp_message, response);
        if (error == 1) {
            recovered = true;
        }
        return error;
    }
    return 0;
}

uint16_t TPProtocol::createCANTPMessage(TPMessage& tp_message, localPacketHandler* response, const Message& message, uint8_t retries) {
    // Check if can is out of commission
    if (xSemaphoreTake(canHandler.getTransmitMutex(), pdMS_TO_TICKS(canHandler.getTransmitTimeout())) == pdFALSE) {
        return 0;
    }
    // bool recovered = false;
    // uint16_t breakerError = CANBreaker(tp_message, response, recovered);
    // if (breakerError != 1) {
    //     if (recovered == true) {
    //         xSemaphoreGive(canHandler.getTransmitMutex());
    //         return 1;
    //     }
    //     xSemaphoreGive(canHandler.getTransmitMutex());
    //     return breakerError;
    // }

    // try sending message
    auto error = createCANTPMessageWithRetry(tp_message, response, message, retries);

    // start FDIR if necessary
    if (error != 1) {
        // CHANGE CAN BUS
        // FDIR
        if (activeBus == Redundant) {
            activeBus = Main;
            canGatekeeperTask->switchActiveBus(Main);
        } else {
            activeBus = Redundant;
            canGatekeeperTask->switchActiveBus(Redundant);
        }
        error = createCANTPMessageWithRetry(tp_message, response, message, retries);

        if (error != 1) {
            uint32_t can1error = HAL_FDCAN_GetError(&hfdcan1);

            FDCAN_ErrorCountersTypeDef CAN1errorCounter;
            HAL_FDCAN_GetErrorCounters(&hfdcan1, &CAN1errorCounter);
            FDCAN_ErrorCountersTypeDef CAN2errorCounter;

            FDCAN_ProtocolStatusTypeDef CAN1ProtocolStatus;
            HAL_FDCAN_GetProtocolStatus(&hfdcan1, &CAN1ProtocolStatus);
            FDCAN_ProtocolStatusTypeDef CAN2ProtocolStatus;


            xSemaphoreGive(canHandler.getTransmitMutex());
            return error;
        }
    }
    xSemaphoreGive(canHandler.getTransmitMutex());
    return 1;
}


uint16_t TPProtocol::createCANTPResponse(TPMessage& tp_message, const Message& message, uint8_t retries) {
    if (tp_message.data[0] != Application::Response) {
        return 0;
    }
    return createCANTPMessage(tp_message, nullptr, message, retries);
}

uint16_t TPProtocol::createCANTPMessageWithRetry(TPMessage& tp_message, localPacketHandler* response, const Message& message, uint8_t retries) {
    auto error = 1;
    for (uint32_t i = 0; i < retries; i++) {
        error = createCANTPMessageNoRetransmit(tp_message, response);
        if (error == 1) {
            return 1;
        }
        // COMMSParameters::COMMS_CAN_RETRANSMIT_COUNTER = COMMSParameters::COMMS_CAN_RETRANSMIT_COUNTER + 1;
    }
    return error;
}
uint16_t TPProtocol::createCANTPMessageNoRetransmit(TPMessage& tp_message, localPacketHandler* response) {
    if (tp_message.packet_type_ == Message::TC)
        __NOP();
    uint16_t messageSize = tp_message.data_size_message_; // +1 for  the message ID of the TPProtocol
    IdInfo identifier{};
    identifier.messageType = tp_message.idInfo.messageType;
    identifier.destinationAddress = tp_message.idInfo.destinationAddress;
    identifier.sourceAddress = tp_message.idInfo.sourceAddress;
    identifier.TCTLMID = tp_message.idInfo.TCTLMID;
    identifier.isMulticast = false;
    uint32_t id = TPMessage::encodeId(identifier);

    // Add a dummy byte for single byte messages so that the gatekeeper can distinguish it from trash
    if (messageSize == 1) {
        messageSize = 2;
        tp_message.appendUint8(0xAA);
    }

    // First Frame

    responsePointer = response;
    if (tp_message.data[0] != Application::Response) {
        expectingACK = true;
        ACKReceived = false;
        if (response != nullptr) {
            response->MessageID = CAN::Application::MessageIDs::InvalidMessageID;
        }
    }

    // 4 MSB bits is the Frame Type identifier and the 4 LSB are the leftmost 4 bits of the data length.
    uint8_t firstByte = (First << 6) | ((messageSize >> 8) & 0b111111);
    // Rest of the data length.
    uint8_t secondByte = messageSize & 0xFF;

    etl::array<uint8_t, CAN::MaxPayloadLength> firstFrame = {firstByte, secondByte};

    auto queue_status = canGatekeeperTask->send({id, firstFrame}, 50);
    if (queue_status == 1) {
        xTaskNotifyGive(canGatekeeperTask->taskHandle);
    } else {
        xQueueReset(outgoingQueue);
        return 0;
    }


    // Consecutive Frames
    uint8_t totalConsecutiveFramesNeeded = ceil(static_cast<float>(messageSize) / UsableDataLength);
    for (uint8_t currentConsecutiveFrameCount = 1;
         currentConsecutiveFrameCount <= totalConsecutiveFramesNeeded; currentConsecutiveFrameCount++) {
        uint8_t firstByte = (Consecutive << 6);
        if (currentConsecutiveFrameCount == totalConsecutiveFramesNeeded) {
            firstByte = (Final << 6);
        }
        etl::array<uint8_t, MaxPayloadLength> consecutiveFrame = {firstByte};
        consecutiveFrame.at(1) = currentConsecutiveFrameCount;

        for (uint8_t idx = 0; idx < UsableDataLength; idx++) {
            consecutiveFrame.at(idx + 2) = tp_message.data[idx + UsableDataLength * (currentConsecutiveFrameCount - 1)];
        }
        // Make sure the output buffers do not overflow // Added a small delay every 4 frames
        if (currentConsecutiveFrameCount % 6 == 5) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        auto queue_status = canGatekeeperTask->send({id, consecutiveFrame}, 50);
        if (queue_status == 1) {
            xTaskNotifyGive(canGatekeeperTask->taskHandle);
        } else {
            return 0;
        }
    }

    if (tp_message.data[0] == Application::Response) {
        expectingACK = false;
        ACKReceived = false;
        responsePointer = nullptr;
        LOG_DEBUG << "Sent response";
        return 1;
    }
    if (response != nullptr) {
        canHandler.setACKResponseTimeout(1100);
    } else {
        canHandler.setACKResponseTimeout(1000);
    }
    if (xSemaphoreTake(canHandler.getACKSemaphore(), pdMS_TO_TICKS(canHandler.getACKResponseTimeout())) == pdTRUE) {
        if (response != nullptr) {
            LOG_DEBUG << "CAN RESPONSE!";
        } else {
            // LOG_DEBUG << "CAN ACK received!";
        }
        responsePointer = nullptr;
        expectingACK = false;
        ACKReceived = false;
        return 1;
    }
    auto error = 1;
    if (ACKReceived == true) {
        error = 0;
    } else if (response != nullptr) {
        error = 0;
    } else {
        error = 0;
    }
    expectingACK = false;
    ACKReceived = false;
    responsePointer = nullptr;
    return error;
}