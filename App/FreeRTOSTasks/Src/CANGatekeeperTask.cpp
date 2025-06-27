#include "CANDriver.hpp"
#include "CANGatekeeperTask.hpp"
#include "CANParserTask.hpp"
#include "TPProtocol.hpp"
#include <ApplicationLayer.hpp>

#include "Logger.hpp"


bool expectingACK = false;
bool ACKReceived = false;

struct incomingFIFO incomingFIFO;

localPacketHandler CAN1PacketHandler __attribute__((section(".dtcmram_data_CAN1PacketHandler")));
localPacketHandler CAN2PacketHandler __attribute__((section(".dtcmram_data_CAN2PacketHandler")));
localPacketHandler* responsePointer = nullptr;

void CANGatekeeperTask::printActiveBus() const {
    if (ActiveBus != CAN::Main)
        LOG_DEBUG << "REDUNDANT IS THE ACTIVE CAN BUS";
    else
        LOG_DEBUG << "MAIN IS THE ACTIVE CAN BUS";
}

uint16_t CANGatekeeperTask::handleSingleFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler) {
    uint16_t spacecraft_error_code = 1;
    if (in_frame_handler.Data[1] == CAN::Application::ACK) {
        if (expectingACK) {
            ACKReceived = true;
            if (responsePointer == nullptr) {
                auto ID = static_cast<CAN::Application::MessageIDs>(in_frame_handler.Data[2]);
                xSemaphoreGive(canHandler.getACKSemaphore());
                // LOG_DEBUG << "CAN ACK received, ID: " << static_cast<uint32_t>(ID);
            }
        } else {
            spacecraft_error_code = 0;
        }
    } else {
        spacecraft_error_code = 0;
    }
    return spacecraft_error_code;
}

void CANGatekeeperTask::handleFirstFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler, uint8_t payloadLength) {
    packetHandler.PacketSize = (payloadLength << 8) | in_frame_handler.Data[1];
    packetHandler.PacketSize -= 1; // compensate for ID byte
    packetHandler.TailPointer = 0;
}

uint16_t CANGatekeeperTask::handleConsecutiveFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler) {
    const uint8_t frameNumber = in_frame_handler.Data[1] - 1;
    // Actual data are the first 6 bytes
    for (uint32_t i = 0; i < CAN::TPProtocol::UsableDataLength; i++) {
        if (i + frameNumber == 0) {
            packetHandler.MessageID = in_frame_handler.Data[CAN::TPProtocol::BytesStartingPoint];
        } else {
            if (sizeof(packetHandler.Buffer) / sizeof(packetHandler.Buffer[0]) > (frameNumber * (CAN::TPProtocol::UsableDataLength)) + i - 1) {
                packetHandler.Buffer[(frameNumber * (CAN::TPProtocol::UsableDataLength)) + i - 1] = in_frame_handler.Data[i + CAN::TPProtocol::BytesStartingPoint];
                packetHandler.TailPointer = packetHandler.TailPointer + 1;
            } else {
                packetHandler.TailPointer = 0;
                return 0;
            }
        }
    }
    return 1;
}

uint16_t CANGatekeeperTask::handleFinalFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler, uint16_t ms_to_wait_if_queue_is_full) {
    uint16_t spacecraft_error_code = 0;
    if ((packetHandler.PacketSize - packetHandler.TailPointer) <= (CAN::TPProtocol::UsableDataLength) && packetHandler.PacketSize != 0) {
        for (uint32_t i = 0; (packetHandler.PacketSize > packetHandler.TailPointer); i++) {
            uint8_t FrameNumber = in_frame_handler.Data[1] - 1;
            if ((i + FrameNumber) == 0) {
                packetHandler.MessageID = in_frame_handler.Data[CAN::TPProtocol::BytesStartingPoint];
            } else if (sizeof(packetHandler.Buffer) / sizeof(packetHandler.Buffer[0]) > packetHandler.TailPointer) {
                packetHandler.Buffer[packetHandler.TailPointer] = in_frame_handler.Data[i + CAN::TPProtocol::BytesStartingPoint];
                packetHandler.TailPointer = packetHandler.TailPointer + 1;
            } else {
                packetHandler.TailPointer = 0;
                return 0;
            }
        }
        // Check the BUS
        if (in_frame_handler.bus->Instance == FDCAN1) {
            packetHandler.CANInstance = CAN1;
        } else if (in_frame_handler.bus->Instance == FDCAN2) {
            packetHandler.CANInstance = CAN2;
        } else
            return 0;

        // Completed message processing
        spacecraft_error_code = processCompletedMessage(packetHandler, ms_to_wait_if_queue_is_full);
        if (spacecraft_error_code != 1)
            return spacecraft_error_code;
    } else {
        return 0;
    }
    packetHandler.TailPointer = 0;
    // Success
    return 1;
}

uint16_t CANGatekeeperTask::processCompletedMessage(localPacketHandler& packetHandler, uint16_t ms_to_wait_if_queue_is_full) {
    // Completed packet
    uint16_t spacecraft_error_code;
    if (packetHandler.MessageID == CAN::Application::MessageIDs::Response) {
        if (responsePointer != nullptr) {
            responsePointer->PacketSize = packetHandler.PacketSize;
            responsePointer->MessageID = packetHandler.MessageID;
            responsePointer->TailPointer = packetHandler.TailPointer;
            for (int i = 0; i < packetHandler.PacketSize; i++) {
                responsePointer->Buffer[i] = packetHandler.Buffer[i];
            }
            xSemaphoreGive(canHandler.getACKSemaphore());
        } else {
            spacecraft_error_code = 0;
            return spacecraft_error_code;
        }
    } else {
        //
        auto queue_status = xQueueSendToBack(incomingPacketQueue, &packetHandler, pdMS_TO_TICKS(ms_to_wait_if_queue_is_full));
        if (queue_status == errQUEUE_FULL) {
            spacecraft_error_code = 0;
            return spacecraft_error_code;
        }

        xTaskNotifyGive(canParserTask->taskHandle);
    }
    return 1;
}


uint16_t CANGatekeeperTask::processFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler) {
    // Extract metadata
    const uint8_t metadata = in_frame_handler.Data[0];
    const uint8_t frameType = metadata >> 6;
    const uint8_t payloadLength = metadata & 0x3F;
    uint16_t spacecraft_error_code = 1;
    switch (frameType) {
        case CAN::TPProtocol::Frame::First:
            handleFirstFrame(in_frame_handler, packetHandler, payloadLength);
            break;

        case CAN::TPProtocol::Frame::Single:
            spacecraft_error_code = handleSingleFrame(in_frame_handler, packetHandler);
            break;

        case CAN::TPProtocol::Frame::Consecutive:
            spacecraft_error_code = handleConsecutiveFrame(in_frame_handler, packetHandler);
            break;

        case CAN::TPProtocol::Frame::Final:
            spacecraft_error_code = handleFinalFrame(in_frame_handler, packetHandler, MS_WAIT_FOR_QUEUE_FULL);
            break;
        default:
            spacecraft_error_code = 0;
    }
    return spacecraft_error_code;
}


void CANGatekeeperTask::execute() {
    vTaskDelay(pdMS_TO_TICKS(TASK_WAIT_TO_BEGIN_MS));
    CAN::Packet out_message = {};
    CAN::Frame in_frame_handler = {};
    taskHandle = xTaskGetCurrentTaskHandle();
    while (true) {
        xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &receive_events_, pdMS_TO_TICKS(WAIT_FOR_NOTIFICATION_MS));
        while (uxQueueMessagesWaiting(incomingFrameQueue)) {
            // Get the message pointer from the queue
            xQueueReceive(incomingFrameQueue, &in_frame_handler, pdMS_TO_TICKS(100));

            IdInfo identifier = CAN::TPMessage::decodeId(in_frame_handler.header.Identifier);
            if (identifier.destinationAddress == CAN::TTC && identifier.sourceAddress == CAN::OBC) {
                localPacketHandler* CANPacketHandler = &CAN1PacketHandler;
                uint16_t spacecraft_error_code = 1;
                if (in_frame_handler.bus->Instance == FDCAN1)
                    spacecraft_error_code = processFrame(in_frame_handler, CAN1PacketHandler);
                else
                    spacecraft_error_code = processFrame(in_frame_handler, CAN2PacketHandler);
                //
                if (spacecraft_error_code != 1) {
                    // REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, false, MS_WAIT_FOR_QUEUE_FULL);
                }
            }
        }
        while (uxQueueMessagesWaiting(outgoingQueue)) {
            xQueueReceive(outgoingQueue, &out_message, portMAX_DELAY);

            uint16_t spacecraft_error_code = CAN::send(out_message, ActiveBus);
        }
    }
}