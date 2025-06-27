#include "CANParserTask.hpp"
#include "ApplicationLayer.hpp"
#include "CANGatekeeperTask.hpp"
#include <TPProtocol.hpp>


uint16_t CANParserTask::sendACK(CAN::Application::MessageIDs ID) {
    CAN::TPMessage ACKmessage = {{0, 0, CAN::NodeID, CAN::NodeIDs::OBC, false}}; //{{CAN::NodeID, CAN::NodeIDs::OBC, false}};
    ACKmessage.appendUint8(CAN::Application::MessageIDs::ACK);
    etl::array<uint8_t, CAN::MaxPayloadLength> data = {
        static_cast<uint8_t>(((CAN::TPProtocol::Single << 6) & 0xFF) | (1 & 0b111111))};
    for (size_t idx = 0; idx < 1; idx++) {
        data.at(idx + 1) = ACKmessage.data[idx];
    }
    data.at(2) = ID;

    IdInfo identifier;
    identifier.messageType = ACKmessage.idInfo.messageType;
    identifier.destinationAddress = ACKmessage.idInfo.destinationAddress;
    identifier.sourceAddress = ACKmessage.idInfo.sourceAddress;
    identifier.TCTLMID = ACKmessage.idInfo.TCTLMID;
    identifier.isMulticast = ACKmessage.idInfo.isMulticast;
    uint32_t id = ACKmessage.encodeId(identifier);

    auto queue_status = canGatekeeperTask->send({id, data}, 50);
    if (queue_status == 1) {
        xTaskNotifyGive(canGatekeeperTask->taskHandle);
    } else {
        return 0;
    }
    return 1;
}

uint16_t CANParserTask::handlePacket(const localPacketHandler& CANPacketHandler, uint16_t ms_to_wait_if_queue_is_full, uint8_t retries) {
    // sanity check
    if (CANPacketHandler.PacketSize > 1024)
        return 0;
    // get the message ID
    uint8_t messageID = static_cast<CAN::Application::MessageIDs>(CANPacketHandler.MessageID);

    // rest of the messages
    CAN::TPMessage tp_message;
    tp_message.appendUint8(CANPacketHandler.MessageID);
    for (int i = 0; i < CANPacketHandler.PacketSize; i++) {
        tp_message.appendUint8(CANPacketHandler.Buffer[i]);
    }
    tp_message.idInfo.sourceAddress = CAN::OBC;
    Message default_message{};
    return CAN::TPProtocol::parseMessage(tp_message, default_message, CANPacketHandler.PacketSize, retries);
}


void CANParserTask::execute() {
    vTaskDelay(pdMS_TO_TICKS(TASK_WAIT_TO_BEGIN_MS));
    while (true) {
        xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &received_events_, pdMS_TO_TICKS(WAIT_FOR_NOTIFICATION_MS));
        uint16_t spacecraft_error_code = 1;
        while (uxQueueMessagesWaiting(incomingPacketQueue)) {
            localPacketHandler CANPacketHandler;
            xQueueReceive(incomingPacketQueue, &CANPacketHandler, 0);

            if (uxQueueMessagesWaiting(incomingPacketQueue) == 0) {
                spacecraft_error_code = sendACK(static_cast<CAN::Application::MessageIDs>(CANPacketHandler.MessageID));
                spacecraft_error_code = handlePacket(CANPacketHandler, MS_WAIT_FOR_QUEUE_FULL, 1);
            }
        }
    }
}