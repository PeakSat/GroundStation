#include "RF_TXTask.hpp"
#include "Logger.hpp"

PacketData RF_TXTask::createRandomPacketData(uint16_t length) {
    PacketData data{};
    data.packet[0] = 0;
    for (int i = 1; i < length; i++)
        data.packet[i] = i ;
    data.length = length;
    return data;
}

void RF_TXTask::execute() {
    uint32_t receivedEvents = 0;
    if (xTaskNotifyWait(START_TX_TASK, START_TX_TASK, &receivedEvents, 10000) == pdTRUE) {
        if (receivedEvents & START_TX_TASK) {
            LOG_INFO << "Starting RF TX Task"; // Handle the event
        }
    } else {
        // In case the notification was not received, you can log or handle it
        LOG_ERROR << "RF RX Task notification error.";
    }
    /// Check transceiver connection
    if (xSemaphoreTake(TransceiverHandler::transceiver_semaphore, portMAX_DELAY) == pdTRUE) {
        auto status = transceiver.check_transceiver_connection(error);
        if (status.has_value()) {
            LOG_INFO << "AT86RF215 CONNECTION FROM TX TASK OK";
        } else
            /// TODO: Error handling
            LOG_ERROR << "AT86RF215 ##ERROR## WITH CODE: " << status.error();
        /// Set the down-link frequency
        transceiver.freqSynthesizerConfig.setFrequency_FineResolution_CMN_1(FrequencyUHFTX);
        transceiver.configure_pll(AT86RF215::RF09, transceiver.freqSynthesizerConfig.channelCenterFrequency09, transceiver.freqSynthesizerConfig.channelNumber09, transceiver.freqSynthesizerConfig.channelMode09, transceiver.freqSynthesizerConfig.loopBandwidth09, transceiver.freqSynthesizerConfig.channelSpacing09, error);
        transceiver.chip_reset(error);
        transceiver.setup(error);
        xSemaphoreGive(TransceiverHandler::transceiver_semaphore);
    }

    int i = 1;
    uint8_t counter = 0;
    PacketData packetTestData = createRandomPacketData(MaxPacketLength);
    while (1) {
        /// without delay there is an issue on the receiver. The TX stops the transmission after a while and the receiver loses packets.
        /// But why ? There is also the delay of the TXFE interrupt which is around [1 / (50000 / (packetLenghtInbits)) s]. It should work just fine...
        /// Maybe there is an error either on the tx side or the rx side, which is not printed.
        /// Typically 200ms delay works.
        /// We had the same issue on the campaign.
        vTaskDelay(500);
        if (xSemaphoreTake(TransceiverHandler::transceiver_semaphore, portMAX_DELAY) == pdTRUE) {
            if (i) {
                transceiver.transmitBasebandPacketsTx(RF09, packetTestData.packet.data(), packetTestData.length, error);
                i = 0;
            }
            if (xTaskNotifyWait(0xFFFFFFFF, TXFE, &receivedEvents, pdMS_TO_TICKS(1000))) {
                if (counter == 255)
                    counter = 0;
                counter += 1;
                packetTestData.packet[0] = counter;
                transceiver.transmitBasebandPacketsTx(RF09, packetTestData.packet.data(), packetTestData.length, error);
                LOG_INFO << "packet sent: " << counter;
            }
            receivedEvents = 0;
        }
        xSemaphoreGive(TransceiverHandler::transceiver_semaphore);
    }
}