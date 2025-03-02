#include "RF_RXTask.hpp"
#include "Logger.hpp"
#include <RF_TXTask.hpp>

using namespace AT86RF215;

void RF_RXTask::ensureRxMode() {
    switch (State trx_state = transceiver.get_state(RF09, error)) {
        case RF_NOP:
            LOG_DEBUG << "[RX ENSURE] STATE: NOP";
            break;
        case RF_SLEEP:
            LOG_DEBUG << "[RX ENSURE] STATE: SLEEP";
            break;
        case RF_TRXOFF:
            LOG_DEBUG << "[RX ENSURE] STATE: TRXOFF";
            transceiver.set_state(RF09, RF_TXPREP, error);
            /// the delay here is essential
            vTaskDelay(pdMS_TO_TICKS(20));
            transceiver.set_state(RF09, RF_RX, error);
            break;
        case RF_TX:
            LOG_DEBUG << "[RX ENSURE] STATE: TX";
            transceiver.set_state(RF09, RF_TXPREP, error);
            /// the delay here is essential
            vTaskDelay(pdMS_TO_TICKS(20));
            transceiver.set_state(RF09, RF_RX, error);
            // transceiver.print_state(RF09, error);
            break;
        case RF_RX:
            // LOG_DEBUG << "[RX ENSURE] STATE: RX";
            break;
        case RF_TRANSITION:
            LOG_DEBUG << "[RX ENSURE] STATE: TRANSITION";
            break;
        case RF_RESET:
            LOG_DEBUG << "[RX ENSURE] STATE: RESET";
            break;
        case RF_INVALID:
            LOG_DEBUG << "[RX ENSURE] STATE: INVALID";
            transceiver.set_state(RF09, RF_TRXOFF, error);
            vTaskDelay(pdMS_TO_TICKS(20));
            transceiver.set_state(RF09, RF_TXPREP, error);
            /// the delay here is essential
            vTaskDelay(pdMS_TO_TICKS(20));
            transceiver.set_state(RF09, RF_RX, error);
            LOG_DEBUG << "[RX ENSURE] STATE: INVALID";
            transceiver.print_state(RF09, error);
            break;
        case RF_TXPREP:
            LOG_DEBUG << "[RX ENSURE] STATE: TXPREP";
            transceiver.set_state(RF09, RF_RX, error);
            break;
        default:
            LOG_ERROR << "[RX ENSURE] STATE: UNDEFINED";
            break;
    }
}

[[noreturn]] void RF_RXTask::execute() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    LOG_INFO << "[RF RX TASK]";
    transceiver.freqSynthesizerConfig.setFrequency_FineResolution_CMN_1(FrequencyUHFRX);
    /// Check transceiver connection
    const int MAX_RETRIES = 3;
    int attempt = 0;
    bool success = false;
    while (attempt < MAX_RETRIES) {
        if (xSemaphoreTake(transceiver_handler.resources_mtx, portMAX_DELAY) == pdTRUE) {
            auto status = transceiver.check_transceiver_connection(error);
            if (status.has_value()) {
                success = true;  // Connection successful
                LOG_INFO << "[SPI CONNECTION ESTABLISHED]";
            } else {
                LOG_ERROR << "CONNECTION ##ERROR## WITH CODE: " << status.error();
            }
            transceiver.set_state(RF09, RF_TRXOFF, error);
            transceiver.configure_pll(RF09, transceiver.freqSynthesizerConfig.channelCenterFrequency09,
                                      transceiver.freqSynthesizerConfig.channelNumber09,
                                      transceiver.freqSynthesizerConfig.channelMode09,
                                      transceiver.freqSynthesizerConfig.loopBandwidth09,
                                      transceiver.freqSynthesizerConfig.channelSpacing09, error);
            transceiver.chip_reset(error);

            xSemaphoreGive(transceiver_handler.resources_mtx);

            if (success) {
                break;  // Exit loop if successful
            }

            attempt++;
            LOG_ERROR << "Retrying connection attempt " << attempt << "/" << MAX_RETRIES;
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay before retrying
        }
    }
    if (!success) {
        LOG_ERROR << "Failed to establish connection after " << MAX_RETRIES << " attempts.";
    }
    uint16_t received_length = 0;
    uint32_t drop_counter = 0, rx_total_packets = 0, rx_total_drop_packets = 0;
    uint32_t receivedEvents;
    ensureRxMode();
    while (true) {
        if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_RXFE_RX, pdFALSE, pdTRUE, &receivedEvents, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(transceiver_handler.resources_mtx, portMAX_DELAY) == pdTRUE) {
                auto result = transceiver.get_received_length(RF09, error);
                received_length = result.value();
                int16_t corrected_received_length = received_length - MAGIC_NUMBER;
                int8_t rssi = transceiver.get_rssi(RF09, error);
                uint8_t RX_BUFF[1024]{};
                LOG_DEBUG << "[RX AGC] LENGTH: " << corrected_received_length;
                if (rssi != 127)
                    LOG_DEBUG << "[RX AGC] RSSI [dBm]: " << rssi ;
                if (corrected_received_length > 0 && corrected_received_length <= 256) {
                    rx_total_packets++;
                    LOG_DEBUG << "[RX] total packets c: " << rx_total_packets;
                    drop_counter = 0;
                    for (int i = 0; i < corrected_received_length; i++) {
                        RX_BUFF[i] = transceiver.spi_read_8((BBC0_FBRXS) + i, error);
                        // LOG_DEBUG << "[RX] DATA: " << RX_BUFF[i];
                        if (error != NO_ERRORS)
                            LOG_ERROR << "ERROR" ;
                    }
                    /// TODO: parse the packet because it could be a TM if we are on the COMMS-GS or TC if we are on the COMMS-GS side
                    uint8_t packet_version_number = (RX_BUFF[0] >> 5) & 0x07;  // Top 3 bits
                    uint8_t packet_type = (RX_BUFF[0] >> 4) & 0x01;            // 4th bit
                    uint8_t secondary_header_flag = (RX_BUFF[0] >> 3) & 0x01;  // 5th bit
                    uint16_t application_process_ID = ((RX_BUFF[0] & 0x07) << 8) | RX_BUFF[1];  // Last 3 bits + full RX_BUFF[1]

                    LOG_DEBUG << "Packet Version Number: " << packet_version_number ;
                    LOG_DEBUG << "Packet Type: " << packet_type;
                    LOG_DEBUG << "Secondary Header Flag: " << secondary_header_flag;
                    LOG_DEBUG << "Application Process ID: " << application_process_ID;

                    if (packet_type == TM_PACKET) {
                        LOG_DEBUG << "[RX] TM RECEIVED" ;
                    }
                    else if (packet_type == TC_PACKET) {
                        LOG_DEBUG << "[RX AGC] NEW TC FROM COMMS-GS";
                    }
                    else {
                        LOG_DEBUG << "[RX AGC] Neither TC nor TM";
                    }
                    /// TODO: if the packet is TM print it with the format: New TM [3,25] ... call the TM_HandlingTask
                }
                else {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    drop_counter++;
                    rx_total_drop_packets++;
                    LOG_DEBUG << "[RX DROP] c: " << drop_counter;
                    LOG_DEBUG << "[RX DROP] total packets c: " << rx_total_drop_packets;
                    ensureRxMode();
                }
                ensureRxMode();
                xSemaphoreGive(transceiver_handler.resources_mtx);
            }
        }
    }
}



