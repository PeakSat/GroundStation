#include "RF_RXTask.hpp"
#include "RF_TXTask.hpp"
#include "Logger.hpp"

using namespace AT86RF215;

void RF_RXTask::execute() {
    vTaskDelay(5000);
    LOG_INFO << "RF RX TASK";
    /// Set the Up-link frequency
    transceiver.freqSynthesizerConfig.setFrequency_FineResolution_CMN_1(FrequencyUHFRX);
    /// Check transceiver connection
    if (xSemaphoreTake(TransceiverHandler::transceiver_semaphore, portMAX_DELAY) == pdTRUE) {
        auto status = transceiver.check_transceiver_connection(error);
        if (status.has_value()) {
            LOG_INFO << "AT86RF215 CONNECTION OK";
        } else
            /// TODO: Error handling
            LOG_ERROR << "AT86RF215 ##ERROR## WITH CODE: " << status.error();
        transceiver.configure_pll(AT86RF215::RF09, transceiver.freqSynthesizerConfig.channelCenterFrequency09, transceiver.freqSynthesizerConfig.channelNumber09, transceiver.freqSynthesizerConfig.channelMode09, transceiver.freqSynthesizerConfig.loopBandwidth09, transceiver.freqSynthesizerConfig.channelSpacing09, error);
        transceiver.setup(error);
        xSemaphoreGive(TransceiverHandler::transceiver_semaphore);
    }
    xTaskNotify(rf_txtask->taskHandle, START_TX_TASK, eSetBits);

    /*
        transceiver.set_state(AT86RF215::RF09, State::RF_TXPREP, error);
        vTaskDelay(pdMS_TO_TICKS(20));
        transceiver.set_state(AT86RF215::RF09, State::RF_RX, error);
        if (transceiver.get_state(RF09, error) == RF_RX)
            LOG_DEBUG << "STATE = RX";
    */
    uint32_t receivedEvents = 0;
    uint16_t received_length;
    while (1) {
        vTaskDelay(1000);
        if (xSemaphoreTake(TransceiverHandler::transceiver_semaphore, portMAX_DELAY) == pdTRUE) {
            if (transceiver.get_state(RF09, error) == State::RF_RX)
                LOG_INFO << "RX";
            else {
                LOG_ERROR << "STATE FROM RX TASK: " << transceiver.get_state(RF09, error);
            }
            xSemaphoreGive(TransceiverHandler::transceiver_semaphore);
        }
        /*
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &receivedEvents, 5000)) {
            if ((receivedEvents & RXFE) || (receivedEvents & AGC_HOLD)) {
                LOG_DEBUG << "RECEIVE FRAME END || AGC HOLD";
                if (xSemaphoreTake(TransceiverHandler::transceiver_semaphore, portMAX_DELAY) == pdTRUE) {
                    auto result = transceiver.get_received_length(RF09, error);
                    if (result.has_value()) {
                        received_length = result.value();
                        LOG_INFO << "RX PACKET WITH RECEPTION LENGTH: " << received_length;
                    } else {
                        Error err = result.error();
                        LOG_ERROR << "AT86RF215 ##ERROR## AT RX LENGTH RECEPTION WITH CODE: " << err;
                    }
                    vTaskDelay(20);
                    transceiver.set_state(AT86RF215::RF09, State::RF_RX, error);
                    xSemaphoreGive(TransceiverHandler::transceiver_semaphore);
                }
            }
        }*/
    }
}