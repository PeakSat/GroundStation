#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"
#include "Logger.hpp"
#include <timers.h>
#include "main.h"

void RF_TXTask::ensureTxMode() {
    State state = transceiver.get_state(RF09, error);
    switch (state) {
        case RF_NOP:
            LOG_DEBUG << "[TX ENSURE] STATE: NOP";
            transceiver.set_state(RF09, RF_TRXOFF, error);
        break;
        case RF_SLEEP:
            LOG_DEBUG << "[TX ENSURE] STATE: SLEEP";
            transceiver.set_state(RF09, RF_TRXOFF, error);
        break;
        case RF_TRXOFF:
            LOG_DEBUG << "[TX ENSURE] STATE: TRXOFF";
        break;
        case RF_TX:
            LOG_DEBUG << "[TX ENSURE] STATE: TX";
            transceiver.set_state(RF09, RF_TRXOFF, error);
        break;
        case RF_RX:
            transceiver.set_state(RF09, RF_TRXOFF, error);
            break;
        case RF_TRANSITION:
            vTaskDelay(pdMS_TO_TICKS(20));
            LOG_DEBUG << "[TX ENSURE] STATE: TRANSITION";
        break;
        case RF_RESET:
            LOG_DEBUG << "[TX ENSURE] STATE: RESET";
        break;
        case RF_INVALID:
            LOG_DEBUG << "[TX ENSURE] STATE: INVALID";
            transceiver.set_state(RF09, RF_TRXOFF, error);

        break;
        case RF_TXPREP:
            transceiver.set_state(RF09, RF_TRXOFF, error);
        break;
        default:
            LOG_ERROR << "UNDEFINED";
        break;
    }
}

PacketData RF_TXTask::createRandomPacketData(uint16_t length) {
    PacketData data{};
    for (uint16_t i = 0; i < length; i++)
        data.packet[i] = i;
    data.length = length;
    return data;
}

void RF_TXTask::transmitWithWait(uint8_t* tx_buf, uint16_t length, uint16_t wait_ms_for_txfe, Error& error) {
    ensureTxMode();
    transceiver.transmitBasebandPacketsTx(RF09, tx_buf, length, error);
    for (int i = 0; i < length; i++) {
        __NOP();
        // LOG_DEBUG << "[TX DATA] " << tx_buf[i];
    }

    if (xSemaphoreTake(transceiver_handler.txfeSemaphore_tx, pdMS_TO_TICKS(wait_ms_for_txfe)) == pdTRUE) {
        txfe_counter++;
        LOG_DEBUG << "[TX] TXFE: " << txfe_counter << " [TX] LENGTH: " << length - MAGIC_NUMBER;
        LOG_DEBUG << "[TX] TXFE NOT RECEIVED: " << txfe_not_received;
        LOG_DEBUG << "[TX] RXFE: " << rxfe_received << "[TX] RXFE NOT RECEIVED: " << rxfe_not_received; ;
        transceiver.tx_ongoing = false;
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(1000));
        txfe_not_received++;
        // TODO : RESEND THE PACKET
        LOG_ERROR << "[TX READY] TXFE **NOT** RECEIVED: " << txfe_not_received;
        transceiver.set_state(RF09, RF_TRXOFF, error);
        transceiver.chip_reset(error);
        transceiver.tx_ongoing = false;
        /// TODO: RESEND
    }
}


[[noreturn]] void RF_TXTask::execute() {
    vTaskDelay(pdMS_TO_TICKS(3000));
    PacketData packetTestData = createRandomPacketData(MaxPacketLength);
    StaticTimer_t xTimerBuffer;
    TimerHandle_t xTimer = xTimerCreateStatic(
        "Transmit Timer",
        pdMS_TO_TICKS(TX_TRANSMIT),
        pdTRUE,
        (void *)1,
        [](TimerHandle_t pxTimer) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyIndexedFromISR(
                rf_txtask->taskHandle,
                NOTIFY_INDEX_TRANSMIT,
                TRANSMIT,
                eSetBits,
                &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        },
        &xTimerBuffer);

    if (xTimer != nullptr) {
        if (xTimerStart(xTimer, 0) != pdPASS) {
            LOG_ERROR << "[TX] Failed to start the timer";
        }
        else
            LOG_INFO << "[TX] TX TIMER HAS STARTED";
    }
    else
        LOG_ERROR << "[TX] null timer";
    uint8_t state = 0;
    uint8_t counter = 0;
    uint32_t receivedEventsTransmit;
    // TODO add the rest of TCs
    // Are you alive TC [17,1]
    uint8_t test_array[] = {24, 1, 192, 10, 0, 5, 47, 17, 1, 2, 5};
    size_t size_test_array = sizeof(test_array) / sizeof(test_array[0]);
    uint16_t corrected_tx_length = size_test_array + MAGIC_NUMBER;
    LOG_DEBUG << "[TX] TX LENGTH: " << corrected_tx_length;
    while (true) {
        if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_TRANSMIT, pdFALSE, pdTRUE, &receivedEventsTransmit, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(transceiver_handler.resources_mtx, portMAX_DELAY) == pdTRUE) {
                state = (transceiver.rx_ongoing << 1) | transceiver.tx_ongoing;
                xSemaphoreGive(transceiver_handler.resources_mtx);
            }
            switch (state) {
                case READY: {
                    LOG_DEBUG << "[TX] READY";
                    transmitWithWait(test_array, corrected_tx_length, 250, error);
                    rf_rxtask->ensureRxMode();
                    break;
                }
                case TX_ONG: {
                    LOG_DEBUG << "[TX] TX_ONG";
                    break;
                }
                case RX_ONG: {
                    LOG_DEBUG << "[TX] RX_ONG";
                    if (xSemaphoreTake(transceiver_handler.rxfeSemaphore_tx, pdMS_TO_TICKS(250))) {
                        rxfe_received++;
                        transmitWithWait(test_array, corrected_tx_length, 250, error);
                    }
                    else {
                        rxfe_not_received++;
                        transceiver.set_state(RF09, RF_TRXOFF, error);
                        transceiver.chip_reset(error);
                        transceiver.rx_ongoing = false;
                        // TODO: Send it again
                    }
                    rf_rxtask->ensureRxMode();
                    break;
                }
                case RX_TX_ONG: {
                    LOG_ERROR << "[TX] RXONG & TXONG";
                    break;
                }
                default: {
                    LOG_ERROR << "[TX] Unknown state!";
                    break;
                }
            }
        }
    }
}
