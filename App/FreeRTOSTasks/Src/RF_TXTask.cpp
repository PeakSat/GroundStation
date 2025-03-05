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
    uint32_t crc_value;
    uint8_t local_tx_buf[length];
    for (int i = 0; i < length; i++) {
        local_tx_buf[i] = tx_buf[i];
    }
    crc_value = HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t*>(local_tx_buf), length);
    LOG_DEBUG << "[TX]: CRC TRANSMIT: " << crc_value;
    local_tx_buf[length] = static_cast<uint8_t>(crc_value & 0xFF);          // 0x78
    local_tx_buf[length + 1] = static_cast<uint8_t>((crc_value >> 8)  & 0xFF);  // 0x56
    local_tx_buf[length + 2] = static_cast<uint8_t>((crc_value >> 16) & 0xFF);  // 0x34
    local_tx_buf[length + 3] = static_cast<uint8_t>((crc_value >> 24) & 0xFF);  // 0x12
    uint16_t length_with_crc = length + 4;
    for (int i = 0 ; i < length_with_crc; i++) {
        LOG_DEBUG << local_tx_buf[i];
    }
    transceiver.transmitBasebandPacketsTx(RF09, local_tx_buf, length_with_crc  + MAGIC_NUMBER, error);
    if (xSemaphoreTake(transceiver_handler.txfeSemaphore_tx, pdMS_TO_TICKS(wait_ms_for_txfe)) == pdTRUE) {
        txfe_counter++;
        LOG_DEBUG << "[TX] TXFE: " << txfe_counter << " [TX] LENGTH: " << length_with_crc;
        LOG_DEBUG << "[TX] TXFE NOT RECEIVED: " << txfe_not_received;
        LOG_DEBUG << "[TX] RXFE: " << rxfe_received << "[TX] RXFE NOT RECEIVED: " << rxfe_not_received; ;
        transceiver.tx_ongoing = false;
        // for (int i = 0; i < length_with_crc; i++) {
        //     __NOP();
        //     LOG_DEBUG << "[TX]: " << tx_buf[i];
        // }
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
    uint32_t receivedEventsTransmit;
    // TODO add the rest of TCs
    // Are you alive TC [17,1]
    uint8_t test_array_are_you_alive[11] = {24, 1, 192, 10, 0, 5, 47, 17, 1, 0, 5};
    uint8_t test_array_one_shot[12] = {24, 1, 192, 10, 0, 5, 47, 3, 27, 3, 5, 1};
    // uint8_t test_array_one_shot[] = {24, 1, 192, 10, 0, 5, 47, 17, 1, 0, 5};

    size_t size_test_array_are_you_alive = sizeof(test_array_are_you_alive) / sizeof(test_array_are_you_alive[0]);
    size_t size_test_array_one_shot = sizeof(test_array_one_shot) / sizeof(test_array_one_shot[0]);
    uint16_t corrected_tx_length_are_you_alive = 11;
    uint16_t corrected_tx_length_one_shot = 12;
    uint32_t switch_counter = 0;
    while (true) {
        if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_TRANSMIT, pdFALSE, pdTRUE, &receivedEventsTransmit, portMAX_DELAY) == pdTRUE) {
            switch_counter++;
            if (xSemaphoreTake(transceiver_handler.resources_mtx, portMAX_DELAY) == pdTRUE) {
                state = (transceiver.rx_ongoing << 1) | transceiver.tx_ongoing;
                switch (state) {
                    case READY: {
                        LOG_DEBUG << "[TX] READY";
                        if (switch_counter % 2 == 0) {
                            LOG_DEBUG << "[TX] READY: sending TC[3,27]...";
                            uint8_t* buff_pointer = test_array_one_shot;
                            transmitWithWait(buff_pointer, size_test_array_one_shot, 250, error);
                        }
                        else {
                            LOG_DEBUG << "[TX] READY: sending TC[17,1] to OBC...";
                            uint8_t* buff_pointer = test_array_are_you_alive;
                            transmitWithWait(buff_pointer, size_test_array_are_you_alive, 250, error);
                        }
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
                            if (switch_counter % 2 == 0) {
                                LOG_DEBUG << "[TX] READY: sending TC[3,27]...";
                                transmitWithWait(test_array_one_shot, corrected_tx_length_one_shot, 250, error);
                            }
                            else {
                                LOG_DEBUG << "[TX] READY: sending TC[17,1] to OBC...";
                                transmitWithWait(test_array_are_you_alive, corrected_tx_length_are_you_alive, 250, error);
                            }
                            transceiver.rx_ongoing = false;
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
                xSemaphoreGive(transceiver_handler.resources_mtx);
            }
        }
    }
}
