#include "Logger.hpp"
#include <RF_TXTask.hpp>
#include "TC_HandlingTask.hpp"

void TC_HandlingTask::execute() {
    LOG_DEBUG << "TC handling::execute()";
    // vTaskDelay(5000);
    uint32_t ulNotifiedValue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
while (true) {

    xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
    if (UART_RxMessage_size) {
        for (uint8_t i = 0; i < UART_RxMessage_size; i++) {
            LOG_DEBUG << static_cast<char>(UART_Rx_buffer[i]);
        }
        xTaskNotifyIndexedFromISR(
                rf_txtask->taskHandle,
                NOTIFY_INDEX_TRANSMIT,
                TRANSMIT,
                eSetBits,
                &xHigherPriorityTaskWoken);
    }
    LOG_DEBUG << "running";

// vTaskDelay(pdMS_TO_TICKS(1000));
    }
}