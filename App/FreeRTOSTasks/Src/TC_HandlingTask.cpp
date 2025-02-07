#include "Logger.hpp"
#include <RF_TXTask.hpp>
#include "TC_HandlingTask.hpp"

void TC_HandlingTask::execute() {
    LOG_DEBUG << "TC handling::execute()";
    // vTaskDelay(5000);
    uint32_t ulNotifiedValue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
while (true) {

    xTaskNotifyWait(0, 0, &ulNotifiedValue, 2000);
    if (UART_RxMessage_size) {
if (true) {

}
        UART_RxMessage_size=0;
    }
    LOG_DEBUG << "New TM[3,25] message! 8 1 192 0 0 56 32 3 25 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 66 149 75 77 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1 0 0 0 0 255 20 ";

// vTaskDelay(pdMS_TO_TICKS(1000));
    }
}