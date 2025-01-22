#include "UARTGatekeeperTask.hpp"
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart3;

UARTGatekeeperTask::UARTGatekeeperTask() : Task("UARTGatekeeperTask"), taskStack{} {
    xUartQueue = xQueueCreateStatic(UARTQueueSize, sizeof(etl::string<LOGGER_MAX_MESSAGE_SIZE>),
                                    this->ucQueueStorageArea, &(this->xStaticQueue));
}

void UARTGatekeeperTask::execute() {
    etl::string<LOGGER_MAX_MESSAGE_SIZE> output;
    while (true) {
        xQueueReceive(this->xUartQueue, &output, portMAX_DELAY);
        auto status = HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t*>(output.data()), output.size(), 5000);
    }
}
