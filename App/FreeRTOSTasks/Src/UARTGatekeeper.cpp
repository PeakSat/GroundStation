#pragma once
#include "GlobalVariables.hpp"
#include "UARTGatekeeperTask.hpp"
#include "stm32h7xx_hal.h"
#include "TaskConfigs.hpp"

extern UART_HandleTypeDef huart3;

UARTGatekeeperTask::UARTGatekeeperTask() : Task("UARTGatekeeperTask") {
    xUartQueue = xQueueCreateStatic(UARTQueueSize, sizeof(etl::string<LOGGER_MAX_MESSAGE_SIZE>), this->ucQueueStorageArea, &(this->xStaticQueue));
}

 void UARTGatekeeperTask::execute() {
    UART_Gatekeeper_Semaphore = xSemaphoreCreateBinaryStatic(&UART_Gatekeeper_SemaphoreBuffer);
    etl::string<LOGGER_MAX_MESSAGE_SIZE> output;
    xSemaphoreGive(UART_Gatekeeper_Semaphore);
    uint8_t buffer[LOGGER_MAX_MESSAGE_SIZE];
    while (true) {
        uint32_t received_events = 0;
        if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_UART_GATEKEEPER, pdFALSE, 0xFFFFFFFF, &received_events, portMAX_DELAY) == pdTRUE) {
                while (uxQueueMessagesWaiting(xUartQueue)) {
                    xQueueReceive(xUartQueue, &output, 0);
                    if (xSemaphoreTake(UART_Gatekeeper_Semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                        output.repair();
                        memcpy(buffer, output.data(), output.size());
                        uint16_t local_size =  output.size();
                        bool custom_termination = false;
                        if (custom_termination) {
                            uint8_t fixed_end[2] = {0xAA, 0xBB};
                            memcpy(buffer + local_size, fixed_end, sizeof(fixed_end));
                            local_size = local_size + 2;
                        }

                        auto status = HAL_UART_Transmit_DMA(&huart3, buffer, local_size);
                        if (status != HAL_OK) {
                            // DMA failed — try reset and retry once
                            __HAL_RCC_UART4_FORCE_RESET();
                            __HAL_RCC_UART4_RELEASE_RESET();
                            HAL_UART_DeInit(&huart3);
                            HAL_UART_Init(&huart3);

                            // Retry
                            status = HAL_UART_Transmit_DMA(&huart3, buffer, output.size());

                            if (status != HAL_OK) {
                                // Still failed — give up and release semaphore to avoid deadlock
                                xSemaphoreGive(UART_Gatekeeper_Semaphore);
                            }
                        }
                    } else {
                        // Semaphore take timed out — assume DMA got stuck or TXC callback missed
                        // Reset UART and retry DMA
                        // Retry DMA (skip blocking fallback)
                        auto status = HAL_UART_Transmit_DMA(&huart3, buffer, output.size());
                        if (status != HAL_OK) {
                            // Fallback failed — log error and continue
                        }
                        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
                    }
                }
        }
        else {
            // FDIR
        }
    }

}
