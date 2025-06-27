#include "TCHandlingTask.hpp"
#include "RF_TXTask.hpp"
#include "etl/message.h"
#include <ApplicationLayer.hpp>
#include <TaskConfigs.hpp>

uint16_t TCHandlingTask::startUARTOld(uint8_t* buf, uint16_t size, uint8_t retries, uint16_t delay_btw_retries_ms) {
    uint16_t spacecraft_error_code = 1;

    if (buf == nullptr) {
        spacecraft_error_code = 0;
        return spacecraft_error_code;
    }

    int attempt = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    while (attempt < retries) {
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, buf, size);
        if (status == HAL_OK) {
            // Disable unnecessary DMA interrupts
            __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
            spacecraft_error_code = 1;
            break;
        }
        // Optional: Insert delay between retries if needed
        vTaskDelay(pdMS_TO_TICKS(delay_btw_retries_ms));
        ++attempt;
        // Log retry attempt
    }

    if (status != HAL_OK) {
        spacecraft_error_code = 0;
        LOG_ERROR << "UART DMA receive failed after " << retries << " attempts";
    }

    return spacecraft_error_code;
}

uint16_t TCHandlingTask::startUART(uint8_t* buf, uint16_t size, uint8_t retries, uint16_t delay_btw_retries_ms) {
    uint16_t spacecraft_error_code = 1;

    if (buf == nullptr || size == 0) {
        LOG_ERROR << "[TC HANDLING] Invalid parameters: buf=" << (void*)buf << ", size=" << size;
        return 0;
    }

    int attempt = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    while (attempt < retries) {
        // Check UART state before attempting DMA operation
        if (huart4.RxState != HAL_UART_STATE_READY) {
            LOG_WARNING << "[TC HANDLING] UART not ready, state: " << huart4.RxState << ", attempt: " << attempt + 1;

            // Try to abort any ongoing reception
            HAL_UART_AbortReceive(&huart4);

            // Give some time for cleanup
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Check DMA state
        if (hdma_uart4_rx.State != HAL_DMA_STATE_READY) {
            LOG_WARNING << "[TC HANDLING] DMA not ready, state: " << hdma_uart4_rx.State << ", attempt: " << attempt + 1;

            // Try to abort DMA
            HAL_DMA_Abort(&hdma_uart4_rx);

            // Brief delay for DMA cleanup
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Attempt to start DMA reception
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, buf, size);
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
        if (status == HAL_OK) {
            // Success - configure DMA interrupts as needed
            // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
            // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
            spacecraft_error_code = 1;
            LOG_DEBUG << "[TC HANDLING] UART DMA started successfully on attempt " << attempt + 1;
            break;
        }
        // Log specific error
        const char* error_desc = getHALErrorDescription(status);
        LOG_WARNING << "[TC HANDLING] UART DMA start failed: " << error_desc << " (status=" << status << "), attempt " << attempt + 1 << "/" << retries;

        ++attempt;

        // Delay between retries (skip delay on last failed attempt)
        if (attempt < retries) {
            vTaskDelay(pdMS_TO_TICKS(delay_btw_retries_ms));
        }
    }

    if (status != HAL_OK) {
        spacecraft_error_code = 0;
        LOG_ERROR << "[TC HANDLING] UART DMA receive failed after " << retries << " attempts. Final status: " << status;

        // Additional cleanup on final failure
        HAL_UART_AbortReceive(&huart4);
        HAL_DMA_Abort(&hdma_uart4_rx);
    }

    return spacecraft_error_code;
}

// Helper function to get readable error descriptions
const char* TCHandlingTask::getHALErrorDescription(HAL_StatusTypeDef status) {
    switch(status) {
        case HAL_OK:      return "OK";
        case HAL_ERROR:   return "ERROR";
        case HAL_BUSY:    return "BUSY";
        case HAL_TIMEOUT: return "TIMEOUT";
        default:          return "UNKNOWN";
    }
}

// Additional helper function to check and reset UART if needed
bool TCHandlingTask::resetUARTIfNeeded() {
    // Check if UART is in error state
    if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
        LOG_WARNING << "UART error detected: " << huart4.ErrorCode << ", attempting reset";

        // Clear error flags
        huart4.ErrorCode = HAL_UART_ERROR_NONE;

        // Reset UART peripheral
        __HAL_UART_DISABLE(&huart4);
        vTaskDelay(pdMS_TO_TICKS(5));
        __HAL_UART_ENABLE(&huart4);

        // Brief stabilization delay
        vTaskDelay(pdMS_TO_TICKS(5));

        return true;
    }
    return false;
}


[[noreturn]] void TCHandlingTask::execute() {


    auto uart_status = startUART(tc_buf_dma, MAX_TC_DATA_SIZE, 3, 200);

    while (true) {
        uint32_t received_events = 0;
        if (xTaskNotifyWaitIndexed(10, pdFALSE, 0xFFFFFFFF, &received_events, portMAX_DELAY) == pdTRUE) {



                while (uxQueueMessagesWaiting(QueueHandleUART_)) {
                    if (xQueueReceive(QueueHandleUART_, &tc_uart_handler, 0) == pdTRUE) {
                        // LOG_INFO << "****[TC HANDLING] FROM UART*****, data size: " << tc_uart_handler.data_size;
                        Message tc_message{};

                                }
                            }

                    }
                }
            }
}
