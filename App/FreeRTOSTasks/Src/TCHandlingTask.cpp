#include "TCHandlingTask.hpp"
#include "RF_TXTask.hpp"
#include "TPMessage.hpp"
#include "TPProtocol.hpp"
#include "ApplicationLayer.hpp"


uint16_t TCHandlingTask::startUARTOld(uint8_t* buf, uint16_t size, uint8_t retries, uint16_t delay_btw_retries_ms) {
    uint16_t spacecraft_error_code = 1;

    if (buf == nullptr) {
        spacecraft_error_code = 0;
        return spacecraft_error_code;
    }

    int attempt = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    while (attempt < retries) {
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, size);
        if (status == HAL_OK) {
            // Disable unnecessary DMA interrupts
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_TC);
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
        if (huart3.RxState != HAL_UART_STATE_READY) {
            LOG_WARNING << "[TC HANDLING] UART not ready, state: " << huart3.RxState << ", attempt: " << attempt + 1;

            // Try to abort any ongoing reception
            HAL_UART_AbortReceive(&huart3);

            // Give some time for cleanup
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Check DMA state
        if (hdma_usart3_rx.State != HAL_DMA_STATE_READY) {
            LOG_WARNING << "[TC HANDLING] DMA not ready, state: " << hdma_usart3_rx.State << ", attempt: " << attempt + 1;

            // Try to abort DMA
            HAL_DMA_Abort(&hdma_usart3_rx);

            // Brief delay for DMA cleanup
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Attempt to start DMA reception
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, size);
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
        if (status == HAL_OK) {
            // Success - configure DMA interrupts as needed
            // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
            // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
            spacecraft_error_code = 1;
            // LOG_DEBUG << "[TC HANDLING] UART DMA started successfully on attempt " << attempt + 1;
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
        HAL_UART_AbortReceive(&huart3);
        HAL_DMA_Abort(&hdma_usart3_rx);
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
    if (huart3.ErrorCode != HAL_UART_ERROR_NONE) {
        LOG_WARNING << "UART error detected: " << huart3.ErrorCode << ", attempting reset";

        // Clear error flags
        huart3.ErrorCode = HAL_UART_ERROR_NONE;

        // Reset UART peripheral
        __HAL_UART_DISABLE(&huart3);
        vTaskDelay(pdMS_TO_TICKS(5));
        __HAL_UART_ENABLE(&huart3);

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
                    LOG_INFO << "****[UART]****" << tc_uart_handler.data_size;
                    uint16_t size = tc_uart_handler.data_size;
                    CAN::TPMessage response = {{0, 0, CAN::NodeID, CAN::TTC, false}};
                    response.appendUint8(CAN::Application::LogMessage);

                    for (uint32_t i = 0; i < size; i++) {
                        response.appendUint8(tc_uart_handler.buf[i]);
                    }
                    Message default_message{};
                    auto status = CAN::TPProtocol::createCANTPMessage(response, nullptr, default_message, 1);
                    if (status != 1) {
                        LOG_ERROR << "****[UART]****";
                    }
                }
            }

        }
    }
}
