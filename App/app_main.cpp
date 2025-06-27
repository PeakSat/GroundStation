#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "RF_RXTask.hpp"

/* App includes. */
#include "app_main.h"

#include "CANGatekeeperTask.hpp"
#include "TCHandlingTask.hpp"
#include "UARTGatekeeperTask.hpp"


void app_main( void )
{

    uartGatekeeperTask.emplace();
    uartGatekeeperTask->createTask();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not get here. */
    for(;;);
}
/*-----------------------------------------------------------*/
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            /* Retrieve Rx messages from RX FIFO0 */
            if (incomingFIFO.lastItemPointer >= sizeOfIncommingFrameBuffer) {
                incomingFIFO.lastItemPointer = 0;
            }
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &newFrame.header, newFrame.Data) != HAL_OK) {
                CAN::CANError fdcanErr = mapHALFDCANErrorToFDCANError(hfdcan->ErrorCode);
                reportError(UnifiedModuleError(fdcanErr), true, 0);
            }

            newFrame.bus = hfdcan;
            IdInfo identifier = CAN::TPMessage::decodeId(newFrame.header.Identifier);

            if (identifier.destinationAddress == CAN::TTC && identifier.sourceAddress == CAN::ADCS) {
                CanPacket ADCSframe;
                for (uint32_t i = 0; i < newFrame.header.DataLength; i++) {
                    ADCSframe.canData[i] = newFrame.Data[i];
                }
                ADCSframe.canExtId = newFrame.header.Identifier;
                ADCSframe.idType = CAN_ID_TYPE_EXTENDED;
                ADCSframe.canSize = newFrame.header.DataLength;
                if (xQueueIsQueueFullFromISR(canGatekeeperTask->incomingADCSQueue)) {
                    // REPORT_ERROR_WITH_CONTEXT(TTC_ERROR_INCOMING_CAN_QUEUE_ADCS_FULL, true, 0);
                } else {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    // TODO DEFINE THE QUEUE ON THE ADCS TASK
                    xQueueSendToBackFromISR(canGatekeeperTask->incomingADCSQueue, &ADCSframe, &xHigherPriorityTaskWoken);
                    /// TODO NOTIFY ADCS TASK
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            } else if (newFrame.Data[0] == 0 && newFrame.Data[1] == 0) { /// TODO for ADCS maybe it is nominal, that´s why is on the second else if
                __NOP();
                if (newFrame.bus->Instance == FDCAN1) {
                    __NOP();
                } else if (newFrame.bus->Instance == FDCAN2) {
                    __NOP();
                }
            } else if (identifier.destinationAddress == CAN::TTC && identifier.sourceAddress == CAN::OBC) {
                if (xQueueIsQueueFullFromISR(canGatekeeperTask->incomingFrameQueue)) {
                    // REPORT_ERROR_WITH_CONTEXT(TTC_ERROR_INCOMING_CAN_QUEUE_TTC_OBC_FULL, true, 0)
                } else {
                    // Send the data to the gatekeeper
                    incomingFIFO.lastItemPointer++;
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    xQueueSendToBackFromISR(canGatekeeperTask->incomingFrameQueue, &newFrame, &xHigherPriorityTaskWoken);
                    xHigherPriorityTaskWoken = pdFALSE;
                    xTaskNotifyFromISR(canGatekeeperTask->taskHandle, CAN_GATEKEEPER, eSetBits, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
        }
    }
    // Re-activate the callback
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        /* Notification Error */
        CAN::CANError fdcanErr = mapHALFDCANErrorToFDCANError(hfdcan->ErrorCode);
        // reportError(UnifiedModuleError(fdcanErr), true, 0);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    // Declare a variable to track if a higher priority task is woken up
    BaseType_t xHigherPriorityTaskWoken;
    // Initialize xHigherPriorityTaskWoken to pdFALSE (no higher-priority task woken yet)
    xHigherPriorityTaskWoken = pdFALSE;


    // UART4
    if (huart->Instance == UART4) {
        auto currentsTCBufferTailPointer = static_cast<uint32_t>(Size);

        if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
            if (Size >= MIN_TC_DATA_SIZE && Size <= MAX_TC_DATA_SIZE) {
                // copy the data
                memcpy(tc_uart_handler.buf, huart4.pRxBuffPtr, Size);
                tc_uart_handler.data_size = Size;
                tc_uart_handler.active = true;
                xHigherPriorityTaskWoken = pdFALSE;
                auto queue_status = xQueueSendFromISR(tcHandlingTask->QueueHandleUART_, &tc_uart_handler, &xHigherPriorityTaskWoken);
                if (queue_status == errQUEUE_FULL) {
                    tc_uart_handler.queue_full_active = true;
                    xQueueSendFromISR(tcHandlingTask->QueueHandleUART_, &tc_uart_handler, &xHigherPriorityTaskWoken);
                    xTaskNotifyIndexedFromISR(tcHandlingTask->taskHandle, NOTIFY_INDEX_INCOMING_TC, TC_UART, eSetBits, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                } else {
                    xTaskNotifyIndexedFromISR(tcHandlingTask->taskHandle, NOTIFY_INDEX_INCOMING_TC, TC_UART, eSetBits, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, tc_buf_dma, sizeof(tc_buf_dma));
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == UART4) {
        auto error = huart->ErrorCode;
        HAL_UART_DMAStop(&huart4);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, tc_buf_dma, sizeof(tc_buf_dma));
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    }
}

