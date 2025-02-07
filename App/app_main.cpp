#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* App includes. */
#include "app_main.h"
#include "UARTGatekeeperTask.hpp"
#include <optional>
#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"
#include "at86rf215.hpp"
#include "TC_HandlingTask.hpp"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern SemaphoreHandle_t UART_Gatekeeper_Semaphore;

void app_main( void )
{

    transceiver.setGeneralConfig(GeneralConfiguration::DefaultGeneralConfig());
    transceiver.setRXConfig(RXConfig::DefaultRXConfig());
    transceiver.setTXConfig(TXConfig::DefaultTXConfig());
    transceiver.setBaseBandCoreConfig(BasebandCoreConfig::DefaultBasebandCoreConfig());
    transceiver.setFrequencySynthesizerConfig(FrequencySynthesizer::DefaultFrequencySynthesizerConfig());
    transceiver.setExternalFrontEndControlConfig(ExternalFrontEndConfig::DefaultExternalFrontEndConfig());
    transceiver.setInterruptConfig(InterruptsConfig::DefaultInterruptsConfig());
    transceiver.setRadioInterruptConfig(RadioInterruptsConfig::DefaultRadioInterruptsConfig());
    transceiver.setIQInterfaceConfig(IQInterfaceConfig::DefaultIQInterfaceConfig());

    uartGatekeeperTask.emplace();
    rf_rxtask.emplace();
    rf_txtask.emplace();
    tc_handlingtask.emplace();

    uartGatekeeperTask->createTask();
    // rf_rxtask->createTask();
    // rf_txtask->createTask();
    tc_handlingtask->createTask();


    transceiver_handler.initialize_semaphore();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,UART_Rx_buffer,1024);
    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not get here. */
    for(;;);
}
/*-----------------------------------------------------------*/

extern "C" void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(RF_IRQ_Pin);
    transceiver.handle_irq();
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    // Declare a variable to track if a higher priority task is woken up
    BaseType_t xHigherPriorityTaskWoken;
    // Initialize xHigherPriorityTaskWoken to pdFALSE (no higher-priority task woken yet)
    xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART3) {
        xTaskNotifyFromISR(tc_handlingtask->taskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
        UART_RxMessage_size = Size;
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART_Rx_buffer, 1024) != HAL_OK) {
        // Handle the error (e.g., reset the UART or log the error)
        __NOP();
    }
    // disabling the half buffer interrupt //
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    //  disabling the full buffer interrupt //
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_TC);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        xSemaphoreGiveFromISR(UART_Gatekeeper_Semaphore);
    }
}