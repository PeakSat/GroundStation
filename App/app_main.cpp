#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "RF_RXTask.hpp"

/* App includes. */
#include "app_main.h"
#include "TransceiverTask.hpp"
#include "UARTGatekeeperTask.hpp"
#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"


uint8_t button_flag = 0;

void app_main( void )
{
//    transceiverTask.emplace();
    uartGatekeeperTask.emplace();
    rf_rxtask.emplace();
    rf_txtask.emplace();

//    transceiverTask->createTask();
    uartGatekeeperTask->createTask();
    rf_rxtask->createTask();
    rf_txtask->createTask();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not get here. */
    for(;;);
}
/*-----------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USER_BTN_Pin)
        button_flag = 1;
}

extern "C" void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(USER_BTN_Pin);
    HAL_GPIO_EXTI_IRQHandler(RF_IRQ_Pin);
    rf_rxtask->transceiver.handle_irq();
}
