#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "RF_RXTask.hpp"

/* App includes. */
#include "app_main.h"
#include "UARTGatekeeperTask.hpp"
#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"
#include "at86rf215.hpp"



void app_main( void )
{

    uartGatekeeperTask.emplace();
    rf_rxtask.emplace();
    rf_txtask.emplace();

    uartGatekeeperTask->createTask();
    rf_rxtask->createTask();
    rf_txtask->createTask();
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


    transceiver_handler.initialize_semaphore();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not get here. */
    for(;;);
}
/*-----------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_GPIO_EXTI_IRQHandler(RF_IRQ_Pin);
    transceiver.handle_irq();
}

extern "C" void EXTI15_10_IRQHandler(void) {
    transceiver.handle_irq();
}

extern "C" [[maybe_unused]] void EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(RF_IRQ_Pin);
    transceiver.handle_irq();
}