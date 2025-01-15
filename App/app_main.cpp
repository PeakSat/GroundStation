#include "FreeRTOS.h"
#include "task.h"
#include "main.h"


/* App includes. */
#include "app_main.h"
#include "UARTGatekeeperTask.hpp"
#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"


uint8_t button_flag = 0;

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

//    transceiverTask.emplace();
    rf_rxtask.emplace();
    rf_txtask.emplace();
    uartGatekeeperTask.emplace();

//    transceiverTask->createTask();
    uartGatekeeperTask->createTask();
    rf_rxtask->createTask();
    rf_txtask->createTask();
    TransceiverHandler::initialize_semaphore();


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
    transceiver.handle_irq();
}
