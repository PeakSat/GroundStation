#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "RF_RXTask.hpp"

/* App includes. */
#include "app_main.h"
#include "UARTGatekeeperTask.hpp"
#include "RF_TXTask.hpp"
#include "RF_RXTask.hpp"


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

    uartGatekeeperTask->createTask();
    rf_rxtask->createTask();
    rf_txtask->createTask();


    transceiver_handler.initialize_semaphore();

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