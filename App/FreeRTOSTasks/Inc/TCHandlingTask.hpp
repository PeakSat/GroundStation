#pragma once
#include "Task.hpp"
#include "etl/optional.h"
#include <Message.hpp>
#include <queue.h>
#include <stm32h7xx_hal.h>
#include "GlobalVariables.hpp"

#include <TaskConfigs.hpp>


#define MAX_TC_DATA_SIZE 512
#define MIN_TC_DATA_SIZE 10


extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

inline uint8_t ECSS_TC_BUF[MAX_TC_DATA_SIZE]{};

//
struct TCUARTHandler {
    uint8_t buf[MAX_TC_DATA_SIZE]{};
    uint16_t data_size = 0;
    bool active = false;
    bool queue_full_active = false;
};

inline TCUARTHandler tc_uart_handler;
static uint8_t tc_buf_dma[MAX_TC_DATA_SIZE];
class TCHandlingTask : public Task {
public:
    // CONSTANTS
    constexpr static long ITEM_SIZE_UART = sizeof(tc_uart_handler);
    constexpr static unsigned long QUEUE_LENGTH_UART = 10;
    static constexpr uint16_t TASK_WAIT_TO_BEGIN_MS = 5000;
    static constexpr uint16_t WAIT_QUEUE_DATA_MS = 100;

    // VARIABLES
    bool rf_rx_active_  = false;
    uint16_t size_helper_ = 0;
    //
    uint32_t previousTCUARTBufferTailPointer = 0;


    // QUEUES
    QueueHandle_t QueueHandleUART_{};
    StaticQueue_t incomingTCUARTQueueBuffer{};
    uint8_t incomingTCUARTQueueStorageArea[ITEM_SIZE_UART * QUEUE_LENGTH_UART]{};

    // FREERTOS
    uint32_t received_events_ = 0;


    uint16_t startUARTOld(uint8_t* buf, uint16_t size, uint8_t retries, uint16_t delay_btw_retries_ms);
    static uint16_t startUART(uint8_t* buf, uint16_t size, uint8_t retries, uint16_t delay_btw_retries_ms);
    static const char* getHALErrorDescription(HAL_StatusTypeDef status);
    static bool resetUARTIfNeeded();
    TCHandlingTask() : Task("TC Handling Task") {}


    [[noreturn]] void execute();
    void createTask() {
        this->taskHandle = xTaskCreateStatic(vClassTask<TCHandlingTask>, this->TaskName,
                                             TCHandlingTaskStack, this, TCHandlingTaskPriority,
                                             TCHandlingTaskbuffer, &(this->taskBuffer));
        QueueHandleUART_ = xQueueCreateStatic(ITEM_SIZE_UART, QUEUE_LENGTH_UART, incomingTCUARTQueueStorageArea,
                                          &incomingTCUARTQueueBuffer);


        vQueueAddToRegistry(QueueHandleUART_, "TC UART queue");

        TCQueue = xQueueCreateStatic(TCQueueItemNum, TCItemSize, TCQueueStorageArea, &TCQueueBuffer);
        vQueueAddToRegistry(TCQueue, "TC queue");

    }

private:
};

inline etl::optional<TCHandlingTask> tcHandlingTask;