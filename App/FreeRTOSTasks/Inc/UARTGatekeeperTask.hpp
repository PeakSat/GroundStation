#pragma once

#include "Task.hpp"
#include "queue.h"
#include "etl/string.h"
#include "etl/optional.h"
#define UARTQueueSize 20
#define LOGGER_MAX_MESSAGE_SIZE 1024

/**
 * Contains functionality of a Gatekeeper Task for the UART resource. It has the sole access to UART, to avoid any
 * deadlocks that might be caused by simultaneous requests of access to the same resource. It works by having anyone
 * needing to access UART, send the data in a queue. Then, this task receives queue elements and sends them to UART.
 */


class UARTGatekeeperTask : public Task {
private:
    QueueHandle_t xUartQueue;

    StaticQueue_t xStaticQueue;

    uint8_t ucQueueStorageArea[UARTQueueSize * sizeof(etl::string<LOGGER_MAX_MESSAGE_SIZE>)];

    const static uint16_t TaskStackDepth = 15000;

    StackType_t taskStack[TaskStackDepth];

public:
    void execute();

    UARTGatekeeperTask();

    /**
     * Adds an etl::string to the UART Gatekeeper's queue.
     *
     * This function was added as an extra abstraction layer to house the `xQueueSendToBack` function.
     * It can be used from anywhere in the code to get access to the UART queue/UART Gatekeeper task, without having to
     * know the low level details of the queue.
     *
     * If the queue is full, the string is not added to the queue and is lost.
     * @param message the etl::string to be added in the queue of the UART Gatekeeper task.
     */
    void addToQueue(const etl::string<LOGGER_MAX_MESSAGE_SIZE>& message) {
        xQueueSendToBack(xUartQueue, &message, 0);
    }

    void createTask() {
        this->taskHandle = xTaskCreateStatic(vClassTask<UARTGatekeeperTask>, this->TaskName, UARTGatekeeperTask::TaskStackDepth, this,
                                             tskIDLE_PRIORITY + 3, this->taskStack, &(this->taskBuffer));
    }
};

inline etl::optional<UARTGatekeeperTask> uartGatekeeperTask;
