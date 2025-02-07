#pragma once
#include "Task.hpp"
#include "task.h"
#include "at86rf215.hpp"
#include "etl/array.h"
#include "etl/optional.h"

inline uint8_t UART_Rx_buffer[1024];
inline uint16_t UART_RxMessage_size;

class TC_HandlingTask : public Task {
public:
    TC_HandlingTask() : Task("TC Handling Task") {}
    void execute();
    void createTask() {
        this->taskHandle = xTaskCreateStatic(vClassTask<TC_HandlingTask>, this->TaskName,
                                             this->TaskStackDepth, this, tskIDLE_PRIORITY + 1,
                                             this->taskStack, &(this->taskBuffer));
    }
private:
    constexpr static uint16_t TaskStackDepth = 5000;
    /// Frequency in kHz
    Error error = NO_ERRORS;
    StackType_t taskStack[TaskStackDepth]{};
};

inline etl::optional<TC_HandlingTask> tc_handlingtask;