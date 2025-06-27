#pragma once
#include "ApplicationLayer.hpp"
#include "Task.hpp"
#include <optional>
#include "TaskConfigs.hpp"

struct localPacketHandler;

class CANParserTask : public Task {
public:
    static constexpr uint16_t TASK_WAIT_TO_BEGIN_MS = 500;
    static constexpr uint16_t WAIT_FOR_NOTIFICATION_MS = 1000;
    static constexpr uint16_t MS_WAIT_FOR_QUEUE_FULL = 500;
    void execute();
    static uint16_t sendACK(CAN::Application::MessageIDs ID);
    static uint16_t handlePacket(const localPacketHandler& CANPacketHandler, uint16_t ms_to_wait_if_queue_is_full, uint8_t retries);
    CANParserTask() : Task("CAN Parser") {}
    void createTask() {
        this->taskHandle = xTaskCreateStatic(vClassTask<CANParserTask>, this->TaskName, CANParserTaskStack, this,
                                             2, CANParserTaskbuffer, &(this->taskBuffer));
    }

private:
    uint32_t received_events_ = 0;
};

inline std::optional<CANParserTask> canParserTask;