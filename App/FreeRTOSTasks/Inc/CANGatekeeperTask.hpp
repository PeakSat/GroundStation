#pragma once
#include "Task.hpp"
#include "queue.h"
#include "CANDriver.hpp"
#include <optional>
#include "semphr.h"
#include "TaskConfigs.hpp"


extern bool expectingACK;
extern bool ACKReceived;
extern localPacketHandler* responsePointer;


class CANHandler {
public:
    struct MutexWrapper {
        SemaphoreHandle_t handle;
        StaticSemaphore_t buffer;
        const char* name;
    };

    struct BinarySemaphoreWrapper {
        SemaphoreHandle_t handle;
        StaticSemaphore_t buffer;
        const char* name;
    };

private:
    MutexWrapper transmit_mutex_{
        nullptr,
        {},
        "CAN Transmit Mutex"};
    BinarySemaphoreWrapper ack_semaphore_{
        nullptr,
        {},
        "CAN ACK Semaphore"};
    //
    uint32_t transmit_timeout_ms_ = 10000; // above 8s
    uint32_t ack_response_timeout_ms_ = 1000;

    bool initialized_ = false;

public:
    bool initialize() {
        if (initialized_) return true;

        bool success = true;
        success &= createMutex(transmit_mutex_);
        success &= createBinarySemaphore(ack_semaphore_);

        initialized_ = success;
        return success;
    }

    // Accessors
    SemaphoreHandle_t getTransmitMutex() const { return transmit_mutex_.handle; }
    SemaphoreHandle_t getACKSemaphore() const { return ack_semaphore_.handle; }
    uint32_t getTransmitTimeout() { return transmit_timeout_ms_; }
    uint32_t getACKResponseTimeout() { return ack_response_timeout_ms_; }
    void setTransmitTimeout(uint32_t timeout) { transmit_timeout_ms_ = timeout; }
    void setACKResponseTimeout(uint32_t timeout) { ack_response_timeout_ms_ = timeout; }

private:
    bool createMutex(MutexWrapper& wrapper) {
        wrapper.handle = xSemaphoreCreateMutexStatic(&wrapper.buffer);
        if (wrapper.handle == nullptr) {
            return false;
        }
        return true;
    }

    bool createBinarySemaphore(BinarySemaphoreWrapper& wrapper) {
        wrapper.handle = xSemaphoreCreateBinaryStatic(&wrapper.buffer);
        if (wrapper.handle == nullptr) {
            return false;
        }
        return true;
    }
};

// Global inline instance
inline CANHandler canHandler;

/**
* Every variable needed to control the incoming frames' fifo buffer
* will be stored in this struct.
 */
struct incomingFIFO {
    uint8_t* buffer;
    uint32_t NOfItems;
    uint32_t lastItemPointer;
    incomingFIFO() : buffer(nullptr), NOfItems(0), lastItemPointer(0) {}
    incomingFIFO(uint8_t* externalBuffer, uint32_t NOfItems) : buffer(externalBuffer), NOfItems(NOfItems), lastItemPointer(0) {}
};

extern incomingFIFO incomingFIFO;

struct FrameMetadata {
    uint8_t frame_type;
    uint8_t payload_length;
};

//
static inline uint8_t incomingFrameQueueStorageArea[sizeOfIncommingFrameBuffer * sizeof(CAN::Frame)] __attribute__((section(".dtcmram_data_incomingFrameQueueStorageArea")));
static const uint8_t PacketQueueSize = 40;
static inline uint8_t outgoingQueueStorageArea[PacketQueueSize * sizeof(CAN::Packet)] __attribute__((section(".dtcmram_data_outgoingQueueStorageArea")));

static const uint8_t incomingPacketQueueSize = 3;
static inline uint8_t incomingPacketQueueStorageArea[incomingPacketQueueSize * sizeof(localPacketHandler)] __attribute__((section(".dtcmram_data_incomingPacketQueueStorageArea")));

inline uint8_t incomingBuffer[CANMessageSize * sizeOfIncommingFrameBuffer];

inline QueueHandle_t incomingPacketQueue;
inline QueueHandle_t outgoingQueue;
inline CAN::Frame newFrame;

class CANGatekeeperTask : public virtual Task {
public:
    // CONSTANTS
    static constexpr uint16_t TASK_WAIT_TO_BEGIN_MS = 1000;
    static constexpr uint16_t WAIT_FOR_NOTIFICATION_MS = 1000;
    static constexpr uint16_t MS_WAIT_FOR_QUEUE_FULL = 100;
    /**
     * FreeRTOS queues
     */

    QueueHandle_t outgoingADCSQueue;
    QueueHandle_t incomingADCSQueue;
    QueueHandle_t incomingFrameQueue;
    /**
     * The variables used to hold the queue's data structure.
     */
    static inline StaticQueue_t outgoingQueueBuffer;
    static inline StaticQueue_t outgoingADCSQueueBuffer;
    static inline StaticQueue_t incomingADCSQueueBuffer;
    static inline StaticQueue_t incomingPacketQueueBuffer;
    static inline StaticQueue_t incomingFrameQueueBuffer;

    CAN::ActiveBus ActiveBus = CAN::ActiveBus::Main;

    CANGatekeeperTask() : Task("CANGatekeeperTask") {
        canHandler.initialize();
        CAN::initialize(0);
        incomingFIFO.buffer = incomingBuffer;
        incomingFIFO.NOfItems = sizeOfIncommingFrameBuffer;

        outgoingQueue = xQueueCreateStatic(PacketQueueSize, sizeof(CAN::Packet), outgoingQueueStorageArea,
                                           &outgoingQueueBuffer);
        vQueueAddToRegistry(outgoingQueue, "CAN Outgoing");

        incomingFrameQueue = xQueueCreateStatic(sizeOfIncommingFrameBuffer, sizeof(CAN::Frame), incomingFrameQueueStorageArea,
                                                &incomingFrameQueueBuffer);
        vQueueAddToRegistry(incomingFrameQueue, "CAN Incoming Frame");

        incomingPacketQueue = xQueueCreateStatic(incomingPacketQueueSize, sizeof(localPacketHandler), incomingPacketQueueStorageArea,
                                                 &incomingPacketQueueBuffer);
        vQueueAddToRegistry(incomingPacketQueue, "CAN Incoming Packet");
    }

    static uint16_t send(const CAN::Packet& message, uint16_t wait_for_queue_full_ms = 5000) {

        auto status = xQueueSendToBack(outgoingQueue, &message, pdMS_TO_TICKS(wait_for_queue_full_ms));
        if (status == errQUEUE_FULL) {
            return 0;
        }
        return 1;
    }
    static uint16_t processFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler);
    static uint16_t processCompletedMessage(localPacketHandler& packetHandler, uint16_t ms_to_wait_if_queue_is_full);
    static uint16_t handleFinalFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler, uint16_t ms_to_wait_if_queue_is_full);
    static uint16_t handleSingleFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler);
    static void handleFirstFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler, uint8_t payloadLength);
    static uint16_t handleConsecutiveFrame(const CAN::Frame& in_frame_handler, localPacketHandler& packetHandler);

    void switchActiveBus(CAN::ActiveBus activeBus) {
        this->ActiveBus = activeBus;
    }

    void printActiveBus() const;

    void execute();

    void createTask() {
        this->taskHandle = xTaskCreateStatic(vClassTask<CANGatekeeperTask>, this->TaskName, CANGatekeeperTaskStack, this,
                                             CANGatekeeperTaskPriority, this->taskStack, &(this->taskBuffer));
    }

private:
    StackType_t taskStack[CANGatekeeperTaskStack]{};
    uint32_t receive_events_ = 0;
};

inline std::optional<CANGatekeeperTask> canGatekeeperTask;