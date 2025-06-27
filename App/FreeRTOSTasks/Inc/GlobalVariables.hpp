#pragma once

#include <FreeRTOS.h>
#include <queue.h>
#include "etl/memory.h"


#define OBC_APPLICATION_ID 1
#define TTC_APPLICATION_ID 2
#define TX_RX_BUF_SIZE_BYTES 2048

#define MAX_PARAMETERS 128
#define NUM_OF_TMP_PARAMS 3
#define NUM_OF_PWR_PARAMS 3

#define EPS_BUF_SIZE 500
#define MAX_CW_CHARS 50



#define REPORT_ERROR_WITH_CONTEXT(code, isr, ms_to_wait_if_queue_is_full)        \
do {                                                                         \
char context_buffer[128];                                                \
buildErrorContext(context_buffer, sizeof(context_buffer), __FILE__, __LINE__, __func__); \
reportError(code, isr, ms_to_wait_if_queue_is_full, context_buffer);     \
} while (0)

struct PacketHandler {
    uint8_t buf[TX_RX_BUF_SIZE_BYTES]; // TODO: Make this equal to ECSSMaxMessageSize if applicable
    int16_t data_length;

    // Constructor
    PacketHandler(const uint8_t* init_data = nullptr, int16_t length = 0) : data_length(length) {
        if (init_data && length <= TX_RX_BUF_SIZE_BYTES) {
            memcpy(buf, init_data, length);
        } else {
            memset(buf, 0, sizeof(buf));
            data_length = 0;
        }
    }
};

struct TimersHandler {
    uint32_t timer;
    uint32_t period_ms;

    TimersHandler() {
        timer = 0;
        period_ms = 0;
    }
};



struct PacketHandlerCW {
    uint16_t sequence_length;
    bool active;
    char sequence_buf[MAX_CW_CHARS];

    // Constructor
    PacketHandlerCW(const char* init_data = nullptr, int16_t length = 0)
        : sequence_length(0), active(false) {
        if (init_data && length > 0 && length <= MAX_CW_CHARS) {
            memcpy(sequence_buf, init_data, length);
            sequence_length = length;
        } else {
            memset(sequence_buf, 0, sizeof(sequence_buf));
        }
    }
};

namespace InternalFunctionManagement {
    enum functionID : uint16_t {
        // Original functions
        // TODO ADD MAYBE REAL FUNCTIONS (LIKE FDIR FUNCTIONS)

        // GNSS Task States (5-8)
        GNSS_DISABLED = 5,
        GNSS_NOMINAL = 6,
        GNSS_SAFE = 7,
        GNSS_SCIENCE = 8,

        // RF_RX Task States (9-11)
        RF_RX_DISABLED = 9,
        RF_RX_NOMINAL = 10,
        RF_RX_SAFE = 11,

        // RF_TX Task States (12-14)
        RF_TX_DISABLED = 12,
        RF_TX_NOMINAL = 13,
        RF_TX_SAFE = 14,

        // RF_CW Task States (15-17)
        RF_CW_DISABLED = 15,
        RF_CW_NOMINAL = 16,
        RF_CW_SAFE = 17,

        // RF_ISR Task States (18-20)
        RF_ISR_NOMINAL = 18,
        RF_ISR_SAFE = 19,
        RF_ISR_DISABLED = 20,

        // CAN_GATEKEEPER Task States (21-23)
        CAN_GATEKEEPER_DISABLED = 21,
        CAN_GATEKEEPER_NOMINAL = 22,
        CAN_GATEKEEPER_SAFE = 23,

        // TM Task States (24-26)
        TM_DISABLED = 24,
        TM_NOMINAL = 25,


        MOD_BEACON_DISABLED = 26,
        MOD_BEACON_NOMINAL = 27,


        TIMEKEEPING_DISABLED = 28,
        TIMEKEEPING_NOMINAL = 29,
        TIMEKEEPING_SAFE = 30,

        EPS_DISABLED = 31,
        EPS_NOMINAL = 32,
        EPS_SAFE = 33,

        ALL_TASKS_SAFE = 34,
        ALL_TASKS_NOMINAL = 35,
        ALL_TASKS_DISABLED = 36
    };
}

struct ParsedPacket {
    uint8_t packet_version_number;
    uint8_t packet_type;
    uint8_t secondary_header_flag;
    uint16_t application_process_ID;
    uint16_t packet_data_length;
    uint8_t sequence_flags;
    uint16_t sequence_count;


    explicit ParsedPacket(uint8_t version, uint8_t type, uint8_t header_flag, uint16_t app_id, uint16_t packet_data_length)
        : packet_version_number(version),
          packet_type(type),
          secondary_header_flag(header_flag),
          application_process_ID(app_id),
          packet_data_length(packet_data_length) {}
};

inline bool heartbeatReceived = true;


// TX Queue configuration
constexpr uint8_t TX_QUEUE_ITEM_NUM = 20;
constexpr size_t TX_ITEM_SIZE = sizeof(PacketHandler);
inline QueueHandle_t g_tx_queue;
inline StaticQueue_t g_tx_queue_buffer;
inline uint8_t g_tx_queue_storage_area[TX_QUEUE_ITEM_NUM * TX_ITEM_SIZE] __attribute__((section(".dtcmram_outgoingTMQueueStorageArea")));


// TX Queue configuration
constexpr uint8_t TX_QUEUE_ITEM_NUM_CW = 10;
constexpr size_t TX_ITEM_SIZE_CW = sizeof(PacketHandlerCW);
inline QueueHandle_t g_tx_queue_cw;
inline StaticQueue_t g_tx_queue_buffer_cw;
inline uint8_t g_tx_queue_storage_area_cw[TX_QUEUE_ITEM_NUM * TX_ITEM_SIZE_CW];

inline QueueHandle_t TCQueue;
inline StaticQueue_t TCQueueBuffer;
constexpr uint8_t TCQueueItemNum = 5;
constexpr size_t TCItemSize = sizeof(PacketHandler);
inline uint8_t TCQueueStorageArea[TCQueueItemNum * TCItemSize];


inline uint8_t EPSBuffer[EPS_BUF_SIZE]{};

inline uint8_t g_rx_buf[TX_RX_BUF_SIZE_BYTES] __attribute__((section(".dtcmram_rx_buff"), aligned(4)));


