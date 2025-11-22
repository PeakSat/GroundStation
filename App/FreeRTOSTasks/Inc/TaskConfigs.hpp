#pragma once

/// Events
#define RXFE_RX (1 << 1)
#define UART (1 << 2)
#define TM_MOD_BEACON (1 << 12)
#define TRANSMIT (1 << 13)
#define RXFE_STATE (1 << 14)
#define TM_OBC (1 << 15)
#define TM_HANDLING (1 << 16)
#define TXFE_RX (1 << 17)
#define TC_UART (1 << 18)
#define TC_RF_RX (1 << 19)
#define TC_UART_TC_HANDLING_TASK (1 << 20)
#define TM_HANDLING_GS (1 << 21)
#define CAN_GATEKEEPER (1 << 22)
#define TM_TX (1 << 23)
#define GNSS_INITIAL_TIME (1 << 24)
#define GNSS_ACK (1 << 25)
#define GNSS_MESSAGE_READY (1 << 26)
#define NOTIFICATION_BIT_GNSS_TASK (1 << 28)
#define EVENT_HANDLER_TASK (1 << 29)
#define TX_CW (1 << 30)
#define RF_ISR_ISR (1 << 31)

/// EVENT GROUPS
#define TM_EVENT_GROUP (TM_OBC_TM_HANDLING | TM_COMMS_TM_HANDLING)

/// Indexes
#define NOTIFY_INDEX_TRANSMIT 1
#define NOTIFY_INDEX_RXFE_RX 2
#define NOTIFY_INDEX_RXFE_RX_STATE 3
#define NOTIFY_INDEX_AGC_RELEASE 4
#define NOTIFY_INDEX_RXFE_TX 5
#define NOTIFY_INDEX_TXFE_TX 6
#define NOTIFY_INDEX_TXFE_RX 7
#define NOTIFY_INDEX_AGC 8
#define NOTIFY_INDEX_RXFS 9
#define NOTIFY_INDEX_INCOMING_TC 10
#define NOTIFY_INDEX_RECEIVED_TM 11
#define NOTIFY_INDEX_TIMER 12
#define NOTIFY_INDEX_TIMEKEEPING 13
#define NOTIFY_INDEX_UART_GATEKEEPER 14
#define NOTIFY_INDEX_CHANGE_TASK_STATE 15
#define NOTIFY_INDEX_CHANGE_STATE_EVENTHANDLER 16
#define NOTIFY_INDEX_GNSS_MESSAGE 17
#define NOTIFY_INDEX_GNSS_ACK 18
#define NOTIFY_INDEX_EVENT_HANDLER 19
#define NOTIFY_INDEX_TRANSMIT_CW 20
#define NOTIFY_INDEX_RF_ISR 21
#define NOTIFY_INDEX_TIMER_HOUSEKEEPING 22
#define NOTIFY_INDEX_INTERNAL_TEMPERATURE 23

const BaseType_t CANParserTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t CANGatekeeperTaskPriority = tskIDLE_PRIORITY + 2;
const BaseType_t GNSSTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t RF_RXTaskPriority = tskIDLE_PRIORITY + 2;
const BaseType_t RF_TXTaskPriority = tskIDLE_PRIORITY + 2;
const BaseType_t TCHandlingTaskPriority = tskIDLE_PRIORITY + 2;
const BaseType_t ParameterMonitoringTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t TMHandlingTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t TestTaskPriority = tskIDLE_PRIORITY + 3;
const BaseType_t UARTGatekeeperTaskPriority = tskIDLE_PRIORITY + 5;
const BaseType_t TimeKeepingTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t StateMachineTaskPriority = tskIDLE_PRIORITY + 3;
const BaseType_t EventActionHandlerTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t RF_ISRTaskPriority = tskIDLE_PRIORITY + 3;
const BaseType_t CWTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t ModBeaconTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t HousekeepingTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t ADCSTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t ADCSHousekeepingTaskPriority = tskIDLE_PRIORITY + 1;
const BaseType_t EPSTaskPriority = tskIDLE_PRIORITY + 1;

const uint16_t CANParserTaskStack = 10000;
const uint16_t CANGatekeeperTaskStack = 7000;
const uint16_t GNSSTaskStack = 6000;
const uint16_t RF_RXTaskStack = 8000;
const uint16_t RF_TXTaskStack = 10000;
const uint16_t RF_ISRTaskStack = 6000;
const uint16_t ADCSTaskStack = 4000;
const uint16_t ADCSHousekeepingTaskStack = 6000;
const uint16_t CWTaskStack = 6000;
const uint16_t EPSTaskStack = 4000;
const uint16_t ModBeaconTaskStack = 5000;
const uint16_t HousekeepingTaskStack = 6000;
const uint16_t TCHandlingTaskStack = 8000;
const uint16_t ParameterMonitoringTaskStack = 6000;
const uint16_t TMHandlingTaskStack = 6000;
const uint16_t TestTaskStack = 4000;
const uint16_t UARTGatekeeperTaskStack = 15000;
const uint16_t TimeKeepingTaskStack = 5000;
const uint16_t StateMachineTaskStack = 5000;
const uint16_t EventActionHandlerTaskStack = 6000;

inline StackType_t UARTGatekeeperTaskbuffer[UARTGatekeeperTaskStack];
inline StackType_t CANParserTaskbuffer[CANParserTaskStack];
inline StackType_t TCHandlingTaskbuffer[TCHandlingTaskStack] __attribute__((section(".ram_d2_data")));
