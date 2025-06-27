add_subdirectory(Inc)

add_library(freertos_tasks)

target_sources(freertos_tasks PRIVATE Src/UARTGatekeeper.cpp Src/RF_RXTask.cpp Src/RF_TXTask.cpp Src/CANGatekeeperTask.cpp Src/CANParserTask.cpp Src/TCHandlingTask.cpp Src/GlobalVariables.cpp Src/Message.cpp)

target_include_directories(freertos_tasks PRIVATE Inc)

target_link_libraries(freertos_tasks INTERFACE freertos_tasks_headers at86rf215_inc etl::etl CAN_inc PUBLIC at86rf215_inc at86rf215 CAN_inc CAN freertos_kernel etl::etl log_common)