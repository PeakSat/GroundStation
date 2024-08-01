add_subdirectory(Inc)

add_library(freertos_tasks OBJECT)

target_sources(freertos_tasks PRIVATE Src/TransceiverTask.cpp)

target_link_libraries(freertos_tasks PRIVATE freertos_tasks_headers etl::etl log_common)