add_library(freertos_tasks_headers INTERFACE)
target_include_directories(freertos_tasks_headers INTERFACE .)

target_link_libraries(freertos_tasks_headers
        INTERFACE
        at86rf215_inc freertos_kernel)