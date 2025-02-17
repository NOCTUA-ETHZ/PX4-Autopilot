px4_add_module(
    MODULE drivers__ie_1k2
    MAIN ie_1k2               # <-- This becomes the NSH command name: "IE_1k2"
    COMPILE_FLAGS
        -DDEBUG_BUILD
        -O0
    SRCS
        ie_1k2.cpp
        ie_1k2.hpp
    DEPENDS
        px4_work_queue
)

