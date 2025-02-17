px4_add_module(
    MODULE drivers__ie_1k2
    MAIN ie_1k2
    COMPILE_FLAGS
        -Wno-cast-align # TODO: fix and enable
    SRCS
        ie_1k2.cpp
        ie_1k2.hpp
    DEPENDS
        px4_work_queue
)
