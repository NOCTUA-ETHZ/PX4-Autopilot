px4_add_module(
    MODULE drivers__ie_1k2
    MAIN ie_1k2               # <-- This becomes the NSH command name: "IE_1k2"
    SRCS
        ie_1k2_main.cpp
    DEPENDS
        # for example, if you need px4_work_queue
        px4_work_queue
        # or battery, etc. Add any needed modules/libraries here
)

