;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    SH02 CB01 WS11 WS10 START END - location
    R20 M20 - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE PLATFORM_LEFT PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on R20 SH02)
    (on M20 CB01)
)

(:goal (and
    (on R20 WS10)
    (on M20 WS11)
    )
)

(:metric minimize (total-cost))

)

