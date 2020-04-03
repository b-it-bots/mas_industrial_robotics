;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    SH01 CB01 WS01 WS02 START END - location
    R20 M20 M30-00 M30-01 AXIS BEARING - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE PLATFORM_LEFT PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on R20 SH01)
    (on BEARING CB01)
    (on M30-00 WS01)
    (on M20 WS01)
    (on AXIS WS02)
    (on M30-01 WS01)
)

(:goal (and
    (on R20 WS01)
    (on AXIS WS01)
    (on M20 WS02)
    (on M30-00 WS02)
    (on BEARING WS02)
    )
)

(:metric minimize (total-cost))

)

