;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    DRAW01_LOC SH01 CB01 WS01 WS02 START END - location
    R20 M20 M30-00 M30-01 AXIS BEARING - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE PLATFORM_LEFT PLATFORM_RIGHT - robot_platform
    DRAW01 - drawer
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (located_at DRAW01 DRAW01_LOC)
    (inside R20 DRAW01)
    (on M20 WS01)
    (on AXIS WS02)
    (on BEARING SH01)
)

(:goal (and
    (on R20 WS01)
    (inside M20 DRAW01)
    (inside AXIS DRAW01)
    (inside BEARING DRAW01)
    (not (opened DRAW01))
    )
)

(:metric minimize (total-cost))

)

