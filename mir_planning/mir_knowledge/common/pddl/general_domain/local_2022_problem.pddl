;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS00 WS03 WS05 START - location
    R20 AXIS - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on R20 WS00)
    (on AXIS WS00)
)

(:goal (and
    (on R20 WS05)
    (on AXIS WS05)
    )
)

(:metric minimize (total-cost))

)
