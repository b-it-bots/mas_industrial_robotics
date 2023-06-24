;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS01 WS02 WS03 WS04 WS05 WS06 SH01 RT01 PP01 START EXIT - location
    S40_40_B CONTAINER_RED - object
    YOUBOT-BRSU - robot
    PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)

    (on CONTAINER_RED WS03)
    (on S40_40_B WS04)
    (container CONTAINER_RED)
    (heavy CONTAINER_RED)

    (is_big_enough PLATFORM_RIGHT)
)

(:goal (and
    (in S40_40_B CONTAINER_RED)

    )
)

(:metric minimize (total-cost))

)

