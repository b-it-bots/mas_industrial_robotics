;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS01 WS02 WS03 WS04 WS05 WS06 SH01 RT01 PP01 START EXIT - location
    ALLENKEY AXIS2 BEARING2 DRILL F20_20_B F20_20_G HOUSING M20 M20_100 M30 MOTOR2 S40_40_B S40_40_G SCREWDRIVER SPACER WRENCH CONTAINER_RED CONTAINER_BLUE PP01_CAVITY-00 - object
    YOUBOT-BRSU - robot
    PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    ;(on M20 WS06)
    (on M20 WS04)
    (on S40_40_G WS04)
    (on M20_100 WS04)
    (on M30 WS04)
    (container PP01_CAVITY-00)
    (heavy PP01_CAVITY-00)
    (on PP01_CAVITY-00 PP01)
    ;(is_large S40_40_B)
    ;(is_large S40_40_G)
    (is_big_enough PLATFORM_RIGHT)
)

(:goal (and
    (in M20 PP01_CAVITY-00)
    (in M30 PP01_CAVITY-00)
    (in S40_40_G PP01_CAVITY-00)
    (in M20_100 PP01_CAVITY-00)
    ;(on S40_40_G SH01)
    ;(on HOUSING SH01)
    )
)

(:metric minimize (total-cost))

)

