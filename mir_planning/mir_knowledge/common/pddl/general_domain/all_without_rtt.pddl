;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS01 WS02 WS03 WS04 WS05 WS06 SH01 TT01 PP01 START EXIT - location
    ALLENKEY AXIS2 BEARING2 DRILL F20_20_B F20_20_G HOUSING M20 M20_100 M30 MOTOR2 S40_40_B S40_40_G SCREWDRIVER SPACER WRENCH CONTAINER_BOX_RED CONTAINER_BOX_BLUE - object
    YOUBOT-BRSU - robot
    PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on ALLENKEY WS01)
    (on HOUSING WS01)
    (on S40_40_G WS01)

    (on AXIS2 WS02)
    (on M20 WS02)
    (on SCREWDRIVER WS02)

    (on BEARING2 WS03)
    (on M20_100 WS03)
    (on SPACER WS03)

    (on DRILL WS04)
    (on M30 WS04)
    (on WRENCH WS04)

    (on F20_20_B WS05)
    (on F20_20_G WS05)

    (on MOTOR2 SH01)
    (on S40_40_B SH01)

    (on CONTAINER_BOX_RED WS06)
    (on CONTAINER_BOX_BLUE WS06)
    (container CONTAINER_BOX_RED)
    (heavy CONTAINER_BOX_RED)
    (container CONTAINER_BOX_BLUE)
    (heavy CONTAINER_BOX_BLUE)
    (is_large ALLENKEY)
    (is_large DRILL)
    (is_big_enough PLATFORM_RIGHT)
)

(:goal (and
    (in ALLENKEY CONTAINER_BOX_BLUE)
    (in AXIS2 CONTAINER_BOX_RED)
    
    (on HOUSING WS05)
    (on MOTOR2 WS02)
    (on SCREWDRIVER WS04)
    (on BEARING2 WS01)
    (on SPACER WS01)
    (on DRILL WS03)
    (on WRENCH WS03)

    (on F20_20_G SH01)
    (on S40_40_G SH01)

    (in F20_20_B PP01_CAVITY-00)
    (in S40_40_B PP01_CAVITY-00)
    (in M30 PP01_CAVITY-00)
    (in M20_100 PP01_CAVITY-00)
    (in M20 PP01_CAVITY-00)
    )
)

(:metric minimize (total-cost))

)


