;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
    (:domain general_domain)

    (:objects
        WS01 WS02 WS03 WS04 WS05 WS06 SH01 TT01 PP01 START EXIT - location
        ALLENKEY AXIS2 BEARING2 DRILL F20_20_B F20_20_G HOUSING M20 M20_100 M30 MOTOR2 S40_40_B S40_40_G SCREWDRIVER SPACER WRENCH CONTAINER_BOX_RED CONTAINER_BOX_BLUE PP01_CAVITY-00 - object
        YOUBOT-BRSU - robot
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
    )

    (:init
        ;Cost information starts
        (= (total-cost) 0)
        ;Cost information ends

        (at YOUBOT-BRSU PP01)
        (gripper_is_free YOUBOT-BRSU)
        
        (on ALLENKEY WS01)

        (on M20 WS02)
        (on SCREWDRIVER WS02)

        (on M20_100 WS03)

        (on WRENCH WS04)

        (on BEARING2 TT01)
        (on S40_40_G TT01)

        (on F20_20_G WS05)

        (on MOTOR2 SH01)
        (on S40_40_B SH01)

        (on CONTAINER_BOX_RED WS06)
        (on CONTAINER_BOX_BLUE WS06)
        (container CONTAINER_BOX_RED)
        (heavy CONTAINER_BOX_RED)
        (container CONTAINER_BOX_BLUE)
        (heavy CONTAINER_BOX_BLUE)
        (container PP01_CAVITY-00)
        (heavy PP01_CAVITY-00)
        (on PP01_CAVITY-00 PP01)
        (is_large ALLENKEY)
        (is_large DRILL)
        (is_big_enough PLATFORM_RIGHT)
    )

    (:goal
        (and
            (in ALLENKEY CONTAINER_BOX_BLUE)
            (in M20_100 CONTAINER_BOX_BLUE)
            (in SCREWDRIVER CONTAINER_BOX_RED)
            (in WRENCH CONTAINER_BOX_RED)

            (on MOTOR2 WS04)
            (on S40_40_B WS03)

            (on BEARING2 SH01)
            (on S40_40_G SH01)

            (in F20_20_G PP01_CAVITY-00)
            (in M20 PP01_CAVITY-00)
        )
    )

    (:metric minimize
        (total-cost)
    )

)
