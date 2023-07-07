;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
    (:domain general_domain)

    (:objects
        WS01 WS02 WS03 WS04 WS05 WS06 SH01 RT01 PP01 START EXIT - location
        ALLENKEY AXIS2 BEARING2 DRILL F20_20_B F20_20_G-00 HOUSING M20-00 M20_100-00 M30-00 MOTOR2-00 S40_40_B S40_40_G-00 SCREWDRIVER SPACER WRENCH CONTAINER_RED CONTAINER_BLUE PP01_CAVITY-00 - object
        YOUBOT-BRSU - robot
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
    )

    (:init
        ;Cost information starts
        (= (total-cost) 0)
        ;Cost information ends

        (at YOUBOT-BRSU START)
        (gripper_is_free YOUBOT-BRSU)
        (on M30-00 WS02)
        (on S40_40_G-00 WS02)
        (on MOTOR2-00 WS02)
        (on ALLENKEY WS02)
        (on WRENCH WS02)
        ;(on M30 WS04)
        ;(container PP01_CAVITY-00)
        ;(heavy PP01_CAVITY-00)
        ;(on PP01_CAVITY-00 PP01)
        ;(is_large S40_40_B)
        ;(is_large S40_40_G)
        (is_big_enough PLATFORM_RIGHT)
    )

    (:goal
        (and
            (on M30-00 WS01)
            (on S40_40_G-00 WS01)
            (on MOTOR2-00 WS01)
            (on ALLENKEY WS01)
            (on WRENCH WS01)
        )
    )

    (:metric minimize
        (total-cost)
    )

)