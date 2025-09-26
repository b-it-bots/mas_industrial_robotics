;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
    (:domain general_domain)

    (:objects
        WS01 WS02 WS03 START EXIT - location
        DRILL-00 M20_100-00 M30-00 S40_40_B-00 F20_20_B-00 - object
        YOUBOT-BRSU - robot
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
    )

    (:init
        ;Cost information starts
        (= (total-cost) 0)
        ;Cost information ends

        (at YOUBOT-BRSU START)
        (gripper_is_free YOUBOT-BRSU)


        (on M30-00 WS01)

        (on F20_20_B-00 WS01)
        
        (on M20_100-00 WS02)
        
        (on S40_40_B-00 WS03)
        
        (on DRILL-00 WS03)

        
        (is_large ALLENKEY-00)
        (is_big_enough PLATFORM_RIGHT)
    )

    (:goal
        (and
            (on DRILL-00 WS01)
            (on M30-00 WS02)
            (on F20_20_B-00 WS03)
            (on S40_40_B-00 WS02)
            (on M20_100-00 WS01)
        )
    )

    (:metric minimize
        (total-cost)
    )

)
