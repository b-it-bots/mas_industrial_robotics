;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS01 WS02 WS03 WS04 WS05 WS06 SH01 RT01 PP01 START EXIT - location
    R20 F20_20_B INSULATION_TAPE BEARING_BOX SCREW_DRIVER MOTOR S40_40_B AXIS S40_40_G BRACKET - object
    YOUBOT-BRSU - robot
    PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on S40_40_B WS01)
    (on S40_40_G WS01)
    (on BEARING_BOX WS01)
    (is_large S40_40_B)
    (is_large S40_40_G)
    (is_big_enough PLATFORM_RIGHT)
)

(:goal (and
    (on S40_40_B WS03)
    (on S40_40_G WS03)
    (on BEARING_BOX WS03)
    )
)

(:metric minimize (total-cost))

)

