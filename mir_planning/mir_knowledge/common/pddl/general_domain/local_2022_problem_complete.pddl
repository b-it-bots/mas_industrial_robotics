;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS03 WS05 WS01 START EXIT - location
    R20 F20_20_B INSULATION_TAPE BEARING_BOX SCREW_DRIVER MOTOR S40_40_B AXIS S40_40_G BRACKET TENNIS_BALL PRINGLES SPONGE TOWEL SPOON DISHWASHER_SOAP BROWN_BOX CUP EYE_GLASSES TOOTHBRUSH - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (gripper_is_free YOUBOT-BRSU)
    (on F20_20_B WS03)
    (on AXIS WS03)
)

(:goal (and
    (on F20_20_B WS01)
    (on AXIS WS01)
    )
)

(:metric minimize (total-cost))

)

