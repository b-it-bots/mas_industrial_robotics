;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS00 WS03 WS05 START - location
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
    (on R20 WS00)
    (on F20_20_B WS00)
    (on INSULATION_TAPE WS00)
    (on BEARING_BOX WS00)
    (on SCREW_DRIVER WS00)
    (on MOTOR WS00)
    (on S40_40_B WS00)
    (on AXIS WS00)
    (on S40_40_G WS00)
    (on BRACKET WS00)
    (on TENNIS_BALL WS00)
    (on PRINGLES WS00)
    (on SPONGE WS00)
    (on TOWEL WS00)
    (on SPOON WS00)
    (on DISHWASHER_SOAP WS00)
    (on BROWN_BOX WS00)
    (on CUP WS00)
    (on EYE_GLASSES WS00)
    (on TOOTHBRUSH WS00)
)

(:goal (and
    (on R20 WS05)
    (on F20_20_B WS05)
    (on INSULATION_TAPE WS05)
    (on BEARING_BOX WS05)
    (on SCREW_DRIVER WS05)
    (on MOTOR WS05)
    (on S40_40_B WS05)
    (on AXIS WS05)
    (on S40_40_G WS05)
    (on BRACKET WS05)
    (on TENNIS_BALL WS03)
    (on PRINGLES WS03)
    (on SPONGE WS03)
    (on TOWEL WS03)
    (on SPOON WS03)
    (on DISHWASHER_SOAP WS03)
    (on BROWN_BOX WS03)
    (on CUP WS03)
    (on EYE_GLASSES WS03)
    (on TOOTHBRUSH WS03)
    )
)

(:metric minimize (total-cost))

)

