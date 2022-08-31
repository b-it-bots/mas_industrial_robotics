;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS01 WS04 WS05 START END - location
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
    (on R20 WS01)
    (on F20_20_B WS01)
    (on INSULATION_TAPE WS01)
    (on BEARING_BOX WS01)
    (on SCREW_DRIVER WS01)
    (on MOTOR WS01)
    (on S40_40_B WS01)
    (on AXIS WS01)
    (on S40_40_G WS01)
    (on BRACKET WS01)
    (on TENNIS_BALL WS01)
    (on PRINGLES WS01)
    (on SPONGE WS01)
    (on TOWEL WS01)
    (on SPOON WS01)
    (on DISHWASHER_SOAP WS01)
    (on BROWN_BOX WS01)
    (on CUP WS01)
    (on EYE_GLASSES WS01)
    (on TOOTHBRUSH WS01)
)

(:goal (and
    (on R20 WS04)
    (on F20_20_B WS04)
    (on INSULATION_TAPE WS04)
    (on BEARING_BOX WS04)
    (on SCREW_DRIVER WS04)
    (on MOTOR WS04)
    (on S40_40_B WS04)
    (on AXIS WS04)
    (on S40_40_G WS04)
    (on BRACKET WS04)
    (on TENNIS_BALL WS05)
    (on SPONGE WS05)
    (on TOWEL WS05)
    (on SPOON WS05)
    (on DISHWASHER_SOAP WS05)
    (on BROWN_BOX WS05)
    (on CUP WS05)
    (on EYE_GLASSES WS05)
    (on TOOTHBRUSH WS05)
    )
)

(:metric minimize (total-cost))

)

