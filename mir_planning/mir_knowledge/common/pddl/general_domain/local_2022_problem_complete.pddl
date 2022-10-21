;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    WS00 WS01 WS03 WS05 START - location
    R20 F20_20_B INSULATION_TAPE BEARING_BOX SCREW_DRIVER MOTOR S40_40_B AXIS S40_40_G BRACKET TENNIS_BALL PRINGLES SPONGE TOWEL SPOON DISHWASHER_SOAP BROWN_BOX CUP EYE_GLASSES TOOTHBRUSH - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU WS00)
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
)

(:metric minimize (total-cost))

)

