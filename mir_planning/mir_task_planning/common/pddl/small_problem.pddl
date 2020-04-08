;This PDDL problem definition was made automatically from a KB snapshot
(define (problem general_domain_task)
(:domain general_domain)

(:objects
    CB01 WS02 WS01 SH01 PP01 START END - location
    R20-00 M20-00 BEARING-00 S40_40_G-00 AXIS-00 CONTAINER_BOX_BLUE-00 PP01_CAVITY - object
    YOUBOT-BRSU - robot
    PLATFORM_MIDDLE PLATFORM_LEFT PLATFORM_RIGHT - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (at YOUBOT-BRSU START)
    (container CONTAINER_BOX_BLUE-00)
    (container PP01_CAVITY)
    (gripper_is_free YOUBOT-BRSU)
    (heavy CONTAINER_BOX_BLUE-00)
    (heavy PP01_CAVITY)
    (insertable BEARING-00)
    (insertable AXIS-00)
    (on R20-00 SH01)
    (on M20-00 CB01)
    (on BEARING-00 WS01)
    (on S40_40_G-00 WS02)
    (on AXIS-00 WS02)
    (on CONTAINER_BOX_BLUE-00 WS02)
    (on PP01_CAVITY PP01)
)

(:goal (and
    (on R20-00 WS01)
    (in AXIS-00 PP01_CAVITY)
    (on M20-00 WS02)
    (on S40_40_G-00 SH01)
    (in BEARING-00 CONTAINER_BOX_BLUE-00)
    )
)

(:metric minimize (total-cost))

)

