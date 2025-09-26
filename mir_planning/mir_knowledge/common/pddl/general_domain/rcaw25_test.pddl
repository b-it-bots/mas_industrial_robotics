;This PDDL problem definition was made automatically from a KB snapshot 
(define (problem general_domain_task) 
    (:domain general_domain) 

    (:objects 
        WS01 WS02 WS03 WS04 WS05 SH01 TT01 PP01 START EXIT - location 
        ALLENKEY-00 AXIS2-00 BEARING2-00 DRILL-00 F20_20_B-00 F20_20_G-00 HOUSING-00 M20-00 M20_100-00 M30-00 MOTOR2-00 S40_40_B-00 S40_40_G-00 SCREWDRIVER-00 SPACER-00 WRENCH-00 CONTAINER_BOX_RED-00 CONTAINER_BOX_BLUE-00 PP01_CAVITY-00 - object 
        YOUBOT-BRSU - robot 
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform 
    ) 

    (:init 
        ;Cost information starts 
        (= (total-cost) 0) 
        ;Cost information ends 

        (at YOUBOT-BRSU START) 
        (gripper_is_free YOUBOT-BRSU) 

        (on S40_40_B-00 WS05) 

        (on M20-00 WS02) 
        (on SCREWDRIVER-00 WS02) 

        (on M20_100-00 WS05) 
        (on WRENCH-00 WS03) 
        
        (on HOUSING-00 WS04)

        (on BEARING2-00 WS01)

        (on S40_40_G-00 TT01)

        (on MOTOR2-00 WS01)
        (on ALLENKEY-00 SH01)

        (on CONTAINER_BOX_RED-00 WS05) 
        (on CONTAINER_BOX_BLUE-00 WS03) 
        (container CONTAINER_BOX_RED-00) 
        (heavy CONTAINER_BOX_RED-00) 
        (container CONTAINER_BOX_BLUE-00) 
        (heavy CONTAINER_BOX_BLUE-00) 
        (container PP01_CAVITY-00) 
        (heavy PP01_CAVITY-00) 
        (on PP01_CAVITY-00 PP01) 
        (is_large ALLENKEY-00) 
        (is_large DRILL-00)
        (insertable ALLENKEY-00)
        (insertable SCREWDRIVER-00)
        (insertable WRENCH-00)
        (insertable M20-00)
        (insertable S40_40_G-00)
        (is_big_enough PLATFORM_RIGHT) 
    ) 

    (:goal 
        (and 
            (in ALLENKEY-00 CONTAINER_BOX_BLUE-00) 
            (in SCREWDRIVER-00 CONTAINER_BOX_RED-00) 
            (in WRENCH-00 CONTAINER_BOX_RED-00) 

            (on MOTOR2-00 WS04) 
            (on S40_40_B-00 WS03) 

            (on BEARING2-00 WS03) 
            (in S40_40_G-00 PP01_CAVITY-00) 

            (on M20_100-00 SH01)

            (on HOUSING-00 WS01)
            (in M20-00 PP01_CAVITY-00) 
        ) 
    ) 

    (:metric minimize 
        (total-cost) 
    ) 

) 